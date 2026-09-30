#!/usr/bin/env python3
import argparse
import json
import socket
import struct
import sys
import time

CAN_FRAME_FORMAT = "<IB3x8s"
CAN_SFF_MASK = 0x000007FF

NOISE_ID_MIN = 0x200
NOISE_ID_MAX = 0x2FF

ID_TELEMETRY_MIN = 0x100
ID_TELEMETRY_MAX = 0x103
ID_FAULT = 0x1F0
ID_DIAG_MIN = 0x6F0
ID_DIAG_MAX = 0x6F3


class IsoTpReassembler:
    """Reensambla tramas multi-frame acotado a 4 modulos."""
    def __init__(self):
        self.buffers = {
            0x6F0: None,
            0x6F1: None,
            0x6F2: None,
            0x6F3: None,
        }

    def process(self, can_id: int, payload: bytes):
        if can_id not in self.buffers or not payload:
            return None

        byte0 = payload[0]
        frame_type = (byte0 & 0xF0) >> 4

        # First Frame (FF)
        if frame_type == 0x1:
            if len(payload) < 8:
                self.buffers[can_id] = None
                return None

            length = ((byte0 & 0x0F) << 8) | payload[1]

            if length > 64 or length < 8:
                self.buffers[can_id] = None
                return None

            self.buffers[can_id] = {
                "total_len": length,
                "expected_seq": 1,
                "data": bytearray(payload[2:8])
            }
            return None

        # Consecutive Frame (CF)
        elif frame_type == 0x2:
            session = self.buffers[can_id]
            if session is None:
                return None

            seq = byte0 & 0x0F
            if seq != session["expected_seq"]:
                self.buffers[can_id] = None
                return None

            session["data"].extend(payload[1:])
            session["expected_seq"] = (session["expected_seq"] + 1) % 16

            if len(session["data"]) >= session["total_len"]:
                result_bytes = session["data"][:session["total_len"]]
                self.buffers[can_id] = None
                ts_ns = time.monotonic_ns()
                decoded_str = result_bytes.decode("ascii", errors="replace")
                return decoded_str, ts_ns

            return None

        return None


def open_can_socket(interface_name: str) -> socket.socket:
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((interface_name,))
    sock.settimeout(0.2)
    return sock


def decode_telemetry(payload: bytes):
    v_raw, i_raw, t_raw, status, seq = struct.unpack("<HHBBH", payload[:8])
    return {
        "voltage": round(v_raw * 0.1, 1),
        "current": round(i_raw * 0.01, 2),
        "temp_c": t_raw - 40,
        "enabled": bool(status & 0x01),
        "fault": bool(status & 0x02),
        "derated": bool(status & 0x04),
        "seq": seq,
    }


def decode_fault(payload: bytes):
    return struct.unpack("<BB", payload[:2])


def render_dashboard(telemetry, diags, faults):
    """Renderiza el dashboard en consola para el modo normal."""
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.write("DeepSea CAN Diagnostic Tool\n\n")
    sys.stdout.write("--------------------------\n")
    for i in range(4):
        m = telemetry.get(i)
        if m:
            sys.stdout.write(
                f"Module {i}:  {m['voltage']:>5.1f}V  {m['current']:>6.2f}A   {m['temp_c']:>2}C   "
                f"enabled={str(m['enabled']):<5} fault={str(m['fault']):<5}\n"
            )
        else:
            sys.stdout.write(f"Module {i}:  (esperando telemetria...)\n")
    sys.stdout.write("---------------------------\n")
    sys.stdout.write("Identification strings:\n")
    for can_id in range(0x6F0, 0x6F4):
        can_hex = f"0x{can_id:x}"
        sys.stdout.write(f"  {can_hex}: {diags[can_hex]}\n")
    sys.stdout.write("---------------------------\n")
    sys.stdout.write("Recent faults:\n")
    if faults:
        for mod, code in reversed(faults[-5:]):
            sys.stdout.write(f"  module {mod}, code {code}\n")
    else:
        sys.stdout.write("  (ninguna)\n")
    sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(description="DeepSea CAN Diagnostic Tool")
    parser.add_argument("--iface", default="vcan0", help="Interfaz CAN")
    parser.add_argument("--grader", action="store_true", help="Modo evaluacion")
    args = parser.parse_args()

    try:
        sock = open_can_socket(args.iface)
    except OSError as e:
        print(f"Error al abrir la interfaz {args.iface}: {e}", file=sys.stderr)
        sys.exit(1)

    reassembler = IsoTpReassembler()
    frames_processed = 0
    last_stats_time = time.monotonic()
    last_ui_time = time.monotonic()

    telemetry_state = {}
    faults_state = []
    diag_state = {f"0x{can_id:x}": "(not yet received)" for can_id in range(0x6F0, 0x6F4)}

    try:
        while True:
            now = time.monotonic()

            # Modo Grader: stats periodicos cada 2s
            if args.grader and (now - last_stats_time >= 2.0):
                print(json.dumps({"type": "stats", "frames_processed": frames_processed}), flush=True)
                last_stats_time = now

            # Modo Normal: refrescar pantalla a 5 Hz
            if not args.grader and (now - last_ui_time >= 0.2):
                render_dashboard(telemetry_state, diag_state, faults_state)
                last_ui_time = now

            try:
                raw_frame = sock.recv(16)
            except socket.timeout:
                continue

            if len(raw_frame) < 16:
                continue

            can_id_raw, dlc, payload_padded = struct.unpack(CAN_FRAME_FORMAT, raw_frame)
            can_id = can_id_raw & CAN_SFF_MASK
            data = payload_padded[:dlc]

            # Filtrar ruido (no se procesa ni contabiliza)
            if NOISE_ID_MIN <= can_id <= NOISE_ID_MAX:
                continue

            # 1. Telemetria (0x100 - 0x103)
            if ID_TELEMETRY_MIN <= can_id <= ID_TELEMETRY_MAX:
                if len(data) >= 8:
                    frames_processed += 1
                    mod_id = can_id - ID_TELEMETRY_MIN
                    t = decode_telemetry(data)
                    telemetry_state[mod_id] = t
                    if args.grader:
                        out = {
                            "type": "telemetry",
                            "module": mod_id,
                            "seq": t["seq"],
                            "voltage": t["voltage"],
                            "current": t["current"],
                            "temp_c": t["temp_c"],
                            "enabled": t["enabled"],
                            "fault": t["fault"],
                            "derated": t["derated"]
                        }
                        print(json.dumps(out), flush=True)

            # 2. Codigos de falla (0x1F0)
            elif can_id == ID_FAULT:
                if len(data) >= 2:
                    frames_processed += 1
                    mod_id, code = decode_fault(data)
                    faults_state.append((mod_id, code))
                    if len(faults_state) > 10:
                        faults_state.pop(0)
                    if args.grader:
                        out = {
                            "type": "fault",
                            "module": mod_id,
                            "code": code
                        }
                        print(json.dumps(out), flush=True)

            # 3. Diagnostico multi-trama (0x6F0 - 0x6F3)
            elif ID_DIAG_MIN <= can_id <= ID_DIAG_MAX:
                frames_processed += 1
                res = reassembler.process(can_id, data)
                if res is not None:
                    decoded_str, ts_ns = res
                    can_id_hex = f"0x{can_id:x}"
                    diag_state[can_id_hex] = decoded_str
                    if args.grader:
                        out = {
                            "type": "diag_complete",
                            "can_id": can_id_hex,
                            "string": decoded_str,
                            "ts_ns": ts_ns
                        }
                        print(json.dumps(out), flush=True)

    except KeyboardInterrupt:
        pass
    finally:
        if args.grader:
            print(json.dumps({"type": "stats", "frames_processed": frames_processed}), flush=True)
        sock.close()


if __name__ == "__main__":
    main()
