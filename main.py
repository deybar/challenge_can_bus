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
        # Memoria fija: 4 slots predefinidos para IDs 0x6F0 a 0x6F3
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

            # Validar limites de tamano (8 a 64 bytes)
            if length > 64 or length < 8:
                self.buffers[can_id] = None
                return None

            # Inicia o reinicia sesion (soporta First Frame repetido/reinicio)
            self.buffers[can_id] = {
                "total_len": length,
                "expected_seq": 1,
                "data": bytearray(payload[2:8])
            }
            return None

        # Consecutive Frame (CF)
        elif frame_type == 0x2:
            session = self.buffers[can_id]
            # Descartar CF huerfano si no hay FF previo
            if session is None:
                return None

            seq = byte0 & 0x0F
            # Descartar si el numero de secuencia no es consecutivo
            if seq != session["expected_seq"]:
                self.buffers[can_id] = None
                return None

            session["data"].extend(payload[1:])
            session["expected_seq"] = (session["expected_seq"] + 1) % 16

            # Verificar si se alcanzo la longitud esperada
            if len(session["data"]) >= session["total_len"]:
                result_bytes = session["data"][:session["total_len"]]
                self.buffers[can_id] = None  # Limpiar estado
                ts_ns = time.monotonic_ns()
                decoded_str = result_bytes.decode("ascii", errors="replace")
                return decoded_str, ts_ns

            return None

        return None


def open_can_socket(interface_name: str) -> socket.socket:
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((interface_name,))
    # Timeout pequeno para permitir emitir stats de forma periodica aunque no lleguen tramas
    sock.settimeout(0.5)
    return sock


def decode_telemetry(payload: bytes):
    v_raw, i_raw, t_raw, status, seq = struct.unpack("<HHBBH", payload[:8])
    return {
        "voltage": round(v_raw * 0.1, 2),
        "current": round(i_raw * 0.01, 2),
        "temp_c": t_raw - 40,
        "enabled": bool(status & 0x01),
        "fault": bool(status & 0x02),
        "derated": bool(status & 0x04),
        "seq": seq,
    }


def decode_fault(payload: bytes):
    return struct.unpack("<BB", payload[:2])


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

    if not args.grader:
        print(f"Escuchando en {args.iface} (Modo Terminal)...")

    try:
        while True:
            # Emitir stats periodicos cada 2 segundos en modo grader
            now = time.monotonic()
            if args.grader and (now - last_stats_time >= 2.0):
                print(json.dumps({"type": "stats", "frames_processed": frames_processed}), flush=True)
                last_stats_time = now

            try:
                raw_frame = sock.recv(16)
            except socket.timeout:
                continue

            if len(raw_frame) < 16:
                continue

            can_id_raw, dlc, payload_padded = struct.unpack(CAN_FRAME_FORMAT, raw_frame)
            can_id = can_id_raw & CAN_SFF_MASK
            data = payload_padded[:dlc]

            # Filtrar ruido (no se cuenta)
            if NOISE_ID_MIN <= can_id <= NOISE_ID_MAX:
                continue

            # 1. Telemetria (0x100 - 0x103)
            if ID_TELEMETRY_MIN <= can_id <= ID_TELEMETRY_MAX:
                if len(data) >= 8:
                    frames_processed += 1
                    mod_id = can_id - ID_TELEMETRY_MIN
                    t = decode_telemetry(data)
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
                    else:
                        print(f"[TELEMETRIA] Mod {mod_id}: {t['voltage']}V | {t['current']}A | {t['temp_c']}C")

            # 2. Codigos de falla (0x1F0)
            elif can_id == ID_FAULT:
                if len(data) >= 2:
                    frames_processed += 1
                    mod_id, code = decode_fault(data)
                    if args.grader:
                        out = {
                            "type": "fault",
                            "module": mod_id,
                            "code": code
                        }
                        print(json.dumps(out), flush=True)
                    else:
                        print(f"[FALLA] Mod {mod_id}: codigo {code}")

            # 3. Identificacion multi-trama (0x6F0 - 0x6F3)
            elif ID_DIAG_MIN <= can_id <= ID_DIAG_MAX:
                frames_processed += 1
                res = reassembler.process(can_id, data)
                if res is not None:
                    decoded_str, ts_ns = res
                    can_id_hex = f"0x{can_id:x}"
                    if args.grader:
                        out = {
                            "type": "diag_complete",
                            "can_id": can_id_hex,
                            "string": decoded_str,
                            "ts_ns": ts_ns
                        }
                        print(json.dumps(out), flush=True)
                    else:
                        print(f"[DIAG] Modulo {can_id_hex}: {decoded_str}")

    except KeyboardInterrupt:
        pass
    finally:
        # Emitir stats finales antes de salir
        if args.grader:
            print(json.dumps({"type": "stats", "frames_processed": frames_processed}), flush=True)
        else:
            print(f"\nTotal tramas procesadas: {frames_processed}")
        sock.close()


if __name__ == "__main__":
    main()
