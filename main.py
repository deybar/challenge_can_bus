#!/usr/bin/env python3
"""!
@file main.py
@brief Diagnostic and telemetry monitoring tool for Level 3 DC Fast Chargers.
@details Implements direct raw SocketCAN (AF_CAN) communication using exclusively
         the Python standard library. Handles real-time telemetry extraction,
         fault monitoring, bounded-memory ISO-TP diagnostic reassembly, noise
         filtering, and NDJSON streaming for evaluation environments.
@version 1.0.0
@author Deybar Mora
@date 2026-09-30
"""

import argparse
import json
import socket
import struct
import sys
import time

## @brief Binary layout for standard Linux struct can_frame: ID (4B), DLC (1B), 3B padding, 8B payload.
CAN_FRAME_FORMAT = "<IB3x8s"

## @brief Bitmask for extracting standard 11-bit CAN identifiers.
CAN_SFF_MASK = 0x000007FF

## @brief Minimum CAN ID for the electrical noise injection range.
NOISE_ID_MIN = 0x200

## @brief Maximum CAN ID for the electrical noise injection range.
NOISE_ID_MAX = 0x2FF

## @brief Starting CAN identifier for power module telemetry (Module 0).
ID_TELEMETRY_MIN = 0x100

## @brief Ending CAN identifier for power module telemetry (Module 3).
ID_TELEMETRY_MAX = 0x103

## @brief CAN identifier designated for power module fault code frames.
ID_FAULT = 0x1F0

## @brief Starting CAN identifier for multi-frame identification strings (Module 0).
ID_DIAG_MIN = 0x6F0

## @brief Ending CAN identifier for multi-frame identification strings (Module 3).
ID_DIAG_MAX = 0x6F3


class IsoTpReassembler:
    """!
    @brief ISO 15765-2 (ISO-TP) multi-frame reassembler with strictly bounded memory.
    @details Implements a deterministic state machine to reconstruct multi-frame
             diagnostic identification strings for up to 4 power modules. Resilient
             against orphan consecutive frames, frame restarts, oversized length claims,
             sequence anomalies, and repeated message abandonments.
    """

    def __init__(self):
        """!
        @brief Initializes the reassembler with pre-allocated static slots.
        @note Spatial complexity is bounded to O(1) by maintaining fixed slots
              exclusively for CAN IDs 0x6F0 through 0x6F3.
        """
        self.buffers = {
            0x6F0: None,
            0x6F1: None,
            0x6F2: None,
            0x6F3: None,
        }

    def process(self, can_id: int, payload: bytes):
        """!
        @brief Processes incoming CAN frame payloads for multi-frame reassembly.
        @param can_id Standard 11-bit CAN identifier of the transmitting module.
        @param payload Raw byte array containing frame payload (up to 8 bytes).
        @return A tuple of `(decoded_string, timestamp_ns)` upon complete message
                assembly; `None` if the frame was consumed, rejected, or incomplete.
        @note Abandons in-progress reassembly without side effects upon protocol errors.
        """
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

            # Reject invalid length bounds (must be between 8 and 64 bytes)
            if length > 64 or length < 8:
                self.buffers[can_id] = None
                return None

            # Initialize or reset session (handles mid-message restarts)
            self.buffers[can_id] = {
                "total_len": length,
                "expected_seq": 1,
                "data": bytearray(payload[2:8])
            }
            return None

        # Consecutive Frame (CF)
        elif frame_type == 0x2:
            session = self.buffers[can_id]
            # Drop orphan consecutive frames with no prior First Frame
            if session is None:
                return None

            seq = byte0 & 0x0F
            # Drop attempt on out-of-order sequence counter
            if seq != session["expected_seq"]:
                self.buffers[can_id] = None
                return None

            session["data"].extend(payload[1:])
            session["expected_seq"] = (session["expected_seq"] + 1) % 16

            # Check if all declared bytes have been received
            if len(session["data"]) >= session["total_len"]:
                result_bytes = session["data"][:session["total_len"]]
                self.buffers[can_id] = None
                ts_ns = time.monotonic_ns()
                decoded_str = result_bytes.decode("ascii", errors="replace")
                return decoded_str, ts_ns

            return None

        return None


def open_can_socket(interface_name: str) -> socket.socket:
    """!
    @brief Configures and binds a raw SocketCAN interface.
    @param interface_name Network interface identifier (e.g., 'vcan0').
    @return An initialized and bound raw CAN socket object.
    @throws OSError If network interface binding fails.
    """
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((interface_name,))
    sock.settimeout(0.2)
    return sock


def decode_telemetry(payload: bytes) -> dict:
    """!
    @brief Decodes 8-byte little-endian telemetry payload from a power module.
    @param payload Byte slice containing raw telemetry data.
    @return Dictionary containing parsed engineering values:
            - `voltage` (float): Module voltage in Volts (raw * 0.1).
            - `current` (float): Module current in Amperes (raw * 0.01).
            - `temp_c` (int): Internal module temperature in Celsius (raw - 40).
            - `enabled` (bool): Module active state flag.
            - `fault` (bool): Module fault trip flag.
            - `derated` (bool): Module thermal derating flag.
            - `seq` (int): Incrementing transmission sequence counter.
    """
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


def decode_fault(payload: bytes) -> tuple:
    """!
    @brief Decodes fault notification payload.
    @param payload Byte slice containing module identifier and numeric fault code.
    @return Tuple of `(module_id, fault_code)`.
    """
    return struct.unpack("<BB", payload[:2])


def render_dashboard(telemetry: dict, diags: dict, faults: list) -> None:
    """!
    @brief Renders a clean in-place terminal dashboard using ANSI escape codes.
    @param telemetry Dictionary storing the latest telemetry packet per module index.
    @param diags Dictionary mapping CAN IDs to reassembled diagnostic strings.
    @param faults List storing tuples of recent faults (module_id, code).
    """
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


def main() -> None:
    """!
    @brief Application entry point. Parses CLI options and runs CAN event loop.
    @details Dispatches between interactive terminal dashboard view and
             machine-readable NDJSON streaming depending on `--grader` flag.
    """
    parser = argparse.ArgumentParser(description="DeepSea CAN Diagnostic Tool")
    parser.add_argument("--iface", default="vcan0", help="CAN network interface")
    parser.add_argument("--grader", action="store_true", help="Automated grader NDJSON mode")
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

            # Emit periodic stats line every 2.0s when running under grader mode
            if args.grader and (now - last_stats_time >= 2.0):
                print(json.dumps({"type": "stats", "frames_processed": frames_processed}), flush=True)
                last_stats_time = now

            # Refresh terminal UI at 5 Hz (200 ms) in normal dashboard mode
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

            # Filter out bus noise (0x200 - 0x2FF): never process nor count
            if NOISE_ID_MIN <= can_id <= NOISE_ID_MAX:
                continue

            # 1. Telemetry decoding (0x100 - 0x103)
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

            # 2. Fault code decoding (0x1F0)
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

            # 3. Multi-frame ISO-TP diagnostic decoding (0x6F0 - 0x6F3)
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
