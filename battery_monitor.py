#!/usr/bin/env python3
"""INAV Battery Monitor with Percentage Display

This script connects to an INAV flight controller via MSP and displays battery
information including the percentage calculated by INAV itself.

Based on INAV source code analysis:
- MSP_ANALOG (0x6E/110): Legacy message with basic battery info
- MSP2_INAV_ANALOG (0x2002/8194): Extended message with battery percentage
"""

import argparse
import struct
import sys
import time
from typing import Optional, Tuple, Dict

try:
    from yamspy import MSPy
except ImportError:
    print("ERROR: yamspy not installed. Install with: pip install yamspy", file=sys.stderr)
    sys.exit(1)

# MSP Command Codes
MSP_ANALOG = 110  # 0x6E
MSP2_INAV_ANALOG = 0x2002  # 8194

DEFAULT_DEVICE = "/dev/ttyAMA0"
DEFAULT_BAUD = 115200


def build_msp_request(command: int, v2: bool = False) -> bytes:
    """Construct MSP request frame (host->FC).
    
    Args:
        command: MSP command code
        v2: If True, use MSPv2 framing, else use MSPv1
    """
    payload = b""
    size = len(payload)
    
    if v2:
        # MSPv2 format: $X< + flag + cmd(16bit) + size(16bit) + payload + crc
        flag = 0
        frame = b"$X<" + bytes([flag])
        frame += struct.pack("<HH", command, size)
        frame += payload
        # CRC8_DVB_S2
        crc = 0
        for b in frame[3:]:
            crc ^= b
        return frame + bytes([crc])
    else:
        # MSPv1 format: $M< + size + cmd + payload + checksum
        frame = b"$M<" + bytes([size, command]) + payload
        checksum = size ^ command
        for b in payload:
            checksum ^= b
        return frame + bytes([checksum & 0xFF])


def parse_msp_analog(payload: bytes) -> Optional[Dict]:
    """Parse MSP_ANALOG response (legacy format).
    
    Format (INAV):
    - uint8:  voltage in decivolts (divide by 10 to get volts)
    - uint16: mAh drawn
    - uint16: RSSI
    - int16:  amperage in centiamps (divide by 100 to get amps)
    
    Returns dict with parsed values or None on error.
    """
    if len(payload) < 7:
        return None
    
    voltage_dv = payload[0]  # decivolts
    mah_drawn = struct.unpack_from("<H", payload, 1)[0]
    rssi = struct.unpack_from("<H", payload, 3)[0]
    amperage_ca = struct.unpack_from("<h", payload, 5)[0]  # signed
    
    return {
        'voltage': voltage_dv / 10.0,  # convert to volts
        'mah_drawn': mah_drawn,
        'rssi': rssi,
        'amperage': amperage_ca / 100.0,  # convert to amps
        'percentage': None  # not available in MSP_ANALOG
    }


def parse_msp2_inav_analog(payload: bytes) -> Optional[Dict]:
    """Parse MSP2_INAV_ANALOG response (extended format).
    
    Format (INAV):
    - uint8:  battery flags (bit 0: full when plugged, bit 1: use capacity thresholds, 
                             bits 2-3: battery state, bits 4-7: cell count)
    - uint16: voltage in centivolts (divide by 100 to get volts)
    - uint16: amperage in centiamps (divide by 100 to get amps)
    - uint32: power draw in centiwatts
    - uint32: mAh drawn
    - uint32: mWh drawn
    - uint32: remaining capacity in mAh
    - uint8:  battery percentage (0-100)
    - uint16: RSSI
    
    Returns dict with parsed values or None on error.
    """
    if len(payload) < 23:
        return None
    
    offset = 0
    flags = payload[offset]
    offset += 1
    
    voltage_cv = struct.unpack_from("<H", payload, offset)[0]
    offset += 2
    
    amperage_ca = struct.unpack_from("<H", payload, offset)[0]
    offset += 2
    
    power_cw = struct.unpack_from("<I", payload, offset)[0]
    offset += 4
    
    mah_drawn = struct.unpack_from("<I", payload, offset)[0]
    offset += 4
    
    mwh_drawn = struct.unpack_from("<I", payload, offset)[0]
    offset += 4
    
    remaining_capacity = struct.unpack_from("<I", payload, offset)[0]
    offset += 4
    
    percentage = payload[offset]
    offset += 1
    
    rssi = struct.unpack_from("<H", payload, offset)[0]
    
    # Parse flags
    battery_full = (flags & 0x01) != 0
    use_capacity = (flags & 0x02) != 0
    battery_state = (flags >> 2) & 0x03
    cell_count = (flags >> 4) & 0x0F
    
    return {
        'voltage': voltage_cv / 100.0,  # convert to volts
        'amperage': amperage_ca / 100.0,  # convert to amps
        'power': power_cw / 100.0,  # convert to watts
        'mah_drawn': mah_drawn,
        'mwh_drawn': mwh_drawn,
        'remaining_capacity': remaining_capacity,
        'percentage': percentage,
        'rssi': rssi,
        'cell_count': cell_count,
        'battery_full': battery_full,
        'use_capacity': use_capacity,
        'battery_state': battery_state,
        'cell_voltage': voltage_cv / 100.0 / cell_count if cell_count > 0 else 0.0
    }


class BatteryMonitor:
    """Monitor battery status via MSP."""
    
    def __init__(self, device: str, baud: int, poll_hz: float, use_v2: bool = True, debug: bool = False):
        self.device = device
        self.baud = baud
        self.poll_interval = 1.0 / poll_hz if poll_hz > 0 else 1.0
        self.use_v2 = use_v2
        self.debug = debug
        
        print(f"Connecting to {device} @ {baud} baud...")
        self.board = MSPy(device=device, baudrate=baud, loglevel="DEBUG" if debug else "WARNING")
        
        if self.board.connect() != 0:
            raise RuntimeError("Failed to connect to flight controller")
        
        print(f"Connected! Using {'MSPv2_INAV_ANALOG' if use_v2 else 'MSP_ANALOG'}")
        
        # Test if we can get basic status
        if self.debug:
            print("[DEBUG] Flight controller info:", self.board.MSPFW_VARIANT)
    
    def close(self):
        try:
            self.board.close()
        except Exception:
            pass
    
    def poll_once_yamspy(self) -> Optional[Dict]:
        """Poll using yamspy's built-in methods (simpler)."""
        try:
            # Try to send raw MSP command
            if self.use_v2:
                # MSPv2 - let's try using yamspy's send_RAW_msg
                command_code = MSP2_INAV_ANALOG
                data_length = 0
                data = []
                
                if self.board.send_RAW_msg(MSPy.MSPCodes2['MSP2_INAV_ANALOG'], data_length, data):
                    time.sleep(0.1)
                    # Check if response arrived
                    if hasattr(self.board, 'MSP2_INAV_ANALOG'):
                        response_data = self.board.MSP2_INAV_ANALOG
                        if self.debug:
                            print(f"[DEBUG] Got response from yamspy: {response_data}")
                        # Parse the response
                        if response_data and len(response_data) >= 23:
                            return parse_msp2_inav_analog(bytes(response_data))
            else:
                # MSPv1 - try ANALOG
                if self.board.send_RAW_msg(MSPy.MSPCodes['MSP_ANALOG'], 0, []):
                    time.sleep(0.1)
                    if hasattr(self.board, 'ANALOG'):
                        response_data = self.board.ANALOG
                        if self.debug:
                            print(f"[DEBUG] Got ANALOG response: {response_data}")
                        if response_data and 'voltage' in response_data:
                            # yamspy might parse it already
                            return {
                                'voltage': response_data.get('voltage', 0) / 10.0,
                                'mah_drawn': response_data.get('mAhdrawn', 0),
                                'rssi': response_data.get('rssi', 0),
                                'amperage': response_data.get('amperage', 0) / 100.0,
                                'percentage': None
                            }
        except Exception as e:
            if self.debug:
                print(f"[DEBUG] yamspy method error: {e}")
            pass
        
        return None
    
    def poll_once(self) -> Optional[Dict]:
        """Poll battery telemetry once."""
        ser = self.board.conn
        ser.reset_input_buffer()
        
        # Build and send request
        if self.use_v2:
            frame = build_msp_request(MSP2_INAV_ANALOG, v2=True)
        else:
            frame = build_msp_request(MSP_ANALOG, v2=False)
        
        if self.debug:
            print(f"[DEBUG] Sending ({len(frame)} bytes): {frame.hex()}")
        
        ser.write(frame)
        ser.flush()
        
        # Wait for response
        time.sleep(0.1)  # Increased wait time
        available = ser.in_waiting
        if available <= 0:
            if self.debug:
                print("[DEBUG] No data available")
            return None
        
        response = ser.read(available)
        
        if self.debug:
            print(f"[DEBUG] Response ({len(response)} bytes): {response.hex()}")
        
        # Parse response
        if self.use_v2:
            # MSPv2 response: $X> + flag + cmd(16bit) + size(16bit) + payload + crc
            frame_start = response.find(b"$X>")
            if frame_start == -1 or len(response) < frame_start + 8:
                if self.debug:
                    print("[DEBUG] MSPv2 frame not found or incomplete")
                return None
            
            flag = response[frame_start + 3]
            cmd = struct.unpack_from("<H", response, frame_start + 4)[0]
            size = struct.unpack_from("<H", response, frame_start + 6)[0]
            
            if cmd != MSP2_INAV_ANALOG:
                if self.debug:
                    print(f"[DEBUG] Wrong command in response: {cmd}")
                return None
            
            payload_start = frame_start + 8
            payload_end = payload_start + size
            
            if len(response) < payload_end:
                if self.debug:
                    print(f"[DEBUG] Incomplete payload: expected {size}, got {len(response) - payload_start}")
                return None
            
            payload = response[payload_start:payload_end]
            return parse_msp2_inav_analog(payload)
        
        else:
            # MSPv1 response: $M> + size + cmd + payload + checksum
            frame_start = response.find(b"$M>")
            if frame_start == -1 or len(response) < frame_start + 5:
                return None
            
            size = response[frame_start + 3]
            cmd = response[frame_start + 4]
            
            if cmd != MSP_ANALOG:
                return None
            
            payload_start = frame_start + 5
            payload_end = payload_start + size
            
            if len(response) < payload_end:
                return None
            
            payload = response[payload_start:payload_end]
            return parse_msp_analog(payload)
    
    def run(self):
        """Main monitoring loop."""
        print(f"Polling every {self.poll_interval:.2f}s. Press Ctrl+C to stop.\n")
        
        # Try yamspy method first, then fall back to manual
        use_yamspy = True
        
        try:
            while True:
                data = None
                
                # Try yamspy's built-in method first
                if use_yamspy:
                    data = self.poll_once_yamspy()
                
                # Fall back to manual MSP parsing
                if data is None:
                    data = self.poll_once()
                
                if data is None:
                    print("⚠ No response from flight controller", file=sys.stderr)
                else:
                    if self.use_v2 and data.get('percentage') is not None:
                        # Rich output with percentage
                        state_names = ["OK", "WARNING", "CRITICAL", "NOT_PRESENT"]
                        state_name = state_names[data['battery_state']] if data['battery_state'] < 4 else "UNKNOWN"
                        
                        print(
                            f"🔋 {data['percentage']:3d}% │ "
                            f"{data['voltage']:5.2f}V ({data['cell_voltage']:.3f}V/cell) │ "
                            f"{data['amperage']:5.2f}A │ "
                            f"{data['power']:6.1f}W │ "
                            f"Used: {data['mah_drawn']:5.0f}mAh │ "
                            f"Remain: {data['remaining_capacity']:5.0f}mAh │ "
                            f"Cells: {data['cell_count']}S │ "
                            f"State: {state_name}"
                        )
                    else:
                        # Basic output without percentage
                        print(
                            f"Voltage: {data['voltage']:5.2f}V │ "
                            f"Current: {data['amperage']:5.2f}A │ "
                            f"Used: {data['mah_drawn']:5.0f}mAh"
                        )
                
                time.sleep(self.poll_interval)
        
        except KeyboardInterrupt:
            print("\n\nStopping battery monitor...")
        finally:
            self.close()


def main():
    parser = argparse.ArgumentParser(
        description="Monitor INAV battery telemetry via MSP",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                                    # Use defaults
  %(prog)s --device /dev/ttyUSB0 --baud 57600  # Custom serial port
  %(prog)s --legacy                           # Use old MSP_ANALOG instead of MSPv2
  %(prog)s --poll-hz 5                        # Poll 5 times per second
  %(prog)s --debug                            # Show raw MSP frames
        """
    )
    
    parser.add_argument("--device", default=DEFAULT_DEVICE,
                        help=f"Serial device (default: {DEFAULT_DEVICE})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help=f"Baud rate (default: {DEFAULT_BAUD})")
    parser.add_argument("--poll-hz", type=float, default=2.0,
                        help="Polling frequency in Hz (default: 2.0)")
    parser.add_argument("--legacy", action="store_true",
                        help="Use MSP_ANALOG instead of MSP2_INAV_ANALOG (no percentage)")
    parser.add_argument("--debug", action="store_true",
                        help="Show debug output including raw MSP frames")
    
    args = parser.parse_args()
    
    try:
        monitor = BatteryMonitor(
            device=args.device,
            baud=args.baud,
            poll_hz=args.poll_hz,
            use_v2=not args.legacy,
            debug=args.debug
        )
        monitor.run()
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
