#!/usr/bin/env python3
"""
Simple INAV Battery Monitor using yamspy
Shows battery voltage, current, capacity and percentage
"""

import argparse
import sys
import time

try:
    from yamspy import MSPy
except ImportError:
    print("ERROR: yamspy not installed. Install with: pip install yamspy", file=sys.stderr)
    sys.exit(1)

DEFAULT_DEVICE = "/dev/ttyAMA0"
DEFAULT_BAUD = 115200


def main():
    parser = argparse.ArgumentParser(description="Monitor INAV battery via MSP")
    parser.add_argument("--device", default=DEFAULT_DEVICE, help=f"Serial device (default: {DEFAULT_DEVICE})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate (default: {DEFAULT_BAUD})")
    parser.add_argument("--poll-hz", type=float, default=2.0, help="Polling rate in Hz (default: 2.0)")
    parser.add_argument("--debug", action="store_true", help="Show debug info")
    
    args = parser.parse_args()
    poll_interval = 1.0 / args.poll_hz
    
    print(f"Connecting to {args.device} @ {args.baud} baud...")
    
    board = MSPy(device=args.device, baudrate=args.baud, loglevel="DEBUG" if args.debug else "WARNING")
    
    if board.connect() != 0:
        print("ERROR: Failed to connect to flight controller", file=sys.stderr)
        return 1
    
    print(f"Connected to {board.MSPFW_VARIANT} {board.MSPFW_VERSION}")
    print(f"Target: {board.CONFIG.get('targetName', 'Unknown')}")
    print(f"Polling every {poll_interval:.2f}s. Press Ctrl+C to stop.\n")
    
    try:
        while True:
            # Request ANALOG data (MSP command 110)
            if board.send_RAW_msg(MSPy.MSPCodes['MSP_ANALOG'], data_length=0, data=[]):
                time.sleep(0.05)  # Wait for response
                
                if hasattr(board, 'ANALOG') and board.ANALOG:
                    analog = board.ANALOG
                    
                    # Parse the data - yamspy stores it in a dict
                    voltage = analog.get('voltage', 0) / 10.0  # decivolts to volts
                    mah_drawn = analog.get('mAhdrawn', 0)
                    rssi = analog.get('rssi', 0)
                    amperage = analog.get('amperage', 0) / 100.0  # centiamps to amps
                    
                    # Try to get battery state info
                    cell_count = board.CONFIG.get('voltage_sensor_adc_channel', 0)  # This might not work
                    
                    # Estimate cell count from voltage (rough)
                    if voltage > 0:
                        estimated_cells = int(voltage / 4.2) + 1
                        if estimated_cells in [7, 9, 11]:  # No odd counts above 6S
                            estimated_cells += 1
                        estimated_cells = min(estimated_cells, 12)
                        cell_voltage = voltage / estimated_cells if estimated_cells > 0 else 0
                        
                        # Rough percentage calculation (3.3V = 0%, 4.2V = 100%)
                        percentage = ((cell_voltage - 3.3) / (4.2 - 3.3)) * 100.0
                        percentage = max(0, min(100, percentage))
                        
                        print(
                            f"🔋 ~{percentage:5.1f}% │ "
                            f"{voltage:5.2f}V (~{cell_voltage:.3f}V/cell, ~{estimated_cells}S) │ "
                            f"{amperage:5.2f}A │ "
                            f"Used: {mah_drawn:5.0f}mAh │ "
                            f"RSSI: {rssi}"
                        )
                    else:
                        print("⚠ No battery detected (voltage = 0)")
                else:
                    print("⚠ No ANALOG response", file=sys.stderr)
            else:
                print("⚠ Failed to send MSP command", file=sys.stderr)
            
            time.sleep(poll_interval)
    
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        try:
            board.close()
        except:
            pass
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
