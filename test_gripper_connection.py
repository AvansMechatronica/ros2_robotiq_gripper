#!/usr/bin/env python3
"""
Diagnostic tool for testing Robotiq gripper serial communication.
This script helps identify connection issues before launching the ROS2 driver.
"""

import serial
import time
import sys

def calc_crc(data):
    """Calculate Modbus RTU CRC-16"""
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if (crc & 0x0001):
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

def test_connection(port, baudrate, slave_id):
    """Test connection to Robotiq gripper"""
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1.0,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        
        print(f"✓ Serial port {port} opened successfully")
        print(f"  Baudrate: {baudrate}")
        print(f"  Timeout: {ser.timeout}s")
        
        # Modbus RTU read command: read holding registers
        function = 0x03      # Read holding registers
        address = 0x07D0     # First output register (2000 in decimal)
        count = 0x0006       # Number of registers (6)
        
        cmd = [
            slave_id,
            function,
            (address >> 8) & 0xFF,
            address & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF
        ]
        crc = calc_crc(cmd)
        cmd.extend([crc & 0xFF, (crc >> 8) & 0xFF])
        
        print(f"\n→ Sending Modbus RTU command to slave ID 0x{slave_id:02X}:")
        print(f"  {' '.join([f'{b:02X}' for b in cmd])}")
        
        ser.reset_input_buffer()
        ser.write(bytes(cmd))
        
        time.sleep(0.2)
        
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"\n← Response received ({len(response)} bytes):")
            print(f"  {' '.join([f'{b:02X}' for b in response])}")
            
            # Verify response
            if len(response) >= 3:
                if response[0] == slave_id and response[1] == function:
                    print("\n✓ SUCCESS: Gripper is responding correctly!")
                    print(f"  Slave ID matches: 0x{response[0]:02X}")
                    print(f"  Function code matches: 0x{response[1]:02X}")
                    ser.close()
                    return True
                else:
                    print("\n⚠ WARNING: Response format unexpected")
            else:
                print("\n⚠ WARNING: Response too short")
        else:
            print("\n✗ ERROR: No response from gripper")
            print("\nPossible issues:")
            print("  • Gripper not powered ON")
            print(f"  • Wrong slave ID (tried 0x{slave_id:02X})")
            print(f"  • Wrong baudrate (tried {baudrate})")
            print("  • Cable not connected properly")
            print("  • Gripper needs reset/power cycle")
        
        ser.close()
        return False
        
    except serial.SerialException as e:
        print(f"\n✗ SERIAL ERROR: {e}")
        print("\nTroubleshooting:")
        print(f"  • Check if {port} exists: ls -l {port}")
        print(f"  • Check permissions: sudo chmod 666 {port}")
        print(f"  • Check if port is in use: lsof {port}")
        return False
    except Exception as e:
        print(f"\n✗ UNEXPECTED ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False

def scan_slave_ids(port, baudrate):
    """Scan for gripper on multiple slave IDs"""
    print(f"\n{'='*60}")
    print(f"Scanning for Robotiq gripper on {port} at {baudrate} baud")
    print(f"{'='*60}\n")
    
    # Common slave IDs to try
    slave_ids = [0x09, 0x01] + list(range(0x02, 0x10))
    
    for slave_id in slave_ids:
        print(f"\n[{slave_ids.index(slave_id) + 1}/{len(slave_ids)}] Testing slave ID 0x{slave_id:02X}...")
        if test_connection(port, baudrate, slave_id):
            return slave_id
    
    return None

def main():
    """Main diagnostic routine"""
    print("="*60)
    print("Robotiq Gripper Connection Diagnostic Tool")
    print("="*60)
    
    # Default values
    port = '/dev/ttyUSB0'
    baudrates = [115200, 9600, 19200, 57600]
    
    # Check if port was provided
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    # Check if port exists
    try:
        import os
        if not os.path.exists(port):
            print(f"\n✗ ERROR: Port {port} does not exist!")
            print("\nAvailable ports:")
            import glob
            ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
            if ports:
                for p in ports:
                    print(f"  {p}")
            else:
                print("  No USB serial ports found")
            return 1
    except Exception as e:
        print(f"Warning: Could not check port existence: {e}")
    
    # Try different baudrates
    for baudrate in baudrates:
        print(f"\n\n{'#'*60}")
        print(f"# Testing baudrate: {baudrate}")
        print(f"{'#'*60}")
        
        slave_id = scan_slave_ids(port, baudrate)
        if slave_id is not None:
            print(f"\n{'='*60}")
            print(f"✓ GRIPPER FOUND!")
            print(f"{'='*60}")
            print(f"Port: {port}")
            print(f"Baudrate: {baudrate}")
            print(f"Slave ID: 0x{slave_id:02X}")
            print(f"\nUse these settings in your launch file:")
            print(f"  com_port:={port}")
            print(f"  (baudrate is {baudrate}, slave_id is 0x{slave_id:02X})")
            return 0
    
    print(f"\n{'='*60}")
    print("✗ NO GRIPPER FOUND")
    print(f"{'='*60}")
    print("\nFinal troubleshooting checklist:")
    print("  1. Is the gripper powered ON? (Check LED)")
    print("  2. Is the USB cable connected firmly?")
    print("  3. Does the port exist? ls -l /dev/ttyUSB*")
    print("  4. Do you have permissions? groups | grep dialout")
    print("  5. Try power cycling the gripper")
    print("  6. Try a different USB port on your computer")
    print("  7. Check if another program is using the port")
    return 1

if __name__ == '__main__':
    sys.exit(main())
