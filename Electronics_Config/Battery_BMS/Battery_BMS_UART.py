
import serial
import struct

# Configure UART connection
ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)

# Construct the request packet
start_flag = 0xA5
address = 0x40
data_id = 0x90
data_length = 8
data_content = [0x00] * data_length  # Placeholder 8 bytes

# Calculate checksum (sum of all bytes, only the low byte is used)
packet = [start_flag, address, data_id, data_length] + data_content
checksum = sum(packet) & 0xFF
packet.append(checksum)

# Send the packet
ser.write(bytearray(packet))

