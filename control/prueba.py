import socket
import struct
import time

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
i = 0

while True:
    xe = int(0.1 * 1000)
    ye = int(0.2 * 1000)
    ze = int(0.3 * 1000)
    rollE = int(0.4 * 1000)
    pitchE = int(0.5 * 1000)
    yawE = int(0.6 * 1000)
    
    # Empaquetar 6 enteros int16 (h = short)
    msg = struct.pack('<6h', xe, ye, ze, rollE, pitchE, yawE)

    sock.sendto(msg, (UDP_IP, UDP_PORT))
    time.sleep(0.01)
