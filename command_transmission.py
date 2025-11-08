'''
Run a UDP server on the computer
Give each MCU its own ID
Run a broadcast program like is shown below
Then, add ROS2 hooks so that it can interact with the local planner
'''
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.sendto(b"test message", ('192,168.1.255', 4210))
