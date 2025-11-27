import socket
import time

# Used to broadcast messages to all esp32s
# sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

class CommandTransmission():
    def __init__(self):
        # Storing the IP addresses in a dict so that addressing is in constant time
        self.esp_id = {
            1: "10.42.0.50",
            2: "10.42.0.40",
            3: "10.42.0.30"
        }
        self.ESP32_PORT = 4210
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        


    def send_command(self, id, message):
        ip_address = self.esp_id[id]
        self.sock.sendto(message.encode('utf-8'), (ip_address, self.ESP32_PORT))

        # Echo back messages received
        try:
            data, addr = self.sock.recvfrom(1024)
            print(f"Echo from {addr}: {data.decode()}")
        except socket.timeout:
            print("No echo received.")


if __name__=="__main__":
    data_obj = CommandTransmission()
    
    while True:
        message = input("Type the string you would like to send: ")
        for i in range(1, 4):
            data_obj.send_command(i, message)
        time.sleep(1)
