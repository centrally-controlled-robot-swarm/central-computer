import socket

# Used to broadcast messages to all esp32s
# sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

class CommandTransmission():
    def __init__(self):
        # Storing the IP addresses in a dict so that addressing is in constant time
        self.esp_id = {
            1: "192.168.43.123",
            2: "192.168.43.123",
            3: "192.168.43.123"
        }
        self.ESP32_PORT = 4210
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        


    def send_command(self, id, message):
        ip_address = self.esp_id[id]
        self.sock.sendto(message.encode('utf-8'), (ip_address, self.ESP32_PORT))



if __name__=="__main__":
    data_obj = CommandTransmission()
    
    for i in range(10):
        data_obj.send_command(str(i), 1)

# TODO: Add ROS2 hooks so that this can interact with the local planner