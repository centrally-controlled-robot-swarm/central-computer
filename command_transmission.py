import socket
import time
import random

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
        


    def send_pwm(self, id, pwm_L: int, pwm_R: int):
        ip_address = self.esp_id[id]
        message = f"{pwm_L} {pwm_R}"
        self.sock.sendto(message.encode('utf-8'), (ip_address, self.ESP32_PORT))

        # Echo back messages received
        # try:
        #     data, addr = self.sock.recvfrom(1024)
        #     print(f"Echo from {addr}: {data.decode()}")
        # except socket.timeout:
        #     print("No echo received.")


    def send_halt(self):
        self.send_pwm(1, -1, -1)



if __name__=="__main__":
    data_obj = CommandTransmission()


    # for i in range(10):
    #     command = [(random.randint(0, 50), random.randint(0, 50)) for _ in range(10)]
    #     for l, r in command:
    #         data_obj.send_pwm(3, l, r)
    #         time.sleep(0.1)
    #         data_obj.send_pwm(3, 0, 0)
    #         time.sleep(0.1)
    # data_obj.send_pwm(3, 0, 0)

    
    
    while True:
        pwm_L, pwm_R = input("Enter L and R PWM, separated by a space: ").split()
        data_obj.send_pwm(2, int(pwm_L), int(pwm_R))
        time.sleep(0.05)

