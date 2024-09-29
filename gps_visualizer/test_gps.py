import time
from comms_core import Server, Logger, CustomSocketMessage as csm

if __name__ == "__main__":
    ser = Server()
    ser.start()
    msg = {}
    while True:
        msg['heartbeat'] = 00
        ser.send(csm.encode(msg))
        time.sleep(1.5)