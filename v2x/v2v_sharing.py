#!/usr/bin/env python
from libs.socket_handler import SocketHandler

class V2VSharing:
    def __init__(self, type):
        self.socket_handler = SocketHandler(type)
        self.type = type
        pass
    
    def set_obu(self):
        if self.socket_handler.connect() < 0:
            print("[V2V Sharing] Connection Failed")
            return -1
        if self.socket_handler.register() < 0:
            print("[V2V Sharing] Registratino Failed")
            return -1
        if  self.socket_handler.set_tx() < 0:
            print("[V2V Sharing] Tx Setting Failed")
            return -1
        if  self.socket_handler.set_rx() < 0:
            print("[V2V Sharing] Rx Setting Failed")
            return -1
        return 1

    def execute(self, cnt, state, paths, objects):
        if self.type == 1: #Sender
            return self.socket_handler.tx(cnt)
        elif self.type == 2:
            return self.socket_handler.rx()
        else:
            if self.socket_handler.tx(cnt) < 0 or self.socket_handler.rx() < 0:
                return -1
            else:
                return 1

