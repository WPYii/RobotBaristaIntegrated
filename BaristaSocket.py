#!/usr/bin/python
#coding=utf-8
from socket import *
import sys
import time

def baristasocket():
    PORT = 6355
    s= socket(AF_INET, SOCK_STREAM)
    s.bind(("", PORT))
    s.listen(1)
    print ('Listening.....')
    while True:
        try:
            clientsocket,addr= s.accept()
            print ('Connected by:', addr)
            while True:
                msg=clientsocket.recv(1024)
                adrmsg=str(msg.decode("utf-8"))
                print(adrmsg)
                if not msg:
                    break
        except KeyboardInterrupt:
            print("Terminating.....")
            clientsocket.close()
            exit()
            

    

if __name__ == '__main__':
    baristasocket()