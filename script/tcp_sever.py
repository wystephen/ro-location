# -*- coding:utf-8 -*-
# carete by steve at  2016 / 10 / 12　9:37

from socket import *
from time import ctime

HOST = ''
PORT = 6000
BUFSIZ = 1024
ADDR = (HOST, PORT)

tcpSerSock = socket(AF_INET, SOCK_STREAM)
tcpSerSock.bind(ADDR)
tcpSerSock.listen(5)

while True:
    print 'waiting for connection...'
    tcpCliSock, addr = tcpSerSock.accept()
    print '...connected from:', addr

    while True:
        data = tcpCliSock.recv(BUFSIZ)
        if not data:
            break
        #tcpCliSock.send('[%s] %s' %(ctime(), data))
        print (data)
    print("----------------------------------------------------")
tcpCliSock.close()

tcpSerSock.close()