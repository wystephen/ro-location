# -*- coding:utf-8 -*-
# carete by steve at  2016 / 10 / 12　9:37

from socket import *
from time import ctime
import time


HOST = ''
PORT = 6000
BUFSIZ = 1024
ADDR = (HOST, PORT)

tcpSerSock = socket(AF_INET, SOCK_STREAM)
tcpSerSock.bind(ADDR)
tcpSerSock.listen(5)

t_s = time.localtime()
file_name = 'LOG_{0}_{1}_{2}_{3}_{4}_{5}.data'.format(t_s.tm_year,t_s.tm_mon,t_s.tm_mday,
                                                                  t_s.tm_hour,t_s.tm_min,t_s.tm_sec)
f_handle = open(file_name,'w')

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
        f_handle.write(data)
    print("----------------------------------------------------")
    f_handle.close()
    t_s = time.localtime()
    file_name = 'LOG_{0}_{1}_{2}_{3}_{4}_{5}.data'.format(t_s.tm_year, t_s.tm_mon, t_s.tm_mday,
                                                           t_s.tm_hour, t_s.tm_min, t_s.tm_sec)
    f_handle = open(file_name, 'w')
tcpCliSock.close()

