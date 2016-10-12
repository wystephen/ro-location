# -*- coding:utf-8 -*-
# carete by steve at  2016 / 10 / 10　19:24

import serial
import serial.tools.list_ports


if __name__ == '__main__':
    pllist = list(serial.tools.list_ports.comports())

    if len(pllist) <= 0:
        print 'without any serial device'
    else:
        t = serial.Serial('\\.\COM4',115200)

    while 1:
        all_tmp = t.readline()
        print(all_tmp)
