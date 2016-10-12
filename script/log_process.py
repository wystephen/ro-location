# -*- coding:utf-8 -*-
# carete by steve at  2016 / 10 / 12　10:25

import demjson
import numpy as np

class seq_process:
    def __init__(self):
        print("ini")
        # self.aarange = open('aarange.txt', 'w')
        # self.atrange = open('atrange.txt', 'w')
        #
        # self.aaseq = 0
        # self.atseq = 0
        #
        # self.aadis = np.zeros([3])
        # self.atdis = np.zeros([4])

    def process_file(self,file_name='LOG_2016_10_12_10_15_17.data',out_aa='aarange.txt',out_at='atrange.txt'):
        logf = open(file_name, 'r')

        aarange = open(out_aa, 'w')
        atrange = open(out_at, 'w')

        aaseq = 0
        atseq = 0

        aadis = np.zeros([3])
        atdis = np.zeros([4])

        logf_all = logf.readlines()

        for ll in logf_all:
            jdata = demjson.decode(ll)

            if jdata['type'] == 'a':
                if not (jdata['seq'] == aaseq):
                    if not (aaseq == 0):
                        aarange.write("{0} {1} {2}\n".format(aadis[0], aadis[1], aadis[2]))
                    aaseq = jdata['seq']

                if jdata['aid'] == 0:
                    if jdata['bid'] == 1:
                        aadis[0] = jdata['range']
                    else:
                        aadis[1] = jdata['range']
                else:
                    aadis[2] = jdata['range']

            elif jdata['type'] == 'c':
                if (not (jdata['seq'] == atseq)):
                    if not atseq == 0:
                        atrange.write("{0} {1} {2} {3}\n".format(atdis[0], atdis[1], atdis[2], atdis[3]))
                    atseq = jdata['seq']

                atdis[jdata['beacon_id']] = jdata['range']
            else:
                print("ERROR")


if __name__ == '__main__':
    se = seq_process()
    se.process_file()