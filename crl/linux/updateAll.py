#!/usr/bin/env python

import sys
import os

def runCmd(cmd):
    print cmd;
    os.system(cmd)

target = '/home/e906daq/2.6.1/extensions/linuxvme/AllLinuxROC'
ROCs = [['ROC6', 'ROC16.c', '192.168.24.26'], ['ROC8', 'ROC18.c', '192.168.24.28'], ['ROC10', 'ROC20.c', '192.168.24.30'], ['ROC12', 'ROC22.c', '192.168.24.32'], ['ROC16', 'ROC26.c', '192.168.24.36'], ['ROC17', 'ROC27.c', '192.168.24.37'], ['ROC18', 'ROC28.c', '192.168.24.38']]

if sys.argv[1] == 'all':
    updateList = [roc[0] for roc in ROCs]
else:
    updateList = ['ROC%s' % no for no in sys.argv[1].strip().split(',')]
print updateList

for roc in ROCs:
    if roc[0] not in updateList:
        continue
    print '-------------', roc[0]
    runCmd('scp %s %s:%s' % (roc[1], roc[2], target))
    runCmd('ssh %s "cd %s;make clean;make"' % (roc[2], target))
