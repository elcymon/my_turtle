from __future__ import print_function
import pandas as pd
from matplotlib import pyplot as plt
import time
from subprocess import Popen, PIPE
import sys
def get_quality():
    shell_cmd = 'iwconfig {} | grep Link'.format('wlo1')
    proc = Popen(shell_cmd,shell=True,stdout=PIPE,stderr=PIPE)
    output,err = proc.communicate()
    # print(output)
    msg = output.decode('utf-8').strip()
    m = msg.split('=')[-1]
    # print(msg)
    mt = float(m.split(' ')[0])
    # if mt < 0:
    #     mv = mt

    return mt
def log_signal_quality(filename):
    start = time.time()
    with open(sys.argv[1] + '_' + filename,'w') as f:
        t = time.time() - start
        while (t < 100):
            sys.stderr.write('%.2f\r' % t)
            f.write('{:.2f},{}\n'.format(t,get_quality()))
            time.sleep(0.025)
            t = time.time() - start
            # print('\r',end='')
            
            
        sys.stderr.write('%.2f\n' % t)
            
def plot(outdoorfile,indoorfile):
    fig = plt.figure()
    ax = fig.gca()
    dfindoor = pd.read_csv(indoorfile,names=['Time','RSSI'])
    dfoutdoor = pd.read_csv(outdoorfile,names=['Time','RSSI'])
    
    dfindoor.plot(kind='line',x='Time',y='RSSI',legend=False,ax=ax,label='Indoor')
    dfoutdoor.plot(kind='line',x='Time',y='RSSI',legend=False,ax=ax,label='Outdoor')
    
    ax.set_ylabel('RSSI',fontweight='bold')
    ax.set_xlabel('Time in seconds',fontweight='bold')
    plt.legend()
    fig.savefig(outdoorfile.replace('.log','') + '-vs-' + indoorfile.replace('.log','') + '.png',bbox_inches='tight')


if __name__ == '__main__':
#    log_signal_quality('wifi.log')
    plot('1_wifi.log','1-indoor_wifi.log')