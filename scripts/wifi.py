import time
from subprocess import Popen, PIPE
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
    with open(filename,'w') as f:
        while (time.time() - start < 10):
            f.write('{:.2f},{}\n'.format(time.time() - start,get_quality()))

if __name__ == '__main__':
    log_signal_quality('wifi.log')