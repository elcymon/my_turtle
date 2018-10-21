"""
this is a stripped down version of the SWHear class.
It's designed to hold only a single audio sample in memory.
check my githib for a more complete version:
    http://github.com/swharden
"""

import pyaudio
import time
import numpy as np
import threading
from collections import deque
def getFFT(data,rate):
    """Given some data and rate, returns FFTfreq and FFT (half)."""
    # print(data)
    # data = np.array(list(data),dtype='f')/np.iinfo(np.int16).max
    #data = np.array([np.iinfo(np.int16)]*len(data))
    # with open('dd.txt', 'w') as f:
    #    f.write('\n'.join([str(i) for i in data]))
    #    raw_input('Exit')
    #data = np.array([np.iinfo(np.int16).max,np.iinfo(np.int16).min] * (len(data)/2))
    data=data*np.hamming(len(data))
    fft=np.fft.fft(data)
    fft=np.abs(fft)
    #reference max value is 38983494.75 for int16 max value = 32768 after fft
    #providing each as a reference to maximum value
    #fft = np.array(list(fft),dtype='f')/38983494.75
    #print(np.max(data),np.max(fft))
    #fft=20*np.log10(fft)
    
    freq=np.fft.fftfreq(len(fft),1.0/rate)
    #band pass filter addition
    # aa = fft[(freq > 990)]
    # ww = freq[(freq > 990)]
    # aa = aa[(ww < 1010)]

    # peak = max(aa)
    # f_value = ww[aa >= peak]
    #ff = [str(i) for i in freq]
    #print('\n'.join(ff))

    #print(peak,f_value)
    return freq[:int(len(freq)/2)],fft[:int(len(fft)/2)]

class SWHear():
    """
    The SWHear class is provides access to continuously recorded
    (and mathematically processed) microphone data.
    
    Arguments:
        
        device - the number of the sound card input to use. Leave blank
        to automatically detect one.
        
        rate - sample rate to use. Defaults to something supported.
        
        updatesPerSecond - how fast to record new data. Note that smaller
        numbers allow more data to be accessed and therefore high
        frequencies to be analyzed if using a FFT later
    """

    def __init__(self,device=None,rate=None,updatesPerSecond=10,nchunks = 3):
        self.p=pyaudio.PyAudio()
        self.chunk=4096 # gets replaced automatically
        self.updatesPerSecond=updatesPerSecond
        self.chunksRead=0
        self.device=device
        self.rate=rate
        self.nchunks = nchunks

    ### SYSTEM TESTS

    def valid_low_rate(self,device):
        """set the rate to the lowest supported audio rate."""
        for testrate in [44100]:
            if self.valid_test(device,testrate):
                return testrate
        print("SOMETHING'S WRONG! I can't figure out how to use DEV",device)
        return None

    def valid_test(self,device,rate=44100):
        """given a device ID and a rate, return TRUE/False if it's valid."""
        try:
            self.info=self.p.get_device_info_by_index(device)
            if not self.info["maxInputChannels"]>0:
                return False
            stream=self.p.open(format=pyaudio.paInt16,channels=1,
               input_device_index=device,frames_per_buffer=self.chunk,
               rate=int(self.info["defaultSampleRate"]),input=True)
            stream.close()
            return True
        except:
            return False

    def valid_input_devices(self):
        """
        See which devices can be opened for microphone input.
        call this when no PyAudio object is loaded.
        """
        mics=[]
        for device in range(self.p.get_device_count()):
            if self.valid_test(device):
                mics.append(device)
        if len(mics)==0:
            print("no microphone devices found!")
        else:
            print("found %d microphone devices: %s"%(len(mics),mics))
        return mics

    ### SETUP AND SHUTDOWN

    def initiate(self):
        """run this after changing settings (like rate) before recording"""
        
        if self.device is None:
            self.device=self.valid_input_devices()[1] #pick the first one
        if self.rate is None:
            self.rate=self.valid_low_rate(self.device)
        self.chunk = int(self.rate/self.updatesPerSecond) # hold one tenth of a second in memory
        if not self.valid_test(self.device,self.rate):
            print("guessing a valid microphone device/rate...")
            self.device=self.valid_input_devices()[1] #pick the first one
            self.rate=self.valid_low_rate(self.device)
        self.datax=np.arange(self.chunk * self.nchunks)/float(self.rate)
        msg='recording from "%s" '%self.info["name"]
        msg+='(device %d) '%self.device
        msg+='at %d Hz'%self.rate
        print(msg)

    def close(self):
        """gently detach from things."""
        print(" -- sending stream termination command...")
        self.keepRecording=pyaudio.paComplete #the threads should self-close
        # while(self.t.isAlive()): #wait for all threads to close
        #     time.sleep(.1)
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()

    ### STREAM HANDLING
    def stream_readchunk(self,input_data,frame_count,time_info,status):
        """reads some audio and re-launches itself"""
        # global chunksRead
        try:
            self.streamdata = input_data
            self.curr_data = np.fromstring(input_data,dtype=np.int16)
            self.data.extend(self.curr_data)
            # self.data = np.concatenate((self.prev_data,self.curr_data))
            # if np.array_equal(self.curr_data,self.prev_data):
            #     print 'prev and curr are equal'
            # self.prev_data = self.curr_data
            self.fftx, self.fft = getFFT(self.data,self.rate)

        except Exception as E:
            print(" -- exception! terminating...")
            print(E,"\n"*5)
            self.keepRecording=pyaudio.paComplete
        # if self.keepRecording:
        #     b = 'b'
        #     #self.stream_thread_new()
        # else:
        #     self.stream.close()
        #     self.p.terminate()
        #     print(" -- stream STOPPED")
        self.chunksRead+=1
        # print frame_count
        return (input_data,self.keepRecording)

    

    def stream_thread_new(self):
        self.t=threading.Thread(target=self.stream_readchunk)
        self.t.start()

    def stream_start(self):
        """adds data to self.data until termination signal"""
        global str_read_chunk
        self.initiate()
        print(" -- starting stream")
        self.keepRecording=pyaudio.paContinue # set this to False later to terminate stream
        
        self.data= deque(np.zeros(self.nchunks*self.chunk,dtype=np.int16),maxlen=self.nchunks*self.chunk) # will fill up with threaded recording data
        self.fft=None
        self.dataFiltered=None #same
        # self.prev_data = np.zeros(self.chunk,dtype=np.int16)
        self.curr_data = np.zeros(self.chunk,dtype=np.int16)
        
        
        self.stream=self.p.open(format=pyaudio.paInt16,channels=1,
                      rate=self.rate,input=True,frames_per_buffer=self.chunk,stream_callback=self.stream_readchunk)
        
        
        self.stream.start_stream()
        
        
        # while self.stream.is_active():
        #     print chunksRead,self.chunksRead
        #     # self.chunksRead = chunksRead
        #     time.sleep(0.1)
        # self.close()
        # self.stream.stop_stream()
        # stream.close()
        # p.terminate()
        #self.stream_thread_new()

if __name__=="__main__":
    ear=SWHear(updatesPerSecond=10) # optinoally set sample rate here
    ear.stream_start() #goes forever
    lastRead=ear.chunksRead
    while True:
        while lastRead==ear.chunksRead:
            time.sleep(.01)
        print(ear.chunksRead,len(ear.data))
        lastRead=ear.chunksRead
    print("DONE")
