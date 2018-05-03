import audioop
import time
import sounddevice
import numpy

stream = sounddevice.RawOutputStream(samplerate=12000, channels = 1, dtype='int16', latency = 0.1)

from socket import *
s=socket(AF_INET, SOCK_DGRAM)
s.bind(('',18294))
s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
s.settimeout(1.0) 

s.sendto(b'', ('255.255.255.255', 18294))
send_t = time.time()

outfile = open('data', 'w')
outfile2 = open('data.bin', 'wb')

stream_started = False

start = time.time()
while True:
    if time.time() - send_t > 1:
        s.sendto(b'', ('255.255.255.255', 18294))
        send_t = time.time()

    try:
        m = s.recvfrom(1024)
    except timeout:
        m = None
    
    if m and len(m[0]) > 0:
        t = time.time()
        print '%0.3f' % (t - start), m[1], len(m[0])
        start = t
        m = audioop.alaw2lin(m[0][8:], 2)
        for i in range(len(m) / 2):
            sample = audioop.getsample(m, 2, i)
            outfile.write('%6d\n' % sample)
        
        if not stream_started:
            stream.start()
        
        stream.write(m)
        outfile2.write(m)
        

