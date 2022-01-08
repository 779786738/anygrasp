#-*-coding:utf-8-*-

#引入库
import pyaudio
import wave
import sys

# 定义数据流块
CHUNK = 1024

if len(sys.argv) < 2:
    print("Plays a wave file.\n\nUsage: %s filename.wav" % sys.argv[0])
    sys.exit(-1)

# 只读方式打开wav文件
wf = wave.open(r'D:\\Python\\Lib\\site-packages\\PyQt4\\uic\\test.wav', 'rb')#(sys.argv[1], 'rb')

p = pyaudio.PyAudio()

# 打开数据流
stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                channels=wf.getnchannels(),
                rate=wf.getframerate(),
                output=True)

# 读取数据
data = wf.readframes(CHUNK)

# 播放
while data != '':
    stream.write(data)
    data = wf.readframes(CHUNK)

# 停止数据流
stream.stop_stream()
stream.close()

# 关闭 PyAudio
p.terminate()
