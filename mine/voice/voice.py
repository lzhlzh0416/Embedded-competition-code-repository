import wave
import sys
import json
from vosk import Model, KaldiRecognizer, SetLogLevel

# 设置日志级别
SetLogLevel(-1)

# 打开音频文件
wf = wave.open(sys.argv[1], "rb")
if wf.getnchannels() != 1 or wf.getsampwidth() != 2 or wf.getcomptype() != "NONE":
    print("Audio file must be WAV format mono PCM.")
    sys.exit(1)

# 加载模型
model = Model("vosk-model-small-cn-0.22")
rec = KaldiRecognizer(model, wf.getframerate())
rec.SetWords(True)

str_ret = ""

# 读取音频数据进行识别
while True:
    data = wf.readframes(4000)
    if len(data) == 0:
        break
    if rec.AcceptWaveform(data):
        result = rec.Result()
        result = json.loads(result)
        if 'text' in result:
            str_ret += result['text'] + ' '

# 处理最终结果
result = json.loads(rec.FinalResult())
if 'text' in result:
    str_ret += result['text']

# 将识别结果写入文件
with open("result.txt", "w", encoding="utf-8") as f:
    f.write(str_ret)

