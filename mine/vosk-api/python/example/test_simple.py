import wave
import sys
import json

from vosk import Model, KaldiRecognizer, SetLogLevel

# 设置日志级别
SetLogLevel(-1)

# 检查并打开音频文件
if len(sys.argv) < 2:
    print("请提供音频文件作为参数，例如：python3 test_simple.py <audiofile.wav>")
    sys.exit(1)

audio_file = sys.argv[1]
print(f"正在打开音频文件: {audio_file}")

try:
    wf = wave.open(audio_file, "rb")
except Exception as e:
    print(f"无法打开音频文件: {e}")
    sys.exit(1)

if wf.getnchannels() != 1 or wf.getsampwidth() != 2 or wf.getcomptype() != "NONE":
    print("音频文件必须是 WAV 格式，单声道 PCM。")
    sys.exit(1)

# 设置模型路径
model_path = "/app/mine/vosk-api/python/example/model-small"
print(f"正在加载模型，路径为: {model_path}")

try:
    model = Model(model_path)
except Exception as e:
    print(f"加载模型失败: {e}")
    sys.exit(1)

rec = KaldiRecognizer(model, wf.getframerate())
rec.SetWords(True)

str_ret = ""

print("开始进行语音识别...")
while True:
    data = wf.readframes(4000)
    if len(data) == 0:
        break
    if rec.AcceptWaveform(data):
        result = rec.Result()
        result = json.loads(result)
        if 'text' in result:
            str_ret += result['text'] + ' '

result = json.loads(rec.FinalResult())
if 'text' in result:
    str_ret += result['text']

print("识别完成。识别结果为:")
print(str_ret)