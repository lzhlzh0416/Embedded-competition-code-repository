import wave

def print_wav_info(file_path):
    with wave.open(file_path, 'rb') as wav_file:
        params = wav_file.getparams()
        print(f"声道数: {params.nchannels}")
        print(f"采样率: {params.framerate} Hz")
        print(f"样本宽度: {params.sampwidth} 字节")
        print(f"帧数: {params.nframes}")
        print(f"压缩类型: {params.comptype}")
        print(f"压缩名称: {params.compname}")

if __name__ == "__main__":
    wav_file_path = 'test.wav'  # 替换为您的文件路径
    print_wav_info(wav_file_path)