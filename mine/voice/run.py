import subprocess
import os
import re
import time

def check_dependencies():
    """检查必要的依赖和文件"""
    required_files = [
        "/app/mine/llama.cpp/build/bin/llama-cli",
        "/app/mine/llama.cpp/gguf/qwen2.5-1.5b-instruct-q4_0.gguf",
        "/app/mine/voice/voice.py"
    ]
    
    for file in required_files:
        if not os.path.exists(file):
            print(f"错误：必要文件不存在 - {file}")
            return False
    
    try:
        subprocess.run(["ffmpeg", "-version"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
    except:
        print("错误：ffmpeg未正确安装")
        return False
    
    return True

def convert_webm_to_wav(webm_file, wav_file):
    """优化后的音频转换函数"""
    if not os.path.exists(webm_file):
        print(f"错误：输入文件不存在 - {webm_file}")
        return False

    command = [
        'ffmpeg',
        '-y',                    # 覆盖输出文件而不询问
        '-i', webm_file,
        '-ac', '1',              # 单声道
        '-ar', '16000',          # 16kHz采样率
        '-acodec', 'pcm_s16le',  # 指定PCM编码
        '-hide_banner',          # 隐藏ffmpeg横幅信息
        '-loglevel', 'error',    # 只显示错误信息
        wav_file
    ]
    
    print("开始音频转换...")
    try:
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        print("音频转换成功")
        return True
    except subprocess.CalledProcessError as e:
        print(f"音频转换失败，错误代码: {e.returncode}")
        print(f"错误输出: {e.stderr[:500]}...")
        return False

def run_voice_recognition(audio_file):
    """优化后的语音识别函数"""
    if not os.path.exists(audio_file):
        print(f"错误：音频文件不存在 - {audio_file}")
        return False

    print("开始语音识别...")
    try:
        result = subprocess.run(
            ["python3", "/app/mine/voice/voice.py", audio_file],
            capture_output=True,
            text=True,
            check=True
        )
        print("语音识别成功")
        return True
    except subprocess.CalledProcessError as e:
        print(f"语音识别失败，错误代码: {e.returncode}")
        print(f"错误输出: {e.stderr[:500]}...")
        return False

def run_llama_model(user_input):
    """优化后的LLaMA模型调用函数"""
    # 检查模型文件
    model_path = "/app/mine/llama.cpp/gguf/qwen2.5-1.5b-instruct-q4_0.gguf"
    if not os.path.exists(model_path):
        print(f"模型文件不存在: {model_path}")
        return None

    # 构建命令
    command = [
        "/app/mine/llama.cpp/build/bin/llama-cli",
        "-m", model_path,
        "-n", "128",              # 减少最大token数
        "--threads", str(os.cpu_count() or 4),  # 使用所有CPU核心
        "--no-warmup",
        "--single-turn",
        "-p", user_input
    ]

    print("启动LLaMA模型推理...")
    start_time = time.time()
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=True,
            timeout=300           # 5分钟超时
        )
        print(f"模型推理完成，耗时: {time.time()-start_time:.2f}秒")
        return result.stdout
    except subprocess.TimeoutExpired:
        print("错误：模型推理超时(5分钟)")
        return None
    except subprocess.CalledProcessError as e:
        print(f"模型推理失败，错误代码: {e.returncode}")
        print(f"错误输出: {e.stderr[:500]}...")
        return None

def execute_script(script_name):
    """执行单个脚本"""
    script_path = f"/app/mine/voice/{script_name}"
    if not os.path.exists(script_path):
        print(f"脚本不存在: {script_path}")
        return

    print(f"执行脚本: {script_path}")
    try:
        result = subprocess.run(
            ["python3", script_path],
            capture_output=True,
            text=True,
            check=True
        )
        print(result.stdout.strip())  # 打印脚本输出
        print(f"脚本执行完成: {script_path}")
    except subprocess.CalledProcessError as e:
        print(f"脚本执行失败: {script_path}, 错误: {e.stderr.strip()}")

def process_ai_response(ai_response, webm_file):
    """处理AI响应，保持原始顺序执行脚本"""
    if not ai_response:
        print("无有效的AI响应")
        return

    print("处理AI响应...")
    print(f"原始AI响应: {repr(ai_response)}")
    
    # 提取assistant部分的响应
    assistant_pattern = re.compile(r'assistant\n([^\n]+)')
    assistant_match = assistant_pattern.search(ai_response)
    if not assistant_match:
        print("无法找到assistant响应部分")
        return
    
    clean_response = assistant_match.group(1).split('[')[0].strip()  # 移除[end of text]等部分
    print(f"清理后的响应: {clean_response}")
    
    # 匹配数字部分（只处理实际响应，不处理示例）
    num_pattern = re.compile(r'([一二三四五六七八九十1-9])')
    ordered_numbers = []
    
    # 首先处理逗号分隔的数字
    for num in clean_response.split(','):
        num = num.strip()
        if num in ['一','二','三','四','五','六','七','八','九','十']:
            if num not in ordered_numbers:
                ordered_numbers.append(num)
    
    # 如果没有逗号分隔，尝试直接提取
    if not ordered_numbers:
        for match in num_pattern.finditer(clean_response):
            num = match.group(1)
            if num in ['一','二','三','四','五','六','七','八','九','十'] and num not in ordered_numbers:
                ordered_numbers.append(num)
    
    print(f"最终提取的数字(按顺序): {ordered_numbers}")

    script_map = {
        '一': 'one.py',
        '二': 'two.py',
        '三': 'three.py',
        '四': 'four.py',
        '五': 'five.py',
    }
    
    # 按顺序执行脚本
    for num in ordered_numbers:
        if num in script_map:
            execute_script(script_map[num])

def main():
    webm_file = "/app/mine/voice/output.WebM"
    wav_file = "/app/mine/voice/record.wav"
    
    # 1. 音频处理
    if not convert_webm_to_wav(webm_file, wav_file):
        return
    
    # 2. 语音识别
    if not run_voice_recognition(wav_file):
        return
    
    # 3. 读取识别结果
    result_file = "/app/mine/voice/result.txt"
    if not os.path.exists(result_file):
        print(f"错误：结果文件不存在 - {result_file}")
        return

    try:
        with open(result_file, "r", encoding="utf-8") as f:
            user_input = f.read().strip()
            if not user_input:
                print("语音识别结果为空")
                return
    except IOError as e:
        print(f"读取识别结果失败: {str(e)}")
        return
    
    # 4. 模型推理
    formatted_input = (
        "请从文本中提取所有楼号数字，用中文数字表示，并用逗号分隔。\n"
        "示例1：'三号楼、一号楼、二号楼' → '三,一,二'\n"
        "示例2：'请打开1栋和3栋' → '一,三'\n"
        "示例3：'第2号楼和第5号楼' → '二,五'\n"
        "请只返回数字部分，不要其他内容。\n"
        "输入文本：" + user_input
    )
    ai_response = run_llama_model(formatted_input)
    
    if ai_response:
        # 5. 响应处理
        process_ai_response(ai_response, webm_file)

if __name__ == "__main__":
    if check_dependencies():
        main()