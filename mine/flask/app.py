import cv2
import numpy as np
from flask import Flask, Response, request
from flask_cors import CORS
import os
import time
import logging
import json
import ctypes
from hobot_dnn import pyeasy_dnn as dnn

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)
CORS(app)

# 上传目录配置
UPLOAD_FOLDER = '/app/mine/voice/'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

# 视频识别结果保存路径
RESULT_FILE_PATH = '/app/mine/flask/result.txt'

# 深度学习模型加载
models = dnn.load('../models/fcos_512x512_nv12.bin')

# ---------- 摄像头懒加载初始化 ----------
class CameraManager:
    def __init__(self):
        self.video_device = None
        self.last_init_time = 0
        self.retry_interval = 10  # 失败后重试间隔（秒）

    def find_usb_camera(self):
        """动态查找可用的USB摄像头设备"""
        for dev in [f'/dev/video{i}' for i in range(10)]:  # 检查video0到video9
            if os.path.exists(dev):
                cap = cv2.VideoCapture(dev)
                if cap.isOpened():
                    cap.release()
                    return dev
        return None

    def get_camera(self):
        """获取摄像头设备（带错误重试）"""
        if self.video_device is not None:
            return self.video_device
            
        current_time = time.time()
        if current_time - self.last_init_time < self.retry_interval:
            return None  # 未到重试时间

        try:
            self.video_device = self.find_usb_camera()
            if self.video_device:
                logger.info(f"Camera initialized at {self.video_device}")
            else:
                logger.warning("No camera device found")
        except Exception as e:
            logger.error(f"Camera init failed: {str(e)}")
        finally:
            self.last_init_time = current_time

        return self.video_device

camera_manager = CameraManager()

# 添加必要的 ctypes 结构定义
class hbSysMem_t(ctypes.Structure):
    _fields_ = [
        ("phyAddr", ctypes.c_double),
        ("virAddr", ctypes.c_void_p),
        ("memSize", ctypes.c_int)
    ]

class hbDNNQuantiShift_yt(ctypes.Structure):
    _fields_ = [
        ("shiftLen", ctypes.c_int),
        ("shiftData", ctypes.c_char_p)
    ]

class hbDNNQuantiScale_t(ctypes.Structure):
    _fields_ = [
        ("scaleLen", ctypes.c_int),
        ("scaleData", ctypes.POINTER(ctypes.c_float)),
        ("zeroPointLen", ctypes.c_int),
        ("zeroPointData", ctypes.c_char_p)
    ]

class hbDNNTensorShape_t(ctypes.Structure):
    _fields_ = [
        ("dimensionSize", ctypes.c_int * 8),
        ("numDimensions", ctypes.c_int)
    ]

class hbDNNTensorProperties_t(ctypes.Structure):
    _fields_ = [
        ("validShape", hbDNNTensorShape_t),
        ("alignedShape", hbDNNTensorShape_t),
        ("tensorLayout", ctypes.c_int),
        ("tensorType", ctypes.c_int),
        ("shift", hbDNNQuantiShift_yt),
        ("scale", hbDNNQuantiScale_t),
        ("quantiType", ctypes.c_int),
        ("quantizeAxis", ctypes.c_int),
        ("alignedByteSize", ctypes.c_int),
        ("stride", ctypes.c_int * 8)
    ]

class hbDNNTensor_t(ctypes.Structure):
    _fields_ = [
        ("sysMem", hbSysMem_t * 4),
        ("properties", hbDNNTensorProperties_t)
    ]

class FcosPostProcessInfo_t(ctypes.Structure):
    _fields_ = [
        ("height", ctypes.c_int),
        ("width", ctypes.c_int),
        ("ori_height", ctypes.c_int),
        ("ori_width", ctypes.c_int),
        ("score_threshold", ctypes.c_float),
        ("nms_threshold", ctypes.c_float),
        ("nms_top_k", ctypes.c_int),
        ("is_pad_resize", ctypes.c_int)
    ]

# 加载后处理库
libpostprocess = ctypes.CDLL('/usr/lib/libpostprocess.so')

# 定义后处理函数
get_Postprocess_result = libpostprocess.FcosPostProcess
get_Postprocess_result.argtypes = [ctypes.POINTER(FcosPostProcessInfo_t)]
get_Postprocess_result.restype = ctypes.c_char_p

# 获取 TensorLayout 类型
def get_TensorLayout(Layout):
    if Layout == "NCHW":
        return int(2)
    else:
        return int(0)

# 定义类别名称
def get_classes():
    return np.array(["person", "bicycle", "car",
                     "motorcycle", "airplane", "bus",
                     "train", "truck", "boat",
                     "traffic light", "fire hydrant", "stop sign",
                     "parking meter", "bench", "bird",
                     "cat", "dog", "horse",
                     "sheep", "cow", "elephant",
                     "bear", "zebra", "giraffe",
                     "backpack", "umbrella", "handbag",
                     "tie", "suitcase", "frisbee",
                     "skis", "snowboard", "sports ball",
                     "kite", "baseball bat", "baseball glove",
                     "skateboard", "surfboard", "tennis racket",
                     "bottle", "wine glass", "cup",
                     "fork", "knife", "spoon",
                     "bowl", "banana", "apple",
                     "sandwich", "orange", "broccoli",
                     "carrot", "hot dog", "pizza",
                     "donut", "cake", "chair",
                     "couch", "potted plant", "bed",
                     "dining table", "toilet", "tv",
                     "laptop", "mouse", "remote",
                     "keyboard", "cell phone", "microwave",
                     "oven", "toaster", "sink",
                     "refrigerator", "book", "clock",
                     "vase", "scissors", "teddy bear",
                     "hair drier", "toothbrush"])

# 处理模型输出结果
def process_results(outputs, frame_width, frame_height):
    # 初始化后处理信息
    fcos_postprocess_info = FcosPostProcessInfo_t()
    fcos_postprocess_info.height = 512  # 模型输入高度
    fcos_postprocess_info.width = 512   # 模型输入宽度
    fcos_postprocess_info.ori_height = frame_height
    fcos_postprocess_info.ori_width = frame_width
    fcos_postprocess_info.score_threshold = 0.5
    fcos_postprocess_info.nms_threshold = 0.6
    fcos_postprocess_info.nms_top_k = 5
    fcos_postprocess_info.is_pad_resize = 0
    
    # 准备输出张量
    output_tensors = (hbDNNTensor_t * len(outputs))()
    
    for i in range(len(outputs)):
        output_tensors[i].properties.tensorLayout = get_TensorLayout(models[0].outputs[i].properties.layout)
        if (len(models[0].outputs[i].properties.scale_data) == 0):
            output_tensors[i].properties.quantiType = 0
        else:
            output_tensors[i].properties.quantiType = 2
            scale_data_tmp = models[0].outputs[i].properties.scale_data.reshape(1, 1, 1, models[0].outputs[i].properties.shape[3])
            output_tensors[i].properties.scale.scaleData = scale_data_tmp.ctypes.data_as(ctypes.POINTER(ctypes.c_float))

        for j in range(len(models[0].outputs[i].properties.shape)):
            output_tensors[i].properties.validShape.dimensionSize[j] = models[0].outputs[i].properties.shape[j]
            output_tensors[i].properties.alignedShape.dimensionSize[j] = models[0].outputs[i].properties.shape[j]
    
    # 执行后处理
    strides = [8, 16, 32, 64, 128]
    for i in range(len(strides)):
        if (output_tensors[i].properties.quantiType == 0):
            output_tensors[i].sysMem[0].virAddr = ctypes.cast(outputs[i].buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_float)), ctypes.c_void_p)
            output_tensors[i + 5].sysMem[0].virAddr = ctypes.cast(outputs[i + 5].buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_float)), ctypes.c_void_p)
            output_tensors[i + 10].sysMem[0].virAddr = ctypes.cast(outputs[i + 10].buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_float)), ctypes.c_void_p)
        else:
            output_tensors[i].sysMem[0].virAddr = ctypes.cast(outputs[i].buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)), ctypes.c_void_p)
            output_tensors[i + 5].sysMem[0].virAddr = ctypes.cast(outputs[i + 5].buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)), ctypes.c_void_p)
            output_tensors[i + 10].sysMem[0].virAddr = ctypes.cast(outputs[i + 10].buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)), ctypes.c_void_p)

        libpostprocess.FcosdoProcess(output_tensors[i], output_tensors[i + 5], output_tensors[i + 10], ctypes.pointer(fcos_postprocess_info), i)
    
    # 获取后处理结果
    result_str = get_Postprocess_result(ctypes.pointer(fcos_postprocess_info))
    result_str = result_str.decode('utf-8')
    
    # 解析JSON结果
    try:
        data = json.loads(result_str[14:])
        # 提取检测到的对象信息
        objects = []
        for obj in data:
            objects.append({
                'name': obj['name'],
                'score': obj['score']
            })
        
        # 转换为字符串格式
        result_line = json.dumps(objects)
        return result_line
    except Exception as e:
        logger.error(f"Error parsing results: {e}")
        return "[]"  # 返回空列表表示没有检测到对象

# bgr格式图片转换成 NV12格式
def bgr2nv12_opencv(image):
    height, width = image.shape[0], image.shape[1]
    area = height * width
    yuv420p = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
    y = yuv420p[:area]
    uv_planar = yuv420p[area:].reshape((2, area // 4))
    uv_packed = uv_planar.transpose((1, 0)).reshape((area // 2,))

    nv12 = np.zeros_like(yuv420p)
    nv12[:height * width] = y
    nv12[height * width:] = uv_packed
    return nv12

# ---------- 视频流生成 ----------
def generate_black_frame():
    """生成黑色帧（摄像头不可用时返回）"""
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    _, buffer = cv2.imencode('.jpg', frame)
    return buffer.tobytes()

def generate_frames(frame_rate=30):
    """视频流生成器（带摄像头状态检查）"""
    frame_interval = 1.0 / frame_rate
    camera_device = camera_manager.get_camera()  # 懒加载获取摄像头
    
    if not camera_device:
        # 无摄像头时返回黑色帧
        while True:
            start_time = time.time()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + generate_black_frame() + b'\r\n')
            time.sleep(max(0, frame_interval - (time.time() - start_time)))
    else:
        cap = cv2.VideoCapture(camera_device)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        try:
            while True:
                start_time = time.time()
                success, frame = cap.read()
                if not success:
                    logger.error("Camera frame read failed")
                    break
                
                # 获取帧的宽度和高度
                frame_height, frame_width = frame.shape[:2]
                
                # 图像处理与识别
                h, w = models[0].inputs[0].properties.shape[2], models[0].inputs[0].properties.shape[3]
                resized_data = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)
                nv12_data = bgr2nv12_opencv(resized_data)

                outputs = models[0].forward(nv12_data)

                # 将结果写入文件
                result_line = process_results(outputs, frame_width, frame_height)  # 解析结果并生成字符串
                with open(RESULT_FILE_PATH, "w") as f:
                    f.write(result_line)  # 写入结果文件

                # 编码并返回帧
                _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
                time.sleep(max(0, frame_interval - (time.time() - start_time)))
        finally:
            cap.release()
            logger.info("Camera released")

# ---------- 路由 ----------
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(frame_rate=30),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/upload', methods=['POST'])
def upload_file():
    if 'file' not in request.files:
        return 'No file part', 400
    file = request.files['file']
    if file.filename == '':
        return 'No selected file', 400
        
    file_path = os.path.join(app.config['UPLOAD_FOLDER'], file.filename)
    try:
        file.save(file_path)
        logger.info(f"File saved: {file_path}")
        return 'File uploaded successfully', 200
    except Exception as e:
        logger.error(f"Upload failed: {str(e)}")
        return 'Server error', 500

@app.route('/get_result', methods=['GET'])
def get_result():
    try:
        with open(RESULT_FILE_PATH, 'r') as f:
            data = f.read()
        return data, 200  # 返回文件内容及200状态码
    except Exception as e:
        logger.error(f"Error reading result.txt: {str(e)}")
        return str(e), 500  # 返回错误信息及500状态码

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)