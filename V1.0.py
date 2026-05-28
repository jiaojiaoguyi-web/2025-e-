"""
MIT License
Copyright (c) 2025 BlackCyan
"""

from maix import image, camera, display, app, uart, touchscreen
import time
import cv2
import numpy as np
import os

# ==== 配置 ====
IMG_WIDTH, IMG_HEIGHT = 320, 240
DISP_WIDTH, DISP_HEIGHT = 640, 480
FIT_MODE = image.Fit.FIT_CONTAIN

# Task1固定偏移
LASER_OFFSET_X = 16
LASER_OFFSET_Y = 3

# 持久化: Task2/3可调偏移
OFFSET_FILE = "/usr/task2_offset.txt"

def load_offsets():
    try:
        if os.path.exists(OFFSET_FILE):
            with open(OFFSET_FILE, 'r') as f:
                x, y = map(int, f.read().strip().split())
                return x, y
    except:
        pass
    return LASER_OFFSET_X, LASER_OFFSET_Y  # 默认用宏定义值

def save_offsets(x, y):
    try:
        with open(OFFSET_FILE, 'w') as f:
            f.write(f"{x} {y}")
    except:
        pass

T2ERR_FILE  = "/usr/task2_default_x.txt"
T2MAX_FILE  = "/usr/task2_max_err.txt"

def load_t2_default_x():
    try:
        if os.path.exists(T2ERR_FILE):
            with open(T2ERR_FILE, 'r') as f:
                return int(f.read().strip())
    except:
        pass
    return 50

def save_t2_default_x(val):
    try:
        with open(T2ERR_FILE, 'w') as f:
            f.write(str(val))
    except:
        pass

def load_t2_max_err():
    try:
        if os.path.exists(T2MAX_FILE):
            with open(T2MAX_FILE, 'r') as f:
                return int(f.read().strip())
    except:
        pass
    return 50

def save_t2_max_err(val):
    try:
        with open(T2MAX_FILE, 'w') as f:
            f.write(str(val))
    except:
        pass

adj_offset_x, adj_offset_y = load_offsets()

# 硬件初始化
cam = camera.Camera(IMG_WIDTH, IMG_HEIGHT)
disp = display.Display()
ts = touchscreen.TouchScreen()

# Task2搜索参数(持久化)
TASK2_DEFAULT_X_ERROR = load_t2_default_x()
TASK2_LOST_TOLERANCE  = 8
TASK2_MAX_ERR         = load_t2_max_err()  # 发送误差上限(持久化可调)

# 命中判定参数
HIT_THRESHOLD     = 2
HIT_FRAME_COUNT   = 5

# 状态变量
pressed_already = False
current_task = 0
task_active = False
lost_frame_count = 0
hit_frame_count  = 0
settings_mode    = False  # 设置界面开关(含偏移和T2误差)
smooth_err_x = 0           # 平滑后的误差(限幅过渡, 防止跳变)
smooth_err_y = 0
MAX_ERR_STEP = 15          # 每帧误差最大变化量(像素), 防止云台抽风

# FPS
last_time = time.time()
frame_count = 0
current_fps = 0

# 图像处理
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

# UI: 顶部任务按钮
task1_btn_rect = [5,   5, 72, 30]
task2_btn_rect = [82,  5, 72, 30]
task3_btn_rect = [159, 5, 72, 30]
stop_btn_rect  = [236, 5, 72, 30]

task1_btn_disp = image.resize_map_pos(IMG_WIDTH, IMG_HEIGHT, DISP_WIDTH, DISP_HEIGHT, FIT_MODE, *task1_btn_rect)
task2_btn_disp = image.resize_map_pos(IMG_WIDTH, IMG_HEIGHT, DISP_WIDTH, DISP_HEIGHT, FIT_MODE, *task2_btn_rect)
task3_btn_disp = image.resize_map_pos(IMG_WIDTH, IMG_HEIGHT, DISP_WIDTH, DISP_HEIGHT, FIT_MODE, *task3_btn_rect)
stop_btn_disp  = image.resize_map_pos(IMG_WIDTH, IMG_HEIGHT, DISP_WIDTH, DISP_HEIGHT, FIT_MODE, *stop_btn_rect)

# UI: 设置界面按钮
ox_btns = {
    "X+":   [5, 40,  42, 30],
    "X-":   [5, 73,  42, 30],
    "Y+":   [5, 106, 42, 30],
    "Y-":   [5, 139, 42, 30],
    "Save": [5, 175, 55, 30],
    "EX+":  [68, 40,  42, 30],
    "EX-":  [68, 73,  42, 30],
    "MX+":  [68, 106, 42, 30],
    "MX-":  [68, 139, 42, 30],
}
ox_btns_disp = {}
for k, r in ox_btns.items():
    ox_btns_disp[k] = image.resize_map_pos(IMG_WIDTH, IMG_HEIGHT, DISP_WIDTH, DISP_HEIGHT, FIT_MODE, *r)

# UI: 底部Set入口
set_btn_rect = [8, IMG_HEIGHT - 30, 50, 26]
set_btn_disp = image.resize_map_pos(IMG_WIDTH, IMG_HEIGHT, DISP_WIDTH, DISP_HEIGHT, FIT_MODE, *set_btn_rect)
# Exit按钮（仅在设置界面显示）
exit_btn_rect = [265, 5, 50, 26]
exit_btn_disp = image.resize_map_pos(IMG_WIDTH, IMG_HEIGHT, DISP_WIDTH, DISP_HEIGHT, FIT_MODE, *exit_btn_rect)
COLOR_ACTIVE   = image.Color.from_rgb(0, 255, 0)
COLOR_INACTIVE = image.Color.from_rgb(64, 64, 64)
COLOR_STOP     = image.COLOR_RED
COLOR_OFFSET   = image.COLOR_ORANGE

# 串口通信
class SerialCommunication:
    def __init__(self, device="/dev/ttyS0", baudrate=230400):
        self.device = device
        self.baudrate = baudrate
        self.serial = None
        self.init_serial()

    def init_serial(self):
        try:
            self.serial = uart.UART(self.device, self.baudrate)
            self.serial.write(b"DETECTOR_READY\n")
        except Exception:
            self.serial = None

    def send_error(self, err_x, err_y, laser_ctrl=0x00):
        """发送误差：err_x, err_y 是已计算好的像素误差"""
        if self.serial is None:
            return False
        try:
            if err_x >= 0:
                typeX = 0x02
                valX = min(err_x, 255)
            else:
                typeX = 0x01
                valX = min(-err_x, 255)

            if err_y >= 0:
                typeY = 0x02
                valY = min(err_y, 255)
            else:
                typeY = 0x01
                valY = min(-err_y, 255)

            payload = bytes([0xAA, typeX, valX, typeY, valY, laser_ctrl, 0x55])
            self.serial.write(payload)
            return True
        except Exception:
            return False

    def close(self):
        if self.serial:
            try:
                self.serial.close()
            except:
                pass

serial_comm = SerialCommunication()

# 四边形角点排序
def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect

# 绘制UI按钮
def draw_button(img, rect, label, active, is_stop=False):
    if is_stop:
        bg = COLOR_STOP
    elif active:
        bg = COLOR_ACTIVE
    else:
        bg = COLOR_INACTIVE
    img.draw_rect(rect[0], rect[1], rect[2], rect[3], bg, thickness=-1)
    img.draw_rect(rect[0], rect[1], rect[2], rect[3], image.COLOR_WHITE, thickness=1)
    img.draw_string(rect[0] + 6, rect[1] + 7, label, color=image.COLOR_WHITE)

# 触摸判定
def is_in_button(x, y, disp_rect):
    return disp_rect[0] <= x <= disp_rect[0] + disp_rect[2] and disp_rect[1] <= y <= disp_rect[1] + disp_rect[3]

# ==== 主循环 ====
while not app.need_exit():
    img = cam.read()
    img_disp = img.copy()
    img_raw = image.image2cv(img=img, copy=True)

    # 触摸交互
    x, y, pressed = ts.read()
    if pressed:
        if not pressed_already:
            x_img, y_img = image.resize_map_pos_reverse(IMG_WIDTH, IMG_HEIGHT, DISP_WIDTH, DISP_HEIGHT, FIT_MODE, x, y)

            # 设置界面：仅处理偏移按钮和Exit
            if settings_mode:
                if is_in_button(x, y, exit_btn_disp):
                    settings_mode = False
                    print(">> Exit settings")
                else:
                    for key, disp_rect in ox_btns_disp.items():
                        if is_in_button(x, y, disp_rect):
                            if key == "X+":
                                adj_offset_x += 1
                            elif key == "X-":
                                adj_offset_x -= 1
                            elif key == "Y+":
                                adj_offset_y += 1
                            elif key == "Y-":
                                adj_offset_y -= 1
                            elif key == "EX+":
                                TASK2_DEFAULT_X_ERROR += 5
                            elif key == "EX-":
                                TASK2_DEFAULT_X_ERROR -= 5
                            elif key == "MX+":
                                TASK2_MAX_ERR += 5
                            elif key == "MX-":
                                TASK2_MAX_ERR = max(5, TASK2_MAX_ERR - 5)
                            elif key == "Save":
                                save_offsets(adj_offset_x, adj_offset_y)
                                save_t2_default_x(TASK2_DEFAULT_X_ERROR)
                                save_t2_max_err(TASK2_MAX_ERR)
                                print(f"[Save] x={adj_offset_x}, y={adj_offset_y}, t2x={TASK2_DEFAULT_X_ERROR}, mx={TASK2_MAX_ERR}")
                            break
            else:
                # 主界面：任务按钮 + Set入口
                if is_in_button(x, y, set_btn_disp):
                    settings_mode = True
                    print(">> Enter settings")
                elif is_in_button(x, y, task1_btn_disp):
                    current_task = 1
                    task_active = True
                    print(">> Task1 start")
                elif is_in_button(x, y, task2_btn_disp):
                    current_task = 2
                    task_active = True
                    print(">> Task2 start")
                elif is_in_button(x, y, task3_btn_disp):
                    current_task = 3
                    task_active = True
                    print(">> Task3 start")
                elif is_in_button(x, y, stop_btn_disp):
                    task_active = False
                    print(">> STOP")
        pressed_already = True
        x_img, y_img = image.resize_map_pos_reverse(IMG_WIDTH, IMG_HEIGHT, DISP_WIDTH, DISP_HEIGHT, FIT_MODE, x, y)
        img_disp.draw_circle(int(x_img), int(y_img), 4, image.COLOR_RED, 2)
    else:
        pressed_already = False

    # 矩形检测
    gray = cv2.cvtColor(img_raw, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    closed = cv2.morphologyEx(blur, cv2.MORPH_CLOSE, kernel)
    edged = cv2.Canny(closed, 50, 150)
    contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:10]

    best_rect = None
    best_approx = None
    max_area = 0

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
        if len(approx) == 4:
            area = cv2.contourArea(approx)
            if area > 1000:
                rect = order_points(approx.reshape(4, 2))
                vec1 = rect[1] - rect[0]
                vec2 = rect[2] - rect[1]
                angle_cos = np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))
                if abs(angle_cos) < 0.3 and area > max_area:
                    max_area = area
                    best_rect = rect
                    best_approx = approx

    # 中心计算与发送
    if best_rect is not None:
        for i in range(4):
            pt1 = tuple(best_approx[i][0])
            pt2 = tuple(best_approx[(i + 1) % 4][0])
            img_disp.draw_line(int(pt1[0]), int(pt1[1]), int(pt2[0]), int(pt2[1]), image.COLOR_GREEN, 2)
        center_x = int(np.mean(best_rect[:, 0]))
        center_y = int(np.mean(best_rect[:, 1]))
        img_disp.draw_cross(center_x, center_y, image.COLOR_RED, size=4, thickness=1)

        # ---- 根据任务计算误差 ----
        if current_task == 1:
            # 任务一：固定宏定义偏移
            dx, dy = LASER_OFFSET_X, LASER_OFFSET_Y
            err_x = (160 + dx) - center_x
            err_y = (120 + dy) - center_y
        elif current_task == 2:
            # 任务二：可调偏移
            dx, dy = adj_offset_x, adj_offset_y
            err_x = (160 + dx) - center_x
            err_y = (120 + dy) - center_y
        elif current_task == 3:
            # 任务三：可调偏移 (与Task2共用)
            dx, dy = adj_offset_x, adj_offset_y
            err_x = (160 + dx) - center_x
            err_y = (120 + dy) - center_y
        else:
            dx, dy = LASER_OFFSET_X, LASER_OFFSET_Y
            err_x = (160 + dx) - center_x
            err_y = (120 + dy) - center_y

        # ---- 命中判定 ----
        if task_active and current_task == 2:
            lost_frame_count = 0

        if task_active and current_task == 3:
            laser_ctrl = 0x01
        elif task_active and (current_task == 1 or current_task == 2):
            if abs(err_x) <= HIT_THRESHOLD and abs(err_y) <= HIT_THRESHOLD:
                hit_frame_count += 1
            else:
                hit_frame_count = 0
            laser_ctrl = 0x01 if hit_frame_count >= HIT_FRAME_COUNT else 0x00
        else:
            hit_frame_count = 0
            laser_ctrl = 0x00

        # ---- 误差平滑 + 发送 ----
        if task_active and (current_task == 1 or current_task == 3):
            smooth_err_x += max(-MAX_ERR_STEP, min(MAX_ERR_STEP, err_x - smooth_err_x))
            smooth_err_y += max(-MAX_ERR_STEP, min(MAX_ERR_STEP, err_y - smooth_err_y))
            print(f"err_x = {smooth_err_x}, err_y = {smooth_err_y}, laser = {laser_ctrl}")
            serial_comm.send_error(int(smooth_err_x), int(smooth_err_y), laser_ctrl)
        elif task_active and current_task == 2:
            # 限幅发送, 防止远距离大误差导致云台猛转丢框
            clamp_x = max(-TASK2_MAX_ERR, min(TASK2_MAX_ERR, err_x))
            clamp_y = max(-TASK2_MAX_ERR, min(TASK2_MAX_ERR, err_y))
            print(f"err_x = {clamp_x}, err_y = {clamp_y}, laser = {laser_ctrl}")
            serial_comm.send_error(clamp_x, clamp_y, laser_ctrl)

    else:
        hit_frame_count = 0

        if task_active and current_task == 2:
            lost_frame_count += 1
            if lost_frame_count >= TASK2_LOST_TOLERANCE:
                if TASK2_DEFAULT_X_ERROR >= 0:
                    payload = bytes([0xAA, 0x02, min(TASK2_DEFAULT_X_ERROR, 255), 0x02, 0x00, 0x00, 0x55])
                else:
                    payload = bytes([0xAA, 0x01, min(-TASK2_DEFAULT_X_ERROR, 255), 0x02, 0x00, 0x00, 0x55])
                try:
                    if serial_comm.serial is not None:
                        serial_comm.serial.write(payload)
                        print(f"err_x = {TASK2_DEFAULT_X_ERROR}, err_y = 0  (search)")
                except Exception:
                    pass

        if task_active and current_task == 3:
            # 任务3丢失矩形时, 保持平滑误差不变继续发送(不跳变)
            serial_comm.send_error(int(smooth_err_x), int(smooth_err_y), 0x01)
            print(f"err_x = {int(smooth_err_x)}, err_y = {int(smooth_err_y)}, laser = 1  (hold)")

    # 绘制瞄准点
    if settings_mode or current_task == 2:
        aim_x = int(160 + adj_offset_x)
        aim_y = int(120 + adj_offset_y)
    else:
        aim_x = int(160 + LASER_OFFSET_X)
        aim_y = int(120 + LASER_OFFSET_Y)
    img_disp.draw_cross(aim_x, aim_y, image.COLOR_YELLOW, size=6, thickness=1)

    # 绘制界面
    if settings_mode:
        # 设置界面：显示偏移按钮 + EX按钮 + Exit + 当前值
        for key, r in ox_btns.items():
            img_disp.draw_rect(r[0], r[1], r[2], r[3], COLOR_OFFSET, thickness=2)
            img_disp.draw_string(r[0] + 2, r[1] + 4, key, color=COLOR_OFFSET)
        # Exit按钮
        er = exit_btn_rect
        img_disp.draw_rect(er[0], er[1], er[2], er[3], COLOR_STOP, thickness=2)
        img_disp.draw_string(er[0] + 4, er[1] + 5, "Exit", color=COLOR_STOP)
        # 偏移值 + T2误差值 + 误差上限
        img_disp.draw_string(5, 200, f"x:{adj_offset_x} y:{adj_offset_y}", color=image.COLOR_WHITE)
        img_disp.draw_string(5, 214, f"T2X:{TASK2_DEFAULT_X_ERROR}", color=image.COLOR_YELLOW)
        img_disp.draw_string(5, 228, f"MX:{TASK2_MAX_ERR}", color=image.COLOR_ORANGE)
    else:
        # 主界面：任务按钮 + Set入口
        draw_button(img_disp, task1_btn_rect, "Task1", current_task == 1 and task_active)
        draw_button(img_disp, task2_btn_rect, "Task2", current_task == 2 and task_active)
        draw_button(img_disp, task3_btn_rect, "Task3", current_task == 3 and task_active)
        draw_button(img_disp, stop_btn_rect,  "STOP",  False, is_stop=True)
        # Set按钮 (偏移设置 + T2误差设置)
        sr = set_btn_rect
        img_disp.draw_rect(sr[0], sr[1], sr[2], sr[3], COLOR_OFFSET, thickness=2)
        img_disp.draw_string(sr[0] + 6, sr[1] + 5, "Set", color=COLOR_OFFSET)

    # FPS
    frame_count += 1
    t = time.time()
    if t - last_time >= 1.0:
        current_fps = frame_count
        frame_count = 0
        last_time = t
    img_disp.draw_string(IMG_WIDTH - 70, IMG_HEIGHT - 12, f'FPS:{current_fps}', color=image.COLOR_WHITE)

    disp.show(img_disp, fit=FIT_MODE)
    time.sleep(0.01)
