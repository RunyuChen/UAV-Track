#!/usr/bin/env python
# -*- coding: utf-8-*-
from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import apriltag
import cv2

# -------------全局变量------------------------------------------
distance = 0
flag_error = 0
camera_status = 0
flag_task = 0  # 0: 追踪； 1：悬停等待； 2：寻找着陆点； 3：着陆； 
last_x = 0.0
last_y = 0.0
last_x_land = 0.0
last_y_land = 0.0
count = 0
continue_time = 0
fps = 0
start = 0
end = 0


# -----------飞控函数定义-----------------------------------------


def arm_and_takeoff(targetheight):
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    print("Vehicle now in GUIDED MODE. Have fun!!")

    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for drone to become armed")
        time.sleep(1)
    print("Look out! Virtual props are spinning!")

    vehicle.simple_takeoff(targetheight)  # meters

    while True:
        print("Current Altitude:", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= .95 * targetheight:
            break
        time.sleep(1)
    print("Target altitude reached!!")
    time.sleep(3)


def send_local_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def send_global_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b110111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


# ----------视觉检测函数定义-----------------------------------------


# 检测Apriltag的中心点
def detect_apriltag(cap, detector):
    # 获得图像
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
    # 检测apriltag
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray)
    if len(tags) > 1:  # 如果检测到多个apriltag则返回-1
        return -1
    elif len(tags) == 1:
        for tag in tags:
            return list(tag.center.astype(int))
    # 没检测到目标则返回0
    return 0

# 反馈无人机是否检测到物体
def is_insight(center, w, h):
    if center == 0:
        print('no object detected')
        return False
    if center == -1:
        print('too many objects')
        return False
    else:
        print('find object')
        return True

def reached(center, w, h):
    ratio = 0.45
    if not is_insight(center, w, h):
        print('check whether one center is insight')
        return False
    else:
        if w - ratio * w > center[0] and center[0] > ratio * w and h - ratio * h > center[1] and center[1] > ratio * h:
            print("Reached object center")
            time.sleep(1)
            return True
        else:
            return False


# ----------------运动函数定义--------------------------------------

def move_front_distance(v, duration):
    i = 0
    while i < duration:
        send_global_ned_velocity(v, 0, 0)
        print("Moving toward front")
        time.sleep(1)
        i += 1
    return 0

def move_front(v, duration):
    i = 0
    while i < duration:
        send_global_ned_velocity(v, 0, 0)
        print("Moving toward front")
        i += 1
    return 0


def move_backward(v, duration):
    i = 0
    while i < duration:
        send_global_ned_velocity(-v, 0, 0)
        print("Moving toward backward")
        i += 1
    return 0


def move_left(v, duration):
    i = 0
    while i < duration:
        send_global_ned_velocity(0, -v, 0)
        print("Moving toward left")
        i += 1
    return 0


def move_right(v, duration):
    i = 0
    while i < duration:
        send_global_ned_velocity(0, v, 0)
        print("Moving toward right")
        i += 1
    return 0


def move_up(v, duration):
    i = 0
    while i < duration:
        send_global_ned_velocity(0, 0, -v)
        print("Moving up")
        i += 1
    return 0


def move_down(v, duration):
    i = 0
    while i < duration:
        send_global_ned_velocity(0, 0, v)
        print("Moving DOWN")
        i += 1
    return 0

# # 开环降落控制
# def move_land1(center, width, height):
#     x = center[0]
#     y = center[1]
#     global count
#     count = count + 1
#     ratio1 = 0.3
#     ratio2 = 0.45
    
#     if x < ratio1*width or x > width - ratio1*width:
#         if x < ratio1*width:
#             move_left(0.1, 1)
#         elif x > width - ratio1*width:
#             move_right(0.1, 1)
#     else:
#         if y < ratio1*height or y > height - ratio1*height:
#             if y < ratio1*height:
#                 move_front(0.1, 1)
#             elif y > height - ratio1*height:
#                 move_backward(0.1, 1)
#     if x >= ratio1*width and x <= width - ratio1*width:
#         if y >= ratio1*height and y <= height - ratio1*height:
#             if x < ratio2*width:
#                 move_left(0.01, 1)
#             elif x > width - ratio2*width:
#                 move_right(0.01, 1)
#             if y < ratio2*height:
#                 move_front(0.01, 1)
#             elif y > height - ratio2*height:
#                 move_backward(0.01, 1)
#     # time.sleep(0.2)

# 闭环降落控制
def move_land(center, w, h, duration=1):
    global last_x_land
    global last_y_land
    kp_y = 0.00025
    kd_y = 0.00005
    kp_x = -0.00025
    kd_x = -0.00005
    x = (w / 2 - center[0] - 0)
    y = (h / 2 - center[1] - 0)
    print("x：", x, "y：", y)

    x_out = x * kp_x + (x - last_x_land) * kd_x
    y_out = y * kp_y + (y - last_y_land) * kd_y
    print("x_out：", x_out, "y_out：", y_out)

    if x_out < -0.1:
        x_out = -0.1
    elif x_out > 0.1:
        x_out = 0.1
    if y_out < -0.1:
        y_out = -0.1
    elif y_out > 0.1:
        y_out = 0.1

    last_x_land = x
    last_y_land = y

    i = 0
    while i < duration:
        send_global_ned_velocity(y_out, x_out, 0)
        print("Moving")
        i += 1
    
# def move_track1(center, width, height):
#     x = center[0]
#     y = center[1]
#     ratio = 4
#     if (width-width/ratio) > x > width/ratio:
#         if y > height/2:
#             move_backward(0.1,1)
#         elif y < height/2:
#             move_front(0.1, 1)
#     elif 0 < x < width/ratio:
#         move_left(0.1, 1)
#     elif x > width - width/ratio:
#         move_right(0.1, 1)
        
def move_track(center, w, h, duration=1):
    global last_x
    global last_y
    global count
    count = count + 1
    kp_y = 0.00085
    kd_y = 0.00005
    kp_x = -0.00065
    kd_x = -0.00005
    x = (w / 2 - center[0] - 0)
    y = (h / 2 - center[1] - 0)
    print("x：", x, "y：", y)

    x_out = x * kp_x + (x - last_x) * kd_x
    y_out = y * kp_y + (y - last_y) * kd_y
    print("x_out：", x_out, "y_out：", y_out)

    if x_out < -0.5:
        x_out = -0.5
    elif x_out > 0.5:
        x_out = 0.5
    if y_out < -0.5:
        y_out = -0.5
    elif y_out > 0.5:
        y_out = 0.5

    last_x = x
    last_y = y

    i = 0
    while i < duration:
        send_global_ned_velocity(y_out, x_out, 0)
        print("Moving")
        i += 1

    

# ----------------------------- 工具函数定义 ----------------------------------------------------A
def print_status():
    print('Status List:')

    GREEN = '\033[92m'
    RESET = '\033[0m'

    camera_status_str = ' Camera Status: ' + str(camera_status)
    distance_str = ' Distance: ' + str(distance)
    flag_error_str = ' Flag Error: ' + str(flag_error)
    last_task_str = ' Last Task: ' + str(flag_task)
    local_location = ' Local Location:' + str(vehicle.location.local_frame)
    str_continue_time = ' Continue Time:' + str(continue_time)
    str_fps = ' FPS:' + str(fps)
    

    max_length = 40

    print(GREEN + "┌" + "─" * max_length + "┐" + RESET)

    # 打印内容
    print("│" + camera_status_str.ljust(max_length) + "│")
    print("│" + distance_str.ljust(max_length) + "│")
    print("│" + flag_error_str.ljust(max_length) + "│")
    print("│" + last_task_str.ljust(max_length) + "│")
    print("│" + str_continue_time.ljust(max_length) + "│")
    print("│" + str_fps.ljust(max_length) + "│")
    # print("│" + local_location.ljust(max_length) + "│")

    print(GREEN + "└" + "─" * max_length + "┘" + RESET)



if __name__ == "__main__":
    # 当前连接的 pixhawk 飞控的端口
    connection_string = '/dev/ttyAMA0'  # 现在使用的是USB转ttl接口，连接 pixhawk 飞控
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=None, baud=57600)
    # 连接摄像头
    for i in range(0, 2):
        cap = cv2.VideoCapture(i,cv2.CAP_V4L2)
        if cap.isOpened():
            print("use camera video：", i)
            camera_status = 1
            break
    if not cap.isOpened():
        print("can not open camera")
        flag_error = 1

    at_detector1 = apriltag.Detector(apriltag.DetectorOptions(families='tag25h9'))  # 用来追踪
    at_detector2 = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))  # 用来着陆
    
    cap.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    w = 640
    h = 480
    # fps = cap.get(cv2.CAP_PROP_FPS)
    # print("fps:",fps)
    print("Camera window size：", w, "x", h)

    try:
        print("start takeoff")
        arm_and_takeoff(1)
        if flag_error == 1:
                print('LAND')
                vehicle.mode = VehicleMode('LAND')

                print('Armed False')
                vehicle.armed = False
                while vehicle.armed:
                    print('vehicle arm',vehicle.armed)
                    time.sleep(1)

                print("Close vehicle object")
                vehicle.close()
        move_front_distance(0.1, 5)
        start = time.time()
        while True:
            if flag_error == 1:
                print('LAND')
                vehicle.mode = VehicleMode('LAND')

                print('Armed False')
                vehicle.armed = False
                while vehicle.armed:
                    print('vehicle arm',vehicle.armed)
                    time.sleep(1)

                print("Close vehicle object")
                vehicle.close()
                break

            center1 = detect_apriltag(cap, at_detector1)
            center2 = detect_apriltag(cap, at_detector2)
            if flag_task == 0:  # 追踪
                if is_insight(center2, w, h):
                    flag_task = 1
                    continue
                if is_insight(center1, w, h):
                    move_track(center1, w, h)
                    continue
                else:
                    continue
            elif flag_task == 1:  # 悬停等待
                if is_insight(center2, w, h) and not is_insight(center1, w, h):
                    flag_task = 2
                    continue
                else:
                    time.sleep(0.5)
                    print("waiting center1 move out")
                    continue
            elif flag_task == 2:  # 寻找着陆点
                if reached(center2, w, h):
                    flag_task = 3
                    continue
                else:
                    if is_insight(center2,w,h):
                        print('center2:',center2)
                        move_land(center2, w, h)
                        distance += 0.02
                        print(distance)
                        continue
                    else:
                        continue	
            elif flag_task == 3: # land
                print('LAND')
                vehicle.mode = VehicleMode('LAND')

                print('Armed False')
                vehicle.armed = False
                while vehicle.armed:
                    print('vehicle arm',vehicle.armed)
                    time.sleep(1)

                print("Close vehicle object")
                vehicle.close()

                break
            
        end = time.time()
        continue_time = end - start
        fps = count / continue_time

        cap.release()
        cv2.destroyAllWindows()
        print_status()
        
    except:
        end = time.time()
        continue_time = end - start
        fps = count / continue_time
        print('LAND')
        vehicle.mode = VehicleMode('LAND')

        print('Armed False')
        vehicle.armed = False
        while vehicle.armed:
            print('vehicle arm',vehicle.armed)
            time.sleep(1)


        print("Close vehicle object")
        vehicle.close()

        print('Error break')
        cap.release()
        cv2.destroyAllWindows()

        print_status()

