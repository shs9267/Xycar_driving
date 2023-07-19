#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
import random

#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
image = np.empty(shape=[0]) # 카메라 이미지를 담을 변수
bridge = CvBridge() 
motor = None # 모터 토픽을 담을 변수

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480    # 카메라 이미지 가로x세로 크기

#=============================================
# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
# 카메라 이미지 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 이미지 정보를 꺼내 image 변수에 옮겨 담음.
#=============================================
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

#=============================================
# 모터 토픽을 발행하는 함수  
# 입력으로 받은 angle과 speed 값을 
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):

    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed

    motor.publish(motor_msg)

#=============================================
# 실질적인 메인 함수 
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함. 
#=============================================

# 입력 이미지를 HSV 색 공간으로 변환
def detect_white_lane_fillter_package(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 하얀색 차선을 검출하기 위한 범위 설정
    lower_white = np.array([0, 0, 180])
    upper_white = np.array([0, 0, 255])

    # 범위 내의 픽셀을 마스킹하여 하얀색 차선 검출
    mask = cv2.inRange(hsv, lower_white, upper_white)
    result = cv2.bitwise_and(img, img, mask=mask)

    diameter = 15
    sigmaColor = 75
    sigmaSpace = 75

    # 미디언 필터 적용
    result = cv2.medianBlur(result, 11)

    # 가우시안 블러링 적용
    result = cv2.GaussianBlur(result, (13,13), 0)

    # 양방향 필터 적용
    result = cv2.bilateralFilter(result, diameter, sigmaColor, sigmaSpace)
    
    return result

#def draw_line_and_intersection(img, a, b, c, d):
    

# 왼쪽 차선의 교점을 계산하고 원을 그립니다.
    

def roi(dst):
    x = int(dst.shape[1])
    y = int(dst.shape[0])
    #범위지정
    section = np.array([[40, 200],[40, 1000], [480, 1000],[480, 200]])
    # 마스크 생성
    mask = np.zeros_like(dst) 
    
    if len(dst.shape) > 2:
        # 이미지가 다중 채널 (칼라)인 경우, 채널 수에 맞게 색상 설정
        ignore_mask_color = (255,) * dst.shape[2]
    else:
        # 이미지가 단일 채널 (흑백)인 경우, 흰색으로 설정
        ignore_mask_color = 255

    
    # 관심 영역을 채움
    cv2.fillPoly(mask, np.int32([section]), ignore_mask_color)
    # 관심 영역만 남기고 나머지 부분 제거
    masked_image = cv2.bitwise_and(dst, mask)
    
    return masked_image


def start():

    # 위에서 선언한 변수를 start() 안에서 사용하고자 함
    global motor, image


    #=========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    #=========================================
    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)

    print ("----- Xycar self driving -----")

    prev_distance = 0  # 이전 프레임에서의 거리
    prev_angle = 0  # 이전 프레임에서의 조향각

    # 첫번째 카메라 토픽이 도착할 때까지 기다림.
    while not image.size == (WIDTH * HEIGHT * 3):
        continue
 
    prev_angle = 0  # 이전 프레임에서의 조향각
    target_speed = 10  # 목표 속도 설정
    min_speed = 8  # 최소 속도 설정
    speed_step = 2  # 속도 조정 단계
    avg_x = 0 #avg_x 변수 선언과 초기값

    current_speed = min_speed  # 현재 속도 초기화


    #=========================================
    # 메인 루프 
    # 카메라 토픽이 도착하는 주기에 맞춰 한번씩 루프를 돌면서 
    # "이미지처리 +차선위치찾기 +조향각결정 +모터토픽발행" 
    # 작업을 반복적으로 수행함.
    #=========================================
    while not rospy.is_shutdown():

        # 이미지처리를 위해 카메라 원본이미지를 img에 복사 저장
        img = image.copy()
        white_img = detect_white_lane_fillter_package(img)

        # 이미지의 크기
        img_width = 640
        img_height = 480

        # 변환 전의 좌표
        src = np.float32([[0, 480], [0, 240], [640, 240], [640, 480]])

        # 변환 후의 좌표
        dst = np.float32([[0, 480], [0, 0], [640, 0], [640, 480]])

        M = cv2.getPerspectiveTransform(src, dst)
            
        # 이미지 변환
        transformed_img = cv2.warpPerspective(white_img, M, (img_width, img_height))

        # Canny 엣지 검출
        # threshold1 = 엣지를 검출하기 위한 낮은 임계값입니다. 이 값보다 낮은 강도의 엣지는 거부
        # threshold2 = 엣지를 검출하기 위한 높은 임계값입니다. 이 값보다 높은 강도의 엣지는 확실한 엣지로 간주
        edges = cv2.Canny(transformed_img, threshold1=10, threshold2=20)

        # 확률적 허프 변환을 사용하여 흰색 직선 검출
        # threshold = 직선으로 판단하기 위한 허프 공간의 임계값
        # minLineLength = 검출할 직선의 최소 길이를 지정
        # maxLineGap = 직선으로 간주할 선분들 간의 최대 허용 간격

        roi_image = roi(edges)

        lines = cv2.HoughLinesP(roi_image, rho=1, theta=np.pi / 180, threshold=25, minLineLength=25, maxLineGap=10)



        left_x_coords = []
        left_y_coords = []
        right_x_coords = []
        right_y_coords = []

        lf_x = []
        lf_y = []
        rt_x = []
        rt_y = []

        if lines is not None:
            x_list = []
            min_x = None
            max_x = 0
            for line in lines:
                x1, y1, x2, y2 = line[0]
                dmx = x2 -x1
                if dmx == 0:
                    dmx = 0.01
                    
                slope = (y2 - y1) / dmx
                

                if slope < 0:  # 왼쪽 차선
                    left_x_coords.extend([x1, x2])
                    left_y_coords.extend([y1, y2])
                    

                    lf_x = left_x_coords if not lf_x else []
                    lf_y = left_y_coords if not lf_y else []

                else:  # 오른쪽 차선
                    right_x_coords.extend([x1, x2])
                    right_y_coords.extend([y1, y2])

                    rt_x = right_x_coords if not rt_x else []
                    rt_y = right_y_coords if not rt_y else [] 

                cv2.line(transformed_img, (x1, y1), (x2, y2), (0, 0, 255), 2)

                

                prev_coefficients = [0,0]
            

                # 왼쪽 차선에 대한 선형 회귀
                if len(left_x_coords) > 1 and len(left_y_coords) > 1:
                    left_coefficients = np.polyfit(left_x_coords, left_y_coords, 1)
                    a = left_coefficients[0]
                    b = left_coefficients[1]
                    prev_coefficients = left_coefficients  # 현재 계수를 이전 계수로 저장

                else:
                    a = prev_coefficients[0]
                    b = prev_coefficients[1]
                

                # 오른쪽 차선에 대한 선형 회귀
                if len(right_x_coords) > 1 and len(right_y_coords) > 1:
                    right_coefficients = np.polyfit(right_x_coords, right_y_coords, 1)
                    c = right_coefficients[0]
                    d = right_coefficients[1]
                    prev_coefficients = right_coefficients  # 현재 계수를 이전 계수로 저장

                else:
                    c = prev_coefficients[0]
                    d = prev_coefficients[1]    

                # 화면에 직선과 교점을 그립니다.
                
                if c == 0:
                    c = 0.0001    
                if a == 0:
                    a = 0.0001

                # 중간 선의 y 좌표를 계산합니다.
                mid_y = transformed_img.shape[0] * 3 // 5

                # y=c인 중간 선을 그립니다.
                cv2.line(transformed_img, (0, mid_y), (transformed_img.shape[1] - 1, mid_y), (0, 255, 0), 2)

                if a is not None and b is not None:
                    intersection_x = int((mid_y - b) / a)
                    intersection_y = mid_y
                    x_list.append(intersection_x)
                else:
                    x_list.append(112)
                # 오른쪽 차선의 교점을 계산하고 원을 그립니다.
                if c is not None and d is not None:
                    intersection_w = int((mid_y - d) / c)
                    intersection_z = mid_y

                    x_list.append(intersection_w)
                else:
                    x_list.append(448)

                # 화면의 중앙에 원을 그립니다.
                mid_x = transformed_img.shape[0] * 67 // 100
                cv2.circle(transformed_img, (mid_x, mid_y), 5, (0, 0, 255), -1)
                
            # 화면의 중앙에 원을 그립니다.
            mid_x = img.shape[0] * 67 // 100
            mid_y = img.shape[0] * 3 // 5
            cv2.circle(transformed_img, (mid_x, mid_y), 5, (0, 0, 255), -1)

            first_value = x_list[0]
            for value in x_list:
                if first_value is None or value < first_value:
                    min_x = value
                if first_value is None or value > first_value:
                    max_x = value

            print(min_x, max_x)
            if min_x != None:
                cv2.circle(transformed_img, (min_x, mid_y), 5, (255, 0, 0), -1)
            cv2.circle(transformed_img, (max_x, mid_y), 5, (255, 0, 0), -1)
            

            # 왼쪽 차선과 오른쪽 차선의 평균 값을 계산
            left_lane_avg = np.mean(left_x_coords), np.mean(left_y_coords)
            right_lane_avg = np.mean(right_x_coords), np.mean(right_y_coords)
            


            # 평균 값이 NaN인 경우 예외 처리합니다.
            # 왼쪽 차선이 감지되지 않은 경우 처리할 내용을 작성합니다.
            # 예를 들어, 왼쪽 차선을 무시하고 오른쪽 차선만을 사용할 수 있습니다.
            if np.isnan(left_lane_avg[0]) or np.isnan(left_lane_avg[1]):
                pass

            # 오른쪽 차선이 감지되지 않은 경우 처리할 내용을 작성합니다.
            # 예를 들어, 오른쪽 차선을 무시하고 왼쪽 차선만을 사용할 수 있습니다.
            if np.isnan(right_lane_avg[0]) or np.isnan(right_lane_avg[1]):
                pass

            # 선들의 중심 값에 원을 표기
            if not np.isnan(left_lane_avg[0]) and not np.isnan(right_lane_avg[0]):
                avg_x = int((left_lane_avg[0] + right_lane_avg[0]) / 2)
                mid_y = img.shape[0] * 3 // 5
                cv2.circle(transformed_img, (avg_x, mid_y), 5, (0, 255, 0), -1)

            # 이전 프레임에서의 조향각과 현재 프레임에서의 조향각 계산
            angle = (avg_x - mid_x) * 0.25

            # 조향각에 따라 속도 조정
            if angle != prev_angle:  # 조향각이 틀어졌을 때
                if angle > prev_angle:  # 조향각이 증가했을 때
                    current_speed -= speed_step  # 속도를 감소
                else:  # 조향각이 감소했을 때
                    current_speed += speed_step  # 속도를 증가

                current_speed = max(min_speed, min(target_speed, current_speed))  # 속도 범위 제한
            else:  # 조향각이 틀어지지 않았을 때
                current_speed = target_speed  # 목표 속도로 설정

            # 조향각 범위 제한
            angle = max(-100, min(100, angle))

            print(angle)            

            # drive() 호출. drive()함수 안에서 모터 토픽이 발행됨.
            drive(angle, current_speed)

            prev_angle = angle  # 이전 프레임의 조향각 업데이트

        # 결과 이미지 출력
        cv2.imshow("Camera View", transformed_img)
        cv2.imshow("roi", roi_image)
        # 디버깅을 위해 모니터에 이미지를 디스플레이
        cv2.waitKey(1)       

#=============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임. 
#=============================================
if __name__ == '__main__':
    start()