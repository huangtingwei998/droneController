# -*-coding:utf-8-*-
from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
from socket import *




HOST = '10.13.106.173'
PORT = 21567
BUFSIZ = 1024
ADDR = (HOST, PORT)

tcpSerSock = socket(AF_INET, SOCK_STREAM)
tcpSerSock.bind(ADDR)
tcpSerSock.listen(5)





def arm_and_takeoff(vehicle,aTargetAltitude):
    # 解锁无人机（电机将开始旋转）
    print("Arming motors")
    # 将无人机的飞行模式切换成"GUIDED"（一般建议在GUIDED模式下控制无人机）
    vehicle.mode = VehicleMode("GUIDED")
    # 通过设置vehicle.armed状态变量为True，解锁无人机
    vehicle.armed = True
    # 在无人机起飞之前，确认电机已经解锁
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    # 发送起飞指令
    print("Taking off!")
    # simple_takeoff将发送指令，使无人机起飞并上升到目标高度
    vehicle.simple_takeoff(aTargetAltitude)

    # # 在无人机上升到目标高度之前，阻塞程序
    # while True:
    #     print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    #     # 当高度上升到目标高度的0.95倍时，即认为达到了目标高度，退出循环
    #     # vehicle.location.global_relative_frame.alt为相对于home点的高度
    #     if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
    #         print("Reached target altitude")
    #         break
    #     # 等待1s
    #     time.sleep(1)



#定义方向控制
def send_body_ned_velocity(vehicle,velocity_x, velocity_y, velocity_z, duration=0):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
        0b0000111111000111, # type_mask
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # m/s
        0, 0, 0, # x, y, z acceleration
        0, 0)
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)




def condition_yaw(vehicle,heading, relative=True, clock_wise=True):
    # 使用相对角度或绝对方位
    if relative:
        isRelative = 1
    else:
        isRelative = 0

    # 若使用相对角度，则进行顺时针或逆时针转动
    if clock_wise:
        direction = 1  # "heading"所对应的角度将被加和到当前朝向角
    else:
        direction = -1 # "heading"所对应的角度将被从当前朝向角减去
    if not relative:
        direction = 0

    # 生成CONDITION_YAW命令
    msg = vehicle.message_factory.command_long_encode(
                    0, 0,       # target system, target component
                    mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
                    0,          # confirmation
                    heading,    # param 1, yaw in degrees
                    0,          # param 2, yaw speed (not used)
                    direction,  # param 3, direction
                    isRelative, # param 4, relative or absolute degrees
                    0, 0, 0)    # param 5-7, not used
    # 发送指令
    vehicle.send_mavlink(msg)
    vehicle.flush()

def printdata(data,tcpCliSock):
    tcpCliSock.send(("接收到命令:" + data.decode('utf-8')).encode())
    print(data.decode('utf-8'))


def landDrone(vehicle):
    vehicle.mode = VehicleMode("LAND")
    # 退出之前，清除vehicle对象
    print("Close vehicle object")
    print("关闭连接")
    vehicle.close()

if __name__ == '__main__':

    while True:
        # 建立tcp连接
        print('等待连接....')
        tcpCliSock, addr = tcpSerSock.accept()
        print('成功连接到:', addr)

        # 建立树莓派与pixhawk的连接
        connection_string = '/dev/ttyUSB0'
        print('Connecting to vehicle on: %s' % connection_string)
        # connect函数将会返回一个Vehicle类型的对象，即此处的vehicle
        # 即可认为是无人机的主体，通过vehicle对象，我们可以直接控制无人机
        vehicle = connect(connection_string, wait_ready=True, baud=57600)

        while True:
            # 接收客户端的信息，内容有编码
            data = tcpCliSock.recv(BUFSIZ)
            # 如果内容解码后为“exit” 则退出此次连接
            if data.decode('utf-8') == 'exit':
                break
            # 反馈会系统本次接收已经接收完成
            elif data.decode('utf-8') == 'takeoff':
                arm_and_takeoff(vehicle,1)
                printdata(data,tcpCliSock)

            elif data.decode('utf-8') == 'land':
                landDrone(vehicle)
                printdata(data,tcpCliSock)

            elif data.decode('utf-8') == 'left':
                send_body_ned_velocity(vehicle,0,-0.2,0,1)
                printdata(data,tcpCliSock)

            elif data.decode('utf-8') == 'right':
                send_body_ned_velocity(vehicle,0,0.2,0,1)
                printdata(data,tcpCliSock)

            elif data.decode('utf-8') == 'front':
                send_body_ned_velocity(vehicle,0.2,0,0,1)
                printdata(data,tcpCliSock)

            elif data.decode('utf-8') == 'behind':
                send_body_ned_velocity(vehicle,-0.2,0,0,1)
                printdata(data,tcpCliSock)

            elif data.decode('utf-8') == 'up':
                send_body_ned_velocity(vehicle,0,0,0.2,1)
                printdata(data,tcpCliSock)

            elif data.decode('utf-8') == 'down':
                send_body_ned_velocity(vehicle,0,0,-0.2,1)
                printdata(data,tcpCliSock)

            elif data.decode('utf-8') == 'rolateLeft':
                condition_yaw(vehicle,60,True,True)
                printdata(data, tcpCliSock)

            elif data.decode('utf-8') == 'rolateRight':
                condition_yaw(vehicle,60,True,False)
                printdata(data, tcpCliSock)

            else:
                tcpCliSock.send('命令有误，请重新输入'.encode())
                printdata(data,tcpCliSock)

        tcpCliSock.send('bye'.encode())
        tcpCliSock.close()
    tcpSerSock.close()
