
import time
import ctypes
from ctypes import cast
import os
from HCNetSDK import *
# -*- coding: utf-8 -*-

class HikVisionCam:
    def __init__(self):
        # os.chdir(r'./core/')
        self.Objdll = Objdll
        # self.sDVRIP = "10.17.35.104"
        self.sDVRIP = "192.168.1.64"
        self.wDVRPort = 8000
        self.sUserName = "admin"
        self.sPassword = "Robotdog1"
        self.lUserId = ctypes.c_long(-1)
        self.lChannel = None
        self.handle = ctypes.c_long(-1)  # 布防句柄

    # 用户注册设备 并登入，需要修改IP,账号、密码
    def login(self):
        self.Objdll.NET_DVR_Init()  # SDK初始化
        set_overtime = self.Objdll.NET_DVR_SetConnectTime(5000, 4)  # 设置超时
        if set_overtime:
            print("设置超时时间成功")
        else:
            error_info = self.Objdll.NET_DVR_GetLastError()
            print("设置超时错误信息：" + str(error_info))
            return False
        # 用户注册设备
        # c++传递进去的是byte型数据，需要转成byte型传进去，否则会乱码
        sDVRIP = bytes(self.sDVRIP, "ascii")
        sUserName = bytes(self.sUserName, "ascii")
        sPassword = bytes(self.sPassword, "ascii")
        device_info = NET_DVR_DEVICEINFO_V30()
        self.lUserId = self.Objdll.NET_DVR_Login_V30(sDVRIP, self.wDVRPort, sUserName, sPassword,
                                                     ctypes.byref(device_info))
        if self.lUserId == -1:
            error_info = self.Objdll.NET_DVR_GetLastError()
            print("登录错误信息：" + str(error_info))
            return error_info
        else:
            print("登录成功，用户ID：" + str(self.lUserId))
            return self.lUserId

    def g_fMessageCallBack_Alarm(lCommand, pAlarmer, pAlarmInfo, dwBufLen, pUser):
        """
        解析报警信息
        """
        global alarm_info
        Alarmer = pAlarmer.contents  # 取指针指向的结构体
        alarm_info = []
        single_alrm = {}
        seriel_num = ''
        for n in Alarmer.sSerialNumber[0:48]:
            if n != 0:
                seriel_num += chr(n)
        single_alrm['设备序列号sSerialNumber'] = seriel_num
        if Alarmer.byUserIDValid:
            single_alrm['lUserID'] = Alarmer.lUserID

        # 移动侦测、视频丢失、遮挡、IO信号量等报警信息(V3.0以上版本支持的设备)
        if lCommand == 0x4000:
            print('移动侦测')
            Alarm_struct = cast(pAlarmInfo,
                                LPNET_DVR_ALARMINFO_V30).contents  # 当lCommand是COMM_ALARM时将pAlarmInfo强制转换为NET_DVR_ALARMINFO类型的指针再取值
            single_alrm['dwAlarmType'] = hex(Alarm_struct.dwAlarmType)
            single_alrm['byAlarmOutputNumber'] = Alarm_struct.byAlarmOutputNumber[0]
            single_alrm['byChannel'] = Alarm_struct.byChannel[0]

        # 门禁报警
        if lCommand == 0x5002:
            print('门禁触发报警')
            Alarm_struct = cast(pAlarmInfo,
                                LPNET_DVR_ACS_ALARM_INFO).contents  # 当lCommand是0x5002时将pAlarmInfo强制转换为NET_DVR_ACS_ALARM_INFO类型的指针再取值
            single_alrm['dwSize'] = Alarm_struct.dwSize
            single_alrm['dwMajor'] = hex(Alarm_struct.dwMajor)
            single_alrm['dwMinor'] = hex(Alarm_struct.dwMinor)
            single_alrm['struTime'] = Alarm_struct.struTime.dwYear
            localtime = time.asctime(time.localtime(time.time()))
            single_alrm['localtime'] = localtime

        alarm_info.append(single_alrm)
        print(alarm_info[-1])

    setdvrmsg_callback_func = MSGCallBack(g_fMessageCallBack_Alarm)

    # 报警布防
    def setAlarm(self):
        # 设置回调函数
        self.Objdll.NET_DVR_SetDVRMessageCallBack_V31(self.setdvrmsg_callback_func, None)
        # 设置返回值类型
        self.Objdll.NET_DVR_SetupAlarmChan_V41.restype = ctypes.c_long

        # 启用布防
        struAlarmParam = NET_DVR_SETUPALARM_PARAM()
        struAlarmParam.dwSize = ctypes.sizeof(struAlarmParam)
        struAlarmParam.byAlarmInfoType = 1  # 智能交通报警信息上传类型：0- 老报警信息（NET_DVR_PLATE_RESULT），1- 新报警信息(NET_ITS_PLATE_RESULT)
        struAlarmParam.byDeployType = 1
        self.handle = self.Objdll.NET_DVR_SetupAlarmChan_V41(self.lUserId, ctypes.byref(struAlarmParam))
        if self.handle < 0:
            print("NET_DVR_SetupAlarmChan_V41 失败, error code: %s", self.Objdll.NET_DVR_GetLastError())
            self.Objdll.NET_DVR_Logout(self.lUserId)
            self.Objdll.NET_DVR_Cleanup()

    # 获取报警信息，这里主要是保持连接不断开，从设备接收报警信息
    def getAlarmInfo(self):
        time_flag = 0
        # global alarm_info
        while time_flag < 60:
            print('waite alarm ... %s' % time_flag)
            time_flag += 1
            time.sleep(100)
            if alarm_info:
                print('alarm_info=%s' % str(alarm_info))
                break

    # 登出设备
    def logout(self):
        if(self.handle > -1):
            # 撤销布防
            self.Objdll.NET_DVR_CloseAlarmChan_V30(self.handle)
            # 注销用户
            if (self.Objdll.NET_DVR_Logout(self.lUserId) != True):
                print("登出失败！！！错误码：", self.Objdll.NET_DVR_GetLastError())
            else:
                print("登出成功！！！")
                self.Objdll.NET_DVR_Cleanup()
            # 释放SDK资源
            self.Objdll.NET_DVR_Cleanup()
        else:
            if (self.lUserId >= 0):
                if (self.Objdll.NET_DVR_Logout(self.lUserId) != True):
                    print("登出失败！！！错误码：", self.Objdll.NET_DVR_GetLastError())
                else:
                    print("登出成功！！！")
                    self.Objdll.NET_DVR_Cleanup()


if __name__ == '__main__':
    cam = HikVisionCam()
	
    lUserID = cam.login()  # 登录设备，获取登录句柄
    cam.setAlarm()  # 对设备开启布防
    cam.getAlarmInfo()  # 从设备接收报警事件信息
    cam.logout()  # 退出登录，释放资源