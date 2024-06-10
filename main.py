import serial
import math
import numpy as np
import time
import cv2 as cv

class LidarData:
    def __init__(self, FSA, LSA, CS, Speed, TimeStamp, Degree_angle, Angle_i, Distance_i):
        self.FSA = FSA
        self.LSA = LSA
        self.CS = CS
        self.Speed = Speed
        self.TimeStamp = TimeStamp
        self.Degree_angle = Degree_angle
        self.Angle_i = Angle_i
        self.Distance_i = Distance_i

class img:
    def __init__(self):
        self.ser = serial.Serial(port="COM3",baudrate=230400,timeout=5.0,bytesize=8,parity='N',stopbits=1)
        self.size = 1000
        self.scale = 300
        self.black_img = np.zeros((self.size, self.size, 1), dtype=np.uint8)

    def CalcLidarData(self, str):
        str = str.replace(' ', '')
        # Speed 2字节，因为读取字符串时会颠倒顺序，所以需要反向拼接
        # Lidar的旋转速度，单位（度/秒）
        Speed = int(str[2:4] + str[0:2], 16) / 100
        # Start angle: 数据包的起始角度
        # 单位（0.01度）
        FSA = float(int(str[6:8] + str[4:6], 16)) / 100
        # End angle: 数据包的结束角度
        LSA = float(int(str[-8:-6] + str[-10:-8], 16)) / 100
        # 数据包的捕获时间，单位（毫秒）
        TimeStamp = int(str[-4:-2] + str[-6:-4], 16)
        # CRC 校验
        CS = int(str[-2:], 16)
        Confidence_i = list()
        Angle_i = list()
        Distance_i = list()
        Degree_angle = list()
        # 计算数据包的角度值
        # 除以12表示一个数据包中12个点的间隔
        if(LSA - FSA > 0):
            angleStep = float(LSA - FSA) / 12
        else:
            angleStep = float((LSA + 360) - FSA) / 12
        counter = 0
        circle = lambda deg : deg - 360 if deg >= 360 else deg
        # 分析一个数据包中的12个点
        for i in range(0, 6 * 12, 6):
            # 距离，单位（毫米）
            Distance_i.append(int(str[8+i+2 : 8+i+4] + str[8+i : 8+i+2], 16) / 1000)
            # 数据包中点的角度
            Degree_angle.append(circle(angleStep * counter + FSA))
            Angle_i.append(circle(angleStep * counter + FSA) * math.pi / 180.0)
            counter += 1
        lidarData = LidarData(FSA, LSA, CS, Speed, TimeStamp, Degree_angle, Angle_i, Distance_i)
        # 返回Lidar的数据
        return lidarData

    def output_img(self):
        tmpString = ""
        angles = list()
        distances = list()
        i = 0
        while True:
            loopFlag = True
            flag2c = False
            self.black_img = np.zeros((1000, 1000, 1), dtype=np.uint8)
            if (i % 40 == 39):
                x = distances * np.cos(angles)
                y = distances * np.sin(angles)
                y = y * self.scale + self.size / 2
                x = x * self.scale + self.size / 2
                x_len = len(x)
                y_len = len(y)
                if(x_len == y_len):
                    for j in range(x_len):
                        if (x[j] < self.size and y[j] < self.size):
                            self.black_img[int(x[j]), int(y[j])] = 255
                # 清除数值集合
                angles.clear()
                distances.clear()
                cv.imshow("Lidar", self.black_img)
                cv.waitKey(1)
                time.sleep(0.01)
                i = 0

            while loopFlag:
                # 从串口读取数据
                b = self.ser.read()
                # 将读取的字节转换为整数, big: 字节顺序，最高有效位在前
                tmpInt = int.from_bytes(b, 'big')
                # 0x54 表示数据包的起始
                if (tmpInt == 0x54):
                    tmpString += b.hex() + " "
                    flag2c = True
                    continue
                # 0x2c: 固定值 VerLen
                elif (tmpInt == 0x2c and flag2c):
                    tmpString += b.hex()
                    if (not len(tmpString[0:-5].replace(' ','')) == 90):
                        tmpString = ""
                        loopFlag = False
                        flag2c = False
                        continue
                    # 读取完整的一个 Lidar 数据包后，大小应为 90，获取字符串并传入 CalcLidarData() 函数
                    lidarData = self.CalcLidarData(tmpString[0:-5])
                    # 获取角度和距离值
                    angles.extend(lidarData.Angle_i)
                    distances.extend(lidarData.Distance_i)
                    tmpString = ""
                    loopFlag = False
                else:
                    tmpString += b.hex()+ " "
                flag2c = False
            i += 1


test = img()
test.output_img()
