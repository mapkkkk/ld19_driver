import math

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

def CalcLidarData(str):
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
        # Intensity，光强度，越大表示可信度越高
        # Confidence_i.append(int(str[8+i+4 : 8+i+6], 16))
        # 数据包中点的角度
        Degree_angle.append(circle(angleStep * counter + FSA))
        Angle_i.append(circle(angleStep * counter + FSA) * math.pi / 180.0)
        counter += 1
    lidarData = LidarData(FSA, LSA, CS, Speed, TimeStamp, Degree_angle, Angle_i, Distance_i)
    # 返回Lidar的数据
    return lidarData
