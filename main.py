import serial
from CalcLidarData import CalcLidarData
import matplotlib.pyplot as plt
import math

# 创建一个 matplotlib 的图形
# 图形可以理解为一个画布，我们可以在上面绘制多张图表
fig = plt.figure(figsize=(8,8))

# 在图形上创建一个图表
# 在坐标 (1, 1) 并在图形上具有索引 = 1
# 使用极坐标系，通常用于雷达图
ax = fig.add_subplot(111, projection='polar')
# 图表标题
ax.set_title('Lidar LD19 (exit: Key E)',fontsize=18)

# 串口连接端口
com_port = "COM5"

# 创建一个事件用于 pyplot
# 'key_press_event': 按键事件
# 一个与事件绑定的函数
# 按下 E 键退出
plt.connect('key_press_event', lambda event: exit(1) if event.key == 'e' else None)

# 创建串口连接
ser = serial.Serial(port=com_port,
                    baudrate=230400,
                    timeout=5.0,
                    bytesize=8,
                    parity='N',
                    stopbits=1)

tmpString = ""
lines = list()
angles = list()
distances = list()

i = 0
while True:
    loopFlag = True
    flag2c = False

    if (i % 40 == 39):
        if ('line' in locals()):
            line.remove()

        # 绘制散点图
        # 通常表示两个值之间的相关性，这里是角度和距离
        # c: 颜色, s: 点的大小
        print(len(angles))
        line = ax.scatter(angles, distances, c="blue", s=5)
        # 设置极坐标系中 0 度角的位置偏移
        # 在 Lidar 的坐标系中，0 度角对应 y 轴，需要设置偏移 pi / 2
        ax.set_theta_offset(math.pi / 2)
        # 更新图形，或者延迟一段时间
        plt.pause(0.01)
        # 清除数值集合
        angles.clear()
        distances.clear()
        i = 0

    while loopFlag:
        # 从串口读取数据
        b = ser.read()
        # 将读取的字节转换为整数
        # big: 字节顺序，最高有效位在前
        tmpInt = int.from_bytes(b, 'big')
        # 0x54 表示数据包的起始（LD19 文档）
        if (tmpInt == 0x54):
            tmpString += b.hex() + " "
            flag2c = True
            continue
        # 0x2c: 固定值 VerLen（LD19 文档）
        elif (tmpInt == 0x2c and flag2c):
            tmpString += b.hex()
            if (not len(tmpString[0:-5].replace(' ','')) == 90):
                tmpString = ""
                loopFlag = False
                flag2c = False
                continue
            # 读取完整的一个 Lidar 数据包后，大小应为 90，获取字符串并传入 CalcLidarData() 函数
            lidarData = CalcLidarData(tmpString[0:-5])
            # 获取角度和距离值
            angles.extend(lidarData.Angle_i)
            distances.extend(lidarData.Distance_i)
            # print(distances)
            tmpString = ""
            loopFlag = False
        else:
            tmpString += b.hex()+ " "
        flag2c = False
    i += 1

ser.close()
