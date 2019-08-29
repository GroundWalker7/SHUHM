'''
@File    :   HealthMonitor.py    

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2019/7/12 18:54   gw         0.1         None
'''
import webview
# using pywebview version 2.4
# import serial
# import serial.tools.list_ports
# import re
# import numpy as np
# import sympy
import time
import math
# import matplotlib.pyplot as plt
# from matplotlib.patches import Circle


# numRegex = re.compile(r'-?\d+\.\d+')  # 匹配x,y坐标形式的正则表达式

def getSerialPort(baudrate):
    """
    自动获取串口号
    :param baudrate: 波特率
    :return: 串口名
    """
    port_list = list(serial.tools.list_ports.comports())
    port_list_0 = list(port_list[0])
    port_serial = port_list_0[0]
    ser = serial.Serial(port_serial, baudrate, timeout=0.5)
    return ser


class Api():
    def __init__(self):
        return
    def getData(self, params):
        saveData = []
        points = []
        ser = getSerialPort(115200)
        ser.write(b"start")
        while True:
            data = ser.readline()
            # print(data)
            if data != b"":
                datadecode = data.decode("gbk")
                pointRegex = numRegex.findall(datadecode)
                if pointRegex != []:
                    # print(pointRegex)
                    saveData.append(pointRegex)
                    temp = [pointRegex[3], pointRegex[4]]
                    points.append(temp)
            else:
                print(points)
                print(len(points))
                point_x = []
                point_y = []
                for item in points:
                    point_x.append(float(item[0]))
                    point_y.append(float(item[1]))
                print(point_x)
                print(point_y)
                """
                多项式拟合
                """
                z1 = np.polyfit(point_x, point_y, 5)
                p = np.poly1d(z1)
                # p1[0]-p1[11] 为多项式系数
                ddz = []  # 二阶导数
                for i in range(len(z1) - 2):
                    res = (5 - i) * (4 - i) * z1[i]
                    ddz.append(res)
                ddz = np.array(ddz)
                p2 = np.poly1d(ddz)
                # p2[0]-p2[9] 为二阶导多项式系数

                dz = []  # 一阶导数
                for i in range(len(z1) - 1):
                    res = (5 - i) * z1[i]
                    dz.append(res)
                dz = np.array(dz)
                p1 = np.poly1d(dz)
                # p1[0]-p1[9] 为一阶导多项式系数
                a = sympy.Symbol('x')
                start_time = time.time()
                # print(sympy.solve(p2[9]*a**9 + p2[8]*a**8 + p2[7]*a**7 + p2[6]*a**6 + p2[5]*a**5 + p2[4]*a**4 + p2[3]*a**3 + p2[2]*a**2 + p2[1]*a**1 + p2[0]*a**0,a))
                # print(sympy.solve(p2[5]*a**5 + p2[4]*a**4 + p2[3]*a**3 + p2[2]*a**2 + p2[1]*a**1 + p2[0]*a**0,a))
                # print(sympy.solve(p2[7]*a**7 + p2[6]*a**6 + p2[5]*a**5 + p2[4]*a**4 + p2[3]*a**3 + p2[2]*a**2 + p2[1]*a**1 + p2[0]*a**0,a))
                zero2 = sympy.solve(p2[3] * a ** 3 + p2[2] * a ** 2 + p2[1] * a ** 1 + p2[0] * a ** 0, a)
                zero1 = sympy.solve(p1[4] * a ** 4 + p1[3] * a ** 3 + p1[2] * a ** 2 + p1[1] * a ** 1 + p1[0] * a ** 0,
                                    a)
                print("二阶导")
                print(zero2)
                print("一阶导")
                print(zero1)
                print("time:", time.time() - start_time)
                for i in range(len(zero1)):
                    temp1 = str(zero1[i]).split(" ")
                    if temp1[0][0] != "-":
                        break
                temp1 = float(temp1[0])
                print(temp1)

                for i in range(len(zero2)):
                    temp2 = str(zero2[i]).split(" ")
                    temp2 = float(temp2[0])
                    if temp2 >= temp1:
                        break
                print(temp2)

                # # 拟合曲线图
                # yvals = p(point_x)
                # # yvals2 = p2(point_x)
                # plot1 = plt.plot(point_x, yvals, 'r')
                # # plot2 = plt.plot(point_x, yvals2, 'b')
                ''''''

                # 最小二乘拟合圆
                sum_x = sum_y = 0
                sum_x2 = sum_y2 = 0
                sum_x3 = sum_y3 = 0
                sum_xy = sum_x1y2 = sum_x2y1 = 0

                N = 0
                if len(point_x) >= 3:
                    for i in range(len(point_x)):
                        x = point_x[i]
                        y = point_y[i]
                        if x > temp2:
                            continue
                        # print(x)
                        N += 1
                        x2 = x * x
                        y2 = y * y
                        sum_x += x
                        sum_y += y
                        sum_x2 += x2
                        sum_y2 += y2
                        sum_x3 += x2 * x
                        sum_y3 += y2 * y
                        sum_xy += x * y
                        sum_x1y2 += x * y2
                        sum_x2y1 += x2 * y
                    C = N * sum_x2 - sum_x * sum_x
                    D = N * sum_xy - sum_x * sum_y
                    E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x
                    G = N * sum_y2 - sum_y * sum_y
                    H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y
                    if (C * G - D * D) == 0:
                        return 1
                    a = (H * D - E * G) / (C * G - D * D)
                    b = (H * C - E * D) / (D * D - G * C)
                    c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N

                    center_x = a / (-2)
                    center_y = b / (-2)
                    radius = math.sqrt(a * a + b * b - 4 * c) / 2
                radius = float("%.2f" % radius)
                print("radius: ", radius)

                # if points != []:
                #     # 提取xy,可以用numpy优化
                #     x = []
                #     y = []
                #     for point in points:
                #         x.append(float(point[0]))
                #         y.append(float(point[1]))
                #
                #     # 画图部分
                #     fig = plt.figure()
                #     ax = fig.add_subplot(111)
                #     ax.scatter(x, y, c='r', marker='o', s=10)
                #     plt.grid(ls='--')
                #     plt.gca().set_aspect('equal', adjustable='box')
                #     plt.show()
                return radius

    # def handleData(self, params):
    #     p = []
    #     for i in range(1):
    #         p.append(self.getData(1))
    #     print('-------')
    #     print(p)


if __name__ == "__main__":
    # s = time.time()
    api = Api()

    # api.getData(1)
    # print(time.time()-s)

    # webview.config.gui = "cef"
    webview.create_window("Window Title",
                          "./web/views/index.html",
                          # fullscreen=True,
                          js_api=api)
    webview.start(gui='cef',debug=True)