import sensor, image, time,pyb
from pyb import UART

red_threshold  = (35, 70, -29,  6,-29 ,-11)
#light_threshold  = (10,30, -6,  10,5,12)

sensor.reset()  # 复位并初始化传感器。
#初始化摄像头，reset()是sensor模块里面的函数
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
#设置图像色彩格式，有RGB565色彩图和GRAYSCALE灰度图两种
sensor.set_framesize(sensor.QVGA)  # 将图像大小设置为QVGA (320x240)
#设置图像像素大小，sensor.QQVGA: 160x120，sensor.QQVGA2: 128x160 (一般用于LCD
#扩展板)，sensor.QVGA: 320x240，sensor.VGA: 640x480, sensor.QQCIF: 88x72，sensor.QCIF: 176x144，sensor.CIF: 352x288

sensor.set_windowing((240, 240))
sensor.skip_frames(10)  # 等待设置生效。
sensor.set_auto_whitebal(False) #关闭白平衡。白平衡是默认开启的，在颜色识别中，一定要关闭白平衡。
clock = time.clock() # 追踪帧率

def find_max(blobs):#筛选出最大的色块
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:

            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob

uart = UART(3, 115200)
count_x=0
count_y=0
count_x_output=0
count_y_output=0
number=0
while(True):
    time_start = pyb.millis()
    clock.tick() # Track elapsed milliseconds between snapshots().
    #img = sensor.snapshot() # Take a picture and return the image.
    img = sensor.snapshot().lens_corr(strength = 1.5, zoom = 1)
    img.mean(2)#均值滤波
    img.draw_cross(120,120)
    img.draw_cross(40,40)
    img.draw_cross(40,200)
    img.draw_cross(200,200)
    img.draw_cross(200,40)
    duration = pyb.elapsed_millis(time_start)
    print(duration)
    '''
    blobs = img.find_blobs([light_threshold])#颜色识别函数
    if blobs:
        max_blob = find_max(blobs)#找到最大色块
        Light_X_POS = max_blob.cx()
        Light_Y_POS = max_blob.cy()

        img.draw_rectangle(max_blob.rect()) # rect，画一个矩形框
        img.draw_cross(max_blob.cx(), max_blob.cy()) # cx, cy中心点画一个十字架
        Light = 1
    else:
        Light = 0
        Light_X_POS = 120
        Light_Y_POS = 120
'''
    blobs = img.find_blobs([red_threshold])#颜色识别函数
    if blobs:
        max_blob = find_max(blobs)#找到最大色块
        X_POS = max_blob.cx()
        Y_POS = max_blob.cy()

        img.draw_rectangle(max_blob.rect()) # rect，画一个矩形框
        img.draw_cross(max_blob.cx(), max_blob.cy()) # cx, cy中心点画一个十字架
        ball = 1
    else:
        ball = 0
        X_POS=120
        Y_POS=120

#    if number <= 2:#求平均数
#        count_x= count_x+X_POS
#        count_y= count_y+Y_POS
#   else :
#        number = 0
#        count_x_output=count_x/2
#        count_y_output=count_y/2
#        count_x=0
#       count_y=0
#    number=number+1
#    print(number)
#    print(count_x)
#    print(X_POS)
#    #rx_buf = "R("+str(count_x_output)+", "+ str(count_y_output)+", "+str(ball)+")O"
    Light_X_POS=120
    Light_Y_POS=120
    Light = 0
    rx_buf = "R("+str(X_POS)+", "+ str(Y_POS)+", "+str(ball)+", "+str(Light_X_POS)+", "+ str(Light_Y_POS)+", "+str(Light)+")O"
    print(rx_buf)
    print(clock.fps())
 #   rx_buf = ""
    if len(rx_buf) > 0:
       i = 0
       while i < len(rx_buf):
           temp = rx_buf[i]
           uart.write(temp)
           i = i + 1;
