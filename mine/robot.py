import machine
from machine import Pin,SoftI2C,PWM,ADC
from micropython import const
import framebuf
from neopixel import NeoPixel
import network
from network import WLAN
import time
from machine import CMotor,DEC 
class xyrobot:
    motor=[None,None,None,None]
    coder=[None,None,None,None]
    motorPin=[[18,17],[16,15],[42,41],[40,39]]
    coderPin=[[11,12],[13,14],[35,36],[37,38]]
 
    def waiteS(s):
        time.sleep(s)
    def waiteMs(ms):
        time.sleep_ms(ms)
    '''
  
    超声波

    超声波支持(16,17)(22,21)(0,12)(18,19)四组端口

    '''
    def getUltrasonic(EchoPin,TrigPin):

    #     Trig = Pin(19, Pin.OUT)   #触发超声波模块发射超声波

    #     Echo = Pin(18, Pin.IN)   #接收超声波信号



        Echo = Pin(EchoPin, Pin.IN)

        Trig = Pin(TrigPin, Pin.OUT)



        while True:

            # 给一个高电平触发信号维持20微秒，然后变成低电平

            Trig.value(1)

            time.sleep_us(20)

            Trig.value(0)

            while (Echo.value() == 0):   #如果没有收到信号，再触发一次，发射超声波

                Trig.value(1)

                time.sleep_us(20)

                Trig.value(0)

            if (Echo.value() == 1):     #如果接收到了信号

                ts = time.ticks_us()    #此时定下高电平的开始时刻

                while (Echo.value() == 1):  #如果还是高电平就不断地运行这段代码，直到高电平结束后跳出循环

                    pass

                te = time.ticks_us()   #定下高电平结束时刻

                tc = te - ts      #算出高电平维持时间

        #         distance = str((tc * 0.034) / 2)   #小数点后3位

                distance = str(int((tc * 0.034) / 2))   #取整型后，得到的数值最多为四位数

    #             print('Distance:', distance, 'cm')

            return distance

    # 测试超声波
 
    # while True:

    #     print(getUltrasonic(18,19))

    #     waiteMs(100)

    def InitMotor(port):

        if port<1 or port>4: return
        motorid=port-1
        if xyrobot.motor[motorid] is None:
  
            xyrobot.motor[motorid]=CMotor(motorid,xyrobot.motorPin[motorid][0],xyrobot.motorPin[motorid][1])

            xyrobot.motor[motorid].bind(xyrobot.motorPin[motorid][0],xyrobot.motorPin[motorid][1])

            xyrobot.coder[motorid]=DEC(motorid,xyrobot.coderPin[motorid][0],xyrobot.coderPin[motorid][1])
            xyrobot.coder[motorid].bind_motorid(motorid)
            
            xyrobot.coder[motorid].DCmotor_PID(0.38,0.15,0,10,100,1000)
            if port==1:
                xyrobot.coder[motorid].invert(1)
            if port==2:
                xyrobot.coder[motorid].invert(1)
            if port==3:
                xyrobot.coder[motorid].invert(1)
            if port==4:
                xyrobot.coder[motorid].invert(0)
 
    def setMotor(port,speed):

        xyrobot.InitMotor(port)

        xyrobot.motor[port-1].motor(speed)
    def setDCMotorCL(port,isCL):
        
        if port<1 or port>4: return
        xyrobot.InitMotor(port)
        xyrobot.coder[port-1].setCL(isCL)
    def setRPM(port,RPM):
        if port<1 or port>4: return
        xyrobot.setDCMotorCL(port,1)
        xyrobot.coder[port-1].setRPM(RPM)
    def getRPM(port):
        xyrobot.InitMotor(port)
        
        return xyrobot.coder[port-1].getRPM()
    def getPWM(port):
        xyrobot.InitMotor(port)
        
        return xyrobot.coder[port-1].getPWM()
    def getCode(port):
        
        xyrobot.InitMotor(port)

        return xyrobot.coder[port-1].count()

    '''

    舵机

    舵机需要使用输出引脚(GPIO32、GPIO14、GPIO15、GPIO16、

    GPIO22、GPIO33、GPIO25、GPIO26、GPIO0、GPIO18)。

    '''

    def setServo(pin,angle):
        servo = machine.PWM(machine.Pin(pin),freq=50)
        servo.duty(angle)
    # 测试舵机

    # a = 0

    # while True:

    #     setServoAngle(a,90)

    #     waiteS(1)

    #     setServoAngle(a,30)

    #     waiteS(1)

    '''
    模拟值

    注意：使用WiFi时不能使用ADC2管脚。

    使用ADC1(GPIO34、GPIO37、GPIO32、GPIO36、

    GPIO35、GPIO38、GPIO33、GPIO39)。

    '''
    def getADC(pin):
        adc = ADC(Pin(pin))          # 在引脚上创建ADC对象
        adc.atten(ADC.ATTN_11DB)     # 设置 11dB 衰减输入 (测量电压大致从 0.0v - 3.6v)
    #     print(adc.read())            # 读取测量值, 0-4095 表示电压从 0.0v - 1.0v
        return adc.read()

    # 测试模拟值

    # while True:

    #     print(getADC(36))

    #     waiteMs(100)
    '''

    设置蜂鸣器

    pin:引脚

    音阶：(Hz)

    Do: 262

    Re: 294

    Mi: 330

    Fa: 349

    Sol: 392

    La: 440

    Si: 494



    t:蜂鸣时长

    支持引脚(GPIO32、GPIO14、GPIO15、GPIO16、GPIO22、

    GPIO33、GPIO25、GPIO26、GPIO0、GPIO18)。

    '''
    def setBuzzer(pin,Hz,t):
        global beep
        beep = PWM(Pin(pin,Pin.OUT),freq=0,duty=1000)
        beep.init(duty=1000,freq=Hz)
        time.sleep_ms(t)
        beep.deinit()


    # 测试蜂鸣器

    # a = 18

    # while True:

    #     setBuzzer(a,262,200)

    #     setBuzzer(a,262,400)

    #     setBuzzer(a,349,200)

    #     setBuzzer(a,392,400)

    #     setBuzzer(a,262,200)

    #     setBuzzer(a,349,400)

    #     waiteMs(1000)

    '''

    获取引脚高低电平

    支持输入的引脚（）。

    '''
    def getPinValue(pin):
        p = Pin(pin,Pin.IN)#设置GPIO34为输入
    #     print(pin.value())#察看当前GPIO口状态，0为低电平、1为高电平
        while True:
            if p.value()==1:
                time.sleep_ms(100) #一段时间后再判断按键是否按下，消除抖动
                if p.value()==1:
                    return 1
            if p.value()==0:
                return 0
    '''

    设置引脚高低电平

    支持输出的引脚()。

    '''

    def setPinValue(pin,value):
        p = Pin(pin,Pin.OUT)
        p.value(value)


    '''
    设置3CLED颜色
    支持输出的引脚(GPIO32、GPIO14、GPIO15、GPIO18、GPIO22)。

    '''

    def ColorLED(pin,Red,Green,Blue):
        R = Red           #控制红色强度，值为0到255
        G = Green          #控制绿色强度，值为0到255
        B = Blue          #控制蓝色强度，值为0到255
        p = Pin(pin, Pin.OUT)   # 设置引脚GPIO32来驱动 NeoPixels
        for i in range(3):           #循环写入三次，保证成功设置颜色
            np = NeoPixel(p,1)    # 在GPIO上创建一个 NeoPixel对象，包含1个灯珠
            np[0] = (R,G,B)      #设置第一个灯珠显示数据为RGB
            np.write()               # 写入数据

    # 测试设置3CLED颜色
    # a =32

    # while True:

    #     ColorLED(a,255,0,0)

    #     waiteS(1)

    #     ColorLED(a,0,255,0)

    #     waiteS(1)

    #     ColorLED(a,0,0,255)

    #     waiteS(1)

    #     ColorLED(a,0,0,0)

    #     waiteS(1)

    '''

    设置Wifi AP模式(热点模式)

    例：StartWifiAP("esp32","12345678")

    创建一个Wifi热点

    名称为：esp32，密码为：12345678

    '''

    def StartWifiAP(SSID,PASSWORD):
        global ap
        ap = WLAN(network.AP_IF)#作为网络接入点
        ap.config(essid=SSID,authmode=4,password=PASSWORD)#设置essid（网络名）、authmode（加密模式）、password（密码）
        # ap.ifconfig(('192.168.0.4','255.255.255.0','192.168.0.1','8.8.8.8'))#可自行设置网络参数，如无需修改网络参数请注释本行
        ap.active(True)#启用无线网络
    #     print('Start AP Mode')

    #     ip = ap.ifconfig()#获取当前设备网络参数

    #     print(ip)

    '''

    关闭Wifi AP模式

    '''
    def StopWifiAP():
        ap.active(False)#停用无线网络

    #     print('Stop AP Mode')

    # 测试设置Wifi AP模式

    # StartWifiAP("esp32","12345678")

    # waiteS(30)

    # StopWifiAP()

    '''

    设置Wifi STA模式(连接模式)

    例：StartWifiSTA("mi8","12345678")

    连接到一个名称为：mi8，密码为：12345678 的Wifi网络

    '''

    def StartWifiSTA(ssid,passwd):
        global wlan
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)#激活网络
        wlan.disconnect()#断开WiFi连接
        wlan.connect(ssid,passwd)#连接WiFi
        while(wlan.ifconfig()[0] == '0.0.0.0'):#等待连接
            time.sleep(1)
            if(wlan.isconnected()):
                print('Connection Successful')
                ip = wlan.ifconfig()#获取IP地址
                print(ip)
        return True
    '''

    关闭Wifi STA模式

    '''
    def StopWifiSTA():
        wlan.disconnect()
        print('disconnection')

'''LCD'''

# register definitions

SET_CONTRAST = const(0x81)

SET_ENTIRE_ON = const(0xA4)

SET_NORM_INV = const(0xA6)

SET_DISP = const(0xAE)

SET_MEM_ADDR = const(0x20)
SET_COL_ADDR = const(0x21)
SET_PAGE_ADDR = const(0x22)
SET_DISP_START_LINE = const(0x40)
SET_SEG_REMAP = const(0xA0)
SET_MUX_RATIO = const(0xA8)
SET_COM_OUT_DIR = const(0xC0)
SET_DISP_OFFSET = const(0xD3)
SET_COM_PIN_CFG = const(0xDA)
SET_DISP_CLK_DIV = const(0xD5)
SET_PRECHARGE = const(0xD9)
SET_VCOM_DESEL = const(0xDB)
SET_CHARGE_PUMP = const(0x8D)
    # Subclassing FrameBuffer provides support for graphics primitives

    # http://docs.micropython.org/en/latest/pyboard/library/framebuf.html
class SSD1306(framebuf.FrameBuffer):
    def __init__(self, width, height, external_vcc):
        self.width = width
        self.height = height
        self.external_vcc = external_vcc
        self.pages = self.height // 8
        self.buffer = bytearray(self.pages * self.width)
        super().__init__(self.buffer, self.width, self.height, framebuf.MONO_VLSB, self.width)
        self.init_display()

    def init_display(self):
        for cmd in (
            SET_DISP | 0x00,  # off
            # address setting
            SET_MEM_ADDR,
            0x00,  # horizontal
            # resolution and layout
            SET_DISP_START_LINE | 0x00,
            SET_SEG_REMAP | 0x01,  # column addr 127 mapped to SEG0
            SET_MUX_RATIO,
            self.height - 1,
            SET_COM_OUT_DIR | 0x08,  # scan from COM[N] to COM0
            SET_DISP_OFFSET,
            0x00,
            SET_COM_PIN_CFG,
            0x02 if self.width > 2 * self.height else 0x12,
            # timing and driving scheme
            SET_DISP_CLK_DIV,
            0x80,
            SET_PRECHARGE,
            0x22 if self.external_vcc else 0xF1,
            SET_VCOM_DESEL,
            0x30,  # 0.83*Vcc
            # display
            SET_CONTRAST,
            0xFF,  # maximum
            SET_ENTIRE_ON,  # output follows RAM contents
            SET_NORM_INV,  # not inverted
            # charge pump
            SET_CHARGE_PUMP,
            0x10 if self.external_vcc else 0x14,
            SET_DISP | 0x01,
        ):  # on

            self.write_cmd(cmd)

        self.fill(0)
        self.show()
    def poweroff(self):
        self.write_cmd(SET_DISP | 0x00)

    def poweron(self):
        self.write_cmd(SET_DISP | 0x01)

    def contrast(self, contrast):
        self.write_cmd(SET_CONTRAST)
        self.write_cmd(contrast)

    def invert(self, invert):
        self.write_cmd(SET_NORM_INV | (invert & 1))
    def show(self):
        x0 = 0
        x1 = self.width - 1
        if self.width == 64:
            # displays with width of 64 pixels are shifted by 32
            x0 += 32
            x1 += 32
        self.write_cmd(SET_COL_ADDR)
        self.write_cmd(x0)
        self.write_cmd(x1)
        self.write_cmd(SET_PAGE_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.pages - 1)
        self.write_data(self.buffer)

class SSD1306_I2C(SSD1306):
    def __init__(self, width, height, i2c, addr=0x3C, external_vcc=False):
        self.i2c = i2c
        self.addr = addr
        self.temp = bytearray(2)
        self.write_list = [b"\x40", None]  # Co=0, D/C#=1
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        self.temp[0] = 0x80  # Co=1, D/C#=0
        self.temp[1] = cmd
        self.i2c.writeto(self.addr, self.temp)

    def write_data(self, buf):
        self.write_list[1] = buf
        self.i2c.writevto(self.addr, self.write_list)


class SSD1306_SPI(SSD1306):
    def __init__(self, width, height, spi, dc, res, cs, external_vcc=False):
        self.rate = 10 * 1024 * 1024
        dc.init(dc.OUT, value=0)
        res.init(res.OUT, value=0)
        cs.init(cs.OUT, value=1)
        self.spi = spi
        self.dc = dc
        self.res = res
        self.cs = cs
        import time

        self.res(1)
        time.sleep_ms(1)
        self.res(0)
        time.sleep_ms(10)
        self.res(1)
        super().__init__(width, height, external_vcc)



    def write_cmd(self, cmd):
        self.spi.init(baudrate=self.rate, polarity=0, phase=0)
        self.cs(1)
        self.dc(0)
        self.cs(0)
        self.spi.write(bytearray([cmd]))
        self.cs(1)

    def write_data(self, buf):
        self.spi.init(baudrate=self.rate, polarity=0, phase=0)
        self.cs(1)
        self.dc(1)
        self.cs(0)
        self.spi.write(buf)
        self.cs(1)

    '''
    刷新屏幕
    '''

    def refreshLCD():
        i2c = SoftI2C(scl = Pin(16),sda = Pin(17),freq = 240000000)
        global oled
        oled = SSD1306_I2C(128,64,i2c)

    '''
    显示屏幕
    '''
    def LCDShow():
        oled.show()
    '''
    屏幕输出内容
    data:要输出的内容
    x:x轴坐标
    y:y轴坐标
    例：
    outPutData("Hello World!",0,0)
    在屏幕(0,0)位置输出 Hello World!
    '''
    def outPutData(data,x,y):
        oled.text(data,x,y)

    # 测试LCD,先刷新再显示

    # refreshLCD()

    # outPutData("Hello World!",0,0)

    # LCDShow()

    # waiteMs(1000)

    # refreshLCD()

    # outPutData("Hello Python!",0,8)

    # LCDShow()



