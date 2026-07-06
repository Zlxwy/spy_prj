
from robot import xyrobot,Servo  # 导入xyrobot和Servo类
import time,math  # 导入time和math模块
马达定速系数=3  # 定义电机定速系数
g_wheel2R=5.3  # 定义轮子直径（cm）
g_width=15.8  # 定义轮距（cm）
g_sum_encoders=12*48*4  # 定义编码器总数
g_pi=3.1415926  # 定义圆周率
rate_encoder = g_pi*g_wheel2R/g_sum_encoders  # 计算编码器分辨率

Lerr_D=0  # 初始化D误差变量
P_err_D=0  # 初始化P误差变量
P_tmp = 0  # 初始化P临时变量
I_tmp = 0  # 初始化I临时变量
D_tmp = 0  # 初始化D临时变量
#////////////PID需要使用的变量//////////////////////  # PID变量注释
error = 0  # 初始化误差变量
setDenable=2  # 初始化D使能变量
P=0.0  # 初始化P变量
I=0.0  # 初始化I变量
D = 0.0  # 初始化D变量
PID_value = 0.0  # 初始化PID值
decide = 0  # 初始化决策变量
previous_error = 0  # 初始化前一次误差
previous_I = 0  # 初始化前一次I
S1=0  # 初始化S1变量
S2=0  # 初始化S2变量
S3=0  # 初始化S3变量
S4=0  # 初始化S4变量
S5=0  # 初始化S5变量
#PID需要使用的变量  # PID变量注释
g_last=0  # 初始化g_last变量
g_T1=0  # 初始化g_T1变量
g_T2=0  # 初始化g_T2变量
g_T3=0  # 初始化g_T3变量
g_T4=0  # 初始化g_T4变量
g_T5=0  # 初始化g_T5变量
setDindex=10  # 初始化D索引


MaxI=2000  # 初始化最大I值
w_s=[10,60,90]  # 定义权重数组
weights= [100-w_s[0], 100-w_s[1], 100-w_s[2], 0, -(100-w_s[2]), -(100-w_s[1]), -(100-w_s[0])]  # 计算权重数组
P_30 = 20;  I_30  = 0.001;  D_30 = 0           # 30速度下的PID参数
P_40 = 20;  I_40  = 0.001;  D_40  = 0       # 40速度下的PID参数
P_50 = 20;  I_50  = 0.001;  D_50  = 0        # 50速度下的PID参数
P_60 = 15;  I_60  = 0.0002;  D_60  = 0        # 60速度下的PID参数
P_70 = 15;  I_70  = 0.0005;  D_70  = 0             # 70速度下的PID参数
P_80 = 15;  I_80  = 0.0005;  D_80  = 1        # 80速度下的PID参数
P_90 = 20;  I_90  = 0.0005;  D_90  = 1            # 90速度下的PID参数
P_100 = 20;  I_100 = 0.0005;  D_100 = 1  # 100速度下的PID参数

isdebug=0  # 初始化调试标志
isOutline=0  # 初始化出线标志
g_isLR=0  # 初始化左右标志
OutlineTime=0  # 初始化出线时间
# Kp = 15  # 注释掉的Kp值
# Ki = 0  # 注释掉的Ki值
# Kd = 15  # 注释掉的Kd值

S1_threshold=2500  # 定义S1阈值
S2_threshold=2500  # 定义S2阈值
S3_threshold=2500  # 定义S3阈值
S4_threshold=2500  # 定义S4阈值
S5_threshold=2500  # 定义S5阈值
import ssd1306  # 导入ssd1306模块
from machine import I2C, Pin  # 导入I2C和Pin类
display = ssd1306.SSD1306_I2C(  # 初始化SSD1306显示屏
            128, 64,  # 设置显示屏尺寸
            I2C(0,  # 初始化I2C
                scl=Pin(47, Pin.OUT, Pin.PULL_UP),  # 设置SCL引脚
                sda=Pin(48, Pin.OUT, Pin.PULL_UP),  # 设置SDA引脚
                freq=1000000),  # 设置I2C频率
            addr=0x3c)  # 设置显示屏I2C地址
def _getAllADC():  # 定义获取所有ADC值的函数
    return xyrobot.getADC(4),xyrobot.getADC(5),xyrobot.getADC(1),xyrobot.getADC(2),xyrobot.getADC(9)  # 返回5个ADC值
issee=0  # 初始化issee变量
def _write(txt, line, col=0, reverse=False):  # 定义写文本到显示屏的函数
    if not isdebug:  # 如果不是调试模式
        return  # 直接返回
    global display,issee  # 声明全局变量
    issee=issee+1  # 增加issee计数
    if True:  # 条件为真
        font_width=14  # 设置字体宽度
        font_height=14  # 设置字体高度
        x_offset = col * font_width  # 计算x偏移
        y_offset = line*font_height+3  # 计算y偏移
        display.fill_rect(x_offset,y_offset,font_width*len(txt)+14,font_height-6,reverse)  # 填充矩形区域
        display.text(txt,x_offset,y_offset,not reverse)  # 显示文本
        issee=0  # 重置issee计数
test_min_S2=553  # 初始化S2最小值
test_min_S3=845  # 初始化S3最小值
test_min_S4=991  # 初始化S4最小值
test_max_S2=1927  # 初始化S2最大值
test_max_S3=2889  # 初始化S3最大值
test_max_S4=3289  # 初始化S4最大值
def _扫描中间3个光电():  # 定义扫描中间3个光电传感器的函数
    global display,test_min_S2,test_min_S3,test_min_S4,test_max_S2,test_max_S3,test_max_S4  # 声明全局变量
    test_min_S2=9999  # 重置S2最小值
    test_min_S3=9999  # 重置S3最小值
    test_min_S4=9999  # 重置S4最小值
    test_max_S2=0  # 重置S2最大值
    test_max_S3=0  # 重置S3最大值
    test_max_S4=0  # 重置S4最大值
    while True:  # 无限循环
        ADC_S1,ADC_S2,ADC_S3,ADC_S4,ADC_S5=_getAllADC()  # 获取所有ADC值
        if ADC_S2<test_min_S2:  # 如果ADC_S2小于当前最小值
            test_min_S2=ADC_S2  # 更新最小值
        if ADC_S2>test_max_S2:  # 如果ADC_S2大于当前最大值
            test_max_S2=ADC_S2  # 更新最大值

        if ADC_S3<test_min_S3:  # 如果ADC_S3小于当前最小值
            test_min_S3=ADC_S3  # 更新最小值
        if ADC_S3>test_max_S3:  # 如果ADC_S3大于当前最大值
            test_max_S3=ADC_S3  # 更新最大值

        if ADC_S4<test_min_S4:  # 如果ADC_S4小于当前最小值
            test_min_S4=ADC_S4  # 更新最小值
        if ADC_S4>test_max_S4:  # 如果ADC_S4大于当前最大值
            test_max_S4=ADC_S4  # 更新最大值
        _write(str(test_min_S2) + "," + str(test_max_S2),1)  # 显示S2值
        _write(str(test_min_S3) + "," + str(test_max_S3),2)  # 显示S3值
        _write(str(test_min_S4) + "," + str(test_max_S4),3)  # 显示S4值
        display.show()  # 更新显示屏
        if xyrobot.getPinValue(0)==0:  # 如果引脚0值为0
            _write(str(test_min_S2) + "," + str(test_max_S2)+" o="+str((test_min_S2+test_max_S2)//2),1)  # 显示S2值和阈值
            _write(str(test_min_S3) + "," + str(test_max_S3)+" o="+str((test_min_S3+test_max_S3)//2),2)  # 显示S3值和阈值
            _write(str(test_min_S4) + "," + str(test_max_S4)+" o="+str((test_min_S4+test_max_S4)//2),3)  # 显示S4值和阈值
            display.show()  # 更新显示屏
            break  # 退出循环
            
        if xyrobot.getPinValue(3)==0:  # 如果引脚3值为0
            test_min_S2=9999  # 重置S2最小值
            test_min_S3=9999  # 重置S3最小值
            test_min_S4=9999  # 重置S4最小值
            test_max_S2=0  # 重置S2最大值
            test_max_S3=0  # 重置S3最大值
            test_max_S4=0  # 重置S4最大值
def 设置(S1阈值=1300,S2阈值=1300,S3阈值=1300,S4阈值=1300,S5阈值=1300):  # 定义设置阈值函数
    global S1_threshold,S2_threshold,S3_threshold,S4_threshold,S5_threshold  # 声明全局变量
    S1_threshold=S1阈值  # 设置S1阈值
    S2_threshold=S2阈值  # 设置S2阈值
    S3_threshold=S3阈值  # 设置S3阈值
    S4_threshold=S4阈值  # 设置S4阈值
    S5_threshold=S5阈值  # 设置S5阈值


def _setmotor(sp1,sp2):  # 定义设置电机函数
    xyrobot.setMotor(1,sp1)  # 设置电机1速度
    xyrobot.setMotor(2,sp2)  # 设置电机2速度
def setmotor(sp1,sp2):  # 定义设置电机函数
    xyrobot.setMotor(1,sp1)  # 设置电机1速度
    xyrobot.setMotor(2,sp2)  # 设置电机2速度
g_last1=0  # 初始化g_last1变量
g_last5=0  # 初始化g_last5变量
def _getadState():  # 定义获取ADC状态函数
    global OutlineTime,S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error,setDenable,P_err_D,g_T1,g_T2,g_T3,g_T4,g_T5  # 声明全局变量
    global test_min_S3,test_max_S3,g_last1,g_last5,S1_threshold,S2_threshold,S3_threshold,S4_threshold,S5_threshold  # 声明全局变量
    ADC_S1,ADC_S2,ADC_S3,ADC_S4,ADC_S5=_getAllADC()  # 获取所有ADC值
    if ADC_S1<S1_threshold:  # 如果ADC_S1小于阈值
        S1=1  # S1设为1
    else:  # 否则
        S1=0  # S1设为0
    if ADC_S2<S2_threshold:  # 如果ADC_S2小于阈值
        S2=1  # S2设为1
    else:  # 否则
        S2=0  # S2设为0
    if ADC_S3<S3_threshold:  # 如果ADC_S3小于阈值
        S3=1  # S3设为1
    else:  # 否则
        S3=0  # S3设为0
    if ADC_S4<S4_threshold:  # 如果ADC_S4小于阈值
        S4=1  # S4设为1
    else:  # 否则
        S4=0  # S4设为0
    if ADC_S5<S5_threshold:  # 如果ADC_S5小于阈值
        S5=1  # S5设为1
    else:  # 否则
        S5=0  # S5设为0
    if(S1):  # 如果S1为1
        g_last1=1  # 更新g_last1
        g_last5=0  # 更新g_last5
        g_T1=time.ticks_ms()  # 记录时间
        g_last=1  # 更新g_last
    if(S2):  # 如果S2为1
        g_T2=time.ticks_ms()  # 记录时间
        g_last=2  # 更新g_last
        isOutline=0  # 重置出线标志
    if(S3):  # 如果S3为1
        g_T3=time.ticks_ms()  # 记录时间
        g_last=3  # 更新g_last
        isOutline=0  # 重置出线标志
    if(S4):  # 如果S4为1
       
        g_T4=time.ticks_ms()  # 记录时间
        # if(math.fabs(g_T4-g_T1)>100 and g_T4-g_T3>100):  # 注释掉的条件
        g_last=4  # 更新g_last
        isOutline=0  # 重置出线标志
    if(S5):  # 如果S5为1
        g_last=5  # 更新g_last
        g_last1=0  # 更新g_last1
        g_last5=1  # 更新g_last5
        g_T5=time.ticks_ms()  # 记录时间
        
    if S2==0 and S3==0 and S4==0:  # 如果中间三个传感器都为0
        if ADC_S3 > S3_threshold+300:  # 如果ADC_S3大于阈值+300
            if  g_last<3 :  # 如果g_last小于3
                if time.ticks_ms()-g_T2>100 or g_last1==1 :  # 如果时间差大于100或g_last1为1
                    isOutline=1  # 出线标志设为1
            elif g_last>3 :  # 如果g_last大于3
                if time.ticks_ms()-g_T4>100 or g_last5==1 :  # 如果时间差大于100或g_last5为1
                    isOutline=-1  # 出线标志设为-1
            else:  # 否则
                if g_T2> g_T5:  # 如果g_T2大于g_T5
                    isOutline=1  # 出线标志设为1
                else:  # 否则
                    isOutline=-1  # 出线标志设为-1
        
    # if isOutline==0:  # 注释掉的条件
    #     if((g_T2-g_T3>100 and g_T2>g_T4) or time.ticks_ms()-g_T1<100 ):  # 注释掉的条件
    #         isOutline=1  # 注释掉的代码
    #         OutlineTime=time.ticks_ms()  # 注释掉的代码
    #     elif((g_T4-g_T3>100 and g_T4>g_T2) or time.ticks_ms()-g_T5<100):  # 注释掉的条件
    #         isOutline=-1  # 注释掉的代码
    #         OutlineTime=time.ticks_ms()  # 注释掉的代码
    #     elif g_T1>g_T5:  # 注释掉的条件
    #         isOutline=1  # 注释掉的代码
    #     elif g_T5>g_T1:  # 注释掉的条件
    #         isOutline=-1  # 注释掉的代码
    # else:  # 注释掉的条件
    #     OutlineTime=time.ticks_ms()  # 注释掉的代码
def _read_sensor_error():  # 定义读取传感器误差函数
    global S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,weights  # 声明全局变量
    _getadState()  # 获取ADC状态
    '''  # 多行注释开始
    * 光电几种状态  # 说明
    * 黑    黑  黑：直走  # 说明
    * 白    黑  白：直走  # 说明
    * 黑    白  白：大左转  # 说明
    * 白    白  黑：大右转  # 说明
    * 黑    黑  白：左转  # 说明
    * 白    黑  黑：右转  # 说明
    * 白    白  白：出线  # 说明
    * 黑    白  黑：不存在  # 说明
    '''  # 多行注释结束
    if(S2==1 and S3==1 and S4==1):  # 如果S2,S3,S4都为1
        error = weights[3]  # 设置误差为weights[3]
        I=0  # 重置I
    if(S2==1 and S4==1):  # 如果S2和S4都为1
        error = weights[3]  # 设置误差为weights[3]
        I=0  # 重置I
    elif( S2==0 and S3==1 and S4 == 0):  # 如果S2=0,S3=1,S4=0
        error = weights[3]  # 设置误差为weights[3]
        I=0  # 重置I
        # isOutline=0  # 注释掉的代码
    elif(S2==1 and S3==0 and S4==0):  # 如果S2=1,S3=0,S4=0
        error = weights[1]  # 设置误差为weights[1]
        I=0  # 重置I
    elif(S2==0 and S3==0 and S4==1):  # 如果S2=0,S3=0,S4=1
        error = weights[5]  # 设置误差为weights[5]
        I=0  # 重置I
    elif(S2==1 and S3==1 and S4==0):  # 如果S2=1,S3=1,S4=0
        error = weights[2]  # 设置误差为weights[2]
        I=0  # 重置I
    elif(S2==0 and S3==1 and S4==1):  # 如果S2=0,S3=1,S4=1
        error = weights[4]  # 设置误差为weights[4]
        I=0  # 重置I
    elif S2==0 and S3==0 and S4==0 :  # 如果S2=0,S3=0,S4=0
        if(isOutline > 0 ):  # 如果出线标志大于0
            if time.ticks_ms()-g_T3>50:  # 如果时间差大于50
                error = weights[0]  # 设置误差为weights[0]
        elif (isOutline < 0 ) :  # 如果出线标志小于0
            if time.ticks_ms()-g_T3>50:  # 如果时间差大于50
                error = weights[6]  # 设置误差为weights[6]

def _calc_pid(Kp, Ki, Kd):  # 定义计算PID函数
    global S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error,setDenable,P_err_D  # 声明全局变量
    P = error  # 计算P
    I = I + error  # 计算I
    if(I>MaxI):I=MaxI  # 限制I最大值
    if(I<-MaxI):I=-MaxI  # 限制I最小值
    D = error - previous_error  # 计算D
    if( setDenable<=0 and error!=previous_error and isOutline!=0 ):  # 如果条件满足
       setDenable=setDindex  # 设置D使能
       P_err_D=D  # 保存P误差D
       if(isOutline>0):Lerr_D=-1  # 如果出线标志大于0，设置Lerr_D为-1
       else:Lerr_D=1  # 否则设置Lerr_D为1
       
    if(setDenable>0):  # 如果D使能大于0
        if(Lerr_D>0):D=P_err_D  # 如果Lerr_D大于0，设置D为P_err_D
        else:D=-P_err_D  # 否则设置D为-P_err_D
        setDenable=setDenable-1  # 减少D使能计数
    else:  # 否则
        Lerr_D=0  # 重置Lerr_D
    
    PID_value = (Kp * P) + (Ki * I) + (Kd * D)  # 计算PID值
    previous_error = error  # 保存前一次误差
    
#*************************************  # 分隔符
#实际电机控制变化速度  # 注释
#************************************  # 分隔符

def _motor_control(initial_motor_speed):  # 定义电机控制函数
    global S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error  # 声明全局变量
    left_motor_speed = initial_motor_speed - PID_value  # 计算左电机速度
    right_motor_speed = initial_motor_speed + PID_value  # 计算右电机速度
    
    if(left_motor_speed>100):left_motor_speed=100  # 限制左电机最大速度
    if(left_motor_speed<-55):left_motor_speed=-55  # 限制左电机最小速度
    # if(initial_motor_speed<65):  # 注释掉的条件
    #     if(left_motor_speed<-65):left_motor_speed=-65  # 注释掉的代码
    # else:  # 注释掉的条件
    #     if(left_motor_speed<-100):left_motor_speed=-100  # 注释掉的代码
    if(right_motor_speed>100):right_motor_speed=100  # 限制右电机最大速度
    if(right_motor_speed<-55):right_motor_speed=-55  # 限制右电机最小速度
    # if(initial_motor_speed>65):  # 注释掉的条件
    #     if(right_motor_speed<-65):right_motor_speed=-65  # 注释掉的代码
    # else:  # 注释掉的条件
    #     if(right_motor_speed<-100):right_motor_speed=-100  # 注释掉的代码
    if(isdebug):  # 如果是调试模式
        pass  # 空操作
    _setmotor(left_motor_speed,right_motor_speed)  # 设置电机速度
    # _motorsWritePct(left_motor_speed,right_motor_speed)  # 注释掉的代码


#//////////////////////////////pid line，用哪号光电控制停止/////////////////////////////////////////////////////  # 注释
def 巡线遇线停(速度,sensor):  # 定义巡线遇线停函数
    global display,S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error,g_T1,g_T2,g_T3,g_T4,g_T5  # 声明全局变量
    global w_s,weights,isdebug  # 声明全局变量
    error=0  # 重置误差
    speed=速度  # 设置速度
    # _setPIDspeed(speed)  # 注释掉的代码
    g_T3=2  # 初始化g_T3
    g_T4=3  # 初始化g_T4
    g_T2=4  # 初始化g_T2
    g_T5=0  # 初始化g_T5
    g_T1=1  # 初始化g_T1
    P=0  # 重置P
    I=0  # 重置I
    D=0  # 重置D
    regotime=time.ticks_ms()  # 记录开始时间
    w_s=[10,6,1]  # 设置w_s数组
    weights= [w_s[0], w_s[1], w_s[2], 0,-w_s[2], -w_s[1], -w_s[0]]  # 计算weights数组
    # if(led==1):  # 注释掉的条件
    #     g_isLR=1  # 注释掉的代码
    # if(led==5):  # 注释掉的条件
    #     g_isLR=5  # 注释掉的代码
    isdebug=False  # 设置调试标志为False
    Kp,Ki,Kd=_getPID(速度)  # 获取PID参数
    while(True):  # 无限循环
        #得到PID计算出小车的误差  # 注释
        _read_sensor_error()  # 读取传感器误差
        #填入P和D进行计算小车误差的状态  # 注释
        _calc_pid(Kp,Ki,Kd)  # 计算PID
        _write(str(error)+" ,"+str(PID_value) + " "+str(time.ticks_ms()-g_T2),1)  # 显示误差和PID值
        strADC=str(S1)+" "+str(S2)+" "+str(S3)+" "+str(S4)+" "+str(S5)+ " out:"+str(isOutline)  # 拼接ADC状态字符串
        
        _write(strADC,2)  # 显示ADC状态
        _write(str(P) +" " + str(I) +" "+str(D) ,3)  # 显示P,I,D值
        if isdebug:  # 如果是调试模式
            display.show()  # 更新显示屏
        tt51=math.fabs(g_T1-g_T5)  # 计算g_T1和g_T5的绝对差
        if(sensor == 1):  # 如果传感器是1
            if(S1):  # 如果S1为1
                _setmotor(0,0)  # 停止电机
                break  # 退出循环
        elif(sensor == 5):  # 如果传感器是5
            if(S5):  # 如果S5为1
                _setmotor(0,0)  # 停止电机
                break  # 退出循环
        
        elif(sensor == 234):  # 如果传感器是234
            scount=S2+S3+S4  # 计算S2,S3,S4的和
            if(scount==3):  # 如果和为3
                _setmotor(0,0)  # 停止电机
                break  # 退出循环
        else:  # 否则
           if((S1 or S5) and (g_T5>0 and g_T1>0 and tt51<100)):  # 如果条件满足
                _setmotor(0,0)  # 停止电机
                break  # 退出循环
        
        _motor_control(speed)  # 控制电机
        time.sleep_ms(0)  # 休眠0ms
def 巡线时间(速度,时间ms):  # 定义巡线时间函数
    global display,S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error,g_T1,g_T2,g_T3,g_T4,g_T5  # 声明全局变量
    global w_s,weights,isdebug  # 声明全局变量
    error=0  # 重置误差
    # _setPIDspeed(speed)  # 注释掉的代码
    g_T3=2  # 初始化g_T3
    g_T4=3  # 初始化g_T4
    g_T2=4  # 初始化g_T2
    g_T5=0  # 初始化g_T5
    g_T1=1  # 初始化g_T1
    P=0  # 重置P
    I=0  # 重置I
    D=0  # 重置D
    speed=速度  # 设置速度
    
    w_s=[10,6,1]  # 设置w_s数组
    MaxI=2300  # 设置MaxI值
    weights= [w_s[0], w_s[1], w_s[2], 0,-w_s[2], -w_s[1], -w_s[0]]  # 计算weights数组
    # if(led==1):  # 注释掉的条件
    #     g_isLR=1  # 注释掉的代码
    # if(led==5):  # 注释掉的条件
    #     g_isLR=5  # 注释掉的代码
    isdebug=False  # 设置调试标志为False
    Kp,Ki,Kd=_getPID(速度)  # 获取PID参数
    regotime=time.ticks_ms()  # 记录开始时间
    while(True):  # 无限循环
        #得到PID计算出小车的误差  # 注释
        _read_sensor_error()  # 读取传感器误差
        #填入P和D进行计算小车误差的状态  # 注释
        _calc_pid(Kp,Ki,Kd)  # 计算PID
        _write(str(error)+" ,"+str(PID_value) + " "+str(time.ticks_ms()-g_T2),1)  # 显示误差和PID值
        strADC=str(S1)+" "+str(S2)+" "+str(S3)+" "+str(S4)+" "+str(S5)+ " out:"+str(isOutline)  # 拼接ADC状态字符串
        
        _write(strADC,2)  # 显示ADC状态
        _write(str(P) +" " + str(I) +" "+str(D) ,3)  # 显示P,I,D值
        if isdebug:  # 如果是调试模式
            display.show()  # 更新显示屏
        if time.ticks_ms()-regotime>=时间ms:  # 如果时间达到
            _setmotor(0,0)  # 停止电机
            break  # 退出循环
        _motor_control(speed)  # 控制电机
        time.sleep_ms(0)  # 休眠0ms
def 巡线编码(速度,编码值):  # 定义巡线编码函数
    global display,S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error,g_T1,g_T2,g_T3,g_T4,g_T5  # 声明全局变量
    global w_s,weights,isdebug  # 声明全局变量
   
    error=0  # 重置误差
    # _setPIDspeed(speed)  # 注释掉的代码
    g_T3=2  # 初始化g_T3
    g_T4=3  # 初始化g_T4
    g_T2=4  # 初始化g_T2
    g_T5=0  # 初始化g_T5
    g_T1=1  # 初始化g_T1
    P=0  # 重置P
    I=0  # 重置I
    D=0  # 重置D
    speed=速度  # 设置速度
    
    
    # if(led==1):  # 注释掉的条件
    #     g_isLR=1  # 注释掉的代码
    # if(led==5):  # 注释掉的条件
    #     g_isLR=5  # 注释掉的代码
    isdebug=False  # 设置调试标志为False
    
    regotime=time.ticks_ms()  # 记录开始时间
    code1=xyrobot.getCode(1)  # 获取电机1编码
    code2=xyrobot.getCode(2)  # 获取电机2编码
    Kp,Ki,Kd=_getPID(速度)  # 获取PID参数
    while(True):  # 无限循环
        #得到PID计算出小车的误差  # 注释
        _read_sensor_error()  # 读取传感器误差
        #填入P和D进行计算小车误差的状态  # 注释
        _calc_pid(Kp,Ki,Kd)  # 计算PID
        _write(str(error)+" ,"+str(PID_value) + " "+str(time.ticks_ms()-g_T2),1)  # 显示误差和PID值
        strADC=str(S1)+" "+str(S2)+" "+str(S3)+" "+str(S4)+" "+str(S5)+ " out:"+str(isOutline)  # 拼接ADC状态字符串
        
        _write(strADC,2)  # 显示ADC状态
        _write(str(P) +" " + str(I) +" "+str(D) ,3)  # 显示P,I,D值
        if isdebug:  # 如果是调试模式
            display.show()  # 更新显示屏
        cc=math.fabs(xyrobot.getCode(1)-code1)  # 计算编码变化
        if cc>=编码值:  # 如果编码变化达到
            _setmotor(0,0)  # 停止电机
            break  # 退出循环
        _motor_control(speed)  # 控制电机
        time.sleep_ms(0)  # 休眠0ms
def _getPID(speed):  # 定义获取PID参数函数
    global display,S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error,g_T1,g_T2,g_T3,g_T4,g_T5  # 声明全局变量
    global w_s,weights,isdebug  # 声明全局变量
    if(speed<80):  # 如果速度小于80
        w_s=[10,6,1]  # 设置w_s数组
        MaxI=2500  # 设置MaxI值
        weights= [w_s[0], w_s[1], w_s[2], 0,-w_s[2], -w_s[1], -w_s[0]]  # 计算weights数组
        Kp=8  # 设置Kp值
        Ki=0.03  # 设置Ki值
        Kd=0.01  # 设置Kd值
    else:  # 否则
        w_s=[10,8,2]  # 设置w_s数组
        MaxI=2500  # 设置MaxI值
        weights= [w_s[0], w_s[1], w_s[2], 0,-w_s[2], -w_s[1], -w_s[0]]  # 计算weights数组
        Kp=8  # 设置Kp值
        Ki=0.04  # 设置Ki值
        Kd=0.01  # 设置Kd值
    return Kp,Ki,Kd  # 返回PID参数
#自定义转弯  # 注释
def 自定义转弯(左转速=40,右转速=-40,转到光电=3):  # 定义自定义转弯函数
    global display,S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error,g_T1,g_T2,g_T3,g_T4,g_T5  # 声明全局变量
    global w_s,weights,isdebug  # 声明全局变量
    左转速=左转速*马达定速系数  # 计算左转速
    右转速=右转速*马达定速系数  # 计算右转速
    xyrobot.setRPM(1,左转速)  # 设置左电机转速
    xyrobot.setRPM(2,右转速)  # 设置右电机转速
    while True:  # 无限循环
        _getadState()  # 获取ADC状态
        if(转到光电==1 and S1):  # 如果转到光电为1且S1为1
            xyrobot.setRPM(1,0)  # 停止左电机
            xyrobot.setRPM(2,0)  # 停止右电机
            break  # 退出循环
        if(转到光电==2 and S2):  # 如果转到光电为2且S2为1
            xyrobot.setRPM(1,0)  # 停止左电机
            xyrobot.setRPM(2,0)  # 停止右电机
            break  # 退出循环
        if(转到光电==3 and S3):  # 如果转到光电为3且S3为1
            xyrobot.setRPM(1,0)  # 停止左电机
            xyrobot.setRPM(2,0)  # 停止右电机
            break  # 退出循环
        if(转到光电==4 and S4):  # 如果转到光电为4且S4为1
            xyrobot.setRPM(1,0)  # 停止左电机
            xyrobot.setRPM(2,0)  # 停止右电机
            break  # 退出循环
        if(转到光电==5 and S5):  # 如果转到光电为5且S5为1
            xyrobot.setRPM(1,0)  # 停止左电机
            xyrobot.setRPM(2,0)  # 停止右电机
            break  # 退出循环
def 设置机器参数(轮胎直径cm=5.3,轮距cm=15.8,码盘线数=12,减速比=48):  # 定义设置机器参数函数
    global g_sum_encoders,g_wheel2R,g_width,g_sum_encoders,rate_encoder  # 声明全局变量

    g_width=轮距cm  # 设置轮距
    g_wheel2R=轮胎直径cm  # 设置轮胎直径
    g_sum_encoders=码盘线数*减速比*4  # 计算编码器总数
    
    rate_encoder = g_pi*g_wheel2R/g_sum_encoders  # 计算编码器分辨率
def 走距离cm不循线(转速cms=20,距离cm=100):  # 定义走距离不循线函数
    # if 转速cms>40:  # 注释掉的条件
    #     转速cms=40  # 注释掉的代码
    转速cms=转速cms*马达定速系数  # 计算转速
    global g_sum_encoders,g_wheel2R,g_width,g_sum_encoders,rate_encoder  # 声明全局变量
    转速=(转速cms/50)/rate_encoder  # 计算转速
    
    
    code1=xyrobot.getCode(1)  # 获取电机1编码
    code2=xyrobot.getCode(2)  # 获取电机2编码
    xyrobot.setRPM(1,转速)  # 设置左电机转速
    xyrobot.setRPM(2,转速)  # 设置右电机转速
    print(转速)  # 打印转速
    while True:  # 无限循环
        inc_encoder_L=xyrobot.getCode(1)-code1  # 计算左编码变化
        inc_encoder_R=xyrobot.getCode(2)-code2  # 计算右编码变化
        lenth_error = (inc_encoder_L + inc_encoder_R)/2 * rate_encoder  # 计算距离误差
        if(abs(lenth_error)>=abs(距离cm)):  # 如果距离误差达到
            xyrobot.setMotor(1,0)  # 停止左电机
            xyrobot.setMotor(2,0)  # 停止右电机
            break  # 退出循环
def 转弯_速度单位每秒cm数(左转速cm=20,右转速cm=-20,转到角度=90):  # 定义转弯函数
    左转速cm=左转速cm*马达定速系数  # 计算左转速cm
    右转速cm=右转速cm*马达定速系数  # 计算右转速cm
    global display,S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error,g_T1,g_T2,g_T3,g_T4,g_T5  # 声明全局变量
    global w_s,weights,isdebug,g_sum_encoders,g_wheel2R,g_width,g_sum_encoders,rate_encoder  # 声明全局变量
    左转速=(左转速cm/50)/rate_encoder  # 计算左转速
    右转速=(右转速cm/50)/rate_encoder  # 计算右转速
    xyrobot.clearPID(1)  # 清除电机1PID
    xyrobot.clearPID(2)  # 清除电机2PID

    code1=xyrobot.getCode(1)  # 获取电机1编码
    code2=xyrobot.getCode(2)  # 获取电机2编码

    xyrobot.setRPM(1,int(左转速))  # 设置左电机转速
    xyrobot.setRPM(2,int(右转速))  # 设置右电机转速
    while True:  # 无限循环
        inc_encoder_L=xyrobot.getCode(1)-code1  # 计算左编码变化
        inc_encoder_R=xyrobot.getCode(2)-code2  # 计算右编码变化
        lenth_error = (inc_encoder_L- inc_encoder_R) * rate_encoder  # 计算长度误差
        anlge_z_error = (lenth_error / g_width) *180.0/g_pi  # 计算角度误差

        if(anlge_z_error>=转到角度 or anlge_z_error<=-转到角度):  # 如果角度误差达到
            xyrobot.setMotor(1,0)  # 停止左电机
            xyrobot.setMotor(2,0)  # 停止右电机
            break  # 退出循环
        # display.fill_rect(0,0,50,50,False)  # 注释掉的代码

        # display.text(str(anlge_z_error),0,0,True)  # 注释掉的代码
        # display.text(str(xyrobot.getCode(1)),0,15,True)  # 注释掉的代码
        # display.show()  # 注释掉的代码
        
#走编码不巡线  # 注释
def 走编码不循线(左转速=50,右转速=50,编码值=2000):  # 定义走编码不循线函数
    global display,S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error,g_T1,g_T2,g_T3,g_T4,g_T5  # 声明全局变量
    global w_s,weights,isdebug  # 声明全局变量
    
    code1=xyrobot.getCode(1)  # 获取电机1编码
    code2=xyrobot.getCode(2)  # 获取电机2编码
    xyrobot.setRPM(1,左转速*马达定速系数)  # 设置左电机转速
    xyrobot.setRPM(2,右转速*马达定速系数)  # 设置右电机转速
    while True:  # 无限循环
        if math.fabs(xyrobot.getCode(1)-code1)>=编码值 or math.fabs(xyrobot.getCode(2)-code2)>=编码值:  # 如果编码变化达到
            xyrobot.setMotor(1,0)  # 停止左电机
            xyrobot.setMotor(2,0)  # 停止右电机
            break  # 退出循环


def 设置编码方向(m1=1,m2=0,m3=0,m4=0):  # 定义设置编码方向函数
    xyrobot.setMotorDir(1,0,m1)  # 设置电机1编码方向
    xyrobot.setMotorDir(2,0,m2)  # 设置电机2编码方向
    xyrobot.setMotorDir(3,0,m3)  # 设置电机3编码方向
    xyrobot.setMotorDir(4,0,m4)  # 设置电机4编码方向



#舵机抬升角度  # 注释
def 抬升夹子B4(速度):  # 定义抬升夹子函数
    taisheng=Servo(10)  # 初始化舵机
    taisheng.write(速度)  # 设置舵机角度
    
    taisheng=None  # 释放舵机对象


#舵机夹取角度  # 注释
def 夹子张开与关闭A4(角度):  # 定义夹子张开与关闭函数
    jiazi=Servo(7)  # 初始化舵机
    jiazi.write(角度)  # 设置舵机角度
    
    jiazi=None  # 释放舵机对象

import _thread  # 导入_thread模块

def _机械臂升任务(角度=0, 光电阈值=1000):  # 定义机械臂升任务函数
    
    xyrobot.setServo(6,角度)  # 设置舵机角度
    count=光电阈值/10  # 计算计数
    while True:  # 无限循环
        
        time.sleep_ms(10) #10ms检测一次  # 休眠10ms
        count=count-1  # 减少计数
        if count<=0:  # 如果计数小于等于0
            break  # 退出循环
    xyrobot.setServo(6,90)  # 设置舵机到90度
def _机械臂降任务(角度, 光电阈值):  # 定义机械臂降任务函数
    
    xyrobot.setServo(6,角度)  # 设置舵机角度
    count=光电阈值/10  # 计算计数
    while True:  # 无限循环
        
        time.sleep_ms(10) #10ms检测一次  # 休眠10ms
        count=count-1  # 减少计数
        if count<=0 :  # 如果计数小于等于0
            break  # 退出循环
    xyrobot.setServo(6,100)  # 设置舵机到100度
    time.sleep_ms(200) #多转200ms  # 休眠200ms
    xyrobot.setServo(6,90)  # 设置舵机到90度
def 设置机械臂_升(角度=0,光电阈值=1000):  # 定义设置机械臂升函数
    _thread.start_new_thread(_机械臂升任务, (角度, 光电阈值))  # 启动机械臂升任务
def 设置机械臂_降(角度=180,光电阈值=1000):  # 定义设置机械臂降函数
    _thread.start_new_thread(_机械臂降任务, (角度, 光电阈值))  # 启动机械臂降任务
