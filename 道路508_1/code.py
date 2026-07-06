from robot import xyrobot,Servo
import time,math

Lerr_D=0;P_err_D=0;P_tmp = 0;I_tmp = 0;D_tmp = 0;error = 0;setDenable=2
P=0.0;I=0.0;D = 0.0;PID_value = 0.0
decide = 0 
previous_error = 0
previous_I = 0
S1=0;S2=0;S3=0;S4=0;S5=0
S1_d=0;S2_d=0;S3_d=0;S4_d=0;S5_d=0
S1_threshold=2500;S2_threshold=2500;S3_threshold=2500;S4_threshold=2500;S5_threshold=2500
g_last=0
g_T1=0
g_T2=0
g_T3=0
g_T4=0
g_T5=0
setDindex=10
速度0=0
aaa=0
舵机角度7=0;舵机角度10=0
离开角度=0
MaxI=2300
w_s=[10,60,90]
#        [90,         40,         10,         0,         -10,            -40,            -90]
weights= [100-w_s[0], 100-w_s[1], 100-w_s[2], 0, -(100-w_s[2]), -(100-w_s[1]), -(100-w_s[0])]
P_30 = 20;  I_30  = 0.001;  D_30 = 0           
P_40 = 20;  I_40  = 0.001;  D_40  = 0      
P_50 = 20;  I_50  = 0.001;  D_50  = 0       
P_60 = 20;  I_60  = 0.001;  D_60  = 1       
P_70 = 20;  I_70  = 0.0005;  D_70  = 1             
P_80 = 20;  I_80  = 0.0005;  D_80  = 1        
P_90 = 20;  I_90  = 0.0005;  D_90  = 1           
P_100 = 20;  I_100 = 0.0005;  D_100 = 1
isdebug=0 
isOutline=0
g_isLR=0
OutlineTime=0

# 获取所有光电传感器的ADC电压值
def _getAllADC():
    return xyrobot.getADC(4),xyrobot.getADC(5),xyrobot.getADC(1),xyrobot.getADC(2),xyrobot.getADC(9)

issee=0

test_min_S2=553
test_min_S3=845
test_min_S4=991
test_max_S2=1927
test_max_S3=2889
test_max_S4=3289

def _setmotor(sp1,sp2):
    xyrobot.setMotor(1,sp1)
    xyrobot.setMotor(2,sp2)
g_last1=0
g_last5=0

def _getadState():
    global OutlineTime,S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error,setDenable,P_err_D,g_T1,g_T2,g_T3,g_T4,g_T5
    global test_min_S3,test_max_S3,g_last1,g_last5,S1_threshold,S2_threshold,S3_threshold,S4_threshold,S5_threshold
    ADC_S1,ADC_S2,ADC_S3,ADC_S4,ADC_S5=_getAllADC()
    if ADC_S1<S1_threshold:
        S1=1
    else:
        S1=0
    if ADC_S2<S2_threshold:
        S2=1
    else:
        S2=0
    if ADC_S3<S3_threshold:
        S3=1
    else:
        S3=0
    if ADC_S4<S4_threshold:
        S4=1
    else:
        S4=0
    if ADC_S5<S5_threshold:
        S5=1
    else:
        S5=0
    if(S1):
        g_last1=1
        g_last5=0
        g_T1=time.ticks_ms()
        g_last=1
    if(S2):
        g_T2=time.ticks_ms()
        g_last=2
        isOutline=0
    if(S3):
        g_T3=time.ticks_ms()
        g_last=3
        isOutline=0
    if(S4):
        g_T4=time.ticks_ms()
        g_last=4
        isOutline=0
    if(S5):
        g_last=5
        g_last1=0
        g_last5=1
        g_T5=time.ticks_ms()
    if S2==0 and S3==0 and S4==0:
        if ADC_S3 > S3_threshold+300:
            if  g_last<3 :
                if time.ticks_ms()-g_T2>100 or g_last1==1 :
                    isOutline=1
            elif g_last>3 :
                if time.ticks_ms()-g_T4>100 or g_last5==1 :
                    isOutline=-1
            else:
                if g_T2> g_T5:
                    isOutline=1
                else:
                    isOutline=-1
    
def _read_sensor_error():
    global S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,weights
    _getadState()
    if(S2==1 and S3==1 and S4==1):
        error = weights[3]
        I=0
    if(S2==1 and S4==1):
        error = weights[3]
        I=0
    elif( S2==0 and S3==1 and S4 == 0):
        error = weights[3]
        I=0
    elif(S2==1 and S3==0 and S4==0):
        error = weights[1]
        I=0
    elif(S2==0 and S3==0 and S4==1):
        error = weights[5]
        I=0
    elif(S2==1 and S3==1 and S4==0):
        error = weights[2]
        I=0
    elif(S2==0 and S3==1 and S4==1):
        error = weights[4]
        I=0
    elif S2==0 and S3==0 and S4==0 :
        if(isOutline > 0 ):
            if time.ticks_ms()-g_T3>50:
                error = weights[0]
        elif (isOutline < 0 ) :
            if time.ticks_ms()-g_T3>50:
                error = weights[6]

def _calc_pid(Kp, Ki, Kd):
    global S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error,setDenable,P_err_D
    P = error
    I = I + error
    if(I>MaxI):I=MaxI
    if(I<-MaxI):I=-MaxI
    D = error - previous_error
    if( setDenable<=0 and error!=previous_error and isOutline!=0 ):
       setDenable=setDindex
       P_err_D=D
       if(isOutline>0):Lerr_D=-1
       else:Lerr_D=1
    if(setDenable>0):
        if(Lerr_D>0):D=P_err_D
        else:D=-P_err_D
        setDenable=setDenable-1
    else:
        Lerr_D=0
    PID_value = (Kp * P) + (Ki * I) + (Kd * D)
    previous_error = error

def _motor_control(initial_motor_speed):
    global S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error
    if PID_value>0:
        left_motor_speed = initial_motor_speed - PID_value * 1.5
        right_motor_speed = initial_motor_speed
    elif PID_value < 0:
        left_motor_speed = initial_motor_speed
        right_motor_speed = initial_motor_speed + PID_value * 1.5
    else:
        left_motor_speed = initial_motor_speed
        right_motor_speed = initial_motor_speed
    if(left_motor_speed< (initial_motor_speed*-1)):left_motor_speed=initial_motor_speed*-1
    if(right_motor_speed<(initial_motor_speed*-1)):right_motor_speed=initial_motor_speed*-1
    if(isdebug):
        pass
    _setmotor(left_motor_speed,right_motor_speed)

def _巡线遇线停(速度,sensor,调整力度=1):
    global S1,S2,S3,S4,S5
    global isOutline,error
    global P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error
    global g_T1,g_T2,g_T3,g_T4,g_T5
    global w_s,weights,isdebug,aaa
    error=0
    speed=速度
    g_T3=2
    g_T4=3
    g_T2=4
    g_T5=0
    g_T1=1
    P=0
    I=0
    D=0
    Kp=4
    Ki=0.05
    Kd=0
    regotime=time.ticks_ms()
    w_s=[10,6,1]
    MaxI=2300
    weights= [w_s[0], w_s[1], w_s[2], 0,-w_s[2], -w_s[1], -w_s[0]]
    isdebug=False
    if 调整力度 == 1 :
        Kp=4
        Ki=0.05
        Kd=0
    elif 调整力度 == 2 :
        Kp=6
        Ki=0.07
        Kd=0
    elif 调整力度 == 3 :
        Kp=8
        Ki=0.1
        Kd=0.01
    while(True):
        _read_sensor_error()
        _calc_pid(Kp,Ki,Kd)
        tt51=math.fabs(g_T1-g_T5)
        if(sensor == 1):
            if(S1):
                break
        elif(sensor == 5):
            if(S5):
                break
        elif(sensor == 24):
            scount=S2+S4
            if(scount==2):
                break
        elif(sensor == 234):
            scount=S2+S3+S4
            if(scount==3):
                break
        elif(sensor == 15):
            scount=S1+S5
            if(scount>=1):
                if(S1 == 1 ):
                    aaa=-1
                elif(S5==1):
                    aaa=1
                break
        elif(sensor == 11):
            if(S1):
                _setmotor(100,100)
                time.sleep(0.1)
                while True:
                    _getadState()
                    if S1 == 0 :
                        break
                break
        elif(sensor == 55):
            if(S5):
                _setmotor(100,100)
                time.sleep(0.1)
                while True:
                    _getadState()
                    if S5 == 0 :
                        break
                break
        _motor_control(speed)
    _setmotor(0,0)
        
def _巡线编码(速度,编码值,sensor):
    global S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error,g_T1,g_T2,g_T3,g_T4,g_T5
    global w_s,weights,isdebug
    error=0
    g_T3=2
    g_T4=3
    g_T2=4
    g_T5=0
    g_T1=1
    P=0
    I=0
    D=0
    speed=速度
    w_s=[10,6,1]
    MaxI=2300
    weights= [w_s[0], w_s[1], w_s[2], 0,-w_s[2], -w_s[1], -w_s[0]]
    isdebug=False
    Kp=8
    Ki=0.09
    Kd=0.01
    if(speed<80):
        w_s=[10,6,1]
        MaxI=2300
        weights= [w_s[0], w_s[1], w_s[2], 0,-w_s[2], -w_s[1], -w_s[0]]
        Kp=7
        Ki=0.08
        Kd=0
    if(speed>=90):
        w_s=[10,8,2]
        MaxI=2300
        weights= [w_s[0], w_s[1], w_s[2], 0,-w_s[2], -w_s[1], -w_s[0]]
        Kp=11
        Ki=0.1
        Kd=0
    regotime=time.ticks_ms()
    code1=xyrobot.getCode(1)
    code2=xyrobot.getCode(2)
    while(True):
        _read_sensor_error()
        _calc_pid(Kp,Ki,Kd)
        cc=math.fabs(xyrobot.getCode(1)-code1)
        if cc>=编码值:
            if(sensor == 1):
                speed=35
                if(S1):
                    _setmotor(0,0)
                    break
            elif(sensor == 0):
                _setmotor(0,0)
                break
        _motor_control(speed)

def _自定义转弯(左转速=100,右转速=-100,转到光电=3):
    global S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error,g_T1,g_T2,g_T3,g_T4,g_T5
    global w_s,weights,isdebug
    xyrobot.setRPM(1,左转速)
    xyrobot.setRPM(2,右转速)
    while True:
        _getadState()
        if(转到光电==1 and S1):
            xyrobot.setRPM(1,0)
            xyrobot.setRPM(2,0)
            break
        if(转到光电==2 and S2):
            xyrobot.setRPM(1,0)
            xyrobot.setRPM(2,0)
            break
        if(转到光电==3 and S3):
            xyrobot.setRPM(1,0)
            xyrobot.setRPM(2,0)
            break
        if(转到光电==4 and S4):
            xyrobot.setRPM(1,0)
            xyrobot.setRPM(2,0)
            break
        if(转到光电==5 and S5):
            xyrobot.setRPM(1,0)
            xyrobot.setRPM(2,0)
            break
        if(转到光电==55 and S5):
            time.sleep(0.1)
            while True:
                _getadState()
                if S5 ==0 :
                    break
            xyrobot.setRPM(1,0)
            xyrobot.setRPM(2,0)
            break
        if(转到光电==11 and S1):
            time.sleep(0.1)
            while True:
                _getadState()
                if S1 ==0 :
                    break
            xyrobot.setRPM(1,0)
            xyrobot.setRPM(2,0)
            break

# 控制两个电机，以指定的转速走指定的编码值，然后停止
#                150       160      1100
def _走编码不循线(左转速=100,右转速=100,编码值=2000):
    global S1, S2, S3, S4, S5 # 全局变量，记录光电传感器的ADC电压值
    global isOutline, error # 全局变量，记录是否超出范围和当前误差
    global P, I, D, g_last,MaxI, Lerr_D, PID_value, previous_error # 全局变量，记录PID参数和上一次误差值
    global g_T1, g_T2, g_T3, g_T4, g_T5 # 全局变量，记录时间戳
    global w_s, weights, isdebug # 全局变量，记录权重和是否调试调试模式
    xyrobot.clearPID(1) # 清除电机1的PID状态
    xyrobot.clearPID(2) # 清除电机2的PID状态
    code1 = xyrobot.getCode(1) # 记录电机1初始编码值
    code2 = xyrobot.getCode(2) # 记录电机2初始编码值
    xyrobot.setRPM(1,左转速) # 设置电机1转速
    xyrobot.setRPM(2,右转速) # 设置电机2转速
    LL = 0
    RR = 0
    while True:
        LL = abs( xyrobot.getCode(1) - code1 ) # 记录现在电机1已走的编码值
        RR = abs( xyrobot.getCode(2) - code2 ) # 记录现在电机2已走的编码值
        if LL>=编码值 or RR>=编码值 : # 如果电机1或电机2已走的编码值大于等于目标编码值
            xyrobot.setMotor(1,0) # 电机1停止
            xyrobot.setMotor(2,0) # 电机2停止
            break # 跳出循环，结束函数

def 设置(车号=1,S1高=1000,S1低=500,S2高=1000,S2低=500,S3高=1000,S3低=500,S4高=1000,S4低=500,S5高=1000,S5低=500,全局速度=60):
    global S1_threshold,S2_threshold,S3_threshold,S4_threshold,S5_threshold,速度0
    S1_threshold=(S1高+S1低)/2
    S2_threshold=(S2高+S2低)/2
    S3_threshold=(S3高+S3低)/2
    S4_threshold=(S4高+S4低)/2
    S5_threshold=(S5高+S5低)/2
    S1_d=S1低;S2_d=S2低;S3_d=S3低;S4_d=S4低;S5_d=S5低
    速度0=全局速度

def _刹车(L0=50,R0=50,刹车时间=0.1):
    if 刹车时间!=0:
        xyrobot.setRPM(1,L0)
        xyrobot.setRPM(2,R0)
        time.sleep(刹车时间)         #延时指定时间
        xyrobot.setRPM(1,0)         #电机停止
        xyrobot.setRPM(2,0)

# 简述：
# 如果"速度"为0，则让控制夹子舵机的PWM直接变化到指定数值
# 如果"速度"不为0，则让控制夹子舵机的PWM一点一点变化到指定数值，也就是以指定的时间到达目标角度
def _夹子升降(角度=0,速度d=0.02):
    global 舵机角度7 # 此变量全局有效,记录升降舵机的角度
    jjj=0;ii=0;kkk=0 # 傻逼学生定义变量名称不规范
    taisheng=Servo(7) # 实例化端口7，为升降舵机
    if 速度d==0: # 如果速度为0
        taisheng.write(角度) # 直接设置角度
        舵机角度7=角度 # 记录当前舵机角度
    else: # 如果速度不为0
        jjj = 角度-舵机角度7 # 当前角度和目标角度的差值
        ii = abs(jjj) # 取绝对值给次数重复语句
        kkk=舵机角度7 # 记录当前舵机角度
        # 通过循环，每次变化1，间隔速度d，以指定的时间到达目标角度
        for count in range(ii): # 重复角度差的次数
            if jjj > 0: # 如果角度差是正数
                kkk = kkk + 1 # 每次变化1
            elif jjj < 0 : # 如果角度差是负数
                kkk = kkk - 1 # 每次变化1
            taisheng.write(kkk) # 舵机角度每次变化1,间隔速度d指定的时间
            time.sleep( 速度d) # 延时指定时间
        舵机角度7=kkk # 记录最后的当前角度
    taisheng = None #关闭舵机

def _夹子开合(角度=0,速度d=0.02):
    global 舵机角度10 # 此变量全局有效,记录夹合舵机的角度
    jjj=0
    ii=0
    kkk=0
    jiazi=Servo(10) # 实例化端口10，为夹合舵机
    if 速度d==0: # 如果速度为0
        jiazi.write(角度) # 直接设置角度
        舵机角度10=角度 # 记录当前舵机角度
    else: # 如果速度不为0
        jjj = 角度-舵机角度10 # 当前角度和目标角度的差值
        ii = abs(jjj) # 取绝对值给次数重复语句
        kkk=舵机角度10 # 记录当前舵机角度
        for count in range(ii): # 重复角度差的次数
            if jjj > 0: # 如果角度差是正数
                kkk = kkk + 1 # 每次变化1
            elif jjj < 0 : # 如果角度差是负数
                kkk = kkk - 1 # 每次变化1
            jiazi.write(kkk) # 舵机角度每次变化1,间隔速度d指定的时间
            time.sleep( 速度d) # 延时指定时间
        舵机角度10 = kkk # 记录最后的当前角度
    jiazi = None # 关闭舵机

def _单光4L巡线(速度=50,比例=0.02,sensor=3,刹车=1,左右=-1):
    global ADC_S1,ADC_S2,ADC_S3,ADC_S4,ADC_S5
    global S1_threshold,S2_threshold,S3_threshold,S4_threshold,S5_threshold
    left_motor_speed = 0
    right_motor_speed = 0
    bbb = 1.5 * (1400/(S4_threshold-S4_d ))                         #定义变量
    while(True):
        ADC_S1,ADC_S2,ADC_S3,ADC_S4,ADC_S5=_getAllADC()
        if(sensor == 2):                #如果检测2号
            if ADC_S2<S2_threshold:
                break
        elif(sensor == 3):              #如果检测3号
            if ADC_S3<S3_threshold:
                break
        elif(sensor == 1):              #如果检测3号
            if ADC_S1<S1_threshold:
                break
        bbb = S4_threshold - ADC_S4     #4光感当前光值与中间值比较
        bbb = bbb * 比例                #乘一个比例,比例值由调用时设定
        if 左右==-1 :
            if bbb>0:
                left_motor_speed = 速度 - bbb*1.5
                right_motor_speed = 速度 
            else:
                left_motor_speed = 速度 
                right_motor_speed = 速度 + bbb*1.5
        elif 左右==1 :
            if bbb>0:
                left_motor_speed = 速度 
                right_motor_speed = 速度 - bbb*1.5
            else:
                left_motor_speed = 速度 + bbb*1.5
                right_motor_speed = 速度
        _setmotor(left_motor_speed,right_motor_speed)

    if 刹车 == 1 :
        _刹车(-100,-100,0.1)                   #完成后,刹车
    else:
        xyrobot.setRPM(1,0)         #电机停止
        xyrobot.setRPM(2,0)

def _单光2L巡线(速度=50,比例=0.02,sensor=4,刹车=1,左右=-1):
    global ADC_S1,ADC_S2,ADC_S3,ADC_S4,ADC_S5
    global S1_threshold,S2_threshold,S3_threshold,S4_threshold,S5_threshold
    bbb = 1.5* (1455/(S2_threshold-S2_d ))
    left_motor_speed = 0
    right_motor_speed = 0 
    while(True):
        ADC_S1,ADC_S2,ADC_S3,ADC_S4,ADC_S5=_getAllADC()
        if(sensor == 4):
            if ADC_S4<S4_threshold:
                break
        elif(sensor == 5):    
            if ADC_S5<S5_threshold:
                break
        elif(sensor == 1):    
            if ADC_S1<S1_threshold:
                break
        elif(sensor == 3):    
            if ADC_S3<S3_threshold:
                break
        bbb = S2_threshold - ADC_S2
        bbb = bbb * 比例 
        if 左右==-1 :
            if bbb>0:
                left_motor_speed = 速度 - bbb*1.5
                right_motor_speed = 速度 
            else:
                left_motor_speed = 速度 
                right_motor_speed = 速度 + bbb*1.5
        elif 左右==1 :
            if bbb>0:
                left_motor_speed = 速度 
                right_motor_speed = 速度 - bbb*1.5
            else:
                left_motor_speed = 速度 + bbb*1.5
                right_motor_speed = 速度
        _setmotor(left_motor_speed,right_motor_speed)
    if 刹车 == 1 :
        _刹车(-100,-100,0.1)                   #完成后,刹车
    else:
        xyrobot.setRPM(1,0)         #电机停止
        xyrobot.setRPM(2,0)

def _单光3L巡线(速度=50,比例=0.02,sensor=2,刹车=1,左右=-1):
    global ADC_S1,ADC_S2,ADC_S3,ADC_S4,ADC_S5
    global S1_threshold,S2_threshold,S3_threshold,S4_threshold,S5_threshold
    left_motor_speed = 0
    right_motor_speed = 0 
    bbb = 1.5* (1295/(S3_threshold-S3_d ))
    while(True):
        ADC_S1,ADC_S2,ADC_S3,ADC_S4,ADC_S5=_getAllADC()
        if(sensor == 4):
            if ADC_S4<S4_threshold:
                break
        elif(sensor == 2):    
            if ADC_S2 < S2_threshold:
                break
        elif(sensor == 1):    
            if ADC_S1 < S1_threshold:
                break
        bbb = S3_threshold - ADC_S3
        bbb = bbb * 比例 
        if 左右==-1 :
            if bbb>0:
                left_motor_speed = 速度 - bbb*1.5
                right_motor_speed = 速度 
            else:
                left_motor_speed = 速度 
                right_motor_speed = 速度 + bbb*1.5
        elif 左右==1 :
            if bbb>0:
                left_motor_speed = 速度 
                right_motor_speed = 速度 - bbb*1.5
            else:
                left_motor_speed = 速度 + bbb*1.5
                right_motor_speed = 速度 
        _setmotor(left_motor_speed,right_motor_speed)
    if 刹车 == 1 :
        _刹车(-100,-100,0.1)                   #完成后,刹车
    else:
        xyrobot.setRPM(1,0)         #电机停止
        xyrobot.setRPM(2,0)

def _任务(任务12345=1,区1=2.5,区2=2.5,区3=2.5):
    if 任务12345 == 1 :
        _物料回收(区1)
    elif 任务12345 == 2 :
        _建设服务区(区1,区2)
    elif 任务12345 == 3 :
        _搭建桥梁(区1,区2,区3)
    elif 任务12345 == 4 :
        _挖掘隧道(速度0)
    elif 任务12345 ==5 :
        _建设加油站(区1,区2)

def _物料回收(区1=2.5):
    global 离开角度
    比例=0.015

    if 区1 < 2.5 :                      #12区，靠左
        _单光4L巡线(50,0.016,2,1,-1)           #4光感走左边缘，到达工程区
        if 区1 == 1 :                   #如果放1区
            _走编码不循线(-10, 160, 600)  #左转一点，对正1区
            time.sleep(0.1)             
            _夹子开合(90,0.005)            #放开易拉罐
            time.sleep(0.1)
            _走编码不循线(10, -160, 600)    #后退到见边框线
            离开角度 = -1                #偏左大
        elif 区1 == 2 :                 #如果放2区
            _夹子开合(90,0.005)            #直接放开易拉罐
            time.sleep(0.1)             #不用后退
            离开角度 = -1                #偏左小
    elif 区1>2.5:
        time.sleep(0.2)                 #右转时，4光感经过黑线，所以要延时
        _单光2L巡线(50,0.016,4,1,1)          #2光感走黑线左边缘，到达工程区
        if 区1 == 3 :                   #如果放3区
            _夹子开合(90,0.005)           #直接放开易拉罐
            time.sleep(0.1) 
            离开角度 = 1                #偏右小
        elif 区1 == 4 :                 #如果放4区
            _走编码不循线(160, -10, 800)  #右转一点，对正4区
            time.sleep(0.1)
            _夹子开合(90,0.005)            #放开易拉罐
            time.sleep(0.1)
            _走编码不循线(-160, 10, 800)    #后退到见边框线
            离开角度 = 1                 #偏右大

def _建设服务区(区1=2.5,区2=2.5):
    global 离开角度
    _夹子开合(70,0) 
    if 区1 < 2.5 :                      #12区，靠左
        _单光4L巡线(50,0.016,1,0,-1)          #4光感走左边缘，到达工程区
        if 区1 == 1 :                   #如果取杯子是1区,可能会放23或34两种组合
            _走编码不循线(-20, 80, 800)
            time.sleep(0.1)
            _夹子开合(0,0.005)               #夹取方块
            time.sleep(0.1)
            _夹子升降(140,0.005)
            time.sleep(0.3)
            _自定义转弯(-120, -120, 5)
            time.sleep(0.1)
            if 区2== 2.5 :
                _走编码不循线(80, -10, 1600)
                离开角度=11
            elif 区2 == 3.5 :
                _走编码不循线(90, -10, 2300)
                离开角度=12
        if 区1 == 2 :                   #如果取杯子是2区,放杯子只有34一种组合
            time.sleep(0.1)
            _夹子开合(0,0.001)            #直接夹杯子
            time.sleep(0.3)             
            _夹子升降(140,0.005)          #提起杯子
            if 区2 == 3.5 :             #如果杯子放34之间
                _走编码不循线(80, -10, 1300)
                离开角度=12
                _走编码不循线(100, 100, 100)
    elif 区1>2.5 :                              #34区,靠右
        time.sleep(0.2)                 #右转时，4光感经过黑线，所以要延时
        _单光2L巡线(50,0.015,4,0,1)          #2光感走黑线左边缘，到达工程区
        if 区1 == 3 :                   #如果取杯子是3区,放杯子只有12一种组合
            time.sleep(0.1)
            _夹子开合(0,0.001)            #夹起杯子
            time.sleep(0.3)             
            _夹子升降(140,0.005)
            if 区2 == 1.5 :             #如果杯子放12之间
                _走编码不循线(-10, 80, 1300)
                离开角度=-22
                _走编码不循线(100, 100, 150)
        if 区1 == 4 :                   #如果取杯子是4区,放杯子有12和23两种组合
            _走编码不循线(80, -20, 800)    #后退一点,4光感退回边框外
            time.sleep(0.1)
            _夹子开合(0,0.002)               #夹取方块
            time.sleep(0.1)
            _夹子升降(140,0.002)
            time.sleep(0.3)
            _自定义转弯(-120, -120, 1)
            time.sleep(0.1)
            if 区2== 2.5 :
                _走编码不循线(-10, 80, 1600)
                离开角度=-21
            elif 区2 == 1.5 :
                _走编码不循线(-10, 90, 2300)
                _走编码不循线(100, 100, 150)
                离开角度=-22
    time.sleep(0.1)
    _走编码不循线(100, 100, 100)
    time.sleep(0.1)
    _走编码不循线(-100, -100, 150)            
    _夹子升降(120,0.02)
    time.sleep(0.1)
    _夹子开合(20,0.02)
    _夹子开合(70,0)
    if 离开角度 == 11 :
        _走编码不循线(-100, 10, 500)
        离开角度 = -1     
    elif 离开角度== 12 :
        _走编码不循线(-100, 10, 900)
        离开角度 = -1 
    elif 离开角度== -22 :
        _走编码不循线(10, -100, 1000)
        离开角度 = 1 
    elif 离开角度== -21 :
        _走编码不循线(10, -100, 400)
        离开角度 = 1        
    _自定义转弯(-100, -100, 3)
    _夹子开合(170,0)

def _搭建桥梁(区1=2.5,区2=2.5,区3=2.5):
    global 离开角度
    _夹子开合(60,0)                   #打开夹子
    if 区1 == 2.5 :                     #取方块是23区之间，中间,可能放1区和4区两种组合
        _单光3L巡线(50,0.015,1,0,-1)          #3光感走左边缘，到达工程区
        _夹子开合(0,0.001)                #夹取方块
        time.sleep(0.3)
        _夹子升降(120,0.005) 
        _走编码不循线(-100, -100, 100)
        if 区2 == 1 :                   #如果放1区
            _走编码不循线(-10, 90, 1300) #左转
            _走编码不循线(-100, -100, 100)
            离开角度=-10
        elif 区2 == 4 :                  #如果放4区
            _走编码不循线(150, -10, 1000) #右转
            _走编码不循线(-100, -100, 100)
            离开角度=1
        _夹子升降(100,0.01)           #放下方块
        time.sleep(0.1)
        _夹子开合(20,0.01)
        _夹子开合(60,0)

    elif 区1 == 1 :                     
        _单光4L巡线(50,0.015,2,0,-1)          #单4光感走黑线左边缘,到达工程区
        _走编码不循线(-120, -120, 350)    #后退一点,4光感退回边框外
        time.sleep(0.1)
        _自定义转弯(-150, 10, 55)        #后左转,至5光感见黑再见白
        _走编码不循线(-150, 10, 300)     #左转多一些
        _自定义转弯(150, 150, 4)         #前进到3见边框
        time.sleep(0.1)
        _自定义转弯(80, -10, 1)         #右转至1见边框左边界
        _刹车(-120,10,0.1)
        time.sleep(0.1)
        _夹子开合(0,0.001)               #夹取方块
        time.sleep(0.3)
        _夹子升降(120,0.001)
        _自定义转弯(-150, -150, 5)      #后退至5光感见黑再见白
        _刹车(100,100,0.05)
        time.sleep(0.1)
        if 区2 == 2.5 :
            _走编码不循线(100, -10, 1000)
            _走编码不循线(100, 100, 100)
            _夹子升降(100,0.01)           #放下方块
            time.sleep(0.1)
            _夹子开合(20,0.01)
            _夹子开合(80,0)
            离开角度=-1
            if 区3 == 4 :
                _自定义转弯(-120, 10, 11)    #后退到见边框线
                _走编码不循线(-120, 50, 1000)
                time.sleep(0.1)
                _走编码不循线(-120, -120, 1050)
                _自定义转弯(10, -120, 2)
                _夹子升降(60,0.001)
                _单光2L巡线(30,0.01,3,0,1)
                _走编码不循线(10, -100, 750)    #后退一点,4光感退回边框外
                time.sleep(0.1)
                _走编码不循线(100, 100, 600)     #左转多一些
                _夹子开合(0,0.001) 
                time.sleep(0.1)
                _夹子升降(80,0.001)
                _夹子升降(140,0)
                time.sleep(0.2)
                _走编码不循线(-100, 10, 500) 
                _走编码不循线(-10, 100, 700)
                _夹子升降(125,0.01)           #放下方块
                time.sleep(0.1)
                _夹子开合(20,0.01)
                _夹子开合(60,0)
                离开角度=1 
        elif 区2 == 4 :
            _走编码不循线(-120, 60, 2100)
            time.sleep(0.1)
            _自定义转弯(10, -120, 2)    #右转至4见黑,并强刹车
            _单光2L巡线(30,0.01,3,1,1)
            _走编码不循线(30, -100, 600)
            time.sleep(0.1)
            _走编码不循线(100, 100, 500)
            _夹子升降(100,0.01)           #放下方块
            time.sleep(0.1)
            _夹子开合(20,0.01)
            _夹子开合(80,0)
            离开角度=2
            if 区3 == 2.5 :
                _自定义转弯(-100, -100, 3)    #后退到见边框线
                time.sleep(0.1)
                _自定义转弯(-120, 10, 11)
                _走编码不循线(-100, 10, 400)
                _夹子升降(60,0.01)
                _走编码不循线(100, 100, 300) 
                _夹子开合(0,0.001)               #夹取方块
                time.sleep(0.3)
                _夹子升降(80,0.001)
                _夹子升降(140,0)           #放下方块
                time.sleep(0.4)
                _走编码不循线(10, -100, 500)
                time.sleep(0.1)
                _走编码不循线(100, -10, 600) 
                time.sleep(0.1)
                _走编码不循线(100, 100, 500) 
                _夹子升降(125,0.01)           #放下方块
                time.sleep(0.1)
                _夹子开合(20,0.01)
                _夹子开合(60,0)
                离开角度=2
    elif 区1 == 4 :
        离开角度=1
    if 离开角度 == 2 :
        _走编码不循线(-100, 10, 600)
        _走编码不循线(-100, 100, 1000)
        _自定义转弯(-100, -100, 4)
        #time.sleep(10.1)
    elif 离开角度== -2 :
        _自定义转弯(-100, -100, 3)
        _走编码不循线(100, -100, 700)
        _自定义转弯(-100, -100, 3)
    elif 离开角度 == -10 :
        _走编码不循线(10, -90, 1200)
        离开角度=-1
    _自定义转弯(-100, -100, 3)


def _挖掘隧道(速度=60):
    global ADC_S1,ADC_S2,ADC_S3,ADC_S4,ADC_S5
    global S1_threshold,S2_threshold,S3_threshold,S4_threshold,S5_threshold    
    _巡线遇线停(速度0, 234,1)
    bbb = 1.5 
    比例=0.026
    偏差=S1_threshold - S5_threshold
    left_motor_speed = 0
    right_motor_speed = 0 
    while(True):
        ADC_S1,ADC_S2,ADC_S3,ADC_S4,ADC_S5=_getAllADC()
        if ADC_S4>S4_threshold and ADC_S3>S3_threshold :
                break
        bbb = ADC_S1 - ADC_S5 - 偏差
        bbb = bbb * 比例 
        if bbb>0:
            left_motor_speed = 速度 
            right_motor_speed = 速度 - bbb*1.5
        else:
            left_motor_speed = 速度 + bbb*1.5
            right_motor_speed = 速度 
        
        _setmotor(left_motor_speed,right_motor_speed)
    _自定义转弯(0,180,3)
    _setmotor(0,0)

def _建设加油站(区1=2.5,区2=2.5):
    global 离开角度
    _夹子开合(70,0)                   #打开夹子
    if 区1 < 2.5 :                      #12区，靠左
        if 区1 == 1 :                   #如果取杯子是1区,可能会放23或34两种组合
            _单光4L巡线(50,0.015,3,1,-1)          #4光感走左边缘，到达工程区
            time.sleep(0.1)
            _走编码不循线(-10, 80, 750)
            time.sleep(0.1)
            _夹子开合(0,0.005)               #夹取方块
            time.sleep(0.3)
            _夹子升降(140,0.005)
            time.sleep(0.3)
            _自定义转弯(-120, -120, 5)
            _刹车(120,120,0.1)
            time.sleep(0.1)
            if 区2== 2 :
                _走编码不循线(80, -10, 1200)
                离开角度=-1
            elif 区2 == 3 :
                _走编码不循线(90, -10, 2100)
                离开角度=11
            elif 区2 == 4 :
                _走编码不循线(100, -10, 2900)
                离开角度=12
        elif 区1==2:
            _单光4L巡线(50,0.015,3,0,-1)          #4光感走左边缘，到达工程区
            _走编码不循线(100, 100, 200)
            time.sleep(0.1)
            _夹子开合(0,0.001)               #夹取方块
            time.sleep(0.3)
            _夹子升降(140,0.001)
            time.sleep(0.3)
            if 区2==1 :
                _走编码不循线(-10, 100, 800)
                离开角度=-12
            elif 区2==3 :
                _走编码不循线(100, -10, 800)
                离开角度=11
            elif 区2==4 :
                _走编码不循线(100, -10, 1600)
                离开角度=12           
    elif 区1>2.5:
        if 区1==3:
            time.sleep(0.1)
            _单光2L巡线(50,0.015,3,0,1)          #4光感走左边缘，到达工程区
            _走编码不循线(100, 100, 200)
            time.sleep(0.1)
            _走编码不循线(-10, 100, 50)
            _夹子开合(0,0.001)               #夹取方块
            time.sleep(0.3)
            _夹子升降(140,0.001)
            time.sleep(0.3)
            if 区2==1 :
                _走编码不循线(-10, 100, 1600)
                离开角度=-22
            elif 区2==2 :
                _走编码不循线(-10, 100, 800)
                离开角度=-21
            elif 区2==4 :
                _走编码不循线(100, -10, 800)
                离开角度=22   
        elif 区1 == 4 :                   #如果取杯子是1区,可能会放23或34两种组合
            time.sleep(0.1)
            _单光2L巡线(50,0.015,3,1,1)          #4光感走左边缘，到达工程区
            time.sleep(0.1)
            _走编码不循线(80, -10, 750)
            time.sleep(0.1)
            _夹子开合(0,0.005)               #夹取方块
            time.sleep(0.3)
            _夹子升降(140,0.005)
            time.sleep(0.3)
            _自定义转弯(-120, -120, 1)
            _刹车(120,120,0.1)
            time.sleep(0.1)
            if 区2== 3 :
                _走编码不循线(-10, 80, 1100)
                离开角度=1
            elif 区2 == 2 :
                _走编码不循线(-10, 90, 2000)
                离开角度=-21
            elif 区2 == 1 :
                _走编码不循线(-10, 100, 2800)
                离开角度=-22
    time.sleep(0.1)
    _夹子升降(120,0.01)
    time.sleep(0.1)
    _夹子开合(70,0.01)
    if 离开角度 == 11 :
        _走编码不循线(-100, 10, 700)
        离开角度 = -1
    elif 离开角度 == 12 :
        _走编码不循线(-100, 10, 1600)
        离开角度 = -1
    elif 离开角度 == -12 :
        _走编码不循线(10, -100, 650)
        离开角度 = -1
    elif 离开角度 == -22 :
        _走编码不循线(10, -100, 1600)
        离开角度 = 1
    elif 离开角度 == -21 :
        _走编码不循线(10, -100, 700)
        离开角度 = 1
    elif 离开角度 == 22 :
        _走编码不循线(-100, 10, 700)
        离开角度 = 1
    _自定义转弯(-100, -100, 3)
    time.sleep(0.1)

def 启动(启动距离 = 1100):
    global 舵机角度7,舵机角度10
    _夹子升降(60,0)
    _夹子开合(170,0)
    _走编码不循线(150, 160, 启动距离)
    #_巡线编码(80,9000,1)

#          1        2          0      0       0      0          0
def 路口A(入口1=1,出口1234=3,任务0123=0,区1=2.5,区2=2.5,区3=2.5,完成返回路口=2):
    _夹子升降(60,0)
    if 任务0123 == 1:                       #如果任务1,减速前进
        _夹子开合(90,0)
        _巡线遇线停(60, 5,1)
    else:
        _夹子开合(170,0)
        _巡线遇线停(速度0, 5,1)
    if 出口1234 == 2:                       #判断出口，2直接通过，3则完成指定任务
        _走编码不循线(180, 180, 500)        
    elif 出口1234 == 3 and 任务0123>0 :     #进入任务区
        if 任务0123 == 1:                   #如果是任务1，靠近路口马上夹易拉罐
            _夹子开合(10,0)
        if 区1 < 2.5 :                      #1和2，靠左
            _走编码不循线(150, 150, 1900)     
            _自定义转弯(150, -150, 5)
            _自定义转弯(60, -60, 4)
            _刹车(-100,100,0.1)
            _任务(任务0123,区1,区2,区3)      
        elif 区1 > 2.5 :                    #3和4，靠右
            _走编码不循线(150, 150, 1100)    
            _自定义转弯(150, -150, 5)
            _自定义转弯(90, -90, 2)
            _任务(任务0123,区1,区2,区3)      
        elif 区1 == 2.5 :                   #2和3之间
            _走编码不循线(150, 150, 1600)     
            _自定义转弯(150, -150, 5)
            _自定义转弯(60, -60, 3)
            _刹车(-100,100,0.1)
            _任务(任务0123,区1,区2,区3)      
        if 完成返回路口 == 2 :               #唯一返回路口,2
            _走编码不循线(-150, -150, 900)
            if 离开角度 ==-1 :              #完成任务后,根据返回的状态处理离开动作
                _自定义转弯(-150, 150, 1)
                _自定义转弯(-60, 60, 3)
            elif 离开角度 ==1:    
                _自定义转弯(-180, 180, 5)
                _自定义转弯(-80, 80, 3)

def 路口B(入口1234=1,出口1234=3,任务0123=0,区1=2.5,区2=2.5,区3=2.5,完成返回路口=2):
    _夹子升降(60,0)
    if 任务0123 == 1:                       
        _夹子开合(90,0)
    else:
        _夹子开合(170,0)
    if 入口1234==1:    
        if 出口1234 == 2 :
            if 任务0123 == 1:               #1入口是弧线,要准确夹到易拉罐,需特殊走线
                _巡线编码(70,4500,0)
                time.sleep(0.1)
                _自定义转弯(-10, 150, 5)
                _自定义转弯(180, -10, 4)
                #time.sleep(10)
                _单光4L巡线(45,0.028,1,0,1)
                _夹子开合(0,0) 
                time.sleep(0.3)             #_走编码不循线(150, 150, 400)
            else:
                _巡线遇线停(70, 1,1)                                          
            if 区1 > 2.5 :                  #34，靠右
                _走编码不循线(150, 150, 1500)    
                _自定义转弯(-150, 150, 5)
                _自定义转弯(-60, 60, 2)
                _刹车(100,-100,0.1)
                _任务(任务0123,区1,区2,区3)      
            elif 区1 < 2.5 :                #12，靠左
                _走编码不循线(150, 150, 700)   
                _走编码不循线(-150, 150, 200)
                _自定义转弯(-150, 150, 5)
                time.sleep(0.1)
                _自定义转弯(-100, 100, 4)
                time.sleep(0.1)
                _任务(任务0123,区1,区2,区3)      
            elif 区1 == 2.5 :               #2.5,中间
                _走编码不循线(150, 150, 1000)    
                _自定义转弯(-150, 150, 5)
                _自定义转弯(-120, 120, 3) 
                _任务(任务0123,区1,区2,区3)      
            if 离开角度 ==1:                 #离开的动作
                _走编码不循线(150, -150, 2500) 
                _自定义转弯(10, -180, 4)  
                time.sleep(0.1)
                _巡线遇线停(速度0, 5,1)
                _走编码不循线(180, 180, 1000)
            elif 离开角度 == -1 :   
                _走编码不循线(-150, 150, 2500) 
                _自定义转弯(-180, 10, 2)  
                time.sleep(0.1)
                _巡线遇线停(速度0, 5,1)
                _走编码不循线(180, 180, 1000)
            if 完成返回路口 == 3 :           #完成后返回3或4路线
                _自定义转弯(-150, 150, 5)
                _自定义转弯(-150, 150, 2)
            elif 完成返回路口 == 4 :
                _自定义转弯(-150, 150, 2)
        elif 出口1234 == 3:
            _巡线遇线停(70, 1,1)
            _走编码不循线(150,150, 1200)
            _自定义转弯(-120, 120, 3)
            time.sleep(0.1)
        elif 出口1234 == 4 :
            _巡线遇线停(70, 1,1)
            #time.sleep(10)
            _走编码不循线(180, 50, 1300)
            _自定义转弯(150, -150, 4)
            time.sleep(0.1)
    elif 入口1234 == 3:                 #如果入口3,只需4出口
        _巡线遇线停(速度0, 5,1)
        _走编码不循线(180, 180, 2100)
        if 出口1234 == 4:
            time.sleep(0.1)
            _走编码不循线(-150, 150, 200)
            _自定义转弯(-150, 150, 2)
    elif 入口1234 == 4:                 #如果入口4,只需3出口;一般不会从4入口进
        _巡线遇线停(速度0, 5,1)
        _走编码不循线(150, 150, 1400)
        if 出口1234 == 3:
            time.sleep(0.1)
            _走编码不循线(150, -150, 200)
            _自定义转弯(150, -150, 1)
            _自定义转弯(150, -150, 4)

def 路口C(入口1234=1,出口1234=3,任务0123=0,区1=2.5,区2=2.5,区3=2.5,完成返回路口=2):
    _夹子升降(60,0)
    if 任务0123 == 1:
        _夹子开合(90,0)
    else:
        _夹子开合(170,0)
    if 入口1234==1:
        if 出口1234 == 2 :                  #如果出口2,做隧道
            #time.sleep(10)
            _巡线遇线停(速度0, 15,1)
            _走编码不循线(180, 180, 1200)
            _自定义转弯(-150, 150, 2)
            _任务(任务0123,区1,区2,区3)
        elif 出口1234==3:                   #出口3,做其它任务       
            
            if 任务0123==1:
                _巡线遇线停(60, 15,1)
                _夹子开合(0,0) 
                time.sleep(0.3)
            else:
                _巡线遇线停(速度0, 15,1)
            _走编码不循线(150, 150, 1200)    #因路口离工程区较远,分两段走
            _自定义转弯(150, -150, 4)
            _巡线编码(速度0,7000,0)
            if 区1<2.5:
                _自定义转弯(0, 160, 55)
                _走编码不循线(150, 150, 700)
                _自定义转弯(160, -10, 5)
                _自定义转弯(60, -10, 4)
                _刹车(-100,10,0.1)
            elif 区1==2.5:
                _自定义转弯(0, 180, 55)
                _走编码不循线(180, 180, 300)
                _自定义转弯(180, -10, 4)
                _自定义转弯(60, -10, 3)
                _刹车(-100,10,0.05)
            elif 区1>2.5:    
                _自定义转弯(0, 180, 5)
                _自定义转弯(180, 0, 2)
            _任务(任务0123,区1,区2,区3)
            if 完成返回路口==2:
                if 离开角度 ==1:
                    _走编码不循线(150, -150, 2500) 
                    _自定义转弯(10, -180, 4)  
                    time.sleep(0.1)
                elif 离开角度 == -1 :   
                    _走编码不循线(-150, 150, 2500) 
                    _自定义转弯(-180, 10, 2)  
                _巡线遇线停(速度0, 5,1)
                _走编码不循线(180, 180, 1200)
                _自定义转弯(150, -150, 4)
                _任务(4,0,0,0)

def 路口D(入口1234=1,出口1234=2,任务0123=0,区1=2.5,区2=2.5,区3=2.5,完成返回路口=1):
    _夹子升降(60,0)
    if 任务0123 == 1:
        _夹子开合(90,0)
    else:
        _夹子开合(170,0)
    if 入口1234==1:
        if 出口1234==2:                       #出口2,做任务      
            if 任务0123==1:
                _巡线遇线停(60, 1,1)
                _夹子开合(0,0) 
                time.sleep(0.3)
            else:
                _巡线遇线停(速度0, 1,1)
            if 区1 < 2.5 :                  #12，靠左
                _走编码不循线(150, 150, 900)     
                _自定义转弯(-150, 150, 2)
                _自定义转弯(-80, 80, 4)
                time.sleep(0.1)
                _任务(任务0123,区1,区2,区3)      
            elif 区1 > 2.5 :                 #34，靠右
                _走编码不循线(150, 150, 1800)     
                _自定义转弯(-150, 150, 1)
                #_自定义转弯(-60, 60, 2)
                _刹车(100,-100,0.05)
                time.sleep(0.1)
                _任务(任务0123,区1,区2,区3) 
            elif 区1 == 2.5 :
                _走编码不循线(150, 150, 1000)    
                _自定义转弯(-150, 150, 1)
                _自定义转弯(-120, 120, 3) 
                _任务(任务0123,区1,区2,区3)    
            if 完成返回路口 == 1 :
                if 离开角度 ==1:
                    _走编码不循线(150, -150, 2400) 
                    _自定义转弯(10, -180, 4)  
                    time.sleep(0.1)
                elif 离开角度 == -1 :   
                    _走编码不循线(-150, 150, 2500) 
                    _自定义转弯(-180, 10, 2)  
                
                _巡线遇线停(速度0, 5,1)
                _走编码不循线(180, 180, 1200)
                _自定义转弯(180, -180, 5)
                _自定义转弯(80, -80, 4)
            elif 完成返回路口 == 3 :
                _走编码不循线(150, -150, 2000)                 
                if 离开角度 ==1:
                    _自定义转弯(150, -150, 5)
                elif 离开角度 == -1 :   
                    _自定义转弯(150, -150, 4)  
                _自定义转弯(150, 110, 1)
                time.sleep(0.1)
                _自定义转弯(-10, 150, 3)

        if 出口1234==3:
            _巡线遇线停(速度0, 1,1)
            _自定义转弯(150, 150, 5)
            _自定义转弯(150, -10, 3)

def 路口E(入口1234=3,出口1234=2):    
    if 入口1234==3:
        if 出口1234 == 2 :    
            _巡线遇线停(速度0, 1,1)
            _走编码不循线(180, 180, 1000)
            _自定义转弯(150, -150, 4)
    elif 入口1234==1:
        if 出口1234==2:
            _巡线遇线停(速度0, 55,1)
        if 出口1234==3:    
            _巡线遇线停(速度0, 55,1)
            _走编码不循线(180, 150, 1000)
            _走编码不循线(180, -180, 600)
            _自定义转弯(150, -150, 4)


def 路口F(入口1234=1,出口1234=3,任务0123=0,区1=2.5,区2=2.5,区3=2.5,完成返回路口=3):
    _夹子升降(60,0)
    if 任务0123 == 1:
        _夹子开合(90,0)
    else:
        _夹子开合(170,0)
    if 入口1234==1:
        if 出口1234 == 2 :
            if 任务0123==1:
                _巡线遇线停(60, 5,1)
                _夹子开合(0,0) 
                time.sleep(0.3)
            else:
                _巡线遇线停(速度0, 5,1)
            if 区1 < 2.5 :                  #12，靠左
                _走编码不循线(180, 180, 1200)     
                _自定义转弯(-150, 150, 3)
                time.sleep(0.1)
                _任务(任务0123,区1,区2,区3)      
            elif 区1 > 2.5 :                 #34，靠右
                _走编码不循线(180, 180, 600)  
                time.sleep(0.1)   
                _走编码不循线(-10, 180, 4000)
                _自定义转弯(-150, 10, 1)
                _任务(任务0123,区1,区2,区3) 
            elif 区1 == 2.5 :

                _走编码不循线(180, 180, 450)  
                time.sleep(0.1)   
                _走编码不循线(-10, 180, 3100)
                _自定义转弯(-150, 10, 3)
                _任务(任务0123,区1,区2,区3)    
            if 完成返回路口 == 3 :
                if 离开角度 ==1:
                    _走编码不循线(150, -150, 2400) 
                    _自定义转弯(10, -180, 4)  
                    time.sleep(0.1)
                elif 离开角度 == -1 :   
                    _走编码不循线(-150, 150, 2500) 
                    _自定义转弯(-200, 10, 2)  
                if 完成返回路口 == 3 :
                    _巡线遇线停(速度0, 5,1)
                    _走编码不循线(180, 180, 1500)
                    _自定义转弯(150, -150, 4)
                    _刹车(-100,100,0.05)
                    #_自定义转弯(80, -80, 4)      
        elif 出口1234==3:                       #出口2,做任务
            _巡线遇线停(速度0, 5,1)
            _走编码不循线(180, 180, 1300)
            _自定义转弯(160, -160, 4)
            _刹车(-100,100,0.1)

def 路口G(入口1234=1,出口1234=2,任务0123=0,区1=2.5,区2=2.5,区3=2.5,完成返回路口=3):
    _夹子升降(60,0)
    if 任务0123 == 1:
        _夹子开合(90,0)
    else:
        _夹子开合(170,0)
    if 入口1234==1:
        if 出口1234==2:                       #出口2,做任务      
            if 任务0123==1:
                _巡线遇线停(60, 1,1)
                _夹子开合(0,0) 
                time.sleep(0.3)
            else:
                _巡线遇线停(速度0, 1,1)
            if 区1 < 2.5 :                  #12，靠左
                _走编码不循线(150, 150, 800)     
                _自定义转弯(-150, 150, 2)
                _自定义转弯(-80, 80, 4)
                time.sleep(0.1)
                _任务(任务0123,区1,区2,区3)      
            elif 区1 > 2.5 :                 #34，靠右
                _走编码不循线(150, 150, 1600)     
                _自定义转弯(-150, 150, 1)
                #_自定义转弯(-60, 60, 2)
                _刹车(100,-100,0.05)
                time.sleep(0.1)
                _任务(任务0123,区1,区2,区3) 
            elif 区1 == 2.5 :
                _走编码不循线(150, 150, 900)    
                _自定义转弯(-150, 150, 1)
                _自定义转弯(-120, 120, 3) 
                _任务(任务0123,区1,区2,区3)    
            if 完成返回路口 == 3 :
                if 离开角度 ==1:
                    _走编码不循线(150, -150, 2400) 
                    _自定义转弯(10, -180, 4)  
                    time.sleep(0.1)
                elif 离开角度 == -1 :   
                    _走编码不循线(-150, 150, 2500) 
                    _自定义转弯(-180, 10, 2)  
                _巡线遇线停(速度0, 5,1)
                _走编码不循线(180, 180, 1200)
                _自定义转弯(-150, 150, 2)
                _刹车(100,-100,0.05)
        elif 出口1234==3:
            _巡线遇线停(速度0, 1,1)
            _走编码不循线(180, 180, 1000)
            _自定义转弯(150, -150, 4)
            

def 路口H(入口1234=3,出口1234=1,任务0123=0,区1=2.5,区2=2.5,区3=2.5,完成返回路口=1):
    _夹子升降(60,0)
    if 任务0123 == 1:
        _夹子开合(90,0)
    else:
        _夹子开合(170,0)
    if 入口1234==3:
        if 出口1234 == 1 :
            _巡线遇线停(速度0, 1,1)
            _自定义转弯(150, 150, 5)
            _走编码不循线(150, 150, 800)
            time.sleep(0.1)
            _自定义转弯(-150, 150, 2)
        elif 出口1234==4:
            if 任务0123==1:
                _巡线遇线停(60, 1,1)
                _夹子开合(0,0) 
                time.sleep(0.3)
            else:
                _巡线遇线停(速度0, 1,1)
            if 区1 < 2.5 :                  #12，靠左
                _走编码不循线(150, 150, 1000)     
                time.sleep(0.1)
                _自定义转弯(-150, 150, 5)
                _自定义转弯(-100, 100, 4)
                time.sleep(0.1)
                _任务(任务0123,区1,区2,区3)      
            elif 区1 > 2.5 :                 #34，靠右
                _走编码不循线(150, 150, 2000)  
                time.sleep(0.1)   
                _自定义转弯(-150, 150, 5)
                _自定义转弯(-150, 150, 1)
                _刹车(120,-120,0.1)
                _任务(任务0123,区1,区2,区3) 
            elif 区1 == 2.5 :
                _走编码不循线(150, 150, 1300)  
                time.sleep(0.1) 
                _自定义转弯(-150, 150, 5)
                _自定义转弯(-100, 100, 3)
                _任务(任务0123,区1,区2,区3)    
            if 完成返回路口 == 1 :
                _走编码不循线(-150, -150, 1600)
                if 离开角度 ==-1 :              #完成任务后,根据返回的状态处理离开动作
                    _走编码不循线(180, -180, 400)
                    _自定义转弯(180, -180, 5)
                    _自定义转弯(100, -100, 4)
                elif 离开角度 ==1:    
                    _自定义转弯(180, -180, 5)
                    _自定义转弯(60, -60, 4)

def 路口I(入口1234=1,出口1234=2):    
    if 入口1234==1:
        _巡线遇线停(速度0, 5,1)
        if 出口1234 == 2 :
            _走编码不循线(180, 180, 1400)
            _自定义转弯(-150, 150, 2)
        elif 出口1234==3:
            _走编码不循线(180, 180, 1300)
            _自定义转弯(150, -150, 4)
    elif 入口1234==3:
        if 出口1234 == 2 :    
            _巡线遇线停(速度0, 1,1)
            _走编码不循线(180, -10, 700)

def 路口J(入口1234=1,出口1234=2,任务0123=0,区1=2.5,区2=2.5,区3=2.5,完成返回路口=1):
    _夹子升降(60,0)
    if 任务0123 == 1:
        _夹子开合(90,0)
    else:
        _夹子开合(170,0)
    if 入口1234 == 1 :
        if 任务0123==1:
            _巡线遇线停(60, 1,1)
            _夹子开合(0,0) 
            time.sleep(0.3)
        else:
            _巡线遇线停(速度0, 1,1)
        if 出口1234==2:                     #1入3出,一般不会选择
            if 区1 < 2.5 :                  #12，靠左
                _走编码不循线(150, 150, 900)     
                _自定义转弯(-150, 150, 3)
                #_自定义转弯(-80, 80, 4)
                time.sleep(0.1)
                _任务(任务0123,区1,区2,区3)      
            elif 区1 > 2.5 :                 #34，靠右
                _走编码不循线(150, 150, 1700)     
                _自定义转弯(-150, 150, 1)
                #_自定义转弯(-60, 60, 2)
                #_刹车(100,-100,0.1)
                time.sleep(0.1)
                _任务(任务0123,区1,区2,区3) 
            elif 区1 == 2.5 :
                _走编码不循线(150, 150, 1500)  
                _自定义转弯(-150, 150, 2)
                #_自定义转弯(-100, 100, 3)
                time.sleep(0.1)
                _任务(任务0123,区1,区2,区3) 
            if 离开角度 ==1:
                _走编码不循线(150, -150, 2500) 
                _自定义转弯(10, -180, 4)  
                time.sleep(0.1)
            elif 离开角度 == -1 :   
                _走编码不循线(-150, 150, 2500) 
                _自定义转弯(-180, 10, 2)  
            if 完成返回路口 == 1 :
                _巡线遇线停(速度0, 5,1)
                _走编码不循线(180, 180, 1200)
                _自定义转弯(180, -180, 5)
                _自定义转弯(80, -80, 4)
        elif 出口1234==3:
            if 区1 < 2.5 :                      #1和2，靠左
                _走编码不循线(150, 150, 1900)     
                _自定义转弯(150, -150, 5)
                time.sleep(0.1)
                #_自定义转弯(60, -60, 4)
                #_刹车(-100,100,0.1)
                _任务(任务0123,区1,区2,区3)      
            elif 区1 > 2.5 :                    #3和4，靠右
                _走编码不循线(150, 150, 1100)    
                _自定义转弯(150, -150, 3)
                #_自定义转弯(90, -90, 2)
                time.sleep(0.1)
                _任务(任务0123,区1,区2,区3)      
            elif 区1 == 2.5 :                   #2和3之间
                _走编码不循线(150, 150, 1600)     
                _走编码不循线(150, -150, 800)
                _自定义转弯(150, -150, 4)
                _自定义转弯(60, -60, 3)
                _任务(任务0123,区1,区2,区3)      
            if 离开角度 ==1:
                _走编码不循线(150, -150, 2400) 
                _自定义转弯(10, -180, 4)  
                time.sleep(0.1)
            elif 离开角度 == -1 :   
                _走编码不循线(-150, 150, 2500) 
                _自定义转弯(-180, 10, 2)  
            if 完成返回路口 == 1 :
                _巡线遇线停(速度0, 1,1)
                _走编码不循线(180, 180, 1200)
                _自定义转弯(-180, 180, 1)
                _自定义转弯(-80, 80, 2)    
    elif 入口1234==2:
        if 出口1234==3:
            _巡线遇线停(速度0, 5,1)
            if 区1 < 2.5 :                  #12，靠左
                _走编码不循线(180, 180, 500)     
                _自定义转弯(-10, 180, 55)
                _自定义转弯(150, -10, 4)
                _刹车(-100,10,0.1)
                _任务(任务0123,区1,区2,区3)      
            elif 区1 > 2.5 :                 #34，靠右
                _走编码不循线(180, 180, 500)     
                _自定义转弯(180, -10, 11)
                _刹车(-180,10,0.1)
                _自定义转弯(-10, 100, 2)
                _刹车(10,-100,0.15)
                #time.sleep(10)
                _任务(任务0123,区1,区2,区3) 
            elif 区1 == 2.5 :
                _走编码不循线(180, 180, 500)  
                _自定义转弯(-10, 150, 5)
                _自定义转弯(90, -10, 3)
                _刹车(-100,10,0.1)
                _任务(任务0123,区1,区2,区3)    
            if 离开角度 ==1:
                _走编码不循线(150, -150, 2400) 
                _自定义转弯(10, -180, 4)  
                time.sleep(0.1)
            elif 离开角度 == -1 :   
                _走编码不循线(-150, 150, 2500) 
                _自定义转弯(-180, 10, 2)  
            if 完成返回路口 == 1 :
                _巡线遇线停(速度0, 1,1)
                _走编码不循线(180, 180, 1200)
                _自定义转弯(-180, 180, 1)
                _自定义转弯(-80, 80, 2) 
    elif 入口1234==3:
        if 出口1234==2:
            _巡线遇线停(速度0, 1,1)
            if 区1 < 2.5 :                  #12，靠左
                _走编码不循线(180, 180, 500)     
                _自定义转弯(-10, 180, 55)
                _自定义转弯(150, -10, 4)
                _刹车(-100,10,0.1)
                _任务(任务0123,区1,区2,区3)      
            elif 区1 > 2.5 :                 #34，靠右
                _走编码不循线(180, 180, 500)     
                _自定义转弯(180, -10, 11)
                _刹车(-180,10,0.1)
                _自定义转弯(-10, 100, 2)
                _刹车(10,-100,0.15)
                _任务(任务0123,区1,区2,区3) 
            elif 区1 == 2.5 :
                _走编码不循线(180, 180, 500)  
                _自定义转弯(-10, 150, 5)
                _自定义转弯(90, -10, 3)
                _刹车(-100,10,0.1)
                _任务(任务0123,区1,区2,区3)    
            if 离开角度 ==1:
                _走编码不循线(150, -150, 2400) 
                _自定义转弯(10, -180, 4)  
                time.sleep(0.1)
            elif 离开角度 == -1 :   
                _走编码不循线(-150, 150, 2500) 
                _自定义转弯(-180, 10, 2)  
            if 完成返回路口 == 1 :
                _巡线遇线停(速度0, 5,1)
                _走编码不循线(180, 180, 1200)
                _自定义转弯(180, -180, 5)
                _自定义转弯(80, -80, 4) 


def 路口L(L路口距离=9300,道闸偏转角度=1700,推道闸距离=1900,进入充电距离=2400):     
    _夹子升降(170,0)
    _夹子开合(0,0)
    _巡线编码(速度0,L路口距离,1)
    _刹车(-120,-120,0.1)
    _走编码不循线(-150, 10, 1600)
    time.sleep(0.1)
    _走编码不循线(-10, 150, 道闸偏转角度)   #转向道闸
    time.sleep(0.1)
    _走编码不循线(150, 150, 推道闸距离)
    time.sleep(0.1)
    _自定义转弯(-160, -160, 5)          #后退找回黑线
    _刹车(100,100,0.1)
    _夹子升降(60,0)
    _走编码不循线(180, -10,800)
    _自定义转弯(180, -10, 11)
    time.sleep(0.1)
    _自定义转弯(150, 120, 1)           #单轮右转,至1见黑见白,再见黑见白
    time.sleep(0.1)
    _自定义转弯(-10, 150, 2)
    time.sleep(0.1)
    _巡线遇线停(速度0, 5,1)
    _走编码不循线(150,150,800)
    _自定义转弯(150, 150, 5)
    time.sleep(0.1)
    _自定义转弯(120, -10, 3)
    _刹车(-100,10,0.1)
    _巡线遇线停(60, 5,1)
    _夹子开合(180,0)
    _走编码不循线(150, 150, 进入充电距离)