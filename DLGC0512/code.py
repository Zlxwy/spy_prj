from robot import xyrobot,Servo
import time,math
马达定速系数=3
g_wheel2R=5.3
g_width=15.8
g_sum_encoders=12*48*4
g_pi=3.1415926
rate_encoder = g_pi*g_wheel2R/g_sum_encoders

Lerr_D=0
P_err_D=0
P_tmp = 0
I_tmp = 0
D_tmp = 0 
#////////////PID需要使用的变量//////////////////////
error = 0 
setDenable=2
P=0.0
I=0.0
D = 0.0
PID_value = 0.0
decide = 0 
previous_error = 0
previous_I = 0
S1=0
S2=0
S3=0
S4=0
S5=0
#PID需要使用的变量
g_last=0
g_T1=0
g_T2=0
g_T3=0
g_T4=0
g_T5=0
setDindex=10


MaxI=2000
w_s=[10,60,90]
weights= [100-w_s[0], 100-w_s[1], 100-w_s[2], 0, -(100-w_s[2]), -(100-w_s[1]), -(100-w_s[0])]
P_30 = 20;  I_30  = 0.001;  D_30 = 0           
P_40 = 20;  I_40  = 0.001;  D_40  = 0      
P_50 = 20;  I_50  = 0.001;  D_50  = 0       
P_60 = 15;  I_60  = 0.0002;  D_60  = 0       
P_70 = 15;  I_70  = 0.0005;  D_70  = 0             
P_80 = 15;  I_80  = 0.0005;  D_80  = 1        
P_90 = 20;  I_90  = 0.0005;  D_90  = 1           
P_100 = 20;  I_100 = 0.0005;  D_100 = 1

isdebug=0 
isOutline=0
g_isLR=0
OutlineTime=0
# Kp = 15
# Ki = 0
# Kd = 15

S1_threshold=2500
S2_threshold=2500
S3_threshold=2500
S4_threshold=2500
S5_threshold=2500
import ssd1306
from machine import I2C, Pin
display = ssd1306.SSD1306_I2C(
            128, 64,
            I2C(0,
                scl=Pin(47, Pin.OUT, Pin.PULL_UP),
                sda=Pin(48, Pin.OUT, Pin.PULL_UP),
                freq=1000000),
            addr=0x3c)
def _getAllADC():
    return xyrobot.getADC(4),xyrobot.getADC(5),xyrobot.getADC(1),xyrobot.getADC(2),xyrobot.getADC(9)
issee=0
def _write(txt, line, col=0, reverse=False):
    if not isdebug:
        return 
    global display,issee
    issee=issee+1
    if True:
        font_width=14
        font_height=14
        x_offset = col * font_width
        y_offset = line*font_height+3
        display.fill_rect(x_offset,y_offset,font_width*len(txt)+14,font_height-6,reverse)
        display.text(txt,x_offset,y_offset,not reverse)
        issee=0
test_min_S2=553
test_min_S3=845
test_min_S4=991
test_max_S2=1927
test_max_S3=2889
test_max_S4=3289
def _扫描中间3个光电():
    global display,test_min_S2,test_min_S3,test_min_S4,test_max_S2,test_max_S3,test_max_S4
    test_min_S2=9999
    test_min_S3=9999
    test_min_S4=9999
    test_max_S2=0
    test_max_S3=0
    test_max_S4=0
    while True:
        ADC_S1,ADC_S2,ADC_S3,ADC_S4,ADC_S5=_getAllADC()
        if ADC_S2<test_min_S2:
            test_min_S2=ADC_S2
        if ADC_S2>test_max_S2:
            test_max_S2=ADC_S2

        if ADC_S3<test_min_S3:
            test_min_S3=ADC_S3
        if ADC_S3>test_max_S3:
            test_max_S3=ADC_S3

        if ADC_S4<test_min_S4:
            test_min_S4=ADC_S4
        if ADC_S4>test_max_S4:
            test_max_S4=ADC_S4
        _write(str(test_min_S2) + "," + str(test_max_S2),1)
        _write(str(test_min_S3) + "," + str(test_max_S3),2)
        _write(str(test_min_S4) + "," + str(test_max_S4),3)
        display.show()
        if xyrobot.getPinValue(0)==0:
            _write(str(test_min_S2) + "," + str(test_max_S2)+" o="+str((test_min_S2+test_max_S2)//2),1)
            _write(str(test_min_S3) + "," + str(test_max_S3)+" o="+str((test_min_S3+test_max_S3)//2),2)
            _write(str(test_min_S4) + "," + str(test_max_S4)+" o="+str((test_min_S4+test_max_S4)//2),3)
            display.show()
            break
            
        if xyrobot.getPinValue(3)==0:
            test_min_S2=9999
            test_min_S3=9999
            test_min_S4=9999
            test_max_S2=0
            test_max_S3=0
            test_max_S4=0
def 设置(S1阈值=1300,S2阈值=1300,S3阈值=1300,S4阈值=1300,S5阈值=1300):
    global S1_threshold,S2_threshold,S3_threshold,S4_threshold,S5_threshold
    S1_threshold=S1阈值
    S2_threshold=S2阈值
    S3_threshold=S3阈值
    S4_threshold=S4阈值
    S5_threshold=S5阈值


def _setmotor(sp1,sp2):
    xyrobot.setMotor(1,sp1)
    xyrobot.setMotor(2,sp2)
def setmotor(sp1,sp2):
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
        # if(math.fabs(g_T4-g_T1)>100 and g_T4-g_T3>100):
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
        
    # if isOutline==0:

    #     if((g_T2-g_T3>100 and g_T2>g_T4) or time.ticks_ms()-g_T1<100 ):
    #         isOutline=1
    #         OutlineTime=time.ticks_ms()
    #     elif((g_T4-g_T3>100 and g_T4>g_T2) or time.ticks_ms()-g_T5<100):
    #         isOutline=-1
    #         OutlineTime=time.ticks_ms()
    #     elif g_T1>g_T5:
    #         isOutline=1
    #     elif g_T5>g_T1:
    #         isOutline=-1
    # else:
    #     OutlineTime=time.ticks_ms()
def _read_sensor_error():
    global S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,weights
    _getadState()
    '''
    * 光电几种状态
    * 黑    黑  黑：直走
    * 白    黑  白：直走
    * 黑    白  白：大左转
    * 白    白  黑：大右转
    * 黑    黑  白：左转
    * 白    黑  黑：右转
    * 白    白  白：出线
    * 黑    白  黑：不存在
    '''
    if(S2==1 and S3==1 and S4==1):
        error = weights[3]
        I=0
    if(S2==1 and S4==1):
        error = weights[3]
        I=0
    elif( S2==0 and S3==1 and S4 == 0):
        error = weights[3]
        I=0
        # isOutline=0
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
    
#*************************************
#实际电机控制变化速度
#************************************

def _motor_control(initial_motor_speed):
    global S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error
    left_motor_speed = initial_motor_speed - PID_value
    right_motor_speed = initial_motor_speed + PID_value
    
    if(left_motor_speed>100):left_motor_speed=100
    if(left_motor_speed<-55):left_motor_speed=-55
    # if(initial_motor_speed<65):
    #     if(left_motor_speed<-65):left_motor_speed=-65
    # else:
    #     if(left_motor_speed<-100):left_motor_speed=-100
    if(right_motor_speed>100):right_motor_speed=100
    if(right_motor_speed<-55):right_motor_speed=-55
    # if(initial_motor_speed>65):
    #     if(right_motor_speed<-65):right_motor_speed=-65
    # else:
    #     if(right_motor_speed<-100):right_motor_speed=-100
    if(isdebug):
        pass
    _setmotor(left_motor_speed,right_motor_speed)
    # _motorsWritePct(left_motor_speed,right_motor_speed)


#//////////////////////////////pid line，用哪号光电控制停止/////////////////////////////////////////////////////
def 巡线遇线停(速度,sensor):
    global display,S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error,g_T1,g_T2,g_T3,g_T4,g_T5
    global w_s,weights,isdebug
    error=0
    speed=速度
    # _setPIDspeed(speed)
    g_T3=2
    g_T4=3
    g_T2=4
    g_T5=0
    g_T1=1
    P=0
    I=0
    D=0
    regotime=time.ticks_ms()
    w_s=[10,6,1]
    weights= [w_s[0], w_s[1], w_s[2], 0,-w_s[2], -w_s[1], -w_s[0]]
    # if(led==1):
    #     g_isLR=1
    # if(led==5):
    #     g_isLR=5
    isdebug=False
    Kp,Ki,Kd=_getPID(速度)
    while(True):
        #得到PID计算出小车的误差
        _read_sensor_error()
        #填入P和D进行计算小车误差的状态
        _calc_pid(Kp,Ki,Kd)
        _write(str(error)+" ,"+str(PID_value) + " "+str(time.ticks_ms()-g_T2),1)
        strADC=str(S1)+" "+str(S2)+" "+str(S3)+" "+str(S4)+" "+str(S5)+ " out:"+str(isOutline)
        
        _write(strADC,2)
        _write(str(P) +" " + str(I) +" "+str(D) ,3)
        if isdebug:
            display.show()
        tt51=math.fabs(g_T1-g_T5)
        if(sensor == 1):
            if(S1):
                _setmotor(0,0)
                break
        elif(sensor == 5):
            if(S5):
                _setmotor(0,0)
                break
        
        elif(sensor == 234):
            scount=S2+S3+S4
            if(scount==3):
                _setmotor(0,0)
                break
        else:
           if((S1 or S5) and (g_T5>0 and g_T1>0 and tt51<100)):
                _setmotor(0,0)
                break
        
        _motor_control(speed)
        time.sleep_ms(0)
def 巡线时间(速度,时间ms):
    global display,S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error,g_T1,g_T2,g_T3,g_T4,g_T5
    global w_s,weights,isdebug
    error=0
    # _setPIDspeed(speed)
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
    # if(led==1):
    #     g_isLR=1
    # if(led==5):
    #     g_isLR=5
    isdebug=False
    Kp,Ki,Kd=_getPID(速度)
    regotime=time.ticks_ms()
    while(True):
        #得到PID计算出小车的误差
        _read_sensor_error()
        #填入P和D进行计算小车误差的状态
        _calc_pid(Kp,Ki,Kd)
        _write(str(error)+" ,"+str(PID_value) + " "+str(time.ticks_ms()-g_T2),1)
        strADC=str(S1)+" "+str(S2)+" "+str(S3)+" "+str(S4)+" "+str(S5)+ " out:"+str(isOutline)
        
        _write(strADC,2)
        _write(str(P) +" " + str(I) +" "+str(D) ,3)
        if isdebug:
            display.show()
        if time.ticks_ms()-regotime>=时间ms:
            _setmotor(0,0)
            break
        _motor_control(speed)
        time.sleep_ms(0)
def 巡线编码(速度,编码值):
    global display,S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error,g_T1,g_T2,g_T3,g_T4,g_T5
    global w_s,weights,isdebug
   
    error=0
    # _setPIDspeed(speed)
    g_T3=2
    g_T4=3
    g_T2=4
    g_T5=0
    g_T1=1
    P=0
    I=0
    D=0
    speed=速度
    
    
    # if(led==1):
    #     g_isLR=1
    # if(led==5):
    #     g_isLR=5
    isdebug=False
    
    regotime=time.ticks_ms()
    code1=xyrobot.getCode(1)
    code2=xyrobot.getCode(2)
    Kp,Ki,Kd=_getPID(速度)
    while(True):
        #得到PID计算出小车的误差
        _read_sensor_error()
        #填入P和D进行计算小车误差的状态
        _calc_pid(Kp,Ki,Kd)
        _write(str(error)+" ,"+str(PID_value) + " "+str(time.ticks_ms()-g_T2),1)
        strADC=str(S1)+" "+str(S2)+" "+str(S3)+" "+str(S4)+" "+str(S5)+ " out:"+str(isOutline)
        
        _write(strADC,2)
        _write(str(P) +" " + str(I) +" "+str(D) ,3)
        if isdebug:
            display.show()
        cc=math.fabs(xyrobot.getCode(1)-code1)
        if cc>=编码值:
            _setmotor(0,0)
            break
        _motor_control(speed)
        time.sleep_ms(0)
def _getPID(speed):
    global display,S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error,g_T1,g_T2,g_T3,g_T4,g_T5
    global w_s,weights,isdebug
    if(speed<80):
        w_s=[10,6,1]
        MaxI=2500
        weights= [w_s[0], w_s[1], w_s[2], 0,-w_s[2], -w_s[1], -w_s[0]]
        Kp=8
        Ki=0.03
        Kd=0.01
    else:
        w_s=[10,8,2]
        MaxI=2500
        weights= [w_s[0], w_s[1], w_s[2], 0,-w_s[2], -w_s[1], -w_s[0]]
        Kp=8
        Ki=0.04
        Kd=0.01
    return Kp,Ki,Kd
#自定义转弯
def 自定义转弯(左转速=40,右转速=-40,转到光电=3):
    global display,S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error,g_T1,g_T2,g_T3,g_T4,g_T5
    global w_s,weights,isdebug
    左转速=左转速*马达定速系数
    右转速=右转速*马达定速系数
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
def 设置机器参数(轮胎直径cm=5.3,轮距cm=15.8,码盘线数=12,减速比=48):
    global g_sum_encoders,g_wheel2R,g_width,g_sum_encoders,rate_encoder

    g_width=轮距cm
    g_wheel2R=轮胎直径cm
    g_sum_encoders=码盘线数*减速比*4
    
    rate_encoder = g_pi*g_wheel2R/g_sum_encoders
def 走距离cm不循线(转速cms=20,距离cm=100):
    # if 转速cms>40:
    #     转速cms=40
    转速cms=转速cms*马达定速系数
    global g_sum_encoders,g_wheel2R,g_width,g_sum_encoders,rate_encoder
    转速=(转速cms/50)/rate_encoder
    
    
    code1=xyrobot.getCode(1)
    code2=xyrobot.getCode(2)
    xyrobot.setRPM(1,转速)
    xyrobot.setRPM(2,转速)
    print(转速)
    while True:
        inc_encoder_L=xyrobot.getCode(1)-code1
        inc_encoder_R=xyrobot.getCode(2)-code2
        lenth_error = (inc_encoder_L + inc_encoder_R)/2 * rate_encoder
        if(abs(lenth_error)>=abs(距离cm)):
            xyrobot.setMotor(1,0)
            xyrobot.setMotor(2,0)
            break
def 转弯_速度单位每秒cm数(左转速cm=20,右转速cm=-20,转到角度=90):
    左转速cm=左转速cm*马达定速系数
    右转速cm=右转速cm*马达定速系数
    global display,S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error,g_T1,g_T2,g_T3,g_T4,g_T5
    global w_s,weights,isdebug,g_sum_encoders,g_wheel2R,g_width,g_sum_encoders,rate_encoder
    左转速=(左转速cm/50)/rate_encoder
    右转速=(右转速cm/50)/rate_encoder
    xyrobot.clearPID(1)
    xyrobot.clearPID(2)

    code1=xyrobot.getCode(1)
    code2=xyrobot.getCode(2)

    xyrobot.setRPM(1,int(左转速))
    xyrobot.setRPM(2,int(右转速))
    while True:
        inc_encoder_L=xyrobot.getCode(1)-code1
        inc_encoder_R=xyrobot.getCode(2)-code2
        lenth_error = (inc_encoder_L- inc_encoder_R) * rate_encoder
        anlge_z_error = (lenth_error / g_width) *180.0/g_pi

        if(anlge_z_error>=转到角度 or anlge_z_error<=-转到角度):
            xyrobot.setMotor(1,0)
            xyrobot.setMotor(2,0)
            break
        # display.fill_rect(0,0,50,50,False)

        # display.text(str(anlge_z_error),0,0,True)
        # display.text(str(xyrobot.getCode(1)),0,15,True)
        # display.show()
        
#走编码不巡线
def 走编码不循线(左转速=50,右转速=50,编码值=2000):
    global display,S1,S2,S3,S4,S5,isOutline,error,P,I,D,g_last,MaxI,Lerr_D,PID_value,previous_error,g_T1,g_T2,g_T3,g_T4,g_T5
    global w_s,weights,isdebug
    
    code1=xyrobot.getCode(1)
    code2=xyrobot.getCode(2)
    xyrobot.setRPM(1,左转速*马达定速系数)
    xyrobot.setRPM(2,右转速*马达定速系数)
    while True:
        if math.fabs(xyrobot.getCode(1)-code1)>=编码值 or math.fabs(xyrobot.getCode(2)-code2)>=编码值:
            xyrobot.setMotor(1,0)
            xyrobot.setMotor(2,0)
            break

def 设置编码方向(m1=1,m2=0,m3=0,m4=0):
    xyrobot.setMotorDir(1,0,m1)
    xyrobot.setMotorDir(2,0,m2)
    xyrobot.setMotorDir(3,0,m3)
    xyrobot.setMotorDir(4,0,m4)



#舵机抬升角度
def 抬升夹子B4(速度):
    taisheng=Servo(10)
    taisheng.write(速度)
    
    taisheng=None


#舵机夹取角度
def 夹子张开与关闭A4(角度):
    jiazi=Servo(7)
    jiazi.write(角度)
    
    jiazi=None

import _thread

def _机械臂升任务(角度=0, 光电阈值=1000):
    
    xyrobot.setServo(6,角度)
    count=光电阈值/10
    while True:
        
        time.sleep_ms(10) #10ms检测一次
        count=count-1
        if count<=0:
            break
    xyrobot.setServo(6,90)
def _机械臂降任务(角度, 光电阈值):
    
    xyrobot.setServo(6,角度)
    count=光电阈值/10
    while True:
        
        time.sleep_ms(10) #10ms检测一次
        count=count-1
        if count<=0 :
            break
    xyrobot.setServo(6,100)
    time.sleep_ms(200) #多转200ms
    xyrobot.setServo(6,90)
def 设置机械臂_升(角度=0,光电阈值=1000):
    _thread.start_new_thread(_机械臂升任务, (角度, 光电阈值))
def 设置机械臂_降(角度=180,光电阈值=1000):
    _thread.start_new_thread(_机械臂降任务, (角度, 光电阈值))