from robot import xyrobot, Servo
import XY_OLED as oled
import time, math

# 循迹传感器(tracking_sensor)接线（车头朝前，人在后方视角）：
# 从左到右依次为：A1(4)  A2(5)  A3(6)  B1(1)  B2(2)

# 电机接线（车头朝前，人在后方视角）：
# 从左到右依次为：M1  M4
# 轮子1圈2350个编码值



motor_num = 2  # 电机的数量
motor_index = [1, 4]  # 电机连接的引脚引脚号
motor_dir_forward = [1, 0]  # 电机方向，1为正转，0为反转
motor_code_dir_forward = [1, 0]  # 电机编码方向，1为正，0为负
motor_base_speed = 500  # 电机的基础速度（电机速度范围0~100）

# ts为循迹传感器(tracking_sensor)的缩写
ts_num = 5
ts_index = [4, 5, 6, 1, 2]  # 循迹传感器连接ADC引脚的索引
ts_adc_value = [0, 0, 0, 0, 0]  # 循迹传感器的ADC值，可以通过_get_all_ts_adc_value刷新
ts_adc_threshold = [1500, 1500, 1500, 1500, 1500]  # 循迹传感器的ADC阈值，用于对循迹传感器的ADC值进行二值化
ts_status_bin = [False, False, False, False, False] # 白色为false，黑色为true
ts_offset_weight = [-63.5, -24.0, 0.0, +24.0, +63.5]  # 循迹传感器的偏移权重，用于将二值化后的ADC值转换为实际距离
ts_black_block_num = 0 # 循迹传感器所测量到的黑色块的数量



ts_curr_error = 0.0  # 循迹传感器所测量到的误差
ts_last_error = 0.0  # 上一次的误差
ts_error_diff = 0.0  # 误差差分项
ts_error_integ = 0.0  # 误差积分项
ts_pos_output = 0.0  # 位置控制的输出
ts_pos_kp = 5.0  # 位置控制的Kp系数
ts_pos_ki = 0.0  # 位置控制的Ki系数
ts_pos_kd = 0.0  # 位置控制的Kd系数

target_lsd = 0.0  # 左电机目标速度(left speed)
lsd_curr_error = 0.0  # 左电机速度(left speed)控制的误差
lsd_last_error = 0.0  # 左电机上一次的误差(left speed)
lsd_error_integ = 0.0  # 左电机速度(left speed)控制的误差积分项
lsd_error_diff = 0.0  # 左电机速度(left speed)控制的误差差分项
lsd_output = 0.0  # 左电机速度(left speed)控制的输出
target_rsd = 0.0  # 右电机目标速度(right speed)
rsd_curr_error = 0.0  # 右电机速度(right speed)控制的误差
rsd_last_error = 0.0  # 右电机上一次的误差(right speed)
rsd_error_diff = 0.0  # 右电机速度(right speed)控制的误差差分项
rsd_error_integ = 0.0  # 右电机速度(right speed)控制的误差积分项
rsd_output = 0.0  # 右电机速度(right speed)控制的输出
spd_kp = 2.4  # 速度环控制的Kp系数
spd_ki = 0.2  # 速度环控制的Ki系数
spd_kd = 0.4  # 速度环控制的Kd系数

    # global motor_index, motor_dir_forward, motor_code_dir_forward
    # global ts_num, ts_index, ts_adc_value
    # global ts_adc_threshold, ts_status_bin, ts_offset_weight
    # global ts_curr_error, ts_black_block_num



def CLAMP(x, min, max):
    return max(min, min(max, x))



def _get_all_ts_adc_value():
    global ts_adc_value
    for i in range(ts_num):
        ts_adc_value[i] = xyrobot.getADC(ts_index[i])


def _cnt_black_block_num(status_bin_list): # 统计列表中黑色块(True)的数量
    cnt = 0
    for sbl in status_bin_list:
        if sbl: cnt += 1
    return cnt



def _set_left_motor(speed): # 设置左电机的转速，开环
    global motor_index, motor_dir_forward, motor_code_dir_forward
    if speed < 0: xyrobot.setMotorDir(motor_index[0], not motor_dir_forward[0], motor_code_dir_forward[0]) # 如果速度为负，设置为反转
    else:         xyrobot.setMotorDir(motor_index[0], motor_dir_forward[0], motor_code_dir_forward[0]) # 如果速度为正，设置为正转
    xyrobot.setMotor(motor_index[0], abs(speed)) # 设置左电机的转速，绝对值

def _set_right_motor(speed): # 设置右电机的转速，开环
    global motor_index, motor_dir_forward, motor_code_dir_forward
    if speed < 0: xyrobot.setMotorDir(motor_index[1], not motor_dir_forward[1], motor_code_dir_forward[1]) # 如果速度为负，设置为反转
    else:         xyrobot.setMotorDir(motor_index[1], motor_dir_forward[1], motor_code_dir_forward[1]) # 如果速度为正，设置为正转
    xyrobot.setMotor(motor_index[1], abs(speed)) # 设置右电机的转速，绝对值

def _set_all_motor(speed_l, speed_r): # 设置左右电机的转速，开环
    _set_left_motor(speed_l)
    _set_right_motor(speed_r)


def _get_encoder_and_zero(): # 获取电机的编码值，并清零
    global motor_index
    l = xyrobot.getCode(motor_index[0])
    r = xyrobot.getCode(motor_index[1])
    xyrobot.clearCode(motor_index[0])
    xyrobot.clearCode(motor_index[1])
    return l, r





# 初始化配置
def init_config():
    global motor_index, motor_dir_forward, motor_code_dir_forward
    xyrobot.setMotorDir(
        motor_index[0],
        motor_dir_forward[0],
        motor_code_dir_forward[0]
    ) # 左电机(1)初始化为正转(1)，编码方向为正(1)
    xyrobot.setMotorDir(
        motor_index[1],
        motor_dir_forward[1],
        motor_code_dir_forward[1]
    ) # 右电机(4)初始化为反转(0)，编码方向为负(0)
    xyrobot.clearCode(motor_index[0])
    xyrobot.clearCode(motor_index[1])





def have_a_try():
    global motor_base_speed
    global motor_index, motor_dir_forward, motor_code_dir_forward
    global ts_num, ts_index, ts_adc_value
    global ts_adc_threshold, ts_status_bin, ts_offset_weight
    global ts_curr_error # 循迹传感器所测量到的误差
    global ts_last_error # 上一次的误差
    global ts_error_diff # 误差差分项
    global ts_error_integ # 误差积分项
    global ts_pos_output # 位置控制的输出
    global ts_pos_kp # 位置控制的Kp系数
    global ts_pos_ki # 位置控制的Ki系数
    global ts_pos_kd # 位置控制的Kd系数
    global target_lsd # 左电机目标速度(left speed)
    global lsd_curr_error # 左电机速度(left speed)控制的误差
    global lsd_last_error # 左电机上一次的误差(left speed)
    global lsd_error_integ # 左电机速度(left speed)控制的误差积分项
    global lsd_error_diff # 左电机速度(left speed)控制的误差差分项
    global lsd_output # 左电机速度(left speed)控制的输出
    global target_rsd # 右电机目标速度(right speed)
    global rsd_curr_error # 右电机速度(right speed)控制的误差
    global rsd_last_error # 右电机上一次的误差(right speed)
    global rsd_error_diff # 右电机速度(right speed)控制的误差差分项
    global rsd_error_integ # 右电机速度(right speed)控制的误差积分项
    global rsd_output # 右电机速度(right speed)控制的输出
    global spd_kp # 速度环控制的Kp系数
    global spd_ki # 速度环控制的Ki系数
    global spd_kd # 速度环控制的Kd系数

    _get_all_ts_adc_value()

    ts_status_bin = [False, False, False, False, False]
    ts_curr_error = 0.0
    ts_black_block_num = 0

    for i in range(ts_num): # 如果大于阈值，为黑色，状态置True，否则为False
        ts_status_bin[i] = True if ts_adc_value[i] <= ts_adc_threshold[i] else False
    
    ts_black_block_num = _cnt_black_block_num(ts_status_bin) # 统计黑色块的数量
    for i in range(ts_num): ts_curr_error += ts_offset_weight[i] if ts_status_bin[i] else 0.0 # 用于位置环控制
    left_encoder, right_encoder = _get_encoder_and_zero() # 获取电机的编码值，并清零，用于速度环控制

    ts_curr_error   = 0.68*ts_curr_error + 0.32*ts_last_error # 用于P控制
    ts_error_integ += ts_curr_error # 用于I控制
    ts_error_diff   = ts_curr_error - ts_last_error # 用于D控制
    ts_last_error = ts_curr_error # 更新上一次的误差
    ts_pos_output = ts_pos_kp*ts_curr_error + ts_pos_ki*ts_error_integ + ts_pos_kd*ts_error_diff # 位置控制输出

    target_lsd = motor_base_speed + ts_pos_output # 计算左电机目标速度(left speed)
    lsd_curr_error = target_lsd - left_encoder # 左电机速度(left speed)控制的误差
    lsd_error_integ += lsd_curr_error # 左电机速度(left speed)控制的误差积分项
    lsd_error_diff = lsd_curr_error - lsd_last_error # 左电机速度(left speed)控制的误差差分项
    lsd_last_error = lsd_curr_error # 更新上一次的左电机速度(left speed)控制的误差
    lsd_output = spd_kp*lsd_curr_error + spd_ki*lsd_error_integ + spd_kd*lsd_error_diff # 左电机速度(left speed)控制的输出
    target_rsd = motor_base_speed - ts_pos_output # 计算右电机目标速度(right speed)
    rsd_curr_error = target_rsd - right_encoder # 右电机速度(right speed)控制的误差
    rsd_error_integ += rsd_curr_error # 右电机速度(right speed)控制的误差积分项
    rsd_error_diff = rsd_curr_error - rsd_last_error # 右电机速度(right speed)控制的误差差分项
    rsd_last_error = rsd_curr_error # 更新上一次的右电机速度(right speed)控制的误差
    rsd_output = spd_kp*rsd_curr_error + spd_ki*rsd_error_integ + spd_kd*rsd_error_diff # 右电机速度(right speed)控制的输出

    if ts_black_block_num is not 0: # 如果有黑色块
        _set_all_motor(CLAMP(lsd_output), CLAMP(rsd_output))
        pass  # 占位语句，防止缩进错误
    else: # 如果没有黑色块
        _set_all_motor(0, 0) # 让车子停止


    oled.text('' + "{:<6}".format(str((ts_adc_value[0]))),  0,  0, 0, 0)
    oled.text('' + "{:<6}".format(str((ts_adc_value[1]))), 40,  0, 0, 0)
    oled.text('' + "{:<6}".format(str((ts_adc_value[2]))), 80,  0, 0, 0)
    oled.text('' + "{:<6}".format(str((ts_adc_value[3]))),  0, 16, 0, 0)
    oled.text('' + "{:<6}".format(str((ts_adc_value[4]))), 40, 16, 0, 0)
    oled.text('' + "{:<6}".format(str((ts_black_block_num))), 80, 16, 0, 0)

    oled.text('' + "{:<6}".format(str((left_encoder))), 0, 32, 0, 0)
    oled.text('' + "{:<6}".format(str((right_encoder))), 40, 32, 0, 0)

    oled.text('' + "{:<6}".format(str((ts_pos_output))), 0, 48, 0, 0)

    oled.show()

