from robot import xyrobot, Servo
import XY_OLED as oled
import time, math

# 循迹传感器(tracking_sensor)接线（车头朝前，人在后方视角）：
# 从左到右依次为：A1(4)  A2(5)  A3(6)  B1(1)  B2(2)

# 电机接线（车头朝前，人在后方视角）：
# 从左到右依次为：M1  M4
# 轮子1圈2350个编码值



motor_index = [1, 4]  # 电机连接的引脚引脚号
motor_dir_forward = [1, 0]  # 电机方向，1为正转，0为反转
motor_code_dir_forward = [1, 0]  # 电机编码方向，1为正，0为负

# ts为循迹传感器(tracking_sensor)的缩写
ts_num = 5
ts_index = [4, 5, 6, 1, 2]  # 循迹传感器连接ADC引脚的索引
ts_adc_value = [0, 0, 0, 0, 0]  # 循迹传感器的ADC值，可以通过_get_all_ts_adc_value刷新
ts_adc_threshold = [1500, 1500, 1500, 1500, 1500]  # 循迹传感器的ADC阈值，用于对循迹传感器的ADC值进行二值化
ts_status_bin = [False, False, False, False, False] # 白色为false，黑色为true
ts_offset_weight = [-63.5, -24.0, 0.0, +24.0, +63.5]  # 循迹传感器的偏移权重，用于将二值化后的ADC值转换为实际距离
ts_error = 0.0  # 循迹传感器所测量到的误差
ts_black_block_num = 0 # 循迹传感器所测量到的黑色块的数量

    # global motor_index, motor_dir_forward, motor_code_dir_forward
    # global ts_num, ts_index, ts_adc_value
    # global ts_adc_threshold, ts_status_bin, ts_offset_weight
    # global ts_error, ts_black_block_num



def _get_all_ts_adc_value():
    global ts_adc_value
    for i in range(ts_num):
        ts_adc_value[i] = xyrobot.getADC(ts_index[i])




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



def have_a_try():
    global motor_index, motor_dir_forward, motor_code_dir_forward
    global ts_num, ts_index, ts_adc_value
    global ts_adc_threshold, ts_status_bin, ts_offset_weight
    global ts_error, ts_black_block_num

    _get_all_ts_adc_value()

    ts_status_bin = [False, False, False, False, False]
    ts_error = 0.0
    ts_black_block_num = 0

    for i in range(ts_num): # 如果大于阈值，为黑色，状态置True，否则为False
        ts_status_bin[i]    = True if ts_adc_value[i] > ts_adc_threshold[i] else False
        ts_error           += ts_offset_weight[i] if ts_status_bin[i] else 0.0
        ts_black_block_num += 1 if ts_status_bin[i] else 0

    if ts_black_block_num is not 0: # 如果有黑色块
        _set_all_motor(200, 200) # 让车子动一下
    else: # 如果没有黑色块
        _set_all_motor(-200, -200) # 让车子停止

    # if ts_adc_value[4] > ts_adc_threshold[4]: # 如果B2为黑色
    #     _set_all_motor(500, 500) # 让车子动一下
    # else: # 如果B2为白色
    #     _set_all_motor(0, 0) # 让车子停止

    oled.text(''+"{:<6}".format(str((ts_adc_value[0]))),  0,  0, 0, 0)
    oled.text(''+"{:<6}".format(str((ts_adc_value[1]))), 40,  0, 0, 0)
    oled.text(''+"{:<6}".format(str((ts_adc_value[2]))), 80,  0, 0, 0)
    oled.text(''+"{:<6}".format(str((ts_adc_value[3]))),  0, 16, 0, 0)
    oled.text(''+"{:<6}".format(str((ts_adc_value[4]))), 40, 16, 0, 0)

    oled.text(''+"{:<6}".format(str((xyrobot.getCode(1)))), 0, 32, 0, 0)
    oled.text(''+"{:<6}".format(str((xyrobot.getCode(4)))), 40, 32, 0, 0)

    oled.show()


似乎不够完善
