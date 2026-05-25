main.py-主程序
detector.py-检测模块
filter.py-低通滤波器
config.py-参数配置
comm.py-串口通信

UART帧格式：
0xAA + mode + angle_x + angle_y + 0x55

angle_x和angle_y是角度偏差（单位：度），不是像素。MaixCam输出的是“目标相对当前视线的角度偏差”，不是绝对舵机角度，也不是PWM值

angle_x > 0 ：目标在右边，需要云台向右转。

angle_x < 0 ：目标在左边，需要云台向左转。

先测试，如果左边目标但舵机右转，则改angle_x符号。

数值范围限制如下，输入不要超过这个范围
X: ±45°，Y: ±35°

gpt给的STM32处理建议：
1. angle直接作为“误差输入”
2. 做死区（±2°）
3. 做低通或PID滤波
4. 注意方向可能需要取反
5. 不要直接当PWM使用

本系统不是像素控制，是角度控制系统
