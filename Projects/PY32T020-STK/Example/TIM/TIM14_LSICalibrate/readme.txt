================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例配置LSI时钟从MCO（PA8）输出并将TIM14的通道1连接到MCO。将系统时钟配置为24MHz
，TIM14时钟为240KHz，设置重载值为10001，使能TIM14的输入捕获功能，调整LSI trimming
值，LED从闪烁变成常亮表示校准完成。

Function descriptions:
This example configures the LSI clock to output from MCO (PA8) and connects 
channel 1 of TIM14 to MCO. Set the system clock to 24MHz, set the clock of TIM14
to 240KHz, set the overload value to 10001, enable the input capture function of
TIM14, adjust the value of LSI trimming. When the LED changes from blinking to 
steady on, the calibration is complete
================================================================================
测试环境：
测试用板：PY32T020_STK
MDK版本： 5.28
IAR版本： 9.20
GCC 版本：GNU Arm Embedded Toolchain 10.3-2021.10

Test environment:
Test board: PY32T020_STK
MDK Version: 5.28
IAR Version: 9.20
GCC Version: GNU Arm Embedded Toolchain 10.3-2021.10

================================================================================
使用步骤：
1. 编译下载程序到MCU，并运行；
2. 按下用户按键；
3. 等待LED从闪烁变成常亮，用示波器监测PA08引脚上的频率，观察是否校准成功。

Example execution steps:
1. Compile and download the program to the MCU and run it.
2. Press the user button.
3. Wait for LED changes from blinking to steady on and monitor the frequency 
on the PA08 pin with an oscilloscope to see if the calibration is successful
================================================================================
注意事项：
UpLimit 和 LowLimit 值随系统主频变化，计算公式如下：
  UpLimit = 32768*(10000/(systemclock/100)) + 4
  LowLimit = 32768*(10000/(systemclock/100)) - 4

Notes:
UpLimit and LowLimit values vary with the system frequency. The calculation 
formula is as follows:
UpLimit = 32768*(10000/(systemclock/100)) + 4
LowLimit = 32768*(10000/(systemclock/100)) - 4
================================================================================
