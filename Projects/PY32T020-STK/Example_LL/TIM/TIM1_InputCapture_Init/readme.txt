================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了TIM1的输入捕获功能，配置PA3作为输入捕获引脚，PA3每检测到一个下降沿触
发捕获中断,在捕获中断回调函数中翻转LED灯。

Function descriptions:
This example demonstrates the input capture function of TIM1, where PA3 is 
configured as the input capture pin. Every time PA3 detects a falling edge, it 
triggers a capture interrupt and flips the LED light in the capture interrupt 
callback function.
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
1. 下载并运行程序
2. PA3引脚不输入时钟信号的情况下，LED不翻转
3. PA3输入时钟信号，TIM1捕获成功后，LED会翻转

Example execution steps:
1. compile and download the program to MCU and run it;
2. The LED does not toggle when the PA3 pin does not input the clock signal
3. PA3 input clock signal, TIM1 capture success, LED will toggle
================================================================================
注意事项：

Notes:

================================================================================