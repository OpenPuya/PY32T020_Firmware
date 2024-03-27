================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例演示了TIM1的输出比较模式。将捕获/比较通道1（CH1）的输出映射到PA3，开启捕获
/比较通道1（CH1）并设置为比较输出翻转模式，同时开启捕获/比较中断，每次计数值与比
较值匹配时翻转输出电平（PA4），在捕获/比较中断处理中翻转LED灯。

Function descriptions:
This sample demonstrates the output compare mode of TIM1. The output of 
capture/compare channel 1 (CH1) is mapped to pin PA3. Capture/compare 
channel 1 (CH1) is enabled and set to compare output toggle mode. 
Capture/compare interrupt is also enabled. Whenever the counter value 
matches the compare value, the output（PA4） level will toggle. In the interrupt 
handler of capture/compare, the LED will also toggle.
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
1. 编译下载程序到MCU，并运行
2. 使用示波器观察PA3，频率为0.5Hz
3. LED的翻转频率为0.5Hz

Example execution steps:
1. compile and download the program to MCU and run it;
2. Observe PA3 with an oscilloscope ,toggle frequency is 0.5 Hz
3. LED is toggled at 0.5Hz frequency
================================================================================
注意事项：

Notes:

================================================================================