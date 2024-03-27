================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了通过PA6引脚唤醒MCU的功能。下载程序并运行后，LED灯处于常亮状态；按下
用户按键后，LED灯处于常暗状态，且MCU进入STOP模式；拉低PA6引脚后，MCU唤醒，LED灯
处于闪烁状态。

Function descriptions:
This example demonstrates the function of waking up an MCU through the PA6 pin.
After downloading the program and running it, the LED light is constantly on;
After pressing the user button, the LED light is in a constant dark state and
the MCU enters STOP mode; After pulling down the PA6 pin, the MCU wakes up and
the LED light is in a flashing state.
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
2. 小灯处于常亮状态，按下用户按键，LED灯处于常暗状态，且MCU进入STOP模式；
3. 拉低PA6引脚后，MCU唤醒，LED灯处于闪烁状态。

Example execution steps:
1.Compile and download the program to MCU and run it;
2.The small light is in a constant light state, press the user button, the LED
light is in a constant dark state, and the MCU enters STOP mode;
3.After pulling down the PA6 pin, the MCU wakes up and the LED light is in a
flashing state.
================================================================================
注意事项：
1，演示此样例功能时需要断开swd连接线并重新上电，因为默认情况下，仿真器会把
DBGMCU_CR.DBG_STOP置位。

Notes:
1.When demonstrating this sample, disconnect the SWD connection and 
power cycle the board, as the debugger will set DBGMCU_CR.DBG_STOP by default.
================================================================================