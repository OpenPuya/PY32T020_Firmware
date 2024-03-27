================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了COMP比较器的window功能，比较器1的Plus端用比较器2的IO4(1/2VCCA)作为输
入，PB1作为比较器1负端输入，当PB1的电压值大于1.65V时,LED灯灭，小于1.65V时,LED灯亮。
PA2作为比较器2负端输入，当PA2的电压值大于1.65V时,PA4拉低，小于1.65V时,PA4拉高

Function descriptions:
This example demonstrates the window function of the COMP comparator. The Plus
end of comparator 1 uses the IO4 (1/2VCCA) of comparator 2 as the input, and PB1
as the negative end input. When the voltage value of PB1 is greater than 1.65V,
the LED light turns off, and when it is less than 1.65V, the LED light turns on.
PA2 is input as the negative end of comparator 2,when the voltage value of PA2 
is greater than 1.65V,the PA4 pull down, and when it is less than 1.65V, the 
PA4 pull up
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
2. 配置PB1输入大于1.65V电压，LED灯灭；
3. 配置PB1输入小于1.65V电压，LED灯亮。
4. 配置PA2输入大于1.65V电压，PA4拉低。
5. 配置PA2输入小于1.65V电压，PA4拉高。

Example execution steps:
1. Compile and download the program to MCU and run it;
2. Configure PB1 input voltage greater than 1.65V and LED light off;
3. If the input voltage of PB1 is less than 1.65V, the LED light will be on.
4. Configure PA2 input voltage greater than 1.65V and PA4 pull down;
5. If the input voltage of PA2 is less than 1.65V, PA4 pull up;
================================================================================
注意事项：

Notes:
================================================================================