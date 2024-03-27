================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述:
此样例是通过轮询方式对串口外设接口（SPI）与外部设备以全双工串行方式进行通信的演示,
此接口设置为主模式，为外部从设备提供通信时钟SCK。主机通过MOSI引脚发送数据,从MISO
引脚接收从机的数据，数据以主机提供的SCK沿同步被移位，完成全双工通信。

Function descriptions:
This example is a demonstration of communication between the serial peripheral
interface (SPI) and external devices in full duplex serial mode through polling.
This interface is set as the main mode and provides communication clock SCK for
external slave devices. The host sends data through the MOSI pin and receives
data from the slave through the MISO pin. The data is synchronously shifted
along the SCK provided by the host, completing full duplex communication.
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
使用步骤:
1.选择两块PY32T020_STK板，一块作为主机，一块作为从机
2.编译下载主机程序SPI_TwoBoards_FullDuplexMaster_Polling_Init
3.编译下载从机程序SPI_TwoBoards_FullDuplexSlave_Polling_Init
4.主机与从机引脚连接(箭头指向为信号传输方向) 
主机MASTER：         从机SLAVE：
SCK(PA5)   ----->    SCK(PA5)
MISO(PA6)  <-----    MISO(PA6)
MOSI(PA7)  ----->    MOSI(PA7)
NSS(PA4)   ----->    NSS(PA4)
GND        <----->   GND
5.主从机上电
6.按下从机复位按键先运行从机程序，再按下主机用户按键运行主机程序 
7.观察主从机的LED灯，当主机和从机LED灯由常暗转为常亮状态，则表明主机、从机收发数
据成功；当主机或从机LED灯处于闪烁状态，则表明主机、从机收发数据失败。

Example execution steps:
1.Select two PY32T020 blocks_ STK board, one as the master and one as the slave
2.Compile and download master program SPI_TwoBoards_FullDuplexMaster_Polling_Init
3.Compile and download slave program SPI_TwoBoards_FullDuplexSlave_Polling_Init
4.Connection between the master and slave pins (arrow pointing in the direction
of signal transmission)
MASTER：         SLAVE：
SCK(PA5)   ----->    SCK(PA5)
MISO(PA6)  <-----    MISO(PA6)
MOSI(PA7)  ----->    MOSI(PA7)
NSS(PA4)   ----->    NSS(PA4)
GND        <----->   GND
5.Powering on the master and slave machines
6.Press the slave reset button to run the slave program first, and then press
the master user button to run the master program
7. Observe the LED lights of the master and slave. When the LED lights of the 
master and slave turn from normal dark to steady on, it indicates that the 
master and slave receive and send data successfully; When the LED of the host or 
slave is blinking, it indicates that the host or slave fails to receive and send 
data.
================================================================================
注意事项:
1.必须先按从机复位按键使从机程序先运行，再按主机用户按键开始运行主机程序，否则会
导致主从机通信失败。
2.NSS引脚只需要在使用NSS硬件方式时连接（本样例使用的是NSS硬件方式）
4.建议去掉STK板上R21、R1、R7、R8电阻

Notes:
1.You must first press the slave reset button to run the slave program first,
and then press the master user button to start running the master program, otherwise
it will cause communication failure between the master and slave machines.
2.The NSS pin only needs to be connected when using NSS hardware mode
(this example uses NSS hardware mode)
3.It is recommended to remove the R21, R1, R7 and R8 resistors from the STK 
board.
================================================================================