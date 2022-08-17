#include "Cpu0_Main.h"
#include "headfile.h"
#pragma section all "cpu0_dsram"

#define USB_FRAME_HEAD 0x42 // USB通信序列帧头
#define USB_FRAME_END 0x10  // USB通信序列帧尾
#define USB_FRAME_LENGTH 11 // USB通信序列字节长度

#define FILTER 0
#define BUF_LEN 10

/**
 * 
 * @property {uint8} ReceiveStart - The start of the data receiving
 * @property {uint8} ReceiveIndex - The index of the received data.
 * @property {uint8} ReceiveFinished - This is a flag that indicates whether the data has been received
 * and verified.
 * @property {uint8} ReceiveBuff - This is the buffer that receives the data from the USB port.
 * @property {uint8} ReceiveBuffFinished - The data that has been received and verified.
 */
typedef struct
{
    uint8 ReceiveStart;                          //数据接收开始
    uint8 ReceiveIndex;                          //接收序列
    uint8 ReceiveFinished;                       //数据队列接收并校验完成
    uint8 ReceiveBuff[USB_FRAME_LENGTH];         // USB接收队列：临时接收
    uint8 ReceiveBuffFinished[USB_FRAME_LENGTH]; // USB接收队列：校验成功
} USB_STRUCT;

typedef union
{
    unsigned char U8_Buff[4];
    float Float;
} UNION_BIT32;

typedef union
{
    unsigned char U16_Buff[2];
    short I16;
} UNION_BIT16;

USB_STRUCT usbStr;

volatile float speed = 0;
volatile short angle = 0; // positive for right, negative for left
volatile short dir_go_around = 0;
volatile short dir_busyarea = 0;
uint8 uart_buff;

/**
 * If the received data is the frame head, then set the flag of receiving start to 1, and store the
 * data in the buffer. If the receiving start flag is 1 and the index of the buffer is less than the
 * length of the frame, then store the data in the buffer. If the index of the buffer is greater than
 * or equal to the length of the frame, then check the frame end and the checksum. If the frame end and
 * the checksum are correct, then copy the data in the buffer to the finished buffer, set the flag of
 * receiving finished to 1, and parse the data in the finished buffer
 * 
 * @param data the data received by the serial port
 */
void uart_receiveVerify(unsigned char data)
{
    if (data == USB_FRAME_HEAD && !usbStr.ReceiveStart) //监测帧头
    {
        usbStr.ReceiveStart = 1;
        usbStr.ReceiveBuff[0] = data;
        usbStr.ReceiveIndex = 1;
    }
    else if (usbStr.ReceiveStart && usbStr.ReceiveIndex < USB_FRAME_LENGTH) //通信开始|接收队列数据
    {
        usbStr.ReceiveBuff[usbStr.ReceiveIndex] = data;
        usbStr.ReceiveIndex++;
    }

    if (usbStr.ReceiveIndex >= USB_FRAME_LENGTH)
    {
        if (usbStr.ReceiveBuff[USB_FRAME_LENGTH - 1] == USB_FRAME_END) //帧尾
        {
            uint8 check = 0;
            for (int i = 0; i < USB_FRAME_LENGTH - 2; i++)
                check += usbStr.ReceiveBuff[i]; //校验和

            if (check == usbStr.ReceiveBuff[USB_FRAME_LENGTH - 2]) //校验位
            {
                memcpy(usbStr.ReceiveBuffFinished, usbStr.ReceiveBuff, USB_FRAME_LENGTH);
                usbStr.ReceiveFinished = 1;
                uint8 Addr = (uint8)(usbStr.ReceiveBuffFinished[1] & 0xFF);
                uint8 BuffData[6] = {0};
                UNION_BIT32 UnionBit32;
                UNION_BIT16 UnionBit16;
                for (int i = 0; i < 6; i++)
                {
                    BuffData[i] = usbStr.ReceiveBuffFinished[3 + i];
                }
                switch (Addr)
                {
                case 0x01: //智能车速度,智能车姿态（方向）
                    UnionBit32.U8_Buff[0] = BuffData[0];
                    UnionBit32.U8_Buff[1] = BuffData[1];
                    UnionBit32.U8_Buff[2] = BuffData[2];
                    UnionBit32.U8_Buff[3] = BuffData[3];
                    speed = UnionBit32.Float; //电机速度

                    UnionBit16.U16_Buff[0] = BuffData[4];
                    UnionBit16.U16_Buff[1] = BuffData[5];
                    angle = UnionBit16.I16; //舵机方向
                    break;
                case 0x02: //free zone
                    UnionBit16.U16_Buff[0] = BuffData[4];
                    UnionBit16.U16_Buff[1] = BuffData[5];
                    dir_go_around = UnionBit16.I16; //舵机方向
                    break;
                case 0x03: //busy area
                    UnionBit16.U16_Buff[0] = BuffData[4];
                    UnionBit16.U16_Buff[1] = BuffData[5];
                    dir_busyarea = UnionBit16.I16; //舵机方向
                    break;
                }
            }
        }
        usbStr.ReceiveIndex = 0;
        usbStr.ReceiveStart = 0;
    }
}

int core0_main(void)
{
    get_clk(); //获取时钟频率  务必保留
    //用户在此处调用各种初始化函数等

    uart_init(UART_0, 115200, UART0_TX_P14_0, UART0_RX_P15_3);
    enableInterrupts();

    // uart_putstr(UART_0, "\n---uart test---\n");

    //在本库中，对于串口的接收与发送都是通过中断完成的，因此想要正常的使用串口功能务必保证中断是开启的，也就是调用了enableInterrupts()
    //串口的中断函数全部都在isr.c中。

    //串口中断相关的配置参数都在isr_config.h中
    //可配置参数有 ERU_CH0_CH4_INT_SERVICE 和 ERU_CH0_CH4_INT_PRIO
    // UART0_INT_SERVICE 中断服务者，表示改中断由谁处理，0:CPU0 1:CPU1 3:DMA  不可设置为其他值
    // UART0_TX_INT_PRIO 发送中断优先级 优先级范围1-255 越大优先级越高 与平时使用的单片机不一样
    // UART0_RX_INT_PRIO	接收中断优先级
    // UART0_ER_INT_PRIO 错误中断优先级

    usbStr.ReceiveStart = 0;
#if FILTER
    short angle_buf[BUF_LEN] = { 0 };
    int ptr = 0;
#endif

    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
    enableInterrupts();
    while (1)
    {
        if (uart_query(UART_0, &uart_buff))
        {
            uart_receiveVerify(uart_buff);
        }
#if FILTER
        angle_buf[ptr] = angle;
        ptr = (ptr + 1) % BUF_LEN;
        angle = filter(angle_buf);
#endif
    }
}

#pragma section all restore
