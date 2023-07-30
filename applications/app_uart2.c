#include <rtthread.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include <drivers/pin.h>
#include <drv_common.h>
#include <rtthread.h>
#include "app_uart2.h"


#define SAMPLE_UART_NAME       "uart2"/* 根据实际485串口来修改 */

static struct rt_semaphore rx_sem;/* 用于接收消息的信号量 */

rt_sem_t sem_write,sem_read;/* 用于控制和返回数据的信号量 */

rt_device_t uart2_handle;/* 串口句柄 */

rt_uint8_t sum = 0;  /* 校验和 */

rt_uint8_t data[FRAME_NUM];/* 接收数据缓冲区 */

/* 接收数据回调函数 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_sem_release(&rx_sem);

    return RT_EOK;
}


static void serial_thread_entry(void *parameter)
{
//    char str[] = "Frame headers do not match!\r\n";
//    char str1[] = "The checksum does not match!\r\n";
//    char str2[] = "Complete data received!\r\n";
    static rt_uint8_t index = 0;  /* 当前帧数据的索引 */
    rt_uint8_t ch;  /* 接收到的数据 */

    static rt_uint8_t rx_buf[FRAME_NUM];  /* 接收缓冲区，用于存储接收到的数据 */

    while (1)
    {
        /* 从串口读取一个字节的数据，没有读取到则等待接收信号量 */
        while (rt_device_read(uart2_handle, -1, &ch, 1) != 1)
        {
            /* 阻塞等待接收信号量，等到信号量后再次读取数据 */
            rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        }

//        rt_kprintf("ch = %x\r\n",ch);
        /* 处理数据 */
        if (index == 0 && ch != FRAME_HEAD)  /* 如果第一帧数据不是帧头，则丢弃 */
        {
//            RS485_TX;
//            rt_device_write(uart2_handle, 0, str, (sizeof(str) - 1));
//            RS485_RX;
            continue;
        }
        if (index >= FRAME_NUM - 1)  /* 如果接收到了一串完整数据则发送消息 */
        {
            index = 0;
            if(sum == ch)
            {
//                RS485_TX;
//                rt_device_write(uart2_handle, 0, str2, (sizeof(str2) - 1));
//                RS485_RX;
                rt_memcpy(data, rx_buf, sizeof(rx_buf));
                sum = 0;
                if(rx_buf[1] == 0x52)
                {
                    rt_sem_release(sem_write);
//                    rt_kprintf("Complete data received!\r\n");
                }
                else if(rx_buf[1] == 0x51)
                {
                    rt_sem_release(sem_read);
//                    rt_kprintf("Complete data received!\r\n");
                }
                continue;
            }
            else
            {
                sum = 0;
//                RS485_TX;
//                rt_device_write(uart2_handle, 0, str1, (sizeof(str1) - 1));
//                RS485_RX;
                continue;
            }
        }
        rx_buf[index++] = ch;
        sum = ch + sum;

        rt_thread_mdelay(5);
    }
}

 int uart_sample()
{
    rt_err_t ret = RT_EOK;
    char uart_name[RT_NAME_MAX];
//    char str[] = "hello RT-Thread!\r\n";
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */

    rt_strncpy(uart_name, SAMPLE_UART_NAME, RT_NAME_MAX);

#ifdef RS485_WP_PIN
    rt_pin_mode(RS485_WP, PIN_MODE_OUTPUT);
#endif

    /* 查找系统中的串口设备 */
    uart2_handle = rt_device_find(uart_name);
    if (!uart2_handle)
    {
        rt_kprintf("find %s failed!\n", uart_name);
        return RT_ERROR;
    }

    /* step2：修改串口配置参数 */
    config.baud_rate = BAUD_RATE_115200;        //修改波特率为 9600
    config.data_bits = DATA_BITS_8;           //数据位 8
    config.stop_bits = STOP_BITS_1;           //停止位 1
    config.bufsz     = 256 * 200;                   //修改缓冲区 buff size 为 128
    config.parity    = PARITY_NONE;           //无奇偶校验位

    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(uart2_handle, RT_DEVICE_CTRL_CONFIG, &config);

    /* 初始化信号量 */
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
    sem_write = rt_sem_create("sem_write", 0, RT_IPC_FLAG_FIFO);
    sem_read = rt_sem_create("sem_read", 0, RT_IPC_FLAG_FIFO);
    /* 以中断接收及轮询发送模式打开串口设备 */
    rt_device_open(uart2_handle, RT_DEVICE_FLAG_INT_RX);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(uart2_handle, uart_input);
#ifdef RS485_WP_PIN
//    RS485_TX;
//    /* 发送字符串 */
//    rt_device_write(uart2_handle, 0, str, (sizeof(str) - 1));
    RS485_RX;
#endif
    /* 创建 serial 线程 */
    rt_thread_t thread = rt_thread_create("uart2_handle", serial_thread_entry, RT_NULL, 3072, 10, 10);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        ret = RT_ERROR;
    }

    return ret;
}


/* 硬件控制测试 */
void RS485_wp_High(void)
{
    rt_pin_mode(RS485_WP, PIN_MODE_OUTPUT);
    RS485_TX;
}
MSH_CMD_EXPORT(RS485_wp_High, RS485_wp_High);

void RS485_wp_LOW(void)
{
    rt_pin_mode(RS485_WP, PIN_MODE_OUTPUT);
    RS485_RX;
}
MSH_CMD_EXPORT(RS485_wp_LOW, RS485_wp_High);

void read_wp_pin(void)
{
    rt_kprintf("Read Wp pin :%d\n", rt_pin_read(RS485_WP));

}
MSH_CMD_EXPORT(read_wp_pin, read_wp_pin);
