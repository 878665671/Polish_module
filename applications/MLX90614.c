#include "MLX90614.h"
#include <rtthread.h>
#include <rtdevice.h>

/*******************************************************************************
* 函数名: MLX90614  硬件GPIO初始化
* 功能  : MLX90614 完成GPIO端口初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MLX90614_Init( void ) {

    rt_pin_mode(MLX_SCL_PINS, PIN_MODE_OUTPUT);
    rt_pin_mode(MLX_SDA_PINS, PIN_MODE_OUTPUT);
    rt_pin_write(MLX_SCL_PINS, PIN_HIGH);
    rt_pin_write(MLX_SDA_PINS, PIN_HIGH);
}

/*******************************************************************************
* 函数名: MLX90614 发起始位 SMBus_StartBit
* 功能  : MLX90614 发起始位 产生起始位
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SMBus_StartBit( void ) {
    SMBUS_SDA_OUTPUT();
    SMBUS_SDA_H;
    SMBUS_DELAY_5US;
    SMBUS_SCK_H;
    SMBUS_DELAY_5US;
    SMBUS_SDA_L;
    SMBUS_DELAY_5US;
    SMBUS_SCK_L;
    SMBUS_DELAY_5US;
}

/*******************************************************************************
* 函数名: SMBus_StopBit
* 功能: MLX90614 发停止位 STOP condition on SMBus
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SMBus_StopBit( void ) {

    SMBUS_SDA_OUTPUT();
    SMBUS_SCK_L;        // Clear SCL line
    SMBUS_DELAY_5US;   // Wait a few microseconds
    SMBUS_SDA_L;        // Clear SDA line
    SMBUS_DELAY_5US;   // Wait a few microseconds
    SMBUS_SCK_H;        // Set SCL line
    SMBUS_DELAY_5US;   // Stop condition setup time(Tsu:sto=4.0us min)
    SMBUS_SDA_H;        // Set SDA line
}

/*******************************************************************************
* 函数名: SMBus_SendByte
* 功能: MLX90614 发送一个字节 Send a byte on SMBus
* Input          : Tx_buffer
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char SMBus_SendByte( unsigned char Tx_buffer ) {
    unsigned char   Bit_counter;
    unsigned char   Ack_bit;
    unsigned char   bit_out;
    for( Bit_counter=8; Bit_counter; Bit_counter-- ) {
        if( Tx_buffer&0x80 ) { //如果最高位为1
            bit_out=1;   // 把最高位置1
        } else { //如果最高位为0
            bit_out=0;  // 把最高位置0
        }
        SMBus_SendBit( bit_out ); // 把最高位发送出去
        Tx_buffer<<=1;// 左移一位把最高位移出去等待下一个最高位，循环8次，每次都发最高位，就可把一个字节发出去了
    }
    Ack_bit=SMBus_ReceiveBit();
    return  Ack_bit;
}

/*******************************************************************************
* 函数名: SMBus_SendBit
* 功能: MLX90614 发送一个位 Send a bit on SMBus 82.5kHz
* Input          : bit_out
* Output         : None
* Return         : None
*******************************************************************************/
void SMBus_SendBit( unsigned char bit_out ) {
    SMBUS_SDA_OUTPUT();
    if( bit_out==0 ) {
        SMBUS_SDA_L;
    } else {
        SMBUS_SDA_H;
    }
    SMBUS_DELAY_2US;
    SMBUS_SCK_H;                    // Set SCL line
    SMBUS_DELAY_6US;
    SMBUS_SCK_L;                    // Clear SCL line
    SMBUS_DELAY_3US;
    return;
}

/*******************************************************************************
* Function Name  : SMBus_ReceiveBit
* Description    : Receive a bit on SMBus
* Input          : None
* Output         : None
* Return         : Ack_bit
*******************************************************************************/
unsigned char SMBus_ReceiveBit( void ) {
    unsigned char Ack_bit;
    SMBUS_SDA_H;          //引脚靠外部电阻上拉，当作输入

    SMBUS_SDA_INPUT();
    SMBUS_DELAY_2US;
    SMBUS_SCK_H;            // Set SCL line
    SMBUS_DELAY_5US;
    if( rt_pin_read(MLX_SDA_PINS) ) {
        Ack_bit=1;
    } else {
        Ack_bit=0;
    }
    SMBUS_SCK_L;            // Clear SCL line
    SMBUS_DELAY_3US;
    return  Ack_bit;
}

/*******************************************************************************
* 函数名: SMBus_ReceiveByte
* 功能: Receive a byte on SMBus 从SMBus中接受一个字节的数据
* Input          : ack_nack
* Output         : None
* Return         : RX_buffer
*******************************************************************************/
unsigned char SMBus_ReceiveByte( unsigned char ack_nack ) {
    unsigned char   RX_buffer;
    unsigned char   Bit_Counter;
    for( Bit_Counter=8; Bit_Counter; Bit_Counter-- ) {
        if( SMBus_ReceiveBit() ) { // Get a bit from the SDA line
            RX_buffer <<= 1;// If the bit is HIGH save 1  in RX_buffer
            RX_buffer |=0x01;//如果Ack_bit=1，把收到应答信号1与0000 0001 进行或运算，确保为1
        } else {
            RX_buffer <<= 1;// If the bit is LOW save 0 in RX_buffer
            RX_buffer &=0xfe;//如果Ack_bit=1，把收到应答信号0与1111 1110 进行与运算，确保为0
        }
    }
    SMBus_SendBit( ack_nack ); // Sends acknowledgment bit 把应答信号发出去，如果1，就进行下一次通信，如果为0、，就拜拜了
    return RX_buffer;
}

/*******************************************************************************
* 函数名: SMBus_Delay
* 功能: 延时  一次循环约1us
* Input          : time
* Output         : None
* Return         : None
*******************************************************************************/
void SMBus_Delay( unsigned int time ) {

    SMBUS_DELAY_1US;
}

/*******************************************************************************
* 函数名: SMBus_Init
* 功能: SMBus初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MLX90614_SMBus_Init( void ) {
    MLX90614_Init();
}

/*******************************************************************************
 * 函数名: SMBus_ReadMemory
 * 功能: READ _data FROM RAM/EEPROM  从RAM和EEPROM中读取数据
 * Input          : slaveAddress, command
 * Return         : _data
 * SMBus_ReadMemory(0x00, 0x07) 0x00 表示IIC设备的从地址 从0x07这个寄存器开始读取
*******************************************************************************/
unsigned int SMBus_ReadMemory( unsigned char slaveAddress, unsigned char command ) {
    unsigned short _data;         // _data storage (_dataH:_dataL)
    unsigned char Pec;              // PEC byte storage
    unsigned char _dataL=0;         // Low _data byte storage
    unsigned char _dataH=0;         // High _data byte storage
    unsigned char arr[6];           // Buffer for the sent bytes
    unsigned char PecReg;           // Calculated PEC byte storage
    unsigned char ErrorCounter; // Defines the number of the attempts for communication with MLX90614
    ErrorCounter=0x00;  // Initialising of ErrorCounter
    slaveAddress <<= 1; //2-7位表示从机地址 从机地址左移一位，把读写位空出来
    do {
repeat:
        SMBus_StopBit();                //If slave send NACK stop comunication
        --ErrorCounter;                 //Pre-decrement ErrorCounter
        if( !ErrorCounter ) {           //ErrorCounter=0?
            break;                      //Yes,go out from do-while{}
        }
        SMBus_StartBit();               //Start condition
        if( SMBus_SendByte( slaveAddress ) ) { //Send SlaveAddress 最低位Wr=0表示接下来写命令
            goto    repeat;             //Repeat comunication again
        }
        if( SMBus_SendByte( command ) ) { //Send command
            goto    repeat;             //Repeat comunication again
        }
        SMBus_StartBit();                   //Repeated Start condition
        if( SMBus_SendByte( slaveAddress+1 ) ) { //Send SlaveAddress 最低位Rd=1表示接下来读数据
            goto    repeat;                 //Repeat comunication again
        }
        _dataL = SMBus_ReceiveByte( ACK );  //Read low _data,master must send ACK
        _dataH = SMBus_ReceiveByte( ACK ); //Read high _data,master must send ACK
        Pec = SMBus_ReceiveByte( NACK ); //Read PEC byte, master must send NACK
        SMBus_StopBit();                //Stop condition
        arr[5] = slaveAddress;      //
        arr[4] = command;           //
        arr[3] = slaveAddress+1;    //Load array arr
        arr[2] = _dataL;                //
        arr[1] = _dataH;                //
        arr[0] = 0;                 //
        PecReg=PEC_Calculation( arr ); //Calculate CRC 数据校验
    } while( PecReg != Pec ); //If received and calculated CRC are equal go out from do-while{}
    _data = ( _dataH<<8 ) | _dataL; //_data=_dataH:_dataL

    //return 15500;
    return _data;
}

/*******************************************************************************
* 函数名: PEC_calculation
* 功能 : 数据校验
* Input          : pec[]
* Output         : None
* Return         : pec[0]-this byte contains calculated crc value
*******************************************************************************/
unsigned char PEC_Calculation( unsigned char pec[] ) {
    unsigned char   crc[6];//存放多项式
    unsigned char   BitPosition=47;//存放所有数据最高位，6*8=48 最高位就是47位
    unsigned char   shift;
    unsigned char   i;
    unsigned char   j;
    unsigned char   temp;
    do {
        /*Load pattern value 0x00 00 00 00 01 07*/
        crc[5]=0;
        crc[4]=0;
        crc[3]=0;
        crc[2]=0;
        crc[1]=0x01;
        crc[0]=0x07;
        /*Set maximum bit position at 47 ( six bytes byte5...byte0,MSbit=47)*/
        BitPosition=47;
        /*Set shift position at 0*/
        shift=0;
        /*Find first "1" in the transmited message beginning from the MSByte byte5*/
        i=5;
        j=0;
        while( ( pec[i]&( 0x80>>j ) )==0 && i>0 ) {
            BitPosition--;
            if( j<7 ) {
                j++;
            } else {
                j=0x00;
                i--;
            }
        }/*End of while */
        /*Get shift value for pattern value*/
        shift=BitPosition-8;
        /*Shift pattern value */
        while( shift ) {
            for( i=5; i<0xFF; i-- ) {
                if( ( crc[i-1]&0x80 ) && ( i>0 ) ) {
                    temp=1;
                } else {
                    temp=0;
                }
                crc[i]<<=1;
                crc[i]+=temp;
            }/*End of for*/
            shift--;
        }/*End of while*/
        /*Exclusive OR between pec and crc*/
        for( i=0; i<=5; i++ ) {
            pec[i] ^=crc[i];
        }/*End of for*/
    } while( BitPosition>8 ); /*End of do-while*/
    return pec[0];
}

/*******************************************************************************
* 函数名: SMBus_ReadTemp
* 功能: 计算并返回温度值
* Return         : SMBus_ReadMemory(0x00, 0x07)*0.02-273.15
*******************************************************************************/
float SMBus_ReadTemp( void ) {
    float temp;
    temp = ( SMBus_ReadMemory( SA,RAM_ACCESS|RAM_TOBJ1 )*0.02-273.15 );
//    rt_kprintf("tem = %d\n",SMBus_ReadMemory( SA,RAM_ACCESS|RAM_TOBJ1 ));
    return temp;
}
