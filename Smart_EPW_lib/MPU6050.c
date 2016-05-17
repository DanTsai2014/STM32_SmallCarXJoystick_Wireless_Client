
#include "MPU6050.h"
#include "delay.h"
#include "uart.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_gpio.h"
#include "stdbool.h"

void MPU6050_Initialize_1()             //初始化过程 ，其实就是写 5个寄存器
{
    MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS,0x00,MPU6050_RA_PWR_MGMT_1);      // reg107, 唤醒，8M内部时钟源
    MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS,0x07,MPU6050_RA_SMPLRT_DIV);         //采用频率 1000
    MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS,0x06,MPU6050_RA_CONFIG);                 
    MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS,0x01,MPU6050_RA_ACCEL_CONFIG);     //加速度量程 2g
    MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS,0x18,MPU6050_RA_GYRO_CONFIG);          //角速度量程 2000度/s
}
void MPU6050_Initialize_2()
{
    MPU6050_I2C_ByteWrite(MPU6050_SECOND_ADDRESS,0x00,MPU6050_RA_PWR_MGMT_1);
    MPU6050_I2C_ByteWrite(MPU6050_SECOND_ADDRESS,0x07,MPU6050_RA_SMPLRT_DIV);
    MPU6050_I2C_ByteWrite(MPU6050_SECOND_ADDRESS,0x06,MPU6050_RA_CONFIG);
    MPU6050_I2C_ByteWrite(MPU6050_SECOND_ADDRESS,0x01,MPU6050_RA_ACCEL_CONFIG);
    MPU6050_I2C_ByteWrite(MPU6050_SECOND_ADDRESS,0x18,MPU6050_RA_GYRO_CONFIG);
}

void MPU6050_Write(uint8_t slaveAddr, uint8_t regAddr, uint8_t data) 
{
    uint8_t tmp;  
    tmp = data;
    MPU6050_I2C_ByteWrite(slaveAddr,&tmp,regAddr);   
}

void MPU6050_I2C_ByteWrite(u8 slaveAddr, u8 pBuffer, u8 writeAddr)
{
    /* Send START condition */ 
    I2C_GenerateSTART(MPU6050_I2C, ENABLE); //发送I2C1开始信号
    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT));
    /* Send MPU6050 address for write */
    I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Transmitter); // 发送 MPU6050 地址、状态（写）
    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    /* Send the MPU6050's internal address to write to */
    I2C_SendData(MPU6050_I2C, writeAddr);//发送 MPU6050内部某个待写寄存器地址
    /* Test on EV8 and clear it */
    while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    /* Send the byte to be written */
    /*if (pBuffer!=0)*/ I2C_SendData(MPU6050_I2C, pBuffer);//发送要写入的内容
    /* Test on EV8_2 and clear it */
    while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    /* Send STOP condition */
    I2C_GenerateSTOP(MPU6050_I2C, ENABLE);//发送结束信号
}

/**
 * @brief  Initializes the I2C peripheral used to drive the MPU6050
 * @param  None
 * @return None
 */
void MPU6050_I2C_Init()
{
    I2C_InitTypeDef I2C_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    //I2C_DeInit(MPU6050_I2C);
    /* Enable I2C and GPIO clocks */
    RCC_APB1PeriphClockCmd(MPU6050_I2C_RCC_Periph, ENABLE);
    RCC_AHB1PeriphClockCmd(MPU6050_I2C_RCC_Port, ENABLE); // | RCC_AHB1Periph_DMA1

    /* Configure I2C pins: SCL and SDA */
    GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SCL_Pin | MPU6050_I2C_SDA_Pin;
    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //設定腳位為 覆用 ( AF - Alternate Function )
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // set output to open drain --> the line has to be only pulled low, not driven high
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // //两个引脚都加 4.7K 上拉电阻
    GPIO_Init(MPU6050_I2C_Port, &GPIO_InitStructure);

     // 連接 I2C1 到 AF 
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1); // SCL
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA
    //GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
    //GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
    // I2C configuration
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0xc0; //STM32 的自身地址，不与从器件相同即可// MPU6050 7-bit adress = 0x68, 8-bit adress = 0xD0; //MPU6050_DEFAULT_ADDRESS
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = MPU6050_I2C_Speed;

    // Apply I2C configuration after enabling it 
    I2C_Init(MPU6050_I2C, &I2C_InitStructure);
    // I2C Peripheral Enable 
    I2C_Cmd(MPU6050_I2C, ENABLE);
}

void MPU6050_GetRawAccelGyro_1(s16* AccelGyro_1)//读加速度值 和 角速度值
{
	u8 i;
	MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, I2C_Rx_Buffer_1, MPU6050_RA_ACCEL_XOUT_H, 14); //读 RA_ACCEL_XOUT_H 寄存器的值
    //Read Accel data from byte 0 to byte 2
	/* Get acceleration, skip byte 3 of temperature data*/
	for (i = 0; i < 3; i++)
	AccelGyro_1[i] = ((s16) ((u16) I2C_Rx_Buffer_1[2 * i] << 8) + I2C_Rx_Buffer_1[2 * i + 1]);
    //Read Gyro data from byte 4 to byte 6
	/* Get Angular rate */
	for (i = 4; i < 7; i++)//在此跳过温度寄存器，不需要温度值
	AccelGyro_1[i - 1] = ((s16) ((u16) I2C_Rx_Buffer_1[2 * i] << 8) + I2C_Rx_Buffer_1[2 * i + 1]);
}
void MPU6050_GetRawAccelGyro_2(s16* AccelGyro_2)//读加速度值 和 角速度值
{
    u8 i;
    MPU6050_I2C_BufferRead(MPU6050_SECOND_ADDRESS, I2C_Rx_Buffer_2, MPU6050_RA_ACCEL_XOUT_H, 14); //读 RA_ACCEL_XOUT_H 寄存器的值
    //Read Accel data from byte 0 to byte 2
    /* Get acceleration, skip byte 3 of temperature data*/
    for (i = 0; i < 3; i++)
    AccelGyro_2[i] = ((s16) ((u16) I2C_Rx_Buffer_2[2 * i] << 8) + I2C_Rx_Buffer_2[2 * i + 1]);
    //Read Gyro data from byte 4 to byte 6
    /* Get Angular rate */
    for (i = 4; i < 7; i++)//在此跳过温度寄存器，不需要温度值
    AccelGyro_2[i - 1] = ((s16) ((u16) I2C_Rx_Buffer_2[2 * i] << 8) + I2C_Rx_Buffer_2[2 * i + 1]);
}

void MPU6050_I2C_BufferRead(u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead)
{
	// ENTR_CRT_SECTION();
	/* While the bus is busy */
	while (I2C_GetFlagStatus(MPU6050_I2C, I2C_FLAG_BUSY));
	/* Send START condition */
	I2C_GenerateSTART(MPU6050_I2C, ENABLE);
	/* Test on EV5 and clear it */
	while (!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT));
	/* Send MPU6050 address for write */
	I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Transmitter);
	/* Test on EV6 and clear it */
	while (!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(MPU6050_I2C, ENABLE);
	/* Send the MPU6050's internal address to write to */
	I2C_SendData(MPU6050_I2C, readAddr);
	/* Test on EV8 and clear it */
	while (!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	/* Send STRAT condition a second time */
	I2C_GenerateSTART(MPU6050_I2C, ENABLE);
	/* Test on EV5 and clear it */
	while (!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT));
	/* Send MPU6050 address for read */
	I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Receiver);
	/* Test on EV6 and clear it */
	while (!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	/* While there is data to be read */
	while (NumByteToRead)
	{
		if (NumByteToRead == 1)
		{
			/* Disable Acknowledgement */
			I2C_AcknowledgeConfig(MPU6050_I2C, DISABLE);
			/* Send STOP Condition */
			I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
		}
		/* Test on EV7 and clear it */
		if (I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			/* Read a byte from the MPU6050 */
			*pBuffer = I2C_ReceiveData(MPU6050_I2C);
			/* Point to the next location where the byte read will be saved */
			pBuffer++;
			/* Decrement the read bytes counter */
			NumByteToRead--;
		}
	}
	/* Enable Acknowledgement to be ready for another reception */
	I2C_AcknowledgeConfig(MPU6050_I2C, ENABLE);
	// EXT_CRT_SECTION();
}

void MPU6050_SetSleepModeStatus(FunctionalState NewState) 
{
    MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, NewState);
}
void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) 
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);  
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    tmp &= ~(mask); // zero all important bits in existing byte
    tmp |= data; // combine data with existing byte
    MPU6050_I2C_ByteWrite(slaveAddr,&tmp,regAddr);   
}
void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) 
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1); 
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    tmp &= mask;
    tmp >>= (bitStart - length + 1);
    *data = tmp;
}
void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) 
{
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);  
    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
    MPU6050_I2C_ByteWrite(slaveAddr,&tmp,regAddr); 
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, FALSE otherwise
 */
bool MPU6050_TestConnection() 
{
    if(MPU6050_GetDeviceID_1() == 0x34) //0b110100; 8-bit representation in hex = 0x34
      return TRUE;
    else
      return FALSE;
}
uint8_t MPU6050_GetDeviceID_1()
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &tmp);
    return tmp; 
}

