/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include<stdint.h>
#include<string.h>
#include<stdio.h>


#include "stm32f407xx.h"

extern void initialise_monitor_handles();

I2C_Handle_t I2CHandle;
// 7 bit DS1307 address is b1101000
#define DS1307_ADDRESS  0x68
#define SLAVE_ADDR DS1307_ADDRESS

typedef struct
{
	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t dow;
}RTC_Date_t;

typedef struct
{
	uint8_t seconds;
	uint8_t mins;
	uint8_t hours;
	uint8_t format;
}RTC_Time_t;

#define FORMAT_AM 0
#define FORMAT_PM 1
#define FORMAT_24H 2
void clk_init(void)
{

}

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1Handle;

//rcv buffer
uint8_t rcv_buf[32];

/*
 * PB6-> SCL
 * PB7 -> SDA
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins. GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);


	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);


}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn,GpioLed;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	//GPIO_Init(&GpioLed);

}


void DS107_I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	while( I2C_MasterSendDataIT(pI2CHandle,pTxBuffer,Len,SlaveAddr,Sr) != I2C_READY );

}

void DS107_I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	while( I2C_MasterSendDataIT(pI2CHandle,pRxBuffer,Len,SlaveAddr,Sr) != I2C_READY );

}


void wait_till_button_press(void)
{
	while(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13));
	delay();
}

uint8_t is_ds1307_running()
{
	//1. ch = 0 means rtc is not connected or rtc CH is halted
	//2. ch = 1 means rtc is detectd and running
	uint8_t addr_0 = 0x00;

	DS107_I2C_MasterSendDataIT(&I2CHandle,&addr_0,1,SLAVE_ADDR,I2C_ENABLE_SR);

	DS107_I2C_MasterReceiveDataIT(&I2CHandle,&addr_0,1,SLAVE_ADDR,I2C_DISABLE_SR);

	return !(addr_0 >> 7);
}

/**
  * @brief  Convert from 2 digit BCD to Binary.
  */
 uint8_t RTC_Bcd2ToByte(uint8_t Value)
{
  uint8_t tmp = 0;
  tmp = ((uint8_t)(Value & (uint8_t)0xF0) >> (uint8_t)0x4) * 10;
  return (tmp + (Value & (uint8_t)0x0F));
}

 uint8_t RTC_ByteToBcd2(uint8_t Value)
 {
   uint8_t bcdhigh = 0;

   while (Value >= 10)
   {
     bcdhigh++;
     Value -= 10;
   }

   return  ((uint8_t)(bcdhigh << 4) | Value);
 }
uint8_t ds1307_read_from_address(uint8_t addr)
{
 	uint8_t value;
 	DS107_I2C_MasterSendDataIT(&I2CHandle,&addr,1,SLAVE_ADDR,1);
 	DS107_I2C_MasterReceiveDataIT(&I2CHandle,&value,1,SLAVE_ADDR,0);


	return value;
}

void ds1307_write_to_address(uint8_t value, uint8_t addr)
{
 	uint8_t data[2];
 	data[0]= addr;
 	data[1]= value;
 	DS107_I2C_MasterSendDataIT(&I2CHandle,data,2,SLAVE_ADDR,0);
	//I2C_MasterSendData(&I2CHandle,&data,1,SLAVE_ADDR,0);
}

void ds1307_set_current_time(RTC_Time_t *pRTC_Time,I2C_RegDef_t *pI2C)
{

    uint8_t seconds = RTC_ByteToBcd2(pRTC_Time->seconds);
    seconds &= ~( 1 << 7);
	ds1307_write_to_address(seconds,0x00);
	ds1307_write_to_address(RTC_ByteToBcd2(pRTC_Time->mins),0x01);


	if(pRTC_Time->format == FORMAT_24H)
	{
		pRTC_Time->hours &= ~(1 << 6);
	}else
	{
		pRTC_Time->hours |= (1<<6);
		if(pRTC_Time->format == FORMAT_PM)
		{
			pRTC_Time->hours |= ( 1 << 5);
		}else
		{
			pRTC_Time->hours &= ~( 1 << 5);
		}
	}

	ds1307_write_to_address(RTC_ByteToBcd2(pRTC_Time->hours),0x02);
	//I2C_MasterGenerateSTOPCondition(pI2C);

}

void ds1307_set_current_date(RTC_Date_t *pRTC_Date)
{



}


void ds1307_get_current_time(RTC_Time_t *pRTC_Time,I2C_RegDef_t *pI2C)
{
	//RTC_Bcd2ToByte

	uint8_t seconds = ds1307_read_from_address(0x00);
	seconds &= ~( 1 << 7);

	pRTC_Time->seconds = RTC_Bcd2ToByte(seconds);

	pRTC_Time->mins = RTC_Bcd2ToByte(ds1307_read_from_address(0x01));
	pRTC_Time->hours = ds1307_read_from_address(0x02);


//	I2C_MasterGenerateSTOPCondition(pI2C);
	if(pRTC_Time->hours & (1 << 6))
	{
		//12h format
		if(pRTC_Time->hours & ( 1 << 5))
		{
			pRTC_Time->format = FORMAT_PM;
		}else
		{
			pRTC_Time->format = FORMAT_AM;
		}
		pRTC_Time->hours &= ~(0X3 << 5);//Clear 6 and 5

	}else
	{
		pRTC_Time->format = FORMAT_24H;
	}
	pRTC_Time->hours = RTC_Bcd2ToByte(pRTC_Time->hours);
}

void ds1307_get_current_date(RTC_Date_t *pRTC_Date,I2C_RegDef_t *pI2C)
{
	pRTC_Date->dow =  RTC_Bcd2ToByte(ds1307_read_from_address(0x03));
	pRTC_Date->date = RTC_Bcd2ToByte(ds1307_read_from_address(0x04));
	pRTC_Date->month = RTC_Bcd2ToByte(ds1307_read_from_address(0x05));
	pRTC_Date->year = RTC_Bcd2ToByte(ds1307_read_from_address(0x06));
	I2C_MasterGenerateSTOPCondition(pI2C);
}

int main(void)
{
	RTC_Time_t currentTime;

	initialise_monitor_handles();
	//printf("RTC test\n");

	GPIO_ButtonInit();

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	//ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);


	wait_till_button_press();

	if(! is_ds1307_running())
	{
		printf("RTC is not connected or not running\n\r");
	}

		//read time and date and print
		//use Sr
		//seconds
		currentTime.seconds = 30;
		currentTime.mins = 10;
		currentTime.hours = 10;
		currentTime.format = FORMAT_24H;
		ds1307_set_current_time(&currentTime,I2C1);
		memset(&currentTime,0,sizeof(currentTime));

		printf("Current Time is : %d:%d:%d\n",currentTime.hours,currentTime.mins,currentTime.seconds);

	//	printmsg("Current Date is : %02d-%2d-%2d  <%s> \r\n",RTC_DateRead.Month,RTC_DateRead.Date,RTC_DateRead.Year,getDayofweek(RTC_DateRead.WeekDay));



	while(1)
	{
		wait_till_button_press();
		ds1307_get_current_time(&currentTime,I2C1);
		printf("Current Time is : %d:%d:%d\n",currentTime.hours,currentTime.mins,currentTime.seconds);
	}


	for(;;);
}





