#include <stdint.h>
#include <temhumsensor.h>
#include <Ucglib.h>
#include <timer.h>
#include <kalman_filter.h>
#include <stm32f401re_i2c.h>
#include <stm32f401re_spi.h>
#include <string.h>
#include <stm32f401re_gpio.h>
#include <stm32f401re_rcc.h>
#include <stdio.h>

/***********************************************************************************/
#define CYCLE_SEND_DATA_1 1000 // ms

#define CYCLE_SEND_DATA_2 2000 // ms



/*Defined SPI **********************************************************************/

#define SPI1_CS_PORT           				 GPIOB
#define SPI1_CS_PIN            				 GPIO_Pin_6


#define SPI1_RST_PORT          				 GPIOC
#define SPI1_RST_PIN           				 GPIO_Pin_7

#define SPI1_MOSI_PORT         				 GPIOA
#define SPI1_MOSI_PIN          				 GPIO_Pin_7

#define SPI1_SCK_PORT          				 GPIOA
#define SPI1_SCK_PIN           				 GPIO_Pin_5

#define SPI1_RS_PORT           				 GPIOA
#define SPI1_RS_PIN            				 GPIO_Pin_9

#define SPI1_ENABLE_PORT       				 GPIOB
#define SPI1_ENABLE_PIN        				 GPIO_Pin_10

#define SPI1_MODE_PORT          			 GPIOA
#define SPI1_MODE_PIN           			 GPIO_Pin_8

/*Defined I2C ******************************************************************/

#define I2Cx_RCC							 RCC_APB1Periph_I2C1
#define I2Cx_SENSOR							 I2C1
#define I2C_GPIO_RCC		    			 RCC_AHB1Periph_GPIOB
#define I2C_GPIO							 GPIOB
#define I2C_PIN_SDA			    		 	 GPIO_Pin_9
#define I2C_PIN_SCL			    			 GPIO_Pin_8

/*Defined SENSOR ***************************************************************/

#define SI7020_ADDR                          0x40
#define CMDR_MEASURE_VALUE                   0xE0


/*Privated Function ************************************************************/
static void AppCommon();
static void SPI1_Init(void);

static void I2C_Init_temphumi(void);
void I2C_start(void);
void I2C_address_direction(uint8_t address, uint8_t direction);
void I2C_transmit(uint8_t byte);
void I2C_stop(void);
uint8_t I2C_receive_nack(void);
uint8_t I2C_receive_ack(void);
static void TemHumSensor_readRegister(
		uint8_t address,
	    uint8_t* pAddressRegister,
	    uint8_t* pDataRegister,
	    uint8_t Length_Data,
	    uint16_t delay);
void processGetValueSensor(void);
uint32_t GetTemp_Sensor(void);
uint32_t GetHumi_Sensor(void);
void delay_ms(uint32_t ms);
uint32_t CalculatorTime(uint32_t dwTimeInit, uint32_t dwTimeCurrent);
void Scan_SensorLCD(void);
void Scan_TimeSensor(uint32_t byRepeats);
/* Private variables ---------------------------------------------------------*/
static ucg_t ucg;
uint32_t time_initial = 0;
uint32_t time_current, time_initial;
uint32_t idTimer = 0;
static char src1[20] = "";
static char src2[20] = "";
static char src3[20] = "";
static char src4[20] = "";
static uint32_t time_total;
static uint8_t temperature, humidity;
static uint8_t temperature1, humidity1;
//static uint8_t temperature2, humidity2;


int main(void)
{
	AppCommon();

	while(1)
	{
		processGetValueSensor();
		processTimerScheduler();
	}

}

/**
 * @func   delay_ms
 * @brief  delay milisecon
 * @param  None
 * @retval None
 */
void delay_ms(uint32_t ms) {

	uint32_t startTime = GetMilSecTick(); //Lưu lại thời điểm hiện tại.
	while (CalculatorTime(startTime, GetMilSecTick()) <= ms)
		;

//	Đợi cho đến khi hết khoảng time cài đặt

}

/**
 * @func   CalculatorTime
 * @brief  Caculator time button
 * @param  None
 * @retval None
 */
uint32_t CalculatorTime(uint32_t dwTimeInit, uint32_t dwTimeCurrent) {
	uint32_t dwTimeTotal;
	if (dwTimeCurrent >= dwTimeInit) {
		dwTimeTotal = dwTimeCurrent - dwTimeInit;
	} else {
		dwTimeTotal = 0xFFFFFFFFU + dwTimeCurrent - dwTimeInit;
	}
	return dwTimeTotal;

}

/**
 * @func   AppCommon
 * @brief  Init everything
 * @param  None
 * @retval None
 */
static void AppCommon()
{
	SystemCoreClockUpdate();
	TimerInit();
    SPI1_Init();
    I2C_Init_temphumi();
    TemHumSensor_Init();
    Ucglib4WireSWSPI_begin(&ucg, UCG_FONT_MODE_SOLID); //là hàm khởi tạo LCD.
    ucg_ClearScreen(&ucg);
    ucg_SetFont(&ucg, ucg_font_ncenR12_hr);
    ucg_SetColor(&ucg, 0, 255, 255, 255);
    ucg_SetColor(&ucg, 1, 0, 0, 0);
    ucg_SetRotate180(&ucg);
    Scan_TimeSensor(5000);
    time_initial = GetMilSecTick();
}

/**
 * @func   SPI1_Init
 * @brief  Init SPI
 * @param  None
 * @retval None
 */
static void SPI1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOA, GPIOB and GPIOC Clocks enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN | SPI1_MOSI_PIN | SPI1_RS_PIN | SPI1_MODE_PIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI1_CS_PIN | SPI1_ENABLE_PIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI1_RST_PIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
 * @func   I2C_Init_temphumi
 * @brief  Init I2C temp and humi
 * @param  None
 * @retval None
 */
static void I2C_Init_temphumi(void)
{
// Initialization struct
	I2C_InitTypeDef I2C_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;

	// Step 1: Initialize I2C
	RCC_APB1PeriphClockCmd(I2Cx_RCC, ENABLE);
	I2C_InitStruct.I2C_ClockSpeed = 400000;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2Cx_SENSOR, &I2C_InitStruct);
	I2C_Cmd(I2Cx_SENSOR, ENABLE);

	// Step 2: Initialize GPIO as open drain alternate function
	RCC_AHB1PeriphClockCmd(I2C_GPIO_RCC, ENABLE);
	GPIO_InitStruct.GPIO_Pin = I2C_PIN_SCL | I2C_PIN_SDA;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(I2C_GPIO, &GPIO_InitStruct);

	/* Connect PXx to I2C_SCL */
	GPIO_PinAFConfig(I2C_GPIO, GPIO_PinSource8, GPIO_AF_I2C1);

	/* Connect PXx to I2C_SDA */
	GPIO_PinAFConfig(I2C_GPIO, GPIO_PinSource9, GPIO_AF_I2C1);
}

/*
 * @func    I2C_start
 * @brief   Generate I2C start condition
 * @param   None
 * @retval  None
 */
void I2C_start(void)
{
	// Wait until I2Cx is not busy anymore
	while (I2C_GetFlagStatus(I2Cx_SENSOR, I2C_FLAG_BUSY));

	// Generate start condition
	I2C_GenerateSTART(I2Cx_SENSOR, ENABLE);

	// Wait for I2C EV5.
	// It means that the start condition has been correctly released
	// on the I2C bus (the bus is free, no other devices is communicating))
	while (!I2C_CheckEvent(I2Cx_SENSOR, I2C_EVENT_MASTER_MODE_SELECT));
}

/**
 * @func    I2C_address_direction
 * @brief   e
 * @param   None
 * @retval  None
 */
void I2C_address_direction(uint8_t address, uint8_t direction)
{
	// Send slave address
	I2C_Send7bitAddress(I2Cx_SENSOR, address, direction);

	// Wait for I2C EV6
	// It means that a slave acknowledges his address
	if (direction == I2C_Direction_Transmitter)		// truyền
	{
		while (!I2C_CheckEvent(I2Cx_SENSOR, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if (direction == I2C_Direction_Receiver)  // nhận
	{
		while (!I2C_CheckEvent(I2Cx_SENSOR, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}

/**
 * @func    I2C_transmit
 * @brief   Transmit one byte to I2C bus
 * @param   None
 * @retval  None
 */
void I2C_transmit(uint8_t byte)
{
	// Send data byte
	I2C_SendData(I2Cx_SENSOR, byte);
	// Wait for I2C EV8_2.
	// It means that the data has been physically shifted out and
	// output on the bus)
	while (!I2C_CheckEvent(I2Cx_SENSOR, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

/**
 * @func    I2C_stop
 * @brief   Generate I2C stop condition
 * @param   None
 * @retval  None
 */
void I2C_stop(void)
{
	// Generate I2C stop condition
	I2C_GenerateSTOP(I2Cx_SENSOR, ENABLE);
}

/**
 * @func    I2C_receive_nack
 * @brief   receive data
 * @param   None
 * @retval  None
 */
uint8_t I2C_receive_nack(void)
{
	// Disable ACK of received data
	I2C_AcknowledgeConfig(I2Cx_SENSOR, DISABLE);
	// Wait for I2C EV7
	// It means that the data has been received in I2C data register
	while (!I2C_CheckEvent(I2Cx_SENSOR, I2C_EVENT_MASTER_BYTE_RECEIVED));

	// Read and return data byte from I2C data register
	return I2C_ReceiveData(I2Cx_SENSOR);
}


/**
 * @func    I2C_receive_ack
 * @brief   receive data
 * @param   None
 * @retval  None
 */
uint8_t I2C_receive_ack(void)
{
	// Enable ACK of received data
	I2C_AcknowledgeConfig(I2Cx_SENSOR, ENABLE);
	// Wait for I2C EV7
	// It means that the data has been received in I2C data register
	while (!I2C_CheckEvent(I2Cx_SENSOR, I2C_EVENT_MASTER_BYTE_RECEIVED));

	// Read and return data byte from I2C data register
	return I2C_ReceiveData(I2Cx_SENSOR);
}

/**
 * @func    TemHumSensor_readRegister
 * @brief   communicate register in sensor
 * @param   None
 * @retval  None
 */
static void TemHumSensor_readRegister(
		uint8_t address,	// Địa chỉ cảm biến.
	    uint8_t* pAddressRegister, // Địa chỉ của thanh ghi chứa dữ liệu nhiệt độ, độ ẩm.
	    uint8_t* pDataRegister, // Dữ liệu đọc được từ thanh ghi tương ứng.
	    uint8_t Length_Data, //Độ dài Dữ liệu đọc được từ thanh ghi tương ứng.
	    uint16_t delay)	//Dữ liệu đọc được từ thanh ghi tương ứng.
{
	uint8_t LengthCmd = pAddressRegister[0];

	I2C_start();	// conditon start
	I2C_address_direction(address << 1, I2C_Direction_Transmitter); // send slave address - Transmit

	for (uint8_t i = 1; i < LengthCmd; i++) {
		I2C_transmit(pAddressRegister[i]);		// Send Data from register
	}

    if (delay > 0) {
        delay_ms(delay);
    }

	I2C_stop();// condition stop

	I2C_start(); // condition start
	I2C_address_direction(address << 1, I2C_Direction_Receiver); // send slave address - Received

	for (uint8_t i = 0; i < Length_Data; i++)
	{
		if (i == (Length_Data - 1))
		{
			pDataRegister[i] = I2C_receive_nack();  // 	NA
		}
		else
		{
			pDataRegister[i] = I2C_receive_ack();   // A
		}
	}
	I2C_stop();

}


/**
 * @func    GetTemp_Sensor
 * @brief   Get value temperature
 * @param   None
 * @retval  Temperature
 */
uint32_t GetTemp_Sensor(void)
{
	uint32_t RT;
    uint8_t pRT[3] = { 0 };	// pRT[0]: MSB		pRT[1]: LSB

    uint8_t CMD_MEASURE_TEMP[2] =  { 2, 0xE3 }; // gửi độ dài byte cần truyền và CMD_MEASURE chế độ HOLD MASTER MODE

    TemHumSensor_readRegister(SI7020_ADDR, CMD_MEASURE_TEMP, pRT, 3, 4);

    RT = (pRT[0] << 8) + pRT[1];			// RT_CODE (MSB << 8) + LSB		// Temp = ((17572* RT)/(0xFFu +1) - 4685)/100
    RT = ((RT * 17572) >> 16) - 4685;
    RT = RT/100;
    return RT;
}


/**
 * @func    GetHumi_Sensor
 * @brief   Get value humidity
 * @param   None
 * @retval  Humidity
 */
uint32_t GetHumi_Sensor(void)
{
	uint32_t RH;
	uint8_t pRH[3] = { 0 };


	uint8_t CMD_MEASURE_HUMI[2] =  { 2, 0xE5 }; // gửi độ dài byte cần truyền và CMD_MEASURE chế độ HOLD MASTER MODE

    TemHumSensor_readRegister(SI7020_ADDR, CMD_MEASURE_HUMI, pRH, 3, 8);

    RH = (pRH[0] << 8) + pRH[1]; // RH_CODE (MSB << 8) + LSB
    RH = ((RH * 12500) >> 16) - 600;
    RH = RH/100;

    return RH;
}


/**
 * @func 	processGetValueSensor
 * @brief	Get value sensor
 * @param	None
 * @retval	None
 */
void processGetValueSensor(void) {
	// Get timer
	time_current = GetMilSecTick();

	if (time_current >= time_initial) {
		time_total += time_current - time_initial;
	} else {
		time_total += 0xFFFFFFFFU - time_current + time_initial;
	}
	if (time_total == CYCLE_SEND_DATA_1) {
		//Dùng để đọc giá trị nhiệt độ trả về từ cảm biến thông qua giao tiếp I2C.
		temperature = (uint8_t) (GetTemp_Sensor());

		//Dùng để đọc giá trị độ ẩm trả về từ cảm biến thông qua giao tiếp I2C.
		humidity = (uint8_t) (GetHumi_Sensor());

	}
	if(time_total >= CYCLE_SEND_DATA_2)
	{
		temperature1 = (uint8_t) (GetTemp_Sensor());

		humidity1 = (uint8_t) (GetHumi_Sensor());

		// nếu thời gian chênh lệch giữa lần đo 1 và lần đo 2 là 2 thì sẽ cập nhật giá trị sensor
		if( ((temperature1 > temperature ) && (temperature1 - temperature >= 2))
		 || ((temperature1 < temperature ) && (temperature1 - temperature <= 2))
		 || ((humidity1 > humidity ) && (humidity1 - humidity >= 2))
		 || ((humidity1 < humidity ) && (humidity1 - humidity <= 2)))
		{
			ucg_DrawString(&ucg, 0, 32, 0, "Assignment 2");
			memset(src1, 0, sizeof(src1));
			sprintf(src1, " Temp = %d oC  ", temperature1);
			ucg_DrawString(&ucg, 0, 52, 0, src1);

			memset(src2, 0, sizeof(src2));
			sprintf(src2, " Humi = %3d %%   ", humidity1);
			ucg_DrawString(&ucg, 0, 72, 0, src2);
		}

		time_total = 0;
		// Reset lại biến thời gian sau quá trình đọc giá trị nhiệt độ - độ ẩm.

	}
	time_initial = time_current;



}

/**
* @func 	Scan_SensorLCD
* @brief	Scan display value sensor
* @param	None
* @retval	None
*/
void Scan_SensorLCD(void)
{
	ucg_DrawString(&ucg, 0, 32, 0, "Assignment 2");
	memset(src3, 0, sizeof(src3));
	sprintf(src3, " Temp = %d oC  ", temperature);
	ucg_DrawString(&ucg, 0, 52, 0, src3);

	memset(src4, 0, sizeof(src4));
	sprintf(src4, " Humi = %3d %%   ", humidity);
	ucg_DrawString(&ucg, 0, 72, 0, src4);
}
/**
* @func 	Scan_1s
* @brief	Scan period 1s
* @param	None
* @retval	None
*/
void Scan_TimeSensor(uint32_t byRepeats)
{
	if (idTimer != NO_TIMER) {
	TimerStop(idTimer);
	}
	idTimer = TimerStart("Scan_sensor", byRepeats, TIMER_REPEAT_FOREVER, (void*) Scan_SensorLCD, NULL);
}
