/*
 * hr_I2cDevices.c
 *
 *  Created on: 16.08.2016
 *      Author: hrongen
 */

#include <stdio.h>
#include <math.h>
#include <sleep.h>


#include "Hr_xI2C.h"
#include "Hr_I2cDevices.h"

//#define XST_SUCCESS TRUE
//#define XST_FAILURE FALSE




/******************************************************************************************************
 *  INA219 - Bidirectional Current/Power Monitor
 ******************************************************************************************************/
#ifdef Use_INA219

typedef struct
{
	int			ok;
	u8			i2caddr;
	uint16_t	calValue;
	float 		r_shunt;
	float		current_lsb;
	float		power_lsb;
} tINA219;

tINA219		INA219 [INA219_MAXDEVICES];
int			iINA219 = 0;



#define INA219_REG_CONFIG                      	(0x00)		// CONFIG REGISTER (R/W)
#define INA219_CONFIG_RESET                    	(0x8000)  	// Reset Bit

#define INA219_CONFIG_BVOLTAGERANGE_MASK       	(0x2000)  	// Bus Voltage Range Mask
#define INA219_CONFIG_BVOLTAGERANGE_16V        	(0x0000)  	// 0-16V Range
#define INA219_CONFIG_BVOLTAGERANGE_32V        	(0x2000)  	// 0-32V Range

#define INA219_CONFIG_GAIN_MASK                	(0x1800)  	// Gain Mask
#define INA219_CONFIG_GAIN_1_40MV              	(0x0000)  	// Gain 1, 40mV Range
#define INA219_CONFIG_GAIN_2_80MV              	(0x0800)  	// Gain 2, 80mV Range
#define INA219_CONFIG_GAIN_4_160MV             	(0x1000)  	// Gain 4, 160mV Range
#define INA219_CONFIG_GAIN_8_320MV             	(0x1800)  	// Gain 8, 320mV Range

#define INA219_CONFIG_BADCRES_MASK             	(0x0780)  	// Bus ADC Resolution Mask
#define INA219_CONFIG_BADCRES_9BIT             	(0x0080)  	// 9-bit bus res = 0..511
#define INA219_CONFIG_BADCRES_10BIT            	(0x0100)  	// 10-bit bus res = 0..1023
#define INA219_CONFIG_BADCRES_11BIT            	(0x0200)  	// 11-bit bus res = 0..2047
#define INA219_CONFIG_BADCRES_12BIT            	(0x0400)  	// 12-bit bus res = 0..4097

#define INA219_CONFIG_SADCRES_MASK             	(0x0078)  	// Shunt ADC Resolution and Averaging Mask
#define INA219_CONFIG_SADCRES_9BIT_1S_84US     	(0x0000)  	// 1 x 9-bit shunt sample
#define INA219_CONFIG_SADCRES_10BIT_1S_148US   	(0x0008)  	// 1 x 10-bit shunt sample
#define INA219_CONFIG_SADCRES_11BIT_1S_276US   	(0x0010)  	// 1 x 11-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_1S_532US   	(0x0018)  	// 1 x 12-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_2S_1060US  	(0x0048)	// 2 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_4S_2130US  	(0x0050)  	// 4 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_8S_4260US  	(0x0058)  	// 8 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_16S_8510US 	(0x0060)  	// 16 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_32S_17MS   	(0x0068)  	// 32 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_64S_34MS   	(0x0070)  	// 64 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_128S_69MS  	(0x0078)  	// 128 x 12-bit shunt samples averaged together

#define INA219_CONFIG_MODE_MASK                	(0x0007)  	// Operating Mode Mask
#define INA219_CONFIG_MODE_POWERDOWN           	(0x0000)
#define INA219_CONFIG_MODE_SVOLT_TRIGGERED     	(0x0001)
#define INA219_CONFIG_MODE_BVOLT_TRIGGERED     	(0x0002)
#define INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED 	(0x0003)
#define INA219_CONFIG_MODE_ADCOFF              	(0x0004)
#define INA219_CONFIG_MODE_SVOLT_CONTINUOUS    	(0x0005)
#define INA219_CONFIG_MODE_BVOLT_CONTINUOUS    	(0x0006)
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS	(0x0007)

#define INA219_REG_SHUNTVOLTAGE                	(0x01)		// SHUNT VOLTAGE REGISTER (R)
#define INA219_REG_BUSVOLTAGE                  	(0x02)		// BUS VOLTAGE REGISTER (R)
#define INA219_REG_POWER                       	(0x03)		// POWER REGISTER (R)
#define INA219_REG_CURRENT                     	(0x04)		// CURRENT REGISTER (R)
#define INA219_REG_CALIBRATION              	(0x05)		// CALIBRATION REGISTER (R/W)




int	ucIIC_IndexOfAdr(u8 adr)
{
	int i;
	for (i=0; i<INA219_MAXDEVICES; i++)
	{
		if (INA219[i].i2caddr==adr)
		{
//			printf("ucIIC_IndexOfAdr: Adr: 0x%02X -> Index: %d\n",adr,i);
			return i;
		}
	}
#ifdef IIC_DEBUGINFO_INA219
	printf("ucIIC_IndexOfAdr: ERROR no address found !\n");
#endif
	return 0;
}

int  ucIIC_INA219_AddSlave(u8 adr)
{
int i = iINA219;
	INA219[iINA219++].i2caddr = adr;
	return i;
}


u8		SendData[20];
u8		ReadData[20];


int ucIIC_WriteData (int adr, void *SendData, int len)
{
int ret;
	ret = Hr_xI2C_Write (0, adr, SendData, len);
	return (ret);
}


int ucIIC_ReadData (int adr, void *ReadData, int len)
{
int ret;
	ret = Hr_xI2C_Read (0, adr, ReadData, len);
	return (ret);
}


u16	ucIIC_INA219_Write(u8 adr, u8 reg, uint16_t value)
{
int ret;
	SendData[0] = reg;						// Register Pointer
	SendData[1] = (value >> 8) & 0xFF;		// High Byte first
	SendData[2] =  value & 0xFF;			// Low Byte
	ret = Hr_xI2C_Write (0, adr, SendData, 3);
	return ret;
}

u16	ucIIC_INA219_WriteRegPtr(u8 adr, u8 reg)
{
	int ret;
	SendData[0] = reg;			// Register Pointer
	ret = Hr_xI2C_Write (0, adr, SendData, 1);
	return ret;
}

u16	ucIIC_INA219_Read(u8 adr, u8 reg, int16_t *value)
{
uint16_t ret;
	ucIIC_INA219_WriteRegPtr (adr,reg);
	usleep(800); 							// Conversion time at 12 bit is max. 586 us
	ucIIC_ReadData (adr,ReadData,2);
	ret = ((uint16_t)ReadData[0]<<8) | (uint16_t)ReadData[1];
	*value = (int16_t)ret;
	return XST_SUCCESS;
}


/**************************************************************************
    Gets the raw current value (16-bit signed integer, so +-32767)
**************************************************************************/
int16_t ucIIC_INA219_getCurrent_raw(u8 adr)
{
int16_t value;
int	slave = ucIIC_IndexOfAdr(adr);
  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  ucIIC_INA219_Write	(adr, INA219_REG_CALIBRATION, INA219[slave].calValue);
  ucIIC_INA219_Read		(adr, INA219_REG_CURRENT, &value);
  return (int16_t)(value);
}

float ucIIC_INA219_getCurrent_mA(u8 adr)
{
float valueDec;
	int	slave = ucIIC_IndexOfAdr(adr);
	valueDec = ucIIC_INA219_getCurrent_raw(adr);
	return (valueDec * INA219[slave].current_lsb * 1000.0);
}

float ucIIC_INA219_getBusVoltage_V(u8 adr)
{
int16_t value;
	ucIIC_INA219_Read(adr, INA219_REG_BUSVOLTAGE, &value);
	value >>= 3;
	return (value * 0.004);
}

float ucIIC_INA219_getShuntVoltage_V(u8 adr)
{
int16_t value;
	ucIIC_INA219_Read(adr, INA219_REG_SHUNTVOLTAGE, &value);
	return (float)(value / 100000.0);
}

float ucIIC_INA219_getPower_W(u8 adr)
{
int16_t value;
	int	slave = ucIIC_IndexOfAdr(adr);
	ucIIC_INA219_Read(adr, INA219_REG_POWER, &value);
	return (float)(value * INA219[slave].power_lsb);
}



// calibration of equations and device
// shunt_val 		= value of shunt in Ohms
// i_max_expected 	= maximum current draw of bus + shunt
void uIIC_cINA219_Calibrate (u8 adr, float shunt_val, float i_max_expected)
{
uint16_t config;
int	slave;
	slave = ucIIC_IndexOfAdr(adr);

	INA219[slave].r_shunt 		= shunt_val;
	INA219[slave].current_lsb	= i_max_expected / 32768.0;
	INA219[slave].power_lsb 	= 20.0 * INA219[slave].current_lsb;
	INA219[slave].calValue 		= (uint16_t)(0.04096/(INA219[slave].current_lsb*INA219[slave].r_shunt));

#ifdef IIC_DEBUGINFO_INA219
	printf("------------------------------------\n");
	printf("shunt_val         : %f\n",shunt_val);
	printf("i_max_expected    : %f\n",i_max_expected);
	printf("------------------------------------\n");
	printf("current_lsb       : %f\n",INA219[slave].current_lsb);
	printf("power_lsb         : %f\n",INA219[slave].power_lsb);
	printf("r_shunt           : %f\n",INA219[slave].r_shunt);
	printf("cal               : %d\n",INA219[slave].calValue);
	printf("------------------------------------\n");
#endif

#ifdef IIC_DEBUGINFO_INA219
	printf("ucIIC_INA219_Write: Calibration Reg.\n");
#endif
	ucIIC_INA219_Write(adr, INA219_REG_CALIBRATION, INA219[slave].calValue);

#ifdef IIC_DEBUGINFO_INA219
	printf("ucIIC_INA219_Write: Config Reg.\n");
#endif
	config = 	INA219_CONFIG_BVOLTAGERANGE_16V |
						INA219_CONFIG_GAIN_2_80MV |
//						INA219_CONFIG_GAIN_8_320MV |
						INA219_CONFIG_BADCRES_12BIT |
						INA219_CONFIG_SADCRES_12BIT_1S_532US |
						INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	ucIIC_INA219_Write(adr, INA219_REG_CONFIG, config);
}



int INA210_Init (u8 adr)
{

	ucIIC_INA219_AddSlave (AdrINA_VBUS);
	ucIIC_INA219_AddSlave (AdrINA_3V3);
	ucIIC_INA219_AddSlave (AdrINA_A5V);

	INA219[0].ok = (ucIIC_INA219_Write 	(AdrINA_VBUS, 	INA219_REG_CONFIG, INA219_CONFIG_RESET) == XST_SUCCESS);
	INA219[1].ok = (ucIIC_INA219_Write	(AdrINA_3V3, 	INA219_REG_CONFIG, INA219_CONFIG_RESET) == XST_SUCCESS);
	INA219[2].ok = (ucIIC_INA219_Write	(AdrINA_A5V, 	INA219_REG_CONFIG, INA219_CONFIG_RESET) == XST_SUCCESS);


	// Chip Address
	// shunt_val 		= value of shunt in Ohms
	// i_max_expected 	= maximum current draw of bus + shunt
	uIIC_cINA219_Calibrate (AdrINA_VBUS, 	0.05, 3.0);
	uIIC_cINA219_Calibrate (AdrINA_3V3, 	0.05, 3.0);
	uIIC_cINA219_Calibrate (AdrINA_A5V, 	0.05, 3.0);

	return XST_SUCCESS;
}

#endif //Use_INA219


#ifdef Use_SHT21

/******************************************************************************************************
 * SHT21 - Temperature and Humidity
 ******************************************************************************************************/

#define	Device_SHT21		0x80
#define SHT21_cmdReadTEMP	0xE3
#define SHT21_cmdReadRH		0xE5

unsigned short SHT21_ReadAD (int I2cDevId, unsigned char cmd )
{
unsigned char x[4];
unsigned short AdValue;

	x[0] = cmd;						// Read Temperature
	Hr_xI2C_WrRd (I2cDevId, Device_SHT21, 	&x[0], 1,  &x[0], 3);
	AdValue = (x[0]<<8) | x[1];
	AdValue &= ~0x0003;						// clear lower 2 bits (Status bits)
	return (AdValue);
}

float SHT21_ReadTemp (int I2cDevId)
{
float x;
	x = (float) SHT21_ReadAD (I2cDevId, SHT21_cmdReadTEMP);
	x = -46.85 + (175.72 / 65535.0 * x);
	return x;
}

float SHT21_ReadRH (int I2cDevId)
{
float x;
	x = (float) SHT21_ReadAD (I2cDevId, SHT21_cmdReadRH);
	x = -6.0 + (125.0 /65535.0 * x);
	return x;
}

#endif //Use_SHT21



#ifdef Use_RTC1308

//------ Register 0 ..7 are for Realtime Clock
//                8 .. 63 for User Data (56 Bytes)

void	Rtc1308_Write (int I2cDevId, int RegisterAdr, int len, void *p)
{
unsigned char   buf [64];

	buf[0] = RegisterAdr;
	memcpy (&buf[1], p, len);
	Hr_xI2C_Write (I2cDevId, RTC_ADDR, buf, len+1);
}


void	Rtc1308_Read (int I2cDevId, int RegisterAdr, int len, void *p)
{
unsigned char adr;
	adr = RegisterAdr;

	Hr_xI2C_WrRd (I2cDevId, RTC_ADDR, &adr, 1, p, len);
}

void	Rtc1308_SetTime (int I2cDevId, TIMESTRUCT ts)
{
u8				setdata[7];
//twi_package_t	packet;

	setdata[0] = ((ts.sec  / 10) << 4) | (ts.sec  % 10);		// Seconds
	setdata[1] = ((ts.min  / 10) << 4) | (ts.min  % 10);		// Minuten
	setdata[2] = ((ts.hour / 10) << 4) | (ts.hour % 10);		// 24 Std. Mode
	setdata[3] = 1;												// Wochentag: 0 = Sonntag

	setdata[4] = ((ts.day	/ 10) << 4) | (ts.day   % 10);		// Tag
	setdata[5] = ((ts.month / 10) << 4) | (ts.month % 10);		// Monat
	setdata[6] = ((ts.year	/ 10) << 4) | (ts.year  % 10);		// Jahr

	Rtc1308_Write (I2cDevId, 0, 7, (void*)setdata);
}


void	Rtc1308_ReadTime (int I2cDevId, TIMESTRUCT* ts)
{
u8	 rtcdata[8];
	Rtc1308_Read (I2cDevId, 0, 7, (void*)rtcdata);

	ts->sec		= ( ((rtcdata[0]&0x70)>>4)*10) + (rtcdata[0] & 0x0f);
	ts->min		= ( ((rtcdata[1]&0x70)>>4)*10) + (rtcdata[1] & 0x0f);
	ts->hour	= ( ((rtcdata[2]&0x30)>>4)*10) + (rtcdata[2] & 0x0f);

	ts->day		= ( ((rtcdata[4]&0x30)>>4)*10) + (rtcdata[4] & 0x0f);
	ts->month	= ( ((rtcdata[5]&0x10)>>4)*10) + (rtcdata[5] & 0x0f);
	ts->year	= ( ((rtcdata[6]&0xf0)>>4)*10) + (rtcdata[6] & 0x0f);
}


u32 Rtc1308_GetTimeStamp (int I2cDevId)
{
TIMESTRUCT ts;
u32        tx;
	Rtc1308_ReadTime (I2cDevId, &ts);
	tx = ( (ts.hour*3600) + (ts.min*60) + ts.sec) * 1000; // + (xTaskGetTickCount() % 1000);
	return (tx);
}



#endif // Use_RTC1308
