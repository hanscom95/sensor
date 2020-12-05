/**
 * \file
 *
 * \brief CDC Application Main functions
 *
 * Copyright (c) 2011-2012 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <asf.h>
#include "conf_usb.h"
#include "conf_example.h"
#include "ui.h"
#include "uart.h"
#include "twi.h"
#include "tc.h"
#include "delay.h"
#include "linear_acc.h"


#define TWI_SPEED					400000		//i2c speed is 400khz

#define SGA100_BUS_ADDR				(0x64)

#define SGA100_ACC_OUT_X_H			(0x12)   /* X-Axis acceleration data(High Byte) */
#define SGA100_ACC_OUT_X_L			(0x13)   /* X-Axis acceleration data(Low Byte) */
#define SGA100_ACC_OUT_Y_H			(0x14)   /* Y-Axis acceleration data(High Byte) */
#define SGA100_ACC_OUT_Y_L			(0x15)   /* Y-Axis acceleration data(Low Byte) */
#define SGA100_ACC_OUT_Z_H			(0x16)   /* Z-Axis acceleration data(High Byte) */
#define SGA100_ACC_OUT_Z_L			(0x17)   /* Z-Axis acceleration data(Low Byte) */

/** Device sensitivity (scaling) */
#define MICRO_TESLA_PER_COUNT       (0.3)   /* from AK8975 data sheet */

/** \brief SGA100 Register Addresses */
#define SGA100_CTRL1				(0x03)   /* device control register 1 */
#define SGA100_CTRL2				(0x04)   /* device control register 2 */
#define SGA100_CTRL3				(0x05)   /* device control register 3 */

#define TC_TICK_20MS					20
#define TC_TICK_80MS					80
#define TC_TICK_1S						1000

#define OK_MSG_LENGTH					2
#define CONNECT_MSG_LENGTH				7
#define DISCONNECT_MSG_LENGTH			10

#define SGA100_USED

/* SGA100 define register */
#define SGA100_DEV_ID						0x01	/* R */
#define SGA100_STATUS						0x02    /* R */
#define SGA100_CTRL_REG1					0x03	/* RW default 0x61 */
#define SGA100_CTRL_REG2					0x04	/* RW default 0xFC */
#define SGA100_CTRL_REG3					0x05	/* RW default 0x1A */

#define SGA100_INT1_CTRL					0x10	/* RW default 0x1A */
#define SGA100_INT1_MAP_FUNC				0x11	/* RW default 0x00 */
#define SGA100_OUT_DATA_XL					0x12	/* R */
#define SGA100_OUT_DATA_XH					0x13	/* R */
#define SGA100_OUT_DATA_YL					0x14	/* R */
#define SGA100_OUT_DATA_YH					0x15	/* R */
#define SGA100_OUT_DATA_ZL					0x16	/* R */
#define SGA100_OUT_DATA_ZH					0x17	/* R */
#define SGA100_OUT_DATA_TEMP				0x18	/* R */
#define SGA100_USR_GAIN_X_CTRL				0x19	/* RW default 0x80 */
#define SGA100_USR_GAIN_Y_CTRL				0x1A	/* RW default 0x80 */
#define SGA100_USR_GAIN_Z_CTRL				0x1B	/* RW default 0x80 */
#define SGA100_USR_OFS_X_CTRL				0x1C	/* RW default 0x00 */
#define SGA100_USR_OFS_Y_CTRL				0x1D	/* RW default 0x00 */
#define SGA100_USR_OFS_Z_CTRL				0x1E	/* RW default 0x00 */


#define SGA100_DIGITAL_FILTER_CTRL			0x23	/* RW default 0x00 */
#define SGA100_INT_FUNC_CTRL1				0x24	/* RW default 0x00 */
#define SGA100_INT_FUNC_CTRL2				0x25	/* RW default 0x80 */
#define SGA100_FIFO_CTRL					0x27	/* RW default 0x00 */
#define SGA100_FIFO_DATA_STATUS				0x28	/* R */
#define SGA100_INT_FUNC_CTRL1_STATUS		0x29	/* R */
#define SGA100_MOTION_CTRL					0x2A	/* RW default 0x00 */
#define SGA100_MOTION_STATUS				0x2B	/* R */
#define SGA100_MOTION_HIGH_TH				0x2C	/* RW default 0xBE */
#define SGA100_MOTION_HIGH_DUR				0x2D	/* RW default 0x02 */
#define SGA100_MOTION_LOW_TH				0x2E	/* RW default 0x2D */
#define SGA100_MOTION_LOW_DUR				0x2F	/* RW default 0x02 */
#define SGA100_INT1_CNT_DATA				(AVR32_PIN_PA05 & 0x20)	/* R */
#define USE_SPI

/** \brief Sensor Data Descriptor */
typedef struct {		/* Linear axis data */       
	char x1;          /**< x-axis value */
	char x2;          /**< x-axis value */
	char y1;          /**< y-axis value */
	char y2;          /**< y-axis value */
	char z1;          /**< z-axis value */
	char z2;          /**< z-axis value */
} sensor_data_t;

typedef enum _REGISTER_TYPE_PREFIX{
	NORMAL_REGISTER_TYPE_WRITE=0xf1,
	IS3A_PREREGISTER_TYPE_WRITE=0xf2,
	IS3A_POSTREGISTER_TYPE_WRITE=0xf3,
	SAVE_REGISTER_TRIMMINFO_TO_MEMORY=0xf4,
	NORMAL_REGISTER_TYPE_READ=0xf5,
	IS3A_PREREGISTER_TYPE_READ=0xf6,
	IS3A_POSTREGISTER_TYPE_READ=0xf7,
	IS3A_POSTREGISTER_TYPE_MEMBANK_CHANGE_WRITE=0xf8,
}REGISTER_TYPE_PREFIX;

#ifdef USING_LGA12TYPE
//	#define CHIP_SEL PB0 //chip selection (port)
#define CHIP_SEL1 AVR32_PIN_PB00 //chip selection (port)
#define CHIP_SEL2 AVR32_PIN_PB04 //chip selection (port)
#define CHIP_SEL3 AVR32_PIN_PB05 //chip selection (port)
#define CHIP_SEL4 AVR32_PIN_PB06 //chip selection (port)
#endif

static volatile bool main_b_cdc_enable = false;
volatile sensor_data_t sga100_data;

// Flag to update the timer value
volatile static bool update_timer = true;
volatile static bool update_timer_1 = true;
volatile static bool update_timer_2 = true;
// Variable to contain the time ticks occurred
volatile static uint32_t tc_tick = 0;
volatile static uint32_t tc_tick_1 = 0;
volatile static uint32_t tc_tick_2 = 0;

volatile unsigned char 	dbuf[6];
volatile unsigned int 	count=0;
volatile unsigned int data[41];

short outx,outy,outz,outt;//Output Data

volatile unsigned char 	prefix1=0;
volatile unsigned char 	prefix2=0;
volatile unsigned char 	addr1=0;
volatile unsigned char 	addr2=0;
volatile unsigned char 	data1=0;
volatile unsigned char 	data2=0;
volatile unsigned char 	nSensorNum=0;

const char ok_msg[]="OK";
const char connect_msg[]="CONNECT";
const char disconnect_msg[]="DISCONNECT";
volatile static uint16_t buffer_index=0;

int twi_master_setup(void);
void twi_bus_put(uint8_t dev_addr, uint8_t addr, uint8_t data);
size_t twi_bus_write(uint8_t dev_addr, uint8_t addr, const void *data, size_t count);
size_t twi_bus_read(uint8_t dev_addr, uint8_t addr, void *data, size_t count);
void sga100_init(void);
void tc_init(volatile avr32_tc_t *tc);
int read_sensor_data(short *dx,short *dy, short *dz, short *dt); // read data read

uint8_t hextochar_invert_h(uint8_t data);
uint8_t hextochar_invert_l(uint8_t data);

/*************************************************************************************/
// nDeviceCommand :
//select one Sensor Type  < DEVICE_ACCEL_CMD or DEVICE_GYRO_CMD or DEVICE_MAG_CMD>   or  no using  DEVICE_TOTAL_READ_CMD command  or   ACCES_TYPE_READ_CMD //단일 센서의 한개의  Register를 Read 할 경우.  Addr:읽을 Register주소 Data:0
//select one Sensor Type  < DEVICE_ACCEL_CMD or DEVICE_GYRO_CMD or DEVICE_MAG_CMD>   or  no using  DEVICE_TOTAL_READ_CMD command  or   ACCES_TYPE_WRITE_CMD //단일 센서의 한개의  Register를 Write 할 경우. Addr:Write할 Register주소 Data:Write할 8bit Data
//select one Sensor Type  < DEVICE_ACCEL_CMD or DEVICE_GYRO_CMD or DEVICE_MAG_CMD>   or  Using  DEVICE_TOTAL_READ_CMD command  or ACCES_TYPE_READ_CMD //단일 센서의  전체 Output Data를 한번에  Read만 할 경우 전용 . Addr:해당센서의 X,Y,Z Register주소중의 가장 첫번째 주소  Data:0 //주소순서대로 6개를 보내줌.
//select all Type  < DEVICE_ACCEL_CMD and DEVICE_GYRO_CMD and DEVICE_MAG_CMD>   or  Using  DEVICE_TOTAL_READ_CMD command  or ACCES_TYPE_READ_CMD //모든 센서의  전체 출력 Data 보내주는 기능 사용시.   Addr:0 Data:0
//Select No Sensor Type < DEVICE_ACCEL_CMD or DEVICE_GYRO_CMD or DEVICE_MAG_CMD> or No Using DEVICE_TOTAL_READ_CMD or USing Only  ACCES_TYPE_PREVSEND_ACCELDATA_PRINT //이전에 보낸 가속도 센서의 Data를 debuging창에 한번 출력함.  Addr:0 Data:0

////////////////////nDevice Command///////////////
#define DEVICE_ACCEL_CMD						0x80
#define DEVICE_GYRO_CMD							0x40
#define DEVICE_MAG_CMD							0x20

#define DEVICE_TOTAL_READ_CMD					0x10


#define ACCES_TYPE_WRITE_CMD					0x01
#define ACCES_TYPE_READ_CMD						0x02
#define ACCES_TYPE_STREAM_READ_STOP				0x08

#define ACCES_TYPE_PREVSEND_ACCELDATA_PRINT		0x04


////////////////////nDevice Command///////////////
//4byte(0xFE 0xF2 0x0 0x0)를 보내면 전송시작하고 
//4byte(0xFE 0xF8 0x0 0x0)를 서버로 보내면 전송 종료. 
#define CMD_BYTE_SIZE							4 //4BYTE 

volatile bool bServerSend=true;				//Send or Stop
volatile bool bSelectAccel=false;				//Send or Stop
volatile bool bSelectGyro=false;				//Send or Stop
volatile bool bSelectMagnetic=false;			//Send or Stop
volatile bool bClientSend=false;			//Send or Stop

volatile int nClientPort=0;
volatile int nPortIndex=0;						
volatile int nMsgDataStartIndex=0;
volatile int nMsgDataStartIndex2=0;
volatile int nMsgDataEndIndex=0;
volatile int nMsgDataEndIndex2=0;
volatile int nIPStartIndex=0;
volatile int nIPEndIndex=0;

volatile unsigned char g_szServerSendMsg[512]={0};
	
volatile unsigned char szAccelSensorData[64]={0};
volatile unsigned char szMagSensorData[64]={0};
volatile unsigned char szGryoSensorData[64]={0};
volatile unsigned char szSensorData[256]={0};
volatile unsigned char szPacketHeader[5]="PKST";
volatile unsigned char szPacketEnd[3]="\r\n";
int nNextIndex=0;
volatile bool g_bDataSendRepeat=false;
volatile uint8_t connect_cid;
	
// BP Register variable
volatile unsigned char rINDEX=0xff;
volatile unsigned char rSTATUS=0xff;

volatile unsigned char rCTRLREG1=0xff;
volatile unsigned char rCTRLREG2=0xff;
volatile unsigned char rCTRLREG3=0xff;

volatile unsigned char rINT1_CTRL=0xff;
volatile unsigned char rINT1_MAP_FUNC=0xff;
volatile unsigned char rOUT_DATA_XH=0xff;
volatile unsigned char rOUT_DATA_XL=0xff;
volatile unsigned char rOUT_DATA_YH=0xff;
volatile unsigned char rOUT_DATA_YL=0xff;
volatile unsigned char rOUT_DATA_ZH=0xff;
volatile unsigned char rOUT_DATA_ZL=0xff;
volatile unsigned char rOUT_DATA_TEMP=0xff;
volatile unsigned char rUSR_GAIN_X_CTRL=0xff;
volatile unsigned char rUSR_GAIN_Y_CTRL=0xff;
volatile unsigned char rUSR_GAIN_Z_CTRL=0xff;
volatile unsigned char rUSR_OFS_X_CTRL=0xff;
volatile unsigned char rUSR_OFS_Y_CTRL=0xff;
volatile unsigned char rUSR_OFS_Z_CTRL=0xff;


volatile unsigned char rDIGITAL_FILTER_CTRL=0xff;
volatile unsigned char rINT_FUNC_CTRL1=0xff;
volatile unsigned char rINT_FUNC_CTRL2=0xff;
volatile unsigned char rFIFO_CTRL=0xff;
volatile unsigned char rFIFO_DATA_STATUS=0xff;
volatile unsigned char rINT_FUNC_CTRL1_STATUS=0xff;
volatile unsigned char rMOTION_CTRL=0xff;
volatile unsigned char rMOTION_STATUS=0xff;
volatile unsigned char rMOTION_HIGH_TH=0xff;
volatile unsigned char rMOTION_HIGH_DUR=0xff;
volatile unsigned char rMOTION_LOW_TH=0xff;
volatile unsigned char rMOTION_LOW_DUR=0xff;

volatile unsigned char rINT1_CNT_DATA=0xff;


volatile uint8_t nCurrent_ReadSensor=1;
volatile uint8_t nCurrent_WriteSensor=1;

int twi_master_setup(void)
{
	twi_options_t opt;
	int status;
	
	// twi options settings
	opt.pba_hz = sysclk_get_pba_hz();
	opt.speed = TWI_SPEED;
	opt.chip = 0;

	sysclk_enable_pba_module(SYSCLK_TWI);

	// initialize TWI driver with options
	status = twi_master_init(&AVR32_TWI, &opt);
	
	return(status);
}

void twi_bus_put(uint8_t dev_addr, uint8_t addr, uint8_t data)
{
	int status;
	
	status = twi_bus_write(dev_addr, addr, &data, sizeof(uint8_t));
}

size_t twi_bus_write(uint8_t dev_addr, uint8_t addr, const void *data, size_t count)
{
	int status;
	
	twi_package_t const pkg = {
		.chip        = dev_addr,
		.addr        = {addr},
		.addr_length = sizeof(addr),
		.buffer      = (void *)data,
		.length      = count,
	};

	status = twi_master_write(&AVR32_TWI, &pkg);
	return (STATUS_OK == status) ? count : 0;
}

size_t twi_bus_read(uint8_t dev_addr, uint8_t addr, void *data, size_t count)
{
	int status;
	
	twi_package_t const pkg = {
		.chip        = dev_addr,
		.addr        = {addr},
		.addr_length = sizeof(addr),
		.buffer      = data,
		.length      = count,
	};

	status = twi_master_read(&AVR32_TWI, &pkg);
	return (STATUS_OK == status) ? count : 0;
}

void sga100_init(void)
{
	/* Initialize the control registers. */
	//twi_bus_put(SGA100_BUS_ADDR, SGA100_CTRL1, 0);
	//twi_bus_put(SGA100_BUS_ADDR, SGA100_CTRL2, 0);
	//twi_bus_put(SGA100_BUS_ADDR, SGA100_CTRL3, 0);
	
	twi_bus_put(SGA100_BUS_ADDR, SGA100_CTRL1, 0x62);
}

/**
 * \brief TC interrupt.
 *
 * The ISR handles RC compare interrupt and sets the update_timer flag to
 * update the timer value.
 */
#if defined (__GNUC__)
__attribute__((__interrupt__))
#elif defined (__ICCAVR32__)
#pragma handler = EXAMPLE_TC_IRQ_GROUP, 1
__interrupt
#endif
static void tc_irq(void)
{
	// specify that an interrupt has been raised
	if(tc_tick>=TC_TICK_20MS)
	{
		update_timer = true;
	}
	else
	{
		// Increment the ms seconds counter
		tc_tick++;		
	}
	
	if(tc_tick_1>=TC_TICK_80MS)
	{
		update_timer_1 = true;
	}
	else
	{
		// Increment the ms seconds counter
		tc_tick_1++;
	}


	if(tc_tick_2>=TC_TICK_1S)
	{
		update_timer_2 = true;
	}
	else
	{
		// Increment the ms seconds counter
		tc_tick_2++;
	}
		
	// Clear the interrupt flag. This is a side effect of reading the TC SR.
	tc_read_sr(EXAMPLE_TC, EXAMPLE_TC_CHANNEL);
}

/**
 * \brief TC Initialization
 *
 * Initializes and start the TC module with the following:
 * - Counter in Up mode with automatic reset on RC compare match.
 * - fPBA/8 is used as clock source for TC
 * - Enables RC compare match interrupt
 * \param tc Base address of the TC module
 */
void tc_init(volatile avr32_tc_t *tc)
{
	// Options for waveform generation.
	static const tc_waveform_opt_t waveform_opt = {
		// Channel selection.
		.channel  = EXAMPLE_TC_CHANNEL,
		// Software trigger effect on TIOB.
		.bswtrg   = TC_EVT_EFFECT_NOOP,
		// External event effect on TIOB.
		.beevt    = TC_EVT_EFFECT_NOOP,
		// RC compare effect on TIOB.
		.bcpc     = TC_EVT_EFFECT_NOOP,
		// RB compare effect on TIOB.
		.bcpb     = TC_EVT_EFFECT_NOOP,
		// Software trigger effect on TIOA.
		.aswtrg   = TC_EVT_EFFECT_NOOP,
		// External event effect on TIOA.
		.aeevt    = TC_EVT_EFFECT_NOOP,
		// RC compare effect on TIOA.
		.acpc     = TC_EVT_EFFECT_NOOP,
		/*
		 * RA compare effect on TIOA.
		 * (other possibilities are none, set and clear).
		 */
		.acpa     = TC_EVT_EFFECT_NOOP,
		/*
		 * Waveform selection: Up mode with automatic trigger(reset)
		 * on RC compare.
		 */
		.wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,
		// External event trigger enable.
		.enetrg   = false,
		// External event selection.
		.eevt     = 0,
		// External event edge selection.
		.eevtedg  = TC_SEL_NO_EDGE,
		// Counter disable when RC compare.
		.cpcdis   = false,
		// Counter clock stopped with RC compare.
		.cpcstop  = false,
		// Burst signal selection.
		.burst    = false,
		// Clock inversion.
		.clki     = false,
		// Internal source clock 3, connected to fPBA / 8.
		.tcclks   = TC_CLOCK_SOURCE_TC3
	};

	// Options for enabling TC interrupts
	static const tc_interrupt_t tc_interrupt = {
		.etrgs = 0,
		.ldrbs = 0,
		.ldras = 0,
		.cpcs  = 1, // Enable interrupt on RC compare alone
		.cpbs  = 0,
		.cpas  = 0,
		.lovrs = 0,
		.covfs = 0
	};
	// Initialize the timer/counter.
	tc_init_waveform(tc, &waveform_opt);

	/*
	 * Set the compare triggers.
	 * We configure it to count every 1 milliseconds.
	 * We want: (1 / (fPBA / 8)) * RC = 1 ms, hence RC = (fPBA / 8) / 1000
	 * to get an interrupt every 10 ms.
	 */
	tc_write_rc(tc, EXAMPLE_TC_CHANNEL, (sysclk_get_pba_hz() / 8 / 1000));
	// configure the timer interrupt
	tc_configure_interrupts(tc, EXAMPLE_TC_CHANNEL, &tc_interrupt);
	// Start the timer/counter.
	tc_start(tc, EXAMPLE_TC_CHANNEL);
}


int read_sensor_data(short *dx,short *dy, short *dz, short *dt)
{
	uint16_t temp_data;
	int status;

	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_DEV_ID, &rINDEX, 1);//_delay_ms(5);
	
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_STATUS, &rSTATUS, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_CTRL_REG1, &rCTRLREG1, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_CTRL_REG2, &rCTRLREG2, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_CTRL_REG3, &rCTRLREG3, 1);//_delay_ms(5);

	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_INT1_CTRL, &rINT1_CTRL, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_INT1_MAP_FUNC, &rINT1_MAP_FUNC, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_OUT_DATA_XL, &rOUT_DATA_XL, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_OUT_DATA_XH, &rOUT_DATA_XH, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_OUT_DATA_YL, &rOUT_DATA_YL, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_OUT_DATA_YH, &rOUT_DATA_YH, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_OUT_DATA_ZL, &rOUT_DATA_ZL, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_OUT_DATA_ZH, &rOUT_DATA_ZH, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_OUT_DATA_TEMP, &rOUT_DATA_TEMP, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_USR_GAIN_X_CTRL, &rUSR_GAIN_X_CTRL, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_USR_GAIN_Y_CTRL, &rUSR_GAIN_Y_CTRL, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_USR_GAIN_Z_CTRL, &rUSR_GAIN_Z_CTRL, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_USR_OFS_X_CTRL, &rUSR_OFS_X_CTRL, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_USR_OFS_Y_CTRL, &rUSR_OFS_Y_CTRL, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_USR_OFS_Z_CTRL, &rUSR_OFS_Z_CTRL, 1);//_delay_ms(5);
	

	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_DIGITAL_FILTER_CTRL, &rDIGITAL_FILTER_CTRL, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_INT_FUNC_CTRL1, &rINT_FUNC_CTRL1, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_INT_FUNC_CTRL2, &rINT_FUNC_CTRL2, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_FIFO_CTRL, &rFIFO_CTRL, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_FIFO_DATA_STATUS, &rFIFO_DATA_STATUS, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_INT_FUNC_CTRL1_STATUS, &rINT_FUNC_CTRL1_STATUS, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_MOTION_CTRL, &rMOTION_CTRL, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_MOTION_STATUS, &rMOTION_STATUS, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_MOTION_HIGH_TH, &rMOTION_HIGH_TH, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_MOTION_HIGH_DUR, &rMOTION_HIGH_DUR, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_MOTION_LOW_TH, &rMOTION_LOW_TH, 1);//_delay_ms(5);
	status=twi_bus_read(SGA100_BUS_ADDR, SGA100_MOTION_LOW_DUR, &rMOTION_LOW_DUR, 1);//_delay_ms(5);

	temp_data = 0;
	temp_data = rOUT_DATA_XH<<8;
	temp_data |= rOUT_DATA_XL;
	*dx = temp_data;
	
	temp_data = 0;
	temp_data = rOUT_DATA_YH<<8;
	temp_data |= rOUT_DATA_YL;
	*dy = temp_data;
	
	temp_data = 0;
	temp_data = rOUT_DATA_ZH<<8;
	temp_data |= rOUT_DATA_ZL;
	*dy = temp_data;
	
	return 0;
}


/*! \brief Main function:
 *  - Configure the CPU to run at 12MHz
 *  - Configure the USART
 *  - Register the TC interrupt (GCC only)
 *  - Configure, enable the CPCS (RC compare match) interrupt, and start a
 *    TC channel in waveform mode
 *  - In an infinite loop, update the USART message every second.
 */

/*! \brief Main function. Execution starts here.
 */
int main(void)
{
	volatile avr32_tc_t *tc = EXAMPLE_TC;
	char szBuffer[256]={0};
	char nOutIndex[5];
	int status;
	int nMsgSize;
	char sga100[128]={0};
		
	int16_t sga100_z;
	int16_t sga100_x;
	int16_t sga100_y;
	
	static uint8_t user_cal_count=0;
	
	static int32_t sum_of_z=0;
	static int32_t sum_of_x=0;
	static int32_t sum_of_y=0;
	
	int diff_z;
	int diff_x;
	int diff_y;
	
	uint8_t cal_temp_x;
	uint8_t cal_temp_y;
	uint8_t cal_temp_z;
	
	uint8_t bit_cal_temp=0xfc;
	
	char abyData[6];


	irq_initialize_vectors();
	cpu_irq_enable();

	sysclk_init();
	board_init();
	
	// Enable the clock to the selected example Timer/counter peripheral module.
	sysclk_enable_peripheral_clock(EXAMPLE_TC);

	ui_init();
	ui_powerdown();

	// Start USB stack to authorize VBus monitoring
	udc_start();
	
	INTC_register_interrupt(&tc_irq, EXAMPLE_TC_IRQ, EXAMPLE_TC_IRQ_PRIORITY);

	// Initialize the timer module
	tc_init(tc);
		
	//initialize the twi(I2C)
	status = twi_master_setup();
	
	delay_ms(2000);
	
	while (true) {
		if(update_timer_2)
		{
			update_timer_2 =  false;
			tc_tick_2=0;
			LED_Toggle(LED3);
		}
			
		memset(szBuffer,0,sizeof(char)*256);
		
		if(main_b_cdc_enable) {
			if(udi_cdc_is_rx_ready()) {
				dbuf[count++] = udi_cdc_getc();
			}
		}
		
		if( count > 5 )
		{
			if ( ( dbuf[0] == 0xaf ) && ( dbuf[5] == 0x5f ) )
			{
				prefix1=dbuf[1];
				addr1=dbuf[2];
				data1=dbuf[3];
				nSensorNum=dbuf[4];
						
				count=0;
						
				nCurrent_WriteSensor=nSensorNum;
				switch(prefix1){
					case NORMAL_REGISTER_TYPE_WRITE:
						twi_bus_put(SGA100_BUS_ADDR, addr1, data1);
						break;
					default:
						break;
				}
						
			}
			else
			{
				count =0;
			}
		}
		
		if (update_timer)
		{
#ifdef SGA100_USED	
			// Sensor Data Acquisition
			read_sensor_data(&outx,&outy,&outz,&outt);
#endif
			update_timer=false;
			tc_tick=0;
			
			/* magnetometer data read */
			if(update_timer_1)
			{
				update_timer_1=false;
				tc_tick_1=0;
			}
			
			if(1)
			{
				sga100_x = rOUT_DATA_XH;
				sga100_x <<=8;
				sga100_x |= rOUT_DATA_XL;
							
				sga100_y = rOUT_DATA_YH;
				sga100_y <<=8;
				sga100_y |= rOUT_DATA_YL;

				sga100_z = rOUT_DATA_ZH;
				sga100_z <<=8;
				sga100_z |= rOUT_DATA_ZL;
				
				float fAccArray2[3] = {sga100_x, sga100_y, sga100_z};
				float* fAccArray;
				linearAcc(fAccArray2, fAccArray);

				abyData[0] = (int)fAccArray[X] & 0xFF;
				abyData[1] = ((int)fAccArray[X]>>8) & 0xFF;
				abyData[2] = (int)fAccArray[Y] & 0xFF;
				abyData[3] = ((int)fAccArray[Y]>>8) & 0xFF;
				abyData[4] = (int)fAccArray[Z] & 0xFF;
				abyData[5] = ((int)fAccArray[Z]>>8) & 0xFF;
			}
			
			if(main_b_cdc_enable) 
			//if(0)
			{
				
				udi_cdc_putc('P');
				udi_cdc_putc('K');
				udi_cdc_putc('S');
				udi_cdc_putc('T');
				udi_cdc_putc(33);
			
				udi_cdc_putc(nCurrent_ReadSensor);
			
				udi_cdc_putc(rINDEX); //1
						
				udi_cdc_putc(rSTATUS);
				udi_cdc_putc(rCTRLREG1);
				udi_cdc_putc(rCTRLREG2);
				udi_cdc_putc(rCTRLREG3); //5

				udi_cdc_putc(rINT1_CTRL);
				udi_cdc_putc(rINT1_MAP_FUNC);	
				/*udi_cdc_putc(rOUT_DATA_XL);
				udi_cdc_putc(rOUT_DATA_XH);
				udi_cdc_putc(rOUT_DATA_YL); //10
				udi_cdc_putc(rOUT_DATA_YH);
				udi_cdc_putc(rOUT_DATA_ZL);
				udi_cdc_putc(rOUT_DATA_ZH);*/
					
				udi_cdc_putc(abyData[0]);
				udi_cdc_putc(abyData[1]);
				udi_cdc_putc(abyData[2]); //10
				udi_cdc_putc(abyData[3]);
				udi_cdc_putc(abyData[4]);
				udi_cdc_putc(abyData[5]);
				
				udi_cdc_putc(rOUT_DATA_TEMP);
				udi_cdc_putc(rUSR_GAIN_X_CTRL); //15
				udi_cdc_putc(rUSR_GAIN_Y_CTRL);
				udi_cdc_putc(rUSR_GAIN_Z_CTRL);
				udi_cdc_putc(rUSR_OFS_X_CTRL);
				udi_cdc_putc(rUSR_OFS_Y_CTRL);
				udi_cdc_putc(rUSR_OFS_Z_CTRL); //20

				udi_cdc_putc(rDIGITAL_FILTER_CTRL);
				udi_cdc_putc(rINT_FUNC_CTRL1);
				udi_cdc_putc(rINT_FUNC_CTRL2);
				udi_cdc_putc(rFIFO_CTRL);
				udi_cdc_putc(rFIFO_DATA_STATUS); //25
				udi_cdc_putc(rINT_FUNC_CTRL1_STATUS);
				udi_cdc_putc(rMOTION_CTRL);
				udi_cdc_putc(rMOTION_STATUS);
				udi_cdc_putc(rMOTION_HIGH_TH);
				udi_cdc_putc(rMOTION_HIGH_DUR);	//30
				udi_cdc_putc(rMOTION_LOW_TH);
				udi_cdc_putc(rMOTION_LOW_DUR); //32
				
				if(gpio_pin_is_low(AVR32_PIN_PA05)) 
				{
					udi_cdc_putc(0);
				}
				else 
				{
					udi_cdc_putc(1);
				}
				udi_cdc_putc('\r');
				udi_cdc_putc('\n');
			}
		}
	}
}

void main_suspend_action(void)
{
	ui_powerdown();
}

void main_resume_action(void)
{
	ui_wakeup();
}

void main_sof_action(void)
{
	if (!main_b_cdc_enable)
		return;
	ui_process(udd_get_frame_number());
}

bool main_cdc_enable(uint8_t port)
{
	main_b_cdc_enable = true;
	return true;
}

void main_cdc_disable(uint8_t port)
{
	main_b_cdc_enable = false;
}

void main_cdc_set_dtr(uint8_t port, bool b_enable)
{
	if (b_enable) {
		// Host terminal has open COM
		ui_com_open(port);
	}else{
		// Host terminal has close COMF
		ui_com_close(port);
	}
}

uint8_t hextochar_invert_h(uint8_t data)
{
	uint8_t temp;
	
	temp = data & 0xF0;
	
	temp>>=4;
	
	if(temp<0x0a)
	{
		temp += 0x30;
	}
	else
	{
		temp += 0x37;
	}
	
	return temp;
}

uint8_t hextochar_invert_l(uint8_t data)
{
	uint8_t temp;
	
	temp = data & 0x0f;
	
	if(temp<0x0a)
	{
		temp += 0x30;
	}
	else
	{
		temp += 0x37;
	}
	
	return temp;
}
/**
 * \mainpage ASF USB Device CDC
 *
 * \section intro Introduction
 * This example shows how to implement a USB Device CDC
 * on Atmel MCU with USB module.
 * The application note AVR4907 provides more information
 * about this implementation.
 *
 * \section desc Description of the Communication Device Class (CDC)
 * The Communication Device Class (CDC) is a general-purpose way to enable all
 * types of communications on the Universal Serial Bus (USB).
 * This class makes it possible to connect communication devices such as
 * digital telephones or analog modems, as well as networking devices
 * like ADSL or Cable modems.
 * While a CDC device enables the implementation of quite complex devices,
 * it can also be used as a very simple method for communication on the USB.
 * For example, a CDC device can appear as a virtual COM port, which greatly
 * simplifies application development on the host side.
 *
 * \section startup Startup
 * The example is a bridge between a USART from the main MCU
 * and the USB CDC interface.
 *
 * In this example, we will use a PC as a USB host:
 * it connects to the USB and to the USART board connector.
 * - Connect the USART peripheral to the USART interface of the board.
 * - Connect the application to a USB host (e.g. a PC)
 *   with a mini-B (embedded side) to A (PC host side) cable.
 * The application will behave as a virtual COM (see Windows Device Manager).
 * - Open a HyperTerminal on both COM ports (RS232 and Virtual COM)
 * - Select the same configuration for both COM ports up to 115200 baud.
 * - Type a character in one HyperTerminal and it will echo in the other.
 *
 * \note
 * On the first connection of the board on the PC,
 * the operating system will detect a new peripheral:
 * - This will open a new hardware installation window.
 * - Choose "No, not this time" to connect to Windows Update for this installation
 * - click "Next"
 * - When requested by Windows for a driver INF file, select the
 *   atmel_devices_cdc.inf file in the directory indicated in the Atmel Studio
 *   "Solution Explorer" window.
 * - click "Next"
 *
 * \copydoc UI
 *
 * \section example About example
 *
 * The example uses the following module groups:
 * - Basic modules:
 *   Startup, board, clock, interrupt, power management
 * - USB Device stack and CDC modules:
 *   <br>services/usb/
 *   <br>services/usb/udc/
 *   <br>services/usb/class/cdc/
 * - Specific implementation:
 *    - main.c,
 *      <br>initializes clock
 *      <br>initializes interrupt
 *      <br>manages UI
 *      <br>
 *    - uart_xmega.c,
 *      <br>implementation of RS232 bridge for XMEGA parts
 *    - uart_uc3.c,
 *      <br>implementation of RS232 bridge for UC3 parts
 *    - uart_sam.c,
 *      <br>implementation of RS232 bridge for SAM parts
 *    - specific implementation for each target "./examples/product_board/":
 *       - conf_foo.h   configuration of each module
 *       - ui.c        implement of user's interface (leds,buttons...)
 */
