//////////////////////////////////////////////////////////////////////////////
// File Name	: 090409_SMA3920.c
// Title		: SMA3920D Evaluation Kit Firmware
// Revision		: 0.1
// Notes		: 
// SPI			: 250 Khz (CLOCK/64)
// ODR 			: 200 Hz
// RS-232		: 250K BPS
// Target MCU	: Atmel AVR series -> AT90CAN128 F_CPU 16Mhz
// Editor Tabs	: 4
// 
// Revision History:
// When			Who			Description of change
// -----------	-----------	-----------------------
//  2009-04-09	TD			Created the program
//
//////////////////////////////////////////////////////////////////////////////

#ifndef nop
   #define nop()  __asm__ __volatile__ ("nop" ::)
#endif

//************************************************//
// Define                                         //  
//************************************************//
#ifndef F_CPU
	#define F_CPU 16000000
	#define CYCLES_PER_US ((F_CPU+500000)/1000000)    // cpu cycles per microsecond
#endif

#define SPI_DELAY_COUNT		10

//************************************************//
// Include Files                                  //
//************************************************//
#include <avr/io.h>			// include I/O definitions (port names, pin names, etc)
#include <avr/interrupt.h>	// include interrupt support

#include <stdio.h>
#include <util/delay.h>

#include "avrlibdefs.h"		// global AVRLIB defines

//************************************************//
// Global variabels                               //
//************************************************//


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

typedef enum _READ_OTP_REGISTER_TYPE{
  READ_OTP_REGISTER_PRE=0,
  READ_OTP_REGISTER_POST,
}READ_OTP_REGISTER_TYPE;


// UART Data Packet Variables
//volatile unsigned char 	dbuf[8];
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

volatile unsigned char 	wrdata=0;


volatile unsigned char 	raddr=0;
volatile unsigned char 	rdata1=0;
volatile unsigned char 	rdata2=0;

volatile unsigned char 	action_flag=0;
volatile unsigned char 	read_flag=0;
volatile unsigned char  ADC_Conversion_complete=0;
volatile unsigned char	uart_read_flag=0;

//Only 
volatile unsigned char rXSTAGE_GAIN_FACTORY_ONLY=0xff;
volatile unsigned char rYSTAGE_GAIN_FACTORY_ONLY=0xff;
volatile unsigned char rZSTAGE_GAIN_FACTORY_ONLY=0xff;


//* BP Register variable declaration start */

// BP Register variable
volatile unsigned char rINDEX=0xff; 
volatile unsigned char rSTATUS=0xff;

volatile unsigned char rCTRLREG1=0xff; 
volatile unsigned char rCTRLREG2=0xff; 
volatile unsigned char rCTRLREG3=0xff; 

volatile unsigned char rOTP_CTRL=0xff;
volatile unsigned char rMANUAL_BN_SELECT_CTRL=0xff;
volatile unsigned char rOTP_TEST_CTRL=0xff;
volatile unsigned char rOTP_TEST_ADDRESS_CTRL=0xff;
volatile unsigned char rOTP_TEST_DATA_CTRL=0xff;
volatile unsigned char rOTP_Bank_Status=0xff;
volatile unsigned char rOTP_PGM_CYCLE=0xff;

volatile unsigned char rTMODE_CTRL=0xff;
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
volatile unsigned char rTEST_CTRL=0xff;


volatile unsigned char rDIGITAL_FILTER_CTRL=0xff;
volatile unsigned char rINT_FUNC_CTRL1=0xff;
volatile unsigned char rINT_FUNC_CTRL2=0xff;
volatile unsigned char rINT_FUNC_CTRL3=0xff;
volatile unsigned char rFIFO_CTRL=0xff;
volatile unsigned char rFIFO_DATA_STATUS=0xff;
volatile unsigned char rINT_FUNC_CTRL1_STATUS=0xff;
volatile unsigned char rMOTION_CTRL=0xff;
volatile unsigned char rMOTION_STATUS=0xff;
volatile unsigned char rMOTION_HIGH_TH=0xff;
volatile unsigned char rMOTION_HIGH_DUR=0xff;
volatile unsigned char rMOTION_LOW_TH=0xff;
volatile unsigned char rMOTION_LOW_DUR=0xff;
volatile unsigned char rTAP_INT_STATUS=0xff;
volatile unsigned char rTAP_TH=0xff;
volatile unsigned char rTAP_DUR=0xff;
volatile unsigned char rDTAP_TIMEWINDOW_DUR=0xff;
volatile unsigned char rTAP_LATENCY_DUR=0xff;



volatile unsigned char rAFE_WT_MODE=0xff;
volatile unsigned char rCMI=0xff;
volatile unsigned char rTEST_DREGISTER=0xff;
volatile unsigned char rTEST_DREGISTER_STATUS=0xff;
volatile unsigned char rCP_OT_PX=0xff;
volatile unsigned char rCP_OT_NX=0xff;
volatile unsigned char rCP_OT_PY=0xff;
volatile unsigned char rCP_OT_NY=0xff;
volatile unsigned char rCP_OT_PZ=0xff;
volatile unsigned char rCP_OT_NZ=0xff;
volatile unsigned char rPGA_GT_XY=0xff;
volatile unsigned char rPGA_GT_Z=0xff;
volatile unsigned char rTOSC3_VBGR=0xff;
volatile unsigned char rDVP=0xff;
volatile unsigned char rTSEN_DC=0xff;
volatile unsigned char rTOSC1=0xff;
volatile unsigned char rTOSC2=0xff;
volatile unsigned char rDFACT_GT_X=0xff;
volatile unsigned char rDFACT_GT_Y=0xff;
volatile unsigned char rDFACT_GT_Z=0xff;
volatile unsigned char rDFACT_OFS_XYZ=0xff;
volatile unsigned char rDFACT_OT_X=0xff;
volatile unsigned char rDFACT_OT_Y=0xff;
volatile unsigned char rDFACT_OT_Z=0xff;
volatile unsigned char rDSIGN_CONTROL=0xff;
volatile unsigned char rOTP_PM=0xff;

volatile unsigned char rOFFSET_DATA_MSB=0xff;
volatile unsigned char rOFFSET_DATA_LSB=0xff;


volatile uint8_t nCurrent_ReadSensor=1;
volatile uint8_t nCurrent_WriteSensor=1;
//OTP And Register  Trimming


/*
//Define Register Address
//register address
#define BP_REG_CONTROL_REG1_ADDR 0x02  //trimming시 설정필요
#define BP_REG_CONTROL_REG2_ADDR 0x03  //trimming시 설정필요

#define BP_REG_CONTROL_REG3_ADDR 0x04  //default  Register 설정사용할듯.
#define BP_REG_CONTROL_REG4_ADDR 0x05  //trimming시 설정필요
#define BP_REG_CONTROL_ADC_ADDR 0x06  //default register 설정 사용할듯

#define BP_REG_CONTROL_OTP_CONTROL_ADDR 0x07  //저장시 필요.
#define BP_REG_CONTROL_OTP_BANK_ADDR 0x08  //저장시 필요.
#define BP_REG_CONTROL_OTP_DEST_ADDR 0x09  //저장시 필요.
#define BP_REG_CONTROL_OTP_DATA_ADDR 0x0A  //저장시 필요.

#define BP_REG_CONTROL_TESTD_CON_ADDR  0x0B  //Default사용할듯.
#define BP_REG_CONTROL_TEST_PATTERN_ADDR  0x0C  //Default사용할듯.
#define BP_REG_CONTROL_TESTA_CON_ADDR  0x0D  //Default사용할듯.

#define BP_REG_ACCE_GAIN_X_USER_ADDR  0x40  //저장이 안되므로 Gain Trimming시 미세조절의 User설정값  default 기준으로 하면 될듯  0x80이므로 1배
#define BP_REG_ACCE_GAIN_Y_USER_ADDR  0x41  //저장이 안되므로 Gain Trimming시 미세조절의 User설정값  default 기준으로 하면 될듯  0x80이므로 1배
#define BP_REG_ACCE_GAIN_Z_USER_ADDR  0x42  //저장이 안되므로 Gain Trimming시 미세조절의 User설정값  default 기준으로 하면 될듯  0x80이므로 1배

#define BP_REG_ACCE_OFFSET_X_USER_ADDR  0x43  //USER OFFSET조절인데 저장이 안되므로 Offset Trimming시 default가 값 기준으로 하면 될듯  0x00이므로 Offset은 0
#define BP_REG_ACCE_OFFSET_Y_USER_ADDR  0x44  //USER OFFSET조절인데 저장이 안되므로 Offset Trimming시 default가 값 기준으로 하면 될듯  0x00이므로 Offset은 0
#define BP_REG_ACCE_OFFSET_Z_USER_ADDR 0x45  //USER OFFSET조절인데 저장이 안되므로 Offset Trimming시 default가 값 기준으로 하면 될듯  0x00이므로 Offset은 0


#define BP_REG_ACCE_FILTER_CON1_ADDR  0x46   //Digital filter  default bypass라 손댈일 없음
#define BP_REG_ACCE_FILTER_CON2_ADDR  0x47   //Digital filter  default bypass라 손댈일 없음

#define BP_REG_FIFO_CON1_ADDR  0x49   //default로 사용.
#define BP_REG_FIFO_CON2_ADDR  0x4A   //


//OTP Write Address  &&   normal address???? 
#define BP_REG_OTP_BANK_CHANGE_ADDR    0x00   // BANK CHANGE
#define BP_REG_OTP_X_PNode_ADDR    0x01   // //X up
#define BP_REG_OTP_X_NNode_ADDR    0x02   // //X Dn

#define BP_REG_OTP_Y_PNode_ADDR    0x03   // //Y up
#define BP_REG_OTP_Y_NNode_ADDR    0x04   // //Y Dn

#define BP_REG_OTP_Z_PNode_ADDR    0x05   // //Z up
#define BP_REG_OTP_Z_NNode_ADDR    0x06   // //Z Dn


#define BP_REG_OTP_GAINXY_ADDR    0x07   // //2nd Gain Stage X,Y
#define BP_REG_OTP_GAINZLP_ADDR   0x08   // //2nd Gain Z, LP

#define BP_REG_OTP_OSC_ADDR    0x15   // OSC P,N
#define BP_REG_OTP_CBGR_ADDR   0x16   // CBGR
#define BP_REG_OTP_CTEMP_ADDR  0x17   // CTEMP

#define BP_REG_OTP_FACT_GAIN_TRIM_X_ADDR    0x23   //Factory Gain Trimming
#define BP_REG_OTP_FACT_GAIN_TRIM_Y_ADDR    0x24   //Factory Gain Trimming
#define BP_REG_OTP_FACT_GAIN_TRIM_Z_ADDR    0x25   //Factory Gain Trimming

#define BP_REG_OTP_FACT_OFFSET_TRIM_X_ADDR    0x29   //Factory Offset Trimming
#define BP_REG_OTP_FACT_OFFSET_TRIM_Y_ADDR    0x2A   //Factory Offset Trimming
#define BP_REG_OTP_FACT_OFFSET_TRIM_Z_ADDR    0x2B   //Factory Offset Trimming

#define BP_REG_OTP_FACT_OFFSET_TRIM_TEMP_ADDR    0x2C   //Factory Offset Trimming for TemPerature
#define BP_REG_OTP_FACT_OFFSET_TRIM_ADC_H_ADDR    0x2D   //Factory Offset Trimming for ADC
#define BP_REG_OTP_FACT_OFFSET_TRIM_ADC_L_ADDR    0x2E   //Factory Offset Trimming for ADC

#define BP_REG_OTP_SENSOR_WAKEUP_ADDR    0x2F   //Sensor Wakeup Time
//OTP Write Address  &&   normal address???? 

//OTP Write Command Address
#define BP_REG_ROM_SAVE    0x90   //OTP Write Command 
#define BP_PREREGISTER_WRITE_CMD 0x91 //PREREGISTER
*/

/* SGA100 define register */
#define SGA100_DEV_ID						0x01	/* R */
#define SGA100_STATUS						0x02    /* R */
#define SGA100_CTRL_REG1					0x03	/* RW default 0x61 */
#define SGA100_CTRL_REG2					0x04	/* RW default 0xFC */
#define SGA100_CTRL_REG3					0x05	/* RW default 0x1A */

#define SGA100_OTP_CTRL						0x06	/* RW default 0x04 */
#define SGA100_MANUAL_BN_SELECT_CTRL		0x07	/* RW default 0x00 */
#define SGA100_OTP_TEST_CTRL				0x08	/* RW default 0x00 */
#define SGA100_OTP_TEST_ADDRESS_CTRL		0x09	/* RW default 0x00 */
#define SGA100_OTP_TEST_DATA_CTRL			0x0A	/* RW default 0x00 */
#define SGA100_OTP_Bank_Status				0x0B	/* R */
#define SGA100_OTP_PGM_CYCLE				0x0C	/* RW default 0xA0 */

#define SGA100_TMODE_CTRL					0x0F	/* RW default 0x00 */
#define SGA100_INT1_CTRL					0x10	/* RW default 0x1A */
#define SGA100_INT1_MAP_FUNC				0x11	/* RW default 0x00 */
#define SGA100_OUT_DATA_XH					0x12	/* R */
#define SGA100_OUT_DATA_XL					0x13	/* R */
#define SGA100_OUT_DATA_YH					0x14	/* R */
#define SGA100_OUT_DATA_YL					0x15	/* R */
#define SGA100_OUT_DATA_ZH					0x16	/* R */
#define SGA100_OUT_DATA_ZL					0x17	/* R */
#define SGA100_OUT_DATA_TEMP				0x18	/* R */
#define SGA100_USR_GAIN_X_CTRL				0x19	/* RW default 0x80 */
#define SGA100_USR_GAIN_Y_CTRL				0x1A	/* RW default 0x80 */
#define SGA100_USR_GAIN_Z_CTRL				0x1B	/* RW default 0x80 */
#define SGA100_USR_OFS_X_CTRL				0x1C	/* RW default 0x00 */
#define SGA100_USR_OFS_Y_CTRL				0x1D	/* RW default 0x00 */
#define SGA100_USR_OFS_Z_CTRL				0x1E	/* RW default 0x00 */
#define SGA100_TEST_CTRL					0x1F	/* RW default 0x00(Reserved) */


#define SGA100_DIGITAL_FILTER_CTRL			0x33	/* RW default 0x00 */
#define SGA100_INT_FUNC_CTRL1				0x34	/* RW default 0x00 */
#define SGA100_INT_FUNC_CTRL2				0x35	/* RW default 0x80 */
#define SGA100_INT_FUNC_CTRL3				0x36	/* RW default 0x00 */
#define SGA100_FIFO_CTRL					0x37	/* RW default 0x00 */
#define SGA100_FIFO_DATA_STATUS				0x38	/* R */
#define SGA100_INT_FUNC_CTRL1_STATUS		0x39	/* R */
#define SGA100_MOTION_CTRL					0x3A	/* RW default 0x00 */
#define SGA100_MOTION_STATUS				0x3B	/* R */
#define SGA100_MOTION_HIGH_TH				0x3C	/* RW default 0xBE */
#define SGA100_MOTION_HIGH_DUR				0x3D	/* RW default 0x02 */
#define SGA100_MOTION_LOW_TH				0x3E	/* RW default 0x2D */
#define SGA100_MOTION_LOW_DUR				0x3F	/* RW default 0x02 */
#define SGA100_TAP_INT_STATUS				0x40	/* R */
#define SGA100_TAP_TH						0x41	/* RW default 0x40 */
#define SGA100_TAP_DUR						0x42	/* RW default 0x07 */
#define SGA100_DTAP_TIMEWINDOW_DUR			0x43	/* RW default 0x28 */
#define SGA100_TAP_LATENCY_DUR				0x44	/* RW default 0x05 */



#define SGA100_AFE_WT_MODE					0x4C	/* RW default 0x00 */
#define SGA100_CMI							0x4D	/* RW default 0x04 */
#define SGA100_TEST_DREGISTER				0x4E	/* RW default 0x00 */
#define SGA100_TEST_DREGISTER_STATUS		0x4F	/* R */
#define SGA100_CP_OT_PX						0x50	/* RW default 0x00 */
#define SGA100_CP_OT_NX						0x51	/* RW default 0x00 */
#define SGA100_CP_OT_PY						0x52	/* RW default 0x00 */
#define SGA100_CP_OT_NY						0x53	/* RW default 0x00 */
#define SGA100_CP_OT_PZ						0x54	/* RW default 0x00 */
#define SGA100_CP_OT_NZ						0x55	/* RW default 0x00 */
#define SGA100_PGA_GT_XY					0x56	/* RW default 0x66 */
#define SGA100_PGA_GT_Z						0x57	/* RW default 0x60 */
#define SGA100_TOSC3_VBGR					0x58	/* RW default 0x78 */
#define SGA100_DVP							0x59	/* RW default 0x10 */
#define SGA100_TSEN_DC						0x5A	/* RW default 0x02 */
#define SGA100_TOSC1						0x5B	/* RW default 0x3F */
#define SGA100_TOSC2						0x5C	/* RW default 0x1F */
#define SGA100_DFACT_GT_X					0x5D	/* RW default 0x40 */
#define SGA100_DFACT_GT_Y					0x5E	/* RW default 0x40 */
#define SGA100_DFACT_GT_Z					0x5F	/* RW default 0x40 */
#define SGA100_DFACT_OFS_XYZ				0x60	/* RW default 0x07 */
#define SGA100_DFACT_OT_X					0x61	/* RW default 0x00 */
#define SGA100_DFACT_OT_Y					0x62	/* RW default 0x00 */
#define SGA100_DFACT_OT_Z					0x63	/* RW default 0x00 */
#define SGA100_DSIGN_CONTROL				0x64	/* RW default 0x00 */
#define SGA100_OTP_PM						0x65	/* RW default 0x01 */

#define SGA100_OFFSET_DATA_MSB				0x70	/* R */
#define SGA100_OFFSET_DATA_LSB				0x71	/* R */
//not used unsigned char bPreRegisterWrite=0;
		
/* BP Register variable declaration end */



//*************************************************//
// I2C                           //   
//*************************************************//

//#define USE_I2C
//#define USE_SPI


#define USING_LGA12TYPE

//#define USING_LGA16TYPE

//******************* Chip Selection **********/
#ifdef USING_LGA16TYPE
	#define CHIP_SEL PB4 //chip selection (port) 
#endif

#ifdef USING_LGA12TYPE
//	#define CHIP_SEL PB0 //chip selection (port)
	#define CHIP_SEL1 PB0 //chip selection (port)
	#define CHIP_SEL2 PB4 //chip selection (port)
	#define CHIP_SEL3 PB5 //chip selection (port)
	#define CHIP_SEL4 PB6 //chip selection (port)
#endif
//******************* Chip Selection **********/

#define oSCL         PORTB
#define oSDA         PORTB
#define iSDA         PINB
#define oSCL_H       (oSCL|=0x02)
#define oSCL_L       (oSCL&=0xfd)
#define oSDA_H       (oSDA|=0x04)
#define oSDA_L       (oSDA&=0xfb)
#define iSDA_H       ((iSDA & 0x04)!=0)
#define iSDA_L       ((iSDA & 0x04)==0)

#define SCL_OUT_INIT (DDRB|=0x02)
#define SDA_IN_INIT  (DDRB&=0xfb)
#define SDA_OUT_INIT (DDRB|=0x04)

//#define I2C_ID      0x40
#define I2C_ID      0xC8
//#define ADDR_0x100  0x02
#define ADDR_0x100  0x00
#define I2C_WR      0x00
#define I2C_RD      0x01


void I2C_Init();
void I2C_Delay();
void I2C_Start();
void I2C_End();
void I2C_ReStart();
unsigned char I2C_Ack();
unsigned char I2C_ReadAck();
void I2C_NoAck();
void I2C_ByteOut(unsigned char Data);
unsigned char   I2C_ByteIn();
void I2C_Write(unsigned char Addr, unsigned char Data);
unsigned char I2C_Read(unsigned char Addr);
//*************************************************//
// Function declaraion                             //   
//*************************************************//
void init_port(void);			// Port init
void init_timer(void);			// Timer init
void init_uart(void);			// UART init
void init_spi(void);			// SPI init
int init_BP(void);				// BP init
void Sensor_init(void);
void writeEEPROM(unsigned int uiAddress, unsigned char ucData);
unsigned char readEEPROM(unsigned int uiAddress);
int read_sensor_data(short *dx,short *dy, short *dz, short *dt); // read data read
int usart0_sendbyte(char data, FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM(usart0_sendbyte,NULL,_FDEV_SETUP_RW);
void wr_reg(unsigned char, unsigned char);
unsigned char rd_reg(unsigned char addr);
void spi_send_byte(unsigned char);
void _nops_();




void delay(unsigned int count)
{
	while(count--);
}

SIGNAL(SIG_OVERFLOW1)
{
	unsigned char sreg;
	sreg=SREG;

	cli();
	
	// 100HZ
	TCNT1=0xB1DF;

	// 50HZ
	//TCNT1=0x63BF;

	read_flag=1;

	sei();

	SREG=sreg;			
}

void nops(unsigned char num)
{
	unsigned char i;
	for(i=0 ; i<num ; i++)	{
		nop();
	}
}

void Delay_us(unsigned char time_us)
{
 	register unsigned char i;
 	for(i=0; i<time_us; i++){
	 	asm(" PUSH R0 ");
		asm(" POP R0 ");
		asm(" PUSH R0 ");
		asm(" POP R0 ");
		asm(" PUSH R0 ");
		asm(" POP R0 ");
	}
}

void Delay_ms(unsigned char time_ms)
{
 	register unsigned char i;
	for(i=0; i<time_ms; i++){
	 	Delay_us(250);
	 	Delay_us(250);
	 	Delay_us(250);
	 	Delay_us(250);
	}
}



/*******************************************************/
// Function Define                                      / 
/*******************************************************/

/*******************************************************/
// PORT
void init_port(void)
{
 	//MCUCR &= 0xEF;

	// Crystal Oscillator division factor: 1
	CLKPR=0x80;
	CLKPR=0x00;

	// Input/Output Ports initialization
	DDRA=0x00;	PORTA=0x00;			// Port A initialization
	DDRB=0x00;	PORTB=0x00;			// Port B initialization
	DDRC=0x00;	PORTC=0x00;			// Port C initialization
	DDRD=0x00;	PORTD=0x00;			// Port D initialization
	DDRE=0x00;	PORTE=0x00;			// Port E initialization
	DDRF=0x00;	PORTF=0x00;			// Port F initialization
	DDRG=0x00;	PORTG=0x00;			// Port G initialization
}
/*******************************************************/


/*******************************************************/
// timer
void init_timer(void)
{
	// Timer/Counter 0 initialization
	// Clock source: System Clock
	// Clock value: Timer 0 Stopped
	// Mode: Normal top=FFh
	// OC0 output: Disconnected
	TCCR0A=0x00;
	TCNT0=0x00;
	OCR0A=0x00;
	
	// Timer/Counter 1 initialization
	// Clock source: System Clock/8
	// Clock value: Timer 1 Stopped
	// Mode: Normal top=FFFFh
	// OC1A output: Discon.
	// OC1B output: Discon.
	// OC1C output: Discon.
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	TCCR1A=0x00;
	TCCR1B=0x02;
	TCNT1=0x63BF; 	//TCNT1H=0xEC;	100HZ 
					//TCNT1L=0x77;
	ICR1H=0x00;
	ICR1L=0x00;
	OCR1AH=0x00;
	OCR1AL=0x00;
	OCR1BH=0x00;
	OCR1BL=0x00;
	OCR1CH=0x00;
	OCR1CL=0x00;
	
	// Timer/Counter 2 initialization
	// Clock source: System Clock
	// Clock value: Timer 2 Stopped
	// Mode: Normal top=FFh
	// OC2 output: Disconnected
	ASSR=0x00;
	TCCR2A=0x00;
	TCNT2=0x00;
	OCR2A=0x00;
	
	// Timer/Counter 3 initialization
	// Clock source: System Clock
	// Clock value: Timer 3 Stopped
	// Mode: Normal top=FFFFh
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// OC3A output: Discon.
	// OC3B output: Discon.
	// OC3C output: Discon.
	TCCR3A=0x00;
	TCCR3B=0x02;
	TCNT3H=0x00;
	TCNT3L=0x00;
	ICR3H=0x00;
	ICR3L=0x00;
	OCR3AH=0x00;
	OCR3AL=0x00;
	OCR3BH=0x00;
	OCR3BL=0x00;
	OCR3CH=0x00;
	OCR3CL=0x00;
	
	// External Interrupt(s) initialization
	EICRA=0x00;
	EICRB=0x00;
	EIMSK=0x00;
	
	// Timer/Counter 0 Interrupt(s) initialization
	TIMSK0=0x00;
	// Timer/Counter 1 Interrupt(s) initialization
	TIMSK1=0x01;
	// Timer/Counter 2 Interrupt(s) initialization
	TIMSK2=0x00;
	// Timer/Counter 3 Interrupt(s) initialization
	TIMSK3=0x00;
}
/*******************************************************/

/*******************************************************/
// UART
void init_uart(void)
{
	// USART0 initialization
	// Communication Parameters: 8 Data, 1 Stop, No Parity
	// USART0 Receiver: On
	// USART0 Transmitter: On
	// USART0 Mode: Asynchronous
	// Baud Rate : 115200 bps
	UBRR0H = 0x00;
	//UBRR0L = 0x10;		// 0001 0000
	UBRR0L = 0x0F;		// 0001 0000	//<-lsh 0001 0000이 맞는 것 같은데..
	
	UCSR0A = 0x02;		// 0000 0010
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);				// 0x06			// 0000 0110
	UCSR0B = (1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0); // 0x98; 		// 1001 1000

	stdout=&mystdout;	// STDOUT and STDERR device open
}

int usart0_sendbyte(char data, FILE *stream)
{
	// Wait for empty transmit buffer 
	while( !(UCSR0A & 0x20) );
	// Put data into buffer, sends the data
	UDR0 = data;
	return 0;
}


void put_char0(unsigned char data) // transmit a character by USART0 
{
  while((UCSR0A & 0x20) == 0x00);  // data register empty ?
  UDR0 = data;
}


// dbuf[0] : Header
// dbuf[1] : addreas
// dbuf[2] : address
// dbuf[3] : data1
// dbuf[4] : data2
// dbuf[5] : trailer
SIGNAL(SIG_UART0_RECV)
{
	// clear interrupt enable
	unsigned char sreg;
	sreg=SREG;
	cli();
	//SREG=SERG&0x7F;

	// Get received char
	dbuf[count++] = UDR0;

	// set interrupt enable
	//SREG=SERG|0x80;
	SREG=sreg;
	//sei();
}

/*******************************************************/


/*******************************************************/
// SPI
void init_spi(void)
{
#ifdef USING_LGA16TYPE
   	DDRB=0x17;
	PORTB=0x1F;
#endif

#ifdef USING_LGA12TYPE
	//DDRB=0x07;		
	//PORTB=0x0F;
	DDRB=0x77;
	PORTB=0x7F;
#endif
	// SPCR Register //
	// | SPIE	| SPE	| DORD	| MSTR	| CPOL	| CPHA	| SPR1	| SPR0	|
	// SPI MODE: 0
	// SPI: 1.0 Mhz (CLOCK/16)
	// SET SCK low
    //SPCR = 0x5E; // 0x0101 1110 //Mode4 250khz(1/64) //bp300 original

	// SPSR Register //
	// | SPIF 	| WCOL	|  --	|  --	|  --	|  --	|  --	| SPI2X	|
	SPSR = 0x00; // 250KHZ

	//SPCR = 0x56; // 0x0101 0110
	//SPCR = 0x5A; // 0x0101 1010 //I3SA 파형. MODE3 -김경린 선임이 처음에 말한 파형.
	//SPCR = 0x52; // 0x0101 0010  

    //SPCR = 0x56;//정보가 제대로 나오지 않음. 통신이 안되는듯.
    //SPCR = 0x5A;//비트가 밀려서 제대로 된 정보가 나오지 않음.

    //SPCR = 0x52; ->5e와 같은 정보 나옴.
	SPCR = 0x5E; // 0x0101 1010 ->추후에 파형 스샷으로 온 파형. 
}

void spi_send_byte(unsigned char data)
{
    // send a byte over SPI and ignore reply
	SPDR = data;
	while( !(SPSR & (1<<SPIF)));
}

unsigned char spi_transfer_byte(unsigned char data)
{
	// send the given data
    SPDR = data;

    // wait for transfer to complete
    while( (SPSR & (1<<SPIF))==0);
    // *** reading of the SPSR and SPDR are crucial
    // *** to the clearing of the SPIF flag
    // *** in non-interrupt mode
	
	//_delay_us(20);
    // return the received data
    return SPDR;
}

void wr_reg(unsigned char addr, unsigned char data)
{	
#ifdef USE_SPI
//	cbi(PORTB, CHIP_SEL);
	if(nCurrent_WriteSensor==1)
	{
		cbi(PORTB, CHIP_SEL1);
	}
	else if(nCurrent_WriteSensor==2)
	{
		cbi(PORTB, CHIP_SEL2);
	}
	else if(nCurrent_WriteSensor==3)
	{
		cbi(PORTB, CHIP_SEL3);
	}
	else
	{
		cbi(PORTB, CHIP_SEL4);
	}
	_delay_us(3);

	addr <<=1;	//20140724
	
	//spi_send_byte( addr & 0x7F );
	spi_send_byte( addr & 0xFE );
	spi_send_byte( data );

	_delay_us(3);
//  	sbi(PORTB, CHIP_SEL);	
	if(nCurrent_WriteSensor==1)
	{
		sbi(PORTB, CHIP_SEL1);
	}
	else if(nCurrent_WriteSensor==2)
	{
		sbi(PORTB, CHIP_SEL2);
	}
	else if(nCurrent_WriteSensor==3)
	{
		sbi(PORTB, CHIP_SEL3);
	}
	else
	{
		sbi(PORTB, CHIP_SEL4);
	}
#else
    I2C_Write(addr,data);
#endif	
}

unsigned char rd_reg(unsigned char addr)
{
	volatile unsigned char data;

#ifdef USE_SPI
//	cbi(PORTB, CHIP_SEL);
	if(nCurrent_ReadSensor==1)
	{
		cbi(PORTB, CHIP_SEL1);
	}
	else if(nCurrent_ReadSensor==2)
	{
		cbi(PORTB, CHIP_SEL2);
	}
	else if(nCurrent_ReadSensor==3)
	{
		cbi(PORTB, CHIP_SEL3);
	}
	else
	{
		cbi(PORTB, CHIP_SEL4);
	}
	
	_delay_us(3);
	
	addr <<=1;	//20140724
	
	//20140724 spi_send_byte(addr|0x80);
	spi_send_byte(addr|0x01);
	data=spi_transfer_byte(0x00);
	
	_delay_us(3);
// 	sbi(PORTB, CHIP_SEL);
	if(nCurrent_ReadSensor==1)
	{
		sbi(PORTB, CHIP_SEL1);
	}
	else if(nCurrent_ReadSensor==2)
	{
		sbi(PORTB, CHIP_SEL2);
	}
	else if(nCurrent_ReadSensor==3)
	{
		sbi(PORTB, CHIP_SEL3);
	}
	else
	{
		sbi(PORTB, CHIP_SEL4);
	}
#else
    data=I2C_Read(addr);
#endif
	return data;
}


/*******************************************************/

/*******************************************************/
// Register Read & Write
int init_BP(void)
{
	DDRA=0x00;DDRC=0x00;
	PORTA=0x00;PORTC=0x03;

	return 0;
}

void Sensor_init(void)
{
/*
 	wr_reg(BP_REG_CONTROL_REG1_ADDR,0x21);//Powe Mode normal  100hz odr
	wr_reg(BP_REG_CONTROL_REG2_ADDR,0xf0);//Sensor Enable //2G
	wr_reg(BP_REG_ACCE_FILTER_CON1_ADDR,0xA1); //lowpass filter 50hz 
*/
//	wr_reg(SGA100_CTRL_REG1,0x61);	//ODR =50 Hz, Sleep Duration 50ms, Normal Mode
//	wr_reg(SGA100_CTRL_REG2,0xFC);	//X,Y,Z-Axis Enable, X,Y,Z-offset sign value positive, 2g
}


int read_sensor_data(short *dx,short *dy, short *dz, short *dt)
{
	uint16_t temp_data;

	rINDEX=rd_reg(SGA100_DEV_ID);//_delay_ms(5);
	
	rSTATUS=rd_reg(SGA100_STATUS);//_delay_ms(5);
	rCTRLREG1=rd_reg(SGA100_CTRL_REG1);//_delay_ms(5);
	rCTRLREG2=rd_reg(SGA100_CTRL_REG2);//_delay_ms(5);
	rCTRLREG3=rd_reg(SGA100_CTRL_REG3);//_delay_ms(5);

	rOTP_CTRL=rd_reg(SGA100_OTP_CTRL);//_delay_ms(5);
	rMANUAL_BN_SELECT_CTRL=rd_reg(SGA100_MANUAL_BN_SELECT_CTRL);//_delay_ms(5);
	rOTP_TEST_CTRL=rd_reg(SGA100_OTP_TEST_CTRL);//_delay_ms(5);
	rOTP_TEST_ADDRESS_CTRL=rd_reg(SGA100_OTP_TEST_ADDRESS_CTRL);//_delay_ms(5);
	rOTP_TEST_DATA_CTRL=rd_reg(SGA100_OTP_TEST_DATA_CTRL);//_delay_ms(5);
	rOTP_Bank_Status=rd_reg(SGA100_OTP_Bank_Status);//_delay_ms(5);
	rOTP_PGM_CYCLE=rd_reg(SGA100_OTP_PGM_CYCLE);//_delay_ms(5);

	rTMODE_CTRL=rd_reg(SGA100_TMODE_CTRL);//_delay_ms(5);
	rINT1_CTRL=rd_reg(SGA100_INT1_CTRL);//_delay_ms(5);
	rINT1_MAP_FUNC=rd_reg(SGA100_INT1_MAP_FUNC);//_delay_ms(5);
	rOUT_DATA_XH=rd_reg(SGA100_OUT_DATA_XH);//_delay_ms(5);
	rOUT_DATA_XL=rd_reg(SGA100_OUT_DATA_XL);//_delay_ms(5);
	rOUT_DATA_YH=rd_reg(SGA100_OUT_DATA_YH);//_delay_ms(5);
	rOUT_DATA_YL=rd_reg(SGA100_OUT_DATA_YL);//_delay_ms(5);
	rOUT_DATA_ZH=rd_reg(SGA100_OUT_DATA_ZH);//_delay_ms(5);
	rOUT_DATA_ZL=rd_reg(SGA100_OUT_DATA_ZL);//_delay_ms(5);
	rOUT_DATA_TEMP=rd_reg(SGA100_OUT_DATA_TEMP);//_delay_ms(5);
	rUSR_GAIN_X_CTRL=rd_reg(SGA100_USR_GAIN_X_CTRL);//_delay_ms(5);
	rUSR_GAIN_Y_CTRL=rd_reg(SGA100_USR_GAIN_Y_CTRL);//_delay_ms(5);
	rUSR_GAIN_Z_CTRL=rd_reg(SGA100_USR_GAIN_Z_CTRL);//_delay_ms(5);
	rUSR_OFS_X_CTRL=rd_reg(SGA100_USR_OFS_X_CTRL);//_delay_ms(5);
	rUSR_OFS_Y_CTRL=rd_reg(SGA100_USR_OFS_Y_CTRL);//_delay_ms(5);
	rUSR_OFS_Z_CTRL=rd_reg(SGA100_USR_OFS_Z_CTRL);//_delay_ms(5);
	rTEST_CTRL=rd_reg(SGA100_TEST_CTRL);//_delay_ms(5);
	

	rDIGITAL_FILTER_CTRL=rd_reg(SGA100_DIGITAL_FILTER_CTRL);//_delay_ms(5);
	rINT_FUNC_CTRL1=rd_reg(SGA100_INT_FUNC_CTRL1);//_delay_ms(5);
	rINT_FUNC_CTRL2=rd_reg(SGA100_INT_FUNC_CTRL2);//_delay_ms(5);
	rINT_FUNC_CTRL3=rd_reg(SGA100_INT_FUNC_CTRL3);//_delay_ms(5);
	rFIFO_CTRL=rd_reg(SGA100_FIFO_CTRL);//_delay_ms(5);
	rFIFO_DATA_STATUS=rd_reg(SGA100_FIFO_DATA_STATUS);//_delay_ms(5);
	rINT_FUNC_CTRL1_STATUS=rd_reg(SGA100_INT_FUNC_CTRL1_STATUS);//_delay_ms(5);
	rMOTION_CTRL=rd_reg(SGA100_MOTION_CTRL);//_delay_ms(5);
	rMOTION_STATUS=rd_reg(SGA100_MOTION_STATUS);//_delay_ms(5);
	rMOTION_HIGH_TH=rd_reg(SGA100_MOTION_HIGH_TH);//_delay_ms(5);
	rMOTION_HIGH_DUR=rd_reg(SGA100_MOTION_HIGH_DUR);//_delay_ms(5);
	rMOTION_LOW_TH=rd_reg(SGA100_MOTION_LOW_TH);//_delay_ms(5);
	rMOTION_LOW_DUR=rd_reg(SGA100_MOTION_LOW_DUR);//_delay_ms(5);
	rTAP_INT_STATUS=rd_reg(SGA100_TAP_INT_STATUS);//_delay_ms(5);
	rTAP_TH=rd_reg(SGA100_TAP_TH);//_delay_ms(5);
	rTAP_DUR=rd_reg(SGA100_TAP_DUR);//_delay_ms(5);
	rDTAP_TIMEWINDOW_DUR=rd_reg(SGA100_DTAP_TIMEWINDOW_DUR);//_delay_ms(5);
	rTAP_LATENCY_DUR=rd_reg(SGA100_TAP_LATENCY_DUR);//_delay_ms(5);

	rAFE_WT_MODE=rd_reg(SGA100_AFE_WT_MODE);//_delay_ms(5);
	rCMI=rd_reg(SGA100_CMI);//_delay_ms(5);
	rTEST_DREGISTER=rd_reg(SGA100_TEST_DREGISTER);//_delay_ms(5);
	rTEST_DREGISTER_STATUS=rd_reg(SGA100_TEST_DREGISTER_STATUS);//_delay_ms(5);
	rCP_OT_PX=rd_reg(SGA100_CP_OT_PX);//_delay_ms(5);
	rCP_OT_NX=rd_reg(SGA100_CP_OT_NX);//_delay_ms(5);
	rCP_OT_PY=rd_reg(SGA100_CP_OT_PY);//_delay_ms(5);
	rCP_OT_NY=rd_reg(SGA100_CP_OT_NY);//_delay_ms(5);
	rCP_OT_PZ=rd_reg(SGA100_CP_OT_PZ);//_delay_ms(5);
	rCP_OT_NZ=rd_reg(SGA100_CP_OT_NZ);//_delay_ms(5);
	rPGA_GT_XY=rd_reg(SGA100_PGA_GT_XY);//_delay_ms(5);
	rPGA_GT_Z=rd_reg(SGA100_PGA_GT_Z);//_delay_ms(5);
	rTOSC3_VBGR=rd_reg(SGA100_TOSC3_VBGR);//_delay_ms(5);
	rDVP=rd_reg(SGA100_DVP);//_delay_ms(5);
	rTSEN_DC=rd_reg(SGA100_TSEN_DC);//_delay_ms(5);
	rTOSC1=rd_reg(SGA100_TOSC1);//_delay_ms(5);
	rTOSC2=rd_reg(SGA100_TOSC2);//_delay_ms(5);
	rDFACT_GT_X=rd_reg(SGA100_DFACT_GT_X);//_delay_ms(5);
	rDFACT_GT_Y=rd_reg(SGA100_DFACT_GT_Y);//_delay_ms(5);
	rDFACT_GT_Z=rd_reg(SGA100_DFACT_GT_Z);//_delay_ms(5);
	rDFACT_OFS_XYZ=rd_reg(SGA100_DFACT_OFS_XYZ);//_delay_ms(5);
	rDFACT_OT_X=rd_reg(SGA100_DFACT_OT_X);//_delay_ms(5);
	rDFACT_OT_Y=rd_reg(SGA100_DFACT_OT_Y);//_delay_ms(5);
	rDFACT_OT_Z=rd_reg(SGA100_DFACT_OT_Z);//_delay_ms(5);
	rDSIGN_CONTROL=rd_reg(SGA100_DSIGN_CONTROL);//_delay_ms(5);
	rOTP_PM=rd_reg(SGA100_OTP_PM);//_delay_ms(5);

	rOFFSET_DATA_MSB=rd_reg(SGA100_OFFSET_DATA_MSB);//_delay_ms(5);
	rOFFSET_DATA_LSB=rd_reg(SGA100_OFFSET_DATA_LSB);//_delay_ms(5);

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
	



/*******************************************************/


////////////////////////////////* EEPROM Control Start *////////////////////////////////
void writeEEPROM(unsigned int uiAddress, unsigned char ucData)
{
	char sreg;
	sreg = SREG;

	cli();

	while(EECR & (1<<EEWE) ); 	// wait for completion of previous write

	EEAR = uiAddress;
	EEDR = ucData;

	
	EECR |= (1 << EEMWE);		// Write logical one to EEMWE	
	EECR |= (1 << EEWE);		// Start eeprom write by setting EEWE
	
	_delay_ms(20);
	sei();
	SREG = sreg;
}

unsigned char readEEPROM(unsigned int uiAddress)
{
	char sreg;
	sreg = SREG;
	
	cli();

	while ( EECR & ( 1 << EEWE) ); // wait for completion of previous write

	EEAR = uiAddress;
	EECR |= ( 1<< EERE);

	SREG = sreg;
	
	_delay_ms(20);
	sei();
	
	return EEDR;
}
////////////////////////////////* EEPROM Control End *////////////////////////////////



//----- Begin Code ------------------------------------------------------------
int main(void)
{
	
	unsigned int tick_count1, tick_count2;

	// MCU Initialize
	init_port(); 		// initialize the port
	init_timer();		// initialize the timer system
	init_uart();		// initialize the UART0 (serial port)

#ifdef USE_SPI
	init_spi();			// initialize the SPI (SPI) 
#else
	I2C_Init();
#endif
3

	// BP ASIC iniitialize
	init_BP();

	_delay_ms(40);

	sei();

	// timer initialize
	tick_count2=TCNT3;

	//Register Variable iniitialize
	Sensor_init();
	
	nCurrent_ReadSensor=1;	/* 1st sensor in socket */
	nCurrent_WriteSensor=1;
	// Main Loop
	while(1)
	{
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
							wr_reg(addr1, data1);
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
		else
		{
		}	

		// Sensor Data read
		if (read_flag!=0)
		{
			read_flag=0;
			
			// Tick count
			tick_count1=TCNT3;
			tick_count2=tick_count1 - tick_count2;
			
			// Sensor Data Acquisition
			read_sensor_data(&outx,&outy,&outz,&outt);

		    printf("PKST");//Header
			
		    put_char0(75);//count
			
			put_char0(nCurrent_ReadSensor);	/* which sensor data ??? 1st or 2nd or 3rd or 4th */
			
			put_char0(rINDEX);	//1
			
			put_char0(rSTATUS);
			put_char0(rCTRLREG1);
			put_char0(rCTRLREG2);
			put_char0(rCTRLREG3);	//5

			put_char0(rOTP_CTRL);
			put_char0(rMANUAL_BN_SELECT_CTRL);
			put_char0(rOTP_TEST_CTRL);
			put_char0(rOTP_TEST_ADDRESS_CTRL);
			put_char0(rOTP_TEST_DATA_CTRL);	//10
			put_char0(rOTP_Bank_Status);
			put_char0(rOTP_PGM_CYCLE);	

			put_char0(rTMODE_CTRL);
			put_char0(rINT1_CTRL);
			put_char0(rINT1_MAP_FUNC);	//15
			put_char0(rOUT_DATA_XH);
			put_char0(rOUT_DATA_XL);
			put_char0(rOUT_DATA_YH);
			put_char0(rOUT_DATA_YL);
			put_char0(rOUT_DATA_ZH);	//20
			put_char0(rOUT_DATA_ZL);
			put_char0(rOUT_DATA_TEMP);
			put_char0(rUSR_GAIN_X_CTRL);
			put_char0(rUSR_GAIN_Y_CTRL);
			put_char0(rUSR_GAIN_Z_CTRL);	//25
			put_char0(rUSR_OFS_X_CTRL);
			put_char0(rUSR_OFS_Y_CTRL);
			put_char0(rUSR_OFS_Z_CTRL);	
			put_char0(rTEST_CTRL);

			put_char0(rDIGITAL_FILTER_CTRL);	//30
			put_char0(rINT_FUNC_CTRL1);
			put_char0(rINT_FUNC_CTRL2);
			put_char0(rINT_FUNC_CTRL3);
			put_char0(rFIFO_CTRL);
			put_char0(rFIFO_DATA_STATUS);	//35
			put_char0(rINT_FUNC_CTRL1_STATUS);	
			put_char0(rMOTION_CTRL);
			put_char0(rMOTION_STATUS);
			put_char0(rMOTION_HIGH_TH);
			put_char0(rMOTION_HIGH_DUR);	//40
			put_char0(rMOTION_LOW_TH);
			put_char0(rMOTION_LOW_DUR);
			put_char0(rTAP_INT_STATUS);
			put_char0(rTAP_TH);
			put_char0(rTAP_DUR);	//45
			put_char0(rDTAP_TIMEWINDOW_DUR);
			put_char0(rTAP_LATENCY_DUR);	

			put_char0(rAFE_WT_MODE);
			put_char0(rCMI);
			put_char0(rTEST_DREGISTER);	//50
			put_char0(rTEST_DREGISTER_STATUS);
			put_char0(rCP_OT_PX);
			put_char0(rCP_OT_NX);
			put_char0(rCP_OT_PY);
			put_char0(rCP_OT_NY);	//55
			put_char0(rCP_OT_PZ);		
			put_char0(rCP_OT_NZ);
			put_char0(rPGA_GT_XY);
			put_char0(rPGA_GT_Z);
			put_char0(rTOSC3_VBGR);	//60
			put_char0(rDVP);
			put_char0(rTSEN_DC);
			put_char0(rTOSC1);
			put_char0(rTOSC2);
			put_char0(rDFACT_GT_X);	//65
			put_char0(rDFACT_GT_Y);		
			put_char0(rDFACT_GT_Z);
			put_char0(rDFACT_OFS_XYZ);
			put_char0(rDFACT_OT_X);
			put_char0(rDFACT_OT_Y);	//70
			put_char0(rDFACT_OT_Z);
			put_char0(rDSIGN_CONTROL);
			put_char0(rOTP_PM);

			put_char0(rOFFSET_DATA_MSB);
			put_char0(rOFFSET_DATA_LSB);	//75
				
			put_char0('\r');//dummy
			put_char0('\n');

			// Timer Tick Reset
    		tick_count2=tick_count1;
#ifdef USE_SPI			
			nCurrent_ReadSensor++;
			
			if(nCurrent_ReadSensor>=5)	/* total sensor is 4. */
			{
				nCurrent_ReadSensor = 1;
			}
#endif
		}
	}
	return 0;
}


/*************************************************************************************
// I2C Initial Function
*************************************************************************************/

void I2C_Init()
{
#if 0
   DDRB=0x06;    // Output PB1=SCK, PB2=SDA	
   PORTB=0x06;	  // SCK,SDA = 1
#else
   DDRB=0x06;    // Output PB1=SCK, PB2=SDA	
   PORTB=0x07;	  // SCK,SDA = 1   //Chip Select High  SDO- Low
#endif
}


/*************************************************************************************
// I2C Signal Generate Function
*************************************************************************************/
void _nops_()
{
   nop();
   nop();
}

void _nop_()
{
   nop();
   nop();

/*
    nop();
	nop();
	nop();
	nop();
    nop();
	nop();
	nop();
	nop();
    nop();
	nop();


    nop();
	nop();
	nop();
	nop();
    nop();
	nop();
	nop();
	nop();
    nop();
	nop();

    nop();
	nop();
	nop();
	nop();
    nop();
	nop();
	nop();
	nop();
    nop();
	nop();
*/
}

void I2C_Start()
{
	oSDA_L;
	_nop_();
	oSCL_L;
}

void I2C_End()
{
	oSCL_H;
	_nop_();
	oSDA_H;
}

void I2C_ReStart()
{
	oSDA_H;
	_nop_();
	oSCL_H;
	_nop_();
	oSDA_L;
	_nop_();
	oSCL_L;
}

unsigned char I2C_Ack()
{
	unsigned char bAck;
   
	SDA_IN_INIT;
	_nop_();
	oSCL_H;
	_nop_();
   
	if(iSDA_L) 
		bAck=0;
	else       
		bAck=1;
		
	oSCL_L;
	SDA_OUT_INIT;

	return bAck;
}

void I2C_TransferAck()
{  
	oSCL_L;
	_nop_();

	oSDA_L;
	_nop_();
	oSCL_H;
	_nop_();

    _nop_();
	_nop_();
	_nop_();
	_nop_();

	oSCL_L;
	_nop_();
}

unsigned char I2C_ReadAck()
{
	unsigned char bAck;
   
	SDA_IN_INIT;
	_nop_();
	oSCL_H;
	_nop_();
   
	if(iSDA_L) 
		bAck=0;
	else       
		bAck=1;
	_nop_();
	oSCL_L;

	return bAck;
}

void I2C_NoAck()
{
	oSCL_H;
	_nop_();
	oSCL_L;
	_nop_();
	oSDA_L;
}

/*************************************************************************************
// I2C Byte Access Function
*************************************************************************************/
void I2C_ByteOut(unsigned char Data)
{
	unsigned char i;
	for(i=0; i<8; i++)
	{
		if(Data & 0x80) 
			oSDA_H;
		else            
			oSDA_L;
      
		_nop_();
		oSCL_H;
		_nop_();
		oSCL_L;
		_nop_();
      
		Data = (Data<<1);
	}
}

unsigned char I2C_ByteIn()
{
	unsigned char i;
	unsigned char rDat=0;
	unsigned char bDat;
   
	SDA_IN_INIT;
   
	for(i=0; i<8; i++)
	{
		oSCL_H;
		_nop_();

		rDat = rDat<<1;
		
		if(iSDA_H) 
			bDat=1;
		else       
			bDat=0;
		
		rDat = (rDat|bDat);
      
 //     _nop_();
		oSCL_L;
		_nop_();
	    _nop_();
	}

	oSDA_H;
	SDA_OUT_INIT;
   
	return rDat;
}


/*************************************************************************************
// I2C Write Function
*************************************************************************************/
void I2C_Write(unsigned char Addr, unsigned char Data)
{
	unsigned char bAck=0;

//--------------------------------------------- Write Start
	I2C_Start();
//--------------------------------------------- Command Out

	I2C_ByteOut( (I2C_ID | ADDR_0x100 | I2C_WR) );
	nops(40);
//--------------------------------------------- Ack
	bAck=I2C_Ack();
	//if(bAck)
	//printf("I2W SLA:%d\t",bAck);
//--------------------------------------------- Address Out
	I2C_ByteOut(Addr);
	nops(40);
//--------------------------------------------- Ack
	bAck=I2C_Ack();
	//if(bAck)
	//printf("I2W addr:%d\t",bAck);
//--------------------------------------------- Data Out
	I2C_ByteOut(Data);
	nops(40);
//--------------------------------------------- Ack
	bAck=I2C_Ack();
	//if(bAck)
	//printf("I2W data:%d\t",bAck);
//--------------------------------------------- Write End
	I2C_End();
}


/*************************************************************************************
// I2C Read Function
*************************************************************************************/
unsigned char I2C_Read(unsigned char Addr)
{
	unsigned char i2cDat;
	unsigned char bAck=0;

//--------------------------------------------- Write Start
	I2C_Start();
//--------------------------------------------- Write Command Out
	I2C_ByteOut( (I2C_ID | ADDR_0x100 | I2C_WR) );
//--------------------------------------------- Ack
	nops(40);
	bAck=I2C_Ack();
    
	//if(bAck)
	//printf("I2R SLA:%d\t",bAck);
	  
//--------------------------------------------- Address Low Out
	I2C_ByteOut(Addr);
//--------------------------------------------- Ack
	nops(40);
	bAck=I2C_Ack();

	//if(bAck)
    //	printf("I2R addr:%d\t",bAck);
//--------------------------------------------- ReStart
	I2C_ReStart();
//--------------------------------------------- Read Command Out
	I2C_ByteOut( (I2C_ID | ADDR_0x100 | I2C_RD) );
//--------------------------------------------- Ack
	nops(40);

	bAck=I2C_ReadAck();
	nops(40);
	//if(bAck)
	//printf("I2R Read Ack:%d\t",bAck);
//--------------------------------------------- Data Read
	i2cDat=I2C_ByteIn();
//--------------------------------------------- No Ack
	I2C_NoAck();
//--------------------------------------------- Write End
	I2C_End();

	return i2cDat;
}



