/**
 * \file
 *
 * \brief SAM Peripheral DMA Controller Example.
 *
 * Copyright (c) 2011-2015 Atmel Corporation. All rights reserved.
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
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include <string.h> /* memset */
#include <stdio_serial.h>
#include <conf_board.h>
#include <conf_pdc_uart_example.h>

#define BUFFER_SIZE  5

/* WIFI STATUS */
#define WIFI_NO_OPR						0
#define WIFI_ERROR_S					15
#define WIFI_ERROR_RE					16
#define WIFI_BAUD_CHG					1
#define WIFI_CFG_START					2
#define WIFI_CFG_END					3
#define WIFI_CFG_NEWTWORK_START			4
#define WIFI_CFG_NEWTWORK_END			5
#define WIFI_CFG_WMODE_START			6
#define WIFI_CFG_WMODE_END				7
#define WIFI_CREATE_LIMITEDAP_START		8
#define WIFI_CREATE_LIMITEDAP_END		9
#define WIFI_ENABLE_DHCP_SER_START		10
#define WIFI_ENABLE_DHCP_SER_END		11
#define WIFI_UDP_LP_SET_START			12
#define WIFI_UDP_LP_SET_END				13
#define WIFI_UDP_START					14


#define WIFI_BUF_MAX					128
#define OK_MSG_LENGTH					2
#define CONNECT_MSG_LENGTH				7
#define DISCONNECT_MSG_LENGTH			10

/* Pdc transfer buffer */
uint8_t g_uc_pdc_buffer[BUFFER_SIZE];
/* PDC data packet for transfer */
pdc_packet_t g_pdc_uart_packet;
/* Pointer to UART PDC register base */
Pdc *g_p_uart_pdc;

//flag to use wifi status
volatile static uint8_t wifi_opr_mode=WIFI_NO_OPR;

volatile char wifi_buffer[WIFI_BUF_MAX]={0};
const char ok_msg[]="OK";
const char connect_msg[]="CONNECT";
const char disconnect_msg[]="DISCONNECT";
volatile static uint16_t buffer_index=0;
volatile static bool wifi_msg_updated=false;
volatile static bool wifi_Client_msg_updated=false;

volatile int nPortIndex=0;
volatile int nMsgDataStartIndex=0;
volatile int nMsgDataEndIndex=0;
volatile int nIPStartIndex=0;
volatile unsigned char szPacketHeader[5]="PKST";
volatile unsigned char szPacketEnd[3]="\r\n";
volatile bool udp_client_sent=false;

void wifi_module_status_check(void);
void wifi_buffer_update_check(void);
void wifi_buffer_clear(void);


/**
 * \brief Interrupt handler for UART interrupt.
 */

//! [int_handler]
void console_uart_irq_handler(void)
{
	/* Get UART status and check if PDC receive buffer is full */
	//if ((uart_get_status(CONSOLE_UART) & UART_SR_RXBUFF) == UART_SR_RXBUFF) {
		/* Configure PDC for data transfer (RX and TX) */
	//	pdc_rx_init(g_p_uart_pdc, &g_pdc_uart_packet, NULL);
	//	pdc_tx_init(g_p_uart_pdc, &g_pdc_uart_packet, NULL);
	//}
	
	int value;
	usart_getchar(USART0, &value);
	
	if(buffer_index<WIFI_BUF_MAX-1)
	{
		wifi_buffer[buffer_index]=value;
		buffer_index++;
	}
	else
	{
		wifi_buffer_clear();
	}
}
//! [int_handler]

/**
 * \brief Configure UART console.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.paritytype = CONF_UART_PARITY
	};
	
	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}


/**
 * \brief Application entry point for pdc_uart example.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	char szBuffer[128]={0};
	int status;
	int nMsgSize;
	static uint8_t wait_1s=0;
	/* Initialize the SAM system */

	//! [board_setup]
	sysclk_init();
	board_init();
	//! [board_setup]

	/* Initialize the UART console */
	configure_console();

	//! [pdc_config]
	/* Get pointer to UART PDC register base */
	//g_p_uart_pdc = uart_get_pdc_base(CONSOLE_UART);

	/* Initialize PDC data packet for transfer */
	//g_pdc_uart_packet.ul_addr = (uint32_t) g_uc_pdc_buffer;
	//g_pdc_uart_packet.ul_size = BUFFER_SIZE;

	/* Configure PDC for data receive */
	//pdc_rx_init(g_p_uart_pdc, &g_pdc_uart_packet, NULL);

	/* Enable PDC transfers */
	//pdc_enable_transfer(g_p_uart_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);
	//! [pdc_config]

	/* Enable UART IRQ */
	//! [uart_irq]
	uart_enable_interrupt(CONSOLE_UART, UART_IER_RXBUFF);
	//! [uart_irq]

	/* Enable UART interrupt */
	//! [uart_nvic_irq]
	NVIC_EnableIRQ(CONSOLE_UART_IRQn);
	//! [uart_nvic_irq]

	//! [busy_waiting]
	
	wifi_opr_mode=WIFI_NO_OPR;
	
	delay_ms(1000);
	
	while (1) {
		
		wifi_module_status_check();
		
		memset(szBuffer,0,128);
		
		if(wifi_Client_msg_updated)
		{
			wifi_Client_msg_updated=false;
			
			//make accel string
			sprintf(szBuffer,"test1=====");
			usart_write_line(USART0,szBuffer);
						
			usart_write_line(USART0,"\r\n\eE\r\n");
		}
	}
	//! [busy_waiting]
}


void wifi_module_status_check(void)
{
	wifi_buffer_update_check();

	if(wifi_opr_mode==WIFI_NO_OPR)
	{
		wifi_opr_mode = WIFI_CFG_NEWTWORK_START;
		wifi_buffer_clear();
	}
	else if(wifi_opr_mode==WIFI_CFG_NEWTWORK_START)
	{
		usart_write_line(USART0, "AT+NSET=192.168.1.20,255.255.255.0,192.168.1.20\r\n");
		wifi_opr_mode = WIFI_CFG_NEWTWORK_END;
	}
	else if(wifi_opr_mode==WIFI_CFG_NEWTWORK_END)
	{
		if(wifi_msg_updated)
		{
			wifi_msg_updated=false;
			wifi_opr_mode = WIFI_CFG_WMODE_START;
			wifi_buffer_clear();
		}
	}
	else if(wifi_opr_mode==WIFI_CFG_WMODE_START)
	{
		usart_write_line(USART0, "AT+WM=2\r\n");
		wifi_opr_mode = WIFI_CFG_WMODE_END;
	}
	else if(wifi_opr_mode==WIFI_CFG_WMODE_END)
	{
		if(wifi_msg_updated)
		{
			wifi_msg_updated=false;
			wifi_opr_mode = WIFI_ENABLE_DHCP_SER_START;
			wifi_buffer_clear();
		}
	}
	else if(wifi_opr_mode==WIFI_ENABLE_DHCP_SER_START)
	{
		usart_write_line(USART0, "AT+DHCPSRVR=1\r\n");
		wifi_opr_mode = WIFI_ENABLE_DHCP_SER_END;
	}
	else if(wifi_opr_mode==WIFI_ENABLE_DHCP_SER_END)
	{
		if(wifi_msg_updated)
		{
			wifi_msg_updated=false;
			wifi_opr_mode = WIFI_CREATE_LIMITEDAP_START;
			wifi_buffer_clear();
		}
	}
	else if(wifi_opr_mode==WIFI_CREATE_LIMITEDAP_START)
	{
		usart_write_line(USART0, "AT+WA=SE20_AP,,11\r\n");
		wifi_opr_mode = WIFI_CREATE_LIMITEDAP_END;
	}
	else if(wifi_opr_mode==WIFI_CREATE_LIMITEDAP_END)
	{
		if(wifi_msg_updated)
		{
			wifi_msg_updated=false;
			wifi_opr_mode = WIFI_UDP_LP_SET_START;
			wifi_buffer_clear();
		}
	}
	else if(wifi_opr_mode==WIFI_UDP_LP_SET_START)
	{
		usart_write_line(USART0, "AT+NSUDP=48590\r\n");
		wifi_opr_mode = WIFI_UDP_LP_SET_END;
	}
	else if(wifi_opr_mode==WIFI_UDP_LP_SET_END)
	{
		if(wifi_msg_updated)
		{
			wifi_msg_updated=false;
			wifi_Client_msg_updated=false;
			wifi_opr_mode = WIFI_UDP_START;
			wifi_buffer_clear();
		}
	}
	else
	{
	}
}


void wifi_buffer_update_check(void)
{
	uint16_t i;
	
	wifi_msg_updated=true;
	if(wifi_opr_mode!=WIFI_UDP_START)
	{
		for(i=0;i<WIFI_BUF_MAX-1;i++)
		{
			if((wifi_buffer[i]=='\r') && (wifi_buffer[i+1]=='\n'))// Wifi module message
			{
				wifi_msg_updated=true;
			}
		}
	}
	else
	{
		//UDP Start
		for(i=0;i<WIFI_BUF_MAX-1;i++)
		{
			if((wifi_buffer[i]=='P') && (wifi_buffer[i+1]=='K') && (wifi_buffer[i+2]=='S') && (wifi_buffer[i+3]=='T'))//Client Send Message start
			{
				udp_client_sent=true;
				nMsgDataStartIndex=i;
			}
			
			if((wifi_buffer[i]=='\r') && (wifi_buffer[i+1]=='\n'))
			{
				if(udp_client_sent==true)
				{
					wifi_Client_msg_updated=true;
					udp_client_sent=false;
					nMsgDataEndIndex=i-1;
				}
			}
		}
	}
}


void wifi_buffer_clear(void)
{
	memset(wifi_buffer,0,sizeof(char)*WIFI_BUF_MAX);

	buffer_index=0;
}