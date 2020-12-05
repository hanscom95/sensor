/**
 * \file Moon
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
#include <stdio_serial.h>
#include <conf_board.h>
#include <conf_pdc_uart_example.h>
#include <string.h>

#define SEND_BUFFER_SIZE			50	//20160121
#define RECEIVE_BUFFER_SIZE			125	//20160121

#define STRING_EOL    "\r"
#define STRING_HEADER "-- PDC_UART Example --\r\n" \
"-- "BOARD_NAME" --\r\n" \
"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/* Pdc transfer buffer */
uint8_t pdc_send_buffer[SEND_BUFFER_SIZE]={0};				//20160121
uint8_t pdc_receive_buffer[RECEIVE_BUFFER_SIZE]={0};		//20160121
/* PDC data packet for transfer */
pdc_packet_t g_pdc_uart_packet_tx;
pdc_packet_t g_pdc_uart_packet_rx;
/* Pointer to UART PDC register base */
Pdc *g_p_uart_pdc;


/* WIFI STATUS */
#define WIFI_NO_OPR								0

#define WIFI_CFG_START							1
#define WIFI_CFG_END							2

#define WIFI_CFG_NEWTWORK_START					3
#define WIFI_CFG_NEWTWORK_END					4

#define WIFI_CFG_WMODE_START					5
#define WIFI_CFG_WMODE_END						6

#define WIFI_CREATE_LIMITEDAP_START				7
#define WIFI_CREATE_LIMITEDAP_END				8

#define WIFI_ENABLE_DHCP_SER_START				9
#define WIFI_ENABLE_DHCP_SER_END				10

#define WIFI_UDP_LP_SET_START					11
#define WIFI_UDP_LP_SET_END						12

#define WIFI_UDP_START							13
	
uint8_t wifi_set_message_wifi_cfg_start[16]="AT+WRXACTIVE=1\r\n";
uint8_t wifi_set_message_wifi_cfg_network_start[49]="AT+NSET=192.168.1.78,255.255.255.0,192.168.1.78\r\n";
uint8_t wifi_set_message_wifi_cfg_wmode_start[9]="AT+WM=2\r\n";
uint8_t wifi_set_message_wifi_create_limitedap_start[19]="AT+WA=SG78_AP,,11\r\n";
uint8_t wifi_set_message_wifi_dhcp_ser_start[15]="AT+DHCPSRVR=1\r\n";
uint8_t wifi_set_message_wifi_udp_lp_set_start[16]="AT+NSUDP=48578\r\n";
uint8_t wifi_set_message_wifi_udp_start[6]="ATE0\r\n";

volatile static uint8_t wifi_opr_mode=WIFI_NO_OPR;
volatile static bool wifi_msg_updated=false;
volatile static bool wifi_msg_enabled=false;
//uint8_t wifi_buffer[SEND_BUFFER_SIZE]={0};

uint8_t uc_flag;
uint8_t uc_char;
uint8_t uart_buffer_char;

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
	if ((uart_get_status(CONSOLE_UART) & UART_SR_RXBUFF) == UART_SR_RXBUFF) 
	{
		LED_Toggle(LED0);
		uc_flag = uart_read(CONSOLE_UART, &uc_char);
		
		/* Configure PDC for data transfer (RX and TX) */
		wifi_buffer_clear();
		pdc_rx_init(g_p_uart_pdc, &g_pdc_uart_packet_rx, NULL);
	}
	
	if ((uart_get_status(CONSOLE_UART) & UART_SR_TXBUFE) == UART_SR_TXBUFE)
	{
		pdc_tx_init(g_p_uart_pdc, &g_pdc_uart_packet_tx, NULL);
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
	/* Initialize the SAM system */

	//! [board_setup]
	sysclk_init();
	board_init();
	//! [board_setup]

	/* Initialize the UART console */
	configure_console();

	//! [pdc_config]
	/* Get pointer to UART PDC register base */
	g_p_uart_pdc = uart_get_pdc_base(CONSOLE_UART);

	/* Initialize PDC data packet for transfer */
	g_pdc_uart_packet_tx.ul_addr = (uint32_t)pdc_send_buffer;
	g_pdc_uart_packet_tx.ul_size = SEND_BUFFER_SIZE;
	
	/* Initialize PDC data packet for receive */
	g_pdc_uart_packet_rx.ul_addr = (uint32_t)pdc_receive_buffer;
	g_pdc_uart_packet_rx.ul_size = RECEIVE_BUFFER_SIZE;
	
	/* Configure PDC for data transfer */
	pdc_tx_init(g_p_uart_pdc, &g_pdc_uart_packet_tx, NULL);
	
	/* Configure PDC for data receive */
	pdc_rx_init(g_p_uart_pdc, &g_pdc_uart_packet_rx, NULL);

	/* Enable PDC transfers */
	pdc_enable_transfer(g_p_uart_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);
	//! [pdc_config]

	/* Enable UART IRQ */
	//! [uart_irq]
	uart_enable_interrupt(CONSOLE_UART, UART_IER_TXBUFE | UART_IER_RXBUFF);
	//! [uart_irq]

	/* Enable UART interrupt */
	//! [uart_nvic_irq]
	NVIC_EnableIRQ(CONSOLE_UART_IRQn);
	//! [uart_nvic_irq]
	
	//! [busy_waiting]
	while (1) 
	{	
		delay_s(2);
		
		wifi_module_status_check();
		
		if(wifi_msg_enabled)
		{
			g_pdc_uart_packet_tx.ul_addr = (uint32_t) "PKST0000";
			g_pdc_uart_packet_tx.ul_size = sizeof("PKST0000");
		}
	}
	//! [busy_waiting]
}


void wifi_module_status_check(void)
{
	wifi_buffer_update_check();
	
	if(wifi_opr_mode==WIFI_NO_OPR)
	{
		wifi_opr_mode = WIFI_CFG_END;
		//puts((char*)&wifi_set_message_wifi_cfg_start);
		g_pdc_uart_packet_tx.ul_addr = (uint32_t) wifi_set_message_wifi_cfg_start;
		g_pdc_uart_packet_tx.ul_size = sizeof(wifi_set_message_wifi_cfg_start);
	}
	else if(wifi_opr_mode==WIFI_CFG_END)
	{
		if(wifi_msg_updated)
		{
			wifi_msg_updated=false;
			wifi_opr_mode = WIFI_CFG_NEWTWORK_END;
			
			//puts((char*)&wifi_set_message_wifi_cfg_network_start);
			g_pdc_uart_packet_tx.ul_addr = (uint32_t) wifi_set_message_wifi_cfg_network_start;
			g_pdc_uart_packet_tx.ul_size = sizeof(wifi_set_message_wifi_cfg_network_start);	
		}
	}
	else if(wifi_opr_mode==WIFI_CFG_NEWTWORK_END)
	{
		if(wifi_msg_updated)
		{
			wifi_msg_updated=false;
			wifi_opr_mode = WIFI_CFG_WMODE_END;
			
			//puts((char*)&wifi_set_message_wifi_cfg_wmode_start);
			g_pdc_uart_packet_tx.ul_addr = (uint32_t) wifi_set_message_wifi_cfg_wmode_start;
			g_pdc_uart_packet_tx.ul_size = sizeof(wifi_set_message_wifi_cfg_wmode_start);
		}
	}
	else if(wifi_opr_mode==WIFI_CFG_WMODE_END)
	{
		if(wifi_msg_updated)
		{
			wifi_msg_updated=false;
			wifi_opr_mode = WIFI_CREATE_LIMITEDAP_END;
			
			//puts((char*)&wifi_set_message_wifi_create_limitedap_start);
			g_pdc_uart_packet_tx.ul_addr = (uint32_t) wifi_set_message_wifi_create_limitedap_start;
			g_pdc_uart_packet_tx.ul_size = sizeof(wifi_set_message_wifi_create_limitedap_start);
		}
	}
	else if(wifi_opr_mode==WIFI_CREATE_LIMITEDAP_END)
	{
		if(wifi_msg_updated)
		{
			wifi_msg_updated=false;
			wifi_opr_mode = WIFI_ENABLE_DHCP_SER_END;
			
			//puts((char*)&wifi_set_message_wifi_dhcp_ser_start);
			g_pdc_uart_packet_tx.ul_addr = (uint32_t) wifi_set_message_wifi_dhcp_ser_start;
			g_pdc_uart_packet_tx.ul_size = sizeof(wifi_set_message_wifi_dhcp_ser_start);
		}
	}
	else if(wifi_opr_mode==WIFI_ENABLE_DHCP_SER_END)
	{
		if(wifi_msg_updated)
		{
			wifi_msg_updated=false;
			wifi_opr_mode = WIFI_UDP_LP_SET_END;
			
			//puts((char*)&wifi_set_message_wifi_udp_lp_set_start);
			g_pdc_uart_packet_tx.ul_addr = (uint32_t) wifi_set_message_wifi_udp_lp_set_start;
			g_pdc_uart_packet_tx.ul_size = sizeof(wifi_set_message_wifi_udp_lp_set_start);
		}
	}
	else if(wifi_opr_mode==WIFI_UDP_LP_SET_END)
	{
		if(wifi_msg_updated)
		{
			wifi_msg_updated=false;
			wifi_opr_mode = WIFI_UDP_START;
			
			//puts((char*)&wifi_set_message_wifi_udp_start);
			g_pdc_uart_packet_tx.ul_addr = (uint32_t) wifi_set_message_wifi_udp_start;
			g_pdc_uart_packet_tx.ul_size = sizeof(wifi_set_message_wifi_udp_start);
		}
	}
}

void wifi_buffer_update_check(void)
{
	if(wifi_opr_mode!=WIFI_UDP_START)
	{
		uint16_t i;
		
		for(i=0; i<sizeof(pdc_receive_buffer); i++;)
		{
			if((pdc_receive_buffer[i]==13) &&(pdc_receive_buffer[i+1]==10))
			{
				uc_char=0;
				wifi_msg_updated=true;
				wifi_buffer_clear();
			}
		}
	}
	else
	{
		wifi_msg_enabled=true;
	}
}

void wifi_buffer_clear(void)
{
	memset(pdc_receive_buffer,0,RECEIVE_BUFFER_SIZE);
}
