/**
 * \file
 *
 * \brief SAM MCAN basic Quick Start
 *
 * Copyright (C) 2015-2016 Atmel Corporation. All rights reserved.
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

/**
 *  \mainpage MCAN example
 *
 *  \par Purpose
 *
 *  This example demonstrates the basic functions of MCAN controller.
 *
 *  \par Requirements
 *
 *  This package can be used with two SAMV71 Xplained Ultra boards.
 *  The CAN1 in two board should be connected externally before running
 *  the example.
 *
 *  \par Description
 *
 *  In this example, one board sends messages over CAN bus to another board.
 *  The CAN message will display on the terminal window.
 *
 *  \par Usage
 *
 *  -# Build the program and download it into the evaluation board.
 *  -# On the computer, open and configure a terminal application
 *     (e.g., HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 bauds
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Connect CAN1 in two boards.
 *  -# Start the application.
 *  -# Upon startup, the application will output the following lines
 *     on the terminal window.
 *      \code
			"  -- Select the action:\r\n"
			"  0: Set standard filter ID 0: 0x45A, store into Rx buffer. \r\n"
			"  1: Set standard filter ID 1: 0x469, store into Rx FIFO 0. \r\n"
			"  2: Send standard message with ID: 0x45A and 4 byte data 0 to 3. \r\n"
			"  3: Send standard message with ID: 0x469 and 4 byte data 128 to 131. \r\n"
			"  4: Set extended filter ID 0: 0x100000A5, store into Rx buffer. \r\n"
			"  5: Set extended filter ID 1: 0x10000096, store into Rx FIFO 1. \r\n"
			"  6: Send extended message with ID: 0x100000A5 and 8 byte data 0 to 7. \r\n"
			"  7: Send extended message with ID: 0x10000096 and 8 byte data 128 to 135. \r\n"
			"  h: Display menu \r\n\r\n");
 \endcode
 *  -# Press '0' or '1' or '4'  or '5' key in the terminal window to configure one board to
 *     receive CAN message.
 *  -# Press '2' or '3' or '6'  or '7' key in the terminal window to configure another board to
 *     send CAN message. The message will be displayed on the terminal window.
 */

/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include <string.h>
#include <conf_mcan.h>


/* module_inst */
static struct mcan_module mcan_instance;


/* mcan_filter_setting */
#define MCAN_RX_STANDARD_FILTER_INDEX_0    0
#define MCAN_RX_STANDARD_FILTER_INDEX_1    1
#define MCAN_RX_STANDARD_FILTER_ID_0     0x45A
#define MCAN_RX_STANDARD_FILTER_ID_0_BUFFER_INDEX     2
#define MCAN_RX_STANDARD_FILTER_ID_1     0x469
#define MCAN_RX_EXTENDED_FILTER_INDEX_0    0
#define MCAN_RX_EXTENDED_FILTER_INDEX_1    1
#define MCAN_RX_EXTENDED_FILTER_ID_0     0x100000A5
#define MCAN_RX_EXTENDED_FILTER_ID_0_BUFFER_INDEX     1
#define MCAN_RX_EXTENDED_FILTER_ID_1     0x10000096


#define MCAN_RX_FIFO_NUMBER	1


/* mcan_receive_message_setting */
static volatile uint32_t standard_receive_index = 0;
static volatile uint32_t extended_receive_index = 0;
static struct mcan_rx_element_fifo_1 rx_element_fifo_1;

/**
 * \brief Configure UART console.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/**
 * \brief MCAN module initialization.
 *
 */
static void configure_mcan(struct mcan_module* mcan_mod)
{
	/* Initialize the module. */
	struct mcan_config config_mcan;
	mcan_get_config_defaults(&config_mcan);
	mcan_init(mcan_mod, MCAN_MODULE, &config_mcan);


	/* Enable interrupts for this MCAN module */
	irq_register_handler(MCAN1_IRQn, 1);
	mcan_enable_interrupt(&mcan_instance, MCAN_FORMAT_ERROR | MCAN_ACKNOWLEDGE_ERROR | MCAN_BUS_OFF);
	
			
	mcan_start(mcan_mod);
}

/**
 * \brief Configure FIFO1 on the peripheral to accept all extended messages
 */
static void configure_mcan_fifo(struct mcan_module* mcan_mod)
{
	/*  
	 *  Setup rx filtering to accept messages into FIFO1 with extended format
	 *  this accepts all messages
	 */
	struct mcan_extended_message_filter_element et_filter;
	mcan_get_extended_message_filter_element_default(&et_filter);
	et_filter.F1.reg = MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFID2(0) |
			MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFT_CLASSIC;
	mcan_set_rx_extended_filter(mcan_mod, &et_filter, 0);
}

/**
 * \brief Queues up a packet for transmission.
 *
 * \param can Pointer to mcan_module that was initialized
 * \param id_value		Message id value
 * \param data			Data array to transmit
 * \param data_length	Length of data to transmit
 *
 * \return True if successfully queued (space available), false otherwise
 */
static bool mcan_send_message(struct mcan_module* mcan_mod, uint32_t id_value, uint8_t *data, uint32_t data_length)
{
	uint32_t status = mcan_tx_get_fifo_queue_status(mcan_mod);

	//check if fifo is full
	if(status & MCAN_TXFQS_TFQF) {
		return false;
	}

	//get the put index where we put the next packet
	uint32_t put_index = (status & MCAN_TXFQS_TFQPI_Msk) >> MCAN_TXFQS_TFQPI_Pos;

	struct mcan_tx_element tx_element;
	mcan_get_tx_buffer_element_defaults(&tx_element);

	tx_element.T0.reg |= MCAN_TX_ELEMENT_T0_EXTENDED_ID(id_value) | MCAN_TX_ELEMENT_T0_XTD;
	tx_element.T1.bit.DLC = data_length;

	for (uint32_t i = 0; i < data_length; i++) {
		tx_element.data[i] = data[i];
	}

	mcan_set_tx_buffer_element(mcan_mod, &tx_element, put_index);
	mcan_tx_transfer_request(mcan_mod, (1 << put_index));

	return true;
}


/**
 * \brief Get a packet from the software reception buffer
 *
 * \param can Pointer to mcan_module that was initialized
 * \param id_value		Message id value
 * \param data			Data array, should be big enough for message, 8 for CAN, 64 for FD. Otherwise data will be dropped on the copy to array (See data_length param)
 * \param data_length	Data buffer size (set before calling the function), this gets updated in the function call to the actual length of data received if its smaller than the max
 *
 * \return True if packet retrieved, false if no packet available
 */
static bool mcan_get_message(struct mcan_module* mcan_mod, uint32_t* id_value, uint8_t* data, uint8_t* data_length)
{
	uint32_t status = mcan_rx_get_fifo_status(mcan_mod, MCAN_RX_FIFO_NUMBER);

	uint32_t num_elements = status & MCAN_RXF1S_F1FL_Msk;
	uint32_t get_index = (status & MCAN_RXF1S_F1GI_Msk) >> MCAN_RXF1S_F1GI_Pos;

	if (num_elements > 0) {
		mcan_get_rx_fifo_1_element(mcan_mod, &rx_element_fifo_1, get_index);
		mcan_rx_fifo_acknowledge(mcan_mod, MCAN_RX_FIFO_NUMBER, get_index);

		*id_value = rx_element_fifo_1.R0.bit.ID;

		//data overflow check
		if( rx_element_fifo_1.R1.bit.DLC < *data_length ) {
			*data_length = rx_element_fifo_1.R1.bit.DLC;
		}

		for (size_t i = 0; i < *data_length; i++) {
			data[i] = rx_element_fifo_1.data[i];
		}
		
		return true;
	}

	return false;
}

/**
 * \brief Interrupt handler for MCAN,
 *   inlcuding RX,TX,ERROR and so on processes.
 */
void MCAN1_Handler(void)
{
	volatile uint32_t status;
	status = mcan_read_interrupt_status(&mcan_instance);

	if (status & MCAN_BUS_OFF) {
		mcan_clear_interrupt_status(&mcan_instance, MCAN_BUS_OFF);
		mcan_stop(&mcan_instance);
		printf(": MCAN bus off error, re-initialization. \r\n\r\n");
		configure_mcan(&mcan_instance);
	}

	if (status & MCAN_ACKNOWLEDGE_ERROR) {
		mcan_clear_interrupt_status(&mcan_instance, MCAN_ACKNOWLEDGE_ERROR);
		printf("Protocol ACK error, please double check the clock in two boards. \r\n\r\n");
	}

	if (status & MCAN_FORMAT_ERROR) {
		mcan_clear_interrupt_status(&mcan_instance, MCAN_FORMAT_ERROR);
		printf("Protocol format error, please double check the clock in two boards. \r\n\r\n");
	}
}

/**
 * \brief display configuration menu.
 */
static void display_menu(void)
{
	printf("Menu :\r\n"
			"  -- Select the action:\r\n"
			"  0: Send extended message with ID: 0x100000A5 and 8 byte data 0 to 7. \r\n"
			"  1: Send extended message with ID: 0x10000096 and 8 byte data 128 to 135. \r\n"
			"  2: Get a received message from the queue and print it if available. \r\n"
			"  h: Display menu \r\n\r\n");
}

int main(void)
{
	uint8_t key;

	sysclk_init();
	board_init();

	configure_console();
	configure_mcan(&mcan_instance);
	configure_mcan_fifo(&mcan_instance);

	display_menu();

	static uint8_t tx_message_0[8] = { 0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7 };
	static uint8_t tx_message_1[8] = { 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87};
	

	while(1) {
		scanf("%c", (char *)&key);

		switch (key) {
			case 'h':
				display_menu();
				break;

			case '0':
				printf("  0: Send extended message with ID: 0x100000A5 and 8 byte data 0 to 7. \r\n");
				mcan_send_message(&mcan_instance, MCAN_RX_EXTENDED_FILTER_ID_0, tx_message_0, CONF_MCAN_ELEMENT_DATA_SIZE);
				break;

			case '1':
				printf("  1: Send extended message with ID: 0x10000096 and 8 byte data 128 to 135. \r\n");
				mcan_send_message(&mcan_instance, MCAN_RX_EXTENDED_FILTER_ID_1, tx_message_1, CONF_MCAN_ELEMENT_DATA_SIZE);
				break;
				
			case '2': {
					printf("  2: Get and print one received message. \r\n");

					uint32_t rcvd_msg_id = 0;
					uint8_t rcvd_data[8];
					uint8_t rcvd_len = sizeof(rcvd_data);
					if( mcan_get_message(&mcan_instance, &rcvd_msg_id, rcvd_data, &rcvd_len) ) {
						printf("\r\nReceived message id %lu, length %u. \r\n", rcvd_msg_id, rcvd_len );
						
						printf("The received data is: \r\n");
						for (int i = 0; i < rcvd_len; i++) {
							printf("  %d",rcvd_data[i]);
						}
						printf("\r\n\r\n");
					} else {
						printf("  2: No messages found. \r\n");
					}
				}
				break;
			default:
				break;
		}
	}

}
