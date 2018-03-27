#include <asf.h>
#include "usart_driver.h"
#include "time_util.h"
#include "circular_buffer.h"

CIRCBUF_DEF(rxDataBuf, 128);
CIRCBUF_DEF(txDataBuf, 128);

uint16_t rx_timeout = 0;
bool rxRecvFlag = 0;
uint8_t receivedByte = 0;

struct usart_module usart_instance;
bool waiting_for_ack = 0;

volatile bool write_buffer_locked_flag = false;
bool write_buffer_locked(void);
void lock_write_buffer(void);
void unlock_write_buffer(void);

void configure_usart(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);

	config_usart.baudrate    = 115200;
	config_usart.mux_setting = USART_RX_3_TX_2_XCK_3 ;
	config_usart.pinmux_pad0 = PINMUX_UNUSED;
	config_usart.pinmux_pad1 = PINMUX_UNUSED;
	config_usart.pinmux_pad2 = PINMUX_PB10D_SERCOM4_PAD2;
	config_usart.pinmux_pad3 = PINMUX_PB11D_SERCOM4_PAD3;

	while (usart_init(&usart_instance,
			SERCOM4, &config_usart) != STATUS_OK) {
	}
   
    // these lines were required in order to get the receive on charcter
    // interrupt to fire without needing to repeatedly call the built-in
    // receive job function
	SercomUsart *const usart_hw = &(usart_instance.hw->USART);
	usart_hw->INTENSET.reg = SERCOM_USART_INTFLAG_RXC;
	usart_instance.remaining_rx_buffer_length = 1;
	usart_instance.rx_buffer_ptr              = &receivedByte;
	usart_instance.rx_status                  = STATUS_BUSY;

	usart_enable(&usart_instance);
}

void configure_usart_callbacks(void)
{
	usart_register_callback(&usart_instance,
			usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_instance,
			usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);

	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
}

void usart_read_callback(struct usart_module *const usart_module)
{
     //save the character to the buffer
     circular_buf_put(&rxDataBuf, receivedByte);
     // reset timeout timer
     rx_timeout = get_time_ms();
     // indicate that we have received at least one character
     rxRecvFlag = 1; 
     // reset interrupt for next character
	 SercomUsart *const usart_hw = &(usart_module->hw->USART);
	 usart_hw->INTENSET.reg = SERCOM_USART_INTFLAG_RXC;
	 usart_module->remaining_rx_buffer_length = 1;
	 usart_module->rx_buffer_ptr              = &receivedByte;
	 usart_module->rx_status                  = STATUS_BUSY;
}

/******************************************************************
* This function should be called at a rate a few milliseconds longer 
* than the time it will take to send the longest transmission. For
* example if the longest transmission that you expect to receive is 
* 50 bytes then this function should be called at a rate no less than 
* (((1/baudrate) * bits/byte * 50 + .004) seconds.
******************************************************************/
bool USART_dataReady(void)
{
    if(rxRecvFlag == 1)
    {
    	if(time_since_ms(get_time_ms(), rx_timeout) > USART_BYTE_TIMEOUT)
		{
			rxRecvFlag = 0;     
  			return 1;
		}
	}
    return 0;
}

void USART_getRecvData(uint8_t *buffer)
{
    uint8_t index = 0;
    int8_t result = 0;

    result = circular_buf_get(&rxDataBuf, &buffer[index]);
    while(result == 0)
        result = circular_buf_get(&rxDataBuf, &buffer[++index]);
}

void usart_write_callback(struct usart_module *const usart_module)
{
   // see if there is data still in the ring buffer
   // if so, send the data 
}

int write_usart_data(uint8_t *buffer, uint8_t length)
{
   int result = -1;
 
   usart_write_buffer_job(&usart_instance, buffer, length);
   waiting_for_ack = 1;
   return result;
}

bool USART_receivedAck(void)
{
   return waiting_for_ack;
}

int updateUsartTxqueue(uint8_t *data, uint8_t length)
{
   int result = -1;

   return result;
}

bool write_buffer_locked(void)
{
   return write_buffer_locked_flag;
}

void lock_write_buffer(void)
{
   write_buffer_locked_flag = true;
}

void unlock_write_buffer(void)
{
  write_buffer_locked_flag = false;
}
