#include <asf.h>
#include "usart_driver.h"

#define MAX_RX_BUFFER_LENGTH   	10
#define MAX_BUFFER_LENGTH	255
#define DATA_LENGTH 		8

struct usart_module usart_instance;

bool data_ready_flag = 0;
volatile uint8_t rx_buffer[MAX_RX_BUFFER_LENGTH];
uint8_t serial_state = IDLE;
uint16_t data_length = 0;
uint8_t index_count = 0;
uint8_t message[100] = {0};
uint8_t length_byte = 1;

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
        data_ready_flag = 1;
}

void usart_write_callback(struct usart_module *const usart_module)
{

}

int read_usart_data(uint8_t *buffer)
{
        int result = -1;

	usart_read_job(&usart_instance, (uint8_t *)rx_buffer);
        if(data_ready_flag == 1)
        {
           data_ready_flag = 0;
           switch(serial_state)
           {
              case IDLE:
                 if(rx_buffer[0] == 0x00)
                   serial_state = GOT_ZERO;
                 break;
              case GOT_ZERO:
                 if(rx_buffer[0] == 0xAA)
                   serial_state = GOT_AA;
                   data_length = 0;
                 break;
              case GOT_AA:
                 if(length_byte == 1) 
                 {
                    ++length_byte;
                    data_length = rx_buffer[0] << 8;
                 }
                 else
                 {
                    length_byte = 1;
                    data_length |= rx_buffer[0];
                    index_count = 0;
                    serial_state = GOT_LENGTH;
                 }
                 break;
              case GOT_LENGTH:
                 buffer[index_count] =  rx_buffer[0];
                 if(index_count == data_length)
                 {
                   result = 0;
                   serial_state = IDLE;
                 }
                 else
                   ++index_count;
                 break;
              default:
                 break;
           }
        } 
        return result;  
}  

int write_usart_data(uint8_t *buffer, uint8_t length)
{
   int result = -1;
 
   usart_write_buffer_job(&usart_instance, buffer, length);
   return result;
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
