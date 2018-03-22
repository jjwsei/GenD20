#ifndef UART_DRIVER_H
#define UART_DRIVER_H

typedef enum {
   IDLE,
   GOT_ZERO,
   GOT_AA,
   GOT_LENGTH
} serial_states;

void usart_read_callback(struct usart_module *const usart_module);
void usart_write_callback(struct usart_module *const usart_module);
void configure_usart(void);
void configure_usart_callbacks(void);
int read_usart_data(uint8_t *);
int write_usart_data(uint8_t *, uint8_t);
int updateUsartTxqueue(uint8_t*, uint8_t);

#endif

