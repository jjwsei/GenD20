#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#define USART_BYTE_TIMEOUT 100

#define CIRCBUF_DEF(x, y) \
   uint8_t x##_dataSpace[y]; \
   circular_buf_t x = { \
     .buffer = x##_dataSpace, \
     .head = 0, \
     .tail = 0, \
     .size = y 	\
}

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
int write_usart_data(uint8_t *, uint8_t);
int updateUsartTxqueue(uint8_t*, uint8_t);
bool USART_receivedAck(void);
bool USART_dataReady(void);
void USART_getRecvData(uint8_t *buffer);

#endif

