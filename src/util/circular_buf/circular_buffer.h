#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

typedef struct {
  uint8_t* buffer;
  size_t head;
  size_t tail;
  size_t size;
} circular_buf_t;

int circular_buf_reset(circular_buf_t* cbuf);
int circular_buf_put(circular_buf_t * cbuf, uint8_t data);
int circular_buf_get(circular_buf_t * cbuf, uint8_t * data);
bool circular_buf_empty(circular_buf_t cbuf);
bool circular_buf_full(circular_buf_t cbuf);

#endif
