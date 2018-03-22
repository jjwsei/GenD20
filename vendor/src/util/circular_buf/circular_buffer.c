#include <asf.h>
#include "circular_buffer.h"

int circular_buf_reset(circular_buf_t * cbuf)
{
   int r = -1;

   if(cbuf)
   {
      cbuf->head = 0;
      cbuf->tail = 0;
      r = 0;
   }
   return r;
}

bool circular_buf_empty(circular_buf_t cbuf)
{
   // Empty is when head == tail
   return (cbuf.head == cbuf.tail);
}

bool circular_buf_full(circular_buf_t cbuf)
{
   // Full is when head is one position behind tail
   // We waste one space.
   return ((cbuf.head + 1) % cbuf.size) == cbuf.tail;
}

int circular_buf_put(circular_buf_t * cbuf, uint8_t data)
{
   int r = -1;
 
   if(cbuf)
   {
      cbuf->buffer[cbuf->head] = data;
      cbuf->head = (cbuf->head + 1) % cbuf->size;
      if(cbuf->head == cbuf->tail)
      {
         cbuf->tail = (cbuf->tail + 1) % cbuf->size;
      }
      r = 0;
   }
   return r;
}

int circular_buf_get(circular_buf_t * cbuf, uint8_t * data)
{
   int r = -1;

   if(cbuf && data && !circular_buf_empty(*cbuf))
   {
      *data = cbuf->buffer[cbuf->tail];
      cbuf->tail = (cbuf->tail + 1) % cbuf->size;
      r = 0;
   }
   return r;
}

