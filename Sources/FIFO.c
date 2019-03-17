/*! @file
 *
 *  @brief Routines to implement a FIFO buffer.
 *
 *  This contains the structure and "methods" for accessing a byte-wide FIFO.
 *
 *  @author Robert Carey & Harry Mace
 *  @date 2019-03-17
 */

// Include Header Files
#include "FIFO.h"

bool FIFO_Init(TFIFO * const fifo){
  fifo->Start = 0;
  fifo->End = 0;
  fifo->NbBytes = 0;

  return true;
}

bool FIFO_Put(TFIFO * const fifo, const uint8_t data){
  // Check if the buffer is full
  if (fifo->NbBytes >= FIFO_SIZE)
    return false;

  // If not full add new data to the buffer
  fifo->Buffer[fifo->Start]=data;
  fifo->Start++; // Increment start position
  fifo->NbBytes++; // Increment NbBytes in buffer

  // Check if start position needs to be looped
  if (fifo->Start > FIFO_SIZE)
    fifo->Start = 0;

  return true;
}

bool FIFO_Get(TFIFO * const fifo, uint8_t * const dataPtr){
  // Check if there is data in the buffer
  if (fifo->NbBytes <= 0)
    return false;

  // If not empty read data
  (*dataPtr) = fifo->Buffer[fifo->End];
  fifo->End++; // Increment end position
  fifo->NbBytes--; // Decrement NbBytes

  // Check if End position needs to be looped
  if (fifo->End > FIFO_SIZE)
    fifo->End=0;
}
