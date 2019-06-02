/*!
 * @addtogroup FIFO_module FIFO module documentation
 * @{
 */

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


bool FIFO_Init(TFIFO * const fifo)
{
  // Set initial positions for the FIFO buffer
  fifo->Start = 0;
  fifo->End = 0;
  //Semaphore to all buffer access
  fifo->BufferAccess = OS_SemaphoreCreate(1);
  //Initilise Space,Items Available
  fifo->SpaceAvailable = OS_SemaphoreCreate(FIFO_SIZE);
  fifo->ItemsAvailable = OS_SemaphoreCreate(0);


  return true;
}

bool FIFO_Put(TFIFO * const fifo, const uint8_t data)
{
  OS_ERROR error;
  //wait for space to become available
  error = OS_SemaphoreWait(fifo->SpaceAvailable,0);
  if(error)
      return false;
  //obtain exclusive access to the FIFO (after space is available to ensure buffer access isn't locked)
  OS_SemaphoreWait(fifo->BufferAccess,0);

  //Disable interrupts
  OS_DisableInterrupts();

  // If not full add new data to the buffer
  fifo->Buffer[fifo->Start]=data;
  fifo->Start++; // Increment start position

  // Check if start position needs to be looped
  if (fifo->Start > FIFO_SIZE)
    fifo->Start = 0;

  //Enable interrupts
  OS_EnableInterrupts();

  //Release exclusive access to the FIFO
  OS_SemaphoreSignal(fifo->BufferAccess);
  //Increment the number of items in the buffer
  OS_SemaphoreSignal(fifo->ItemsAvailable);
  return true;
}

bool FIFO_Get(TFIFO * const fifo, uint8_t * const dataPtr)
{
  OS_ERROR error;
  //wait for byte to become available
  error = OS_SemaphoreWait(fifo->ItemsAvailable,6);
  if(error)
    return false;
  //obtain exclusive access to the FIFO (after byte is available to ensure buffer access isn't locked)
  OS_SemaphoreWait(fifo->BufferAccess,0);

  //Disable interrupts
  OS_DisableInterrupts();

  // If not empty read data
  (*dataPtr) = fifo->Buffer[fifo->End];
  fifo->End++; // Increment end position

  // Check if End position needs to be looped
  if (fifo->End > FIFO_SIZE)
    fifo->End=0;

  //Enable interrupts
  OS_EnableInterrupts();

  //Release exclusive access to the FIFO
  OS_SemaphoreSignal(fifo->BufferAccess);
  //Increment the number of space in the buffer
  OS_SemaphoreSignal(fifo->SpaceAvailable);
  return true;
}

/*!
 * @}
 */
