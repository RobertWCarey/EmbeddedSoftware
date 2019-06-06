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
  //Semaphore to allow buffer access
  fifo->BufferAccess = OS_SemaphoreCreate(1);
  //Initilise Spaces/Items Available
  fifo->SpaceAvailable = OS_SemaphoreCreate(FIFO_SIZE);
  fifo->ItemsAvailable = OS_SemaphoreCreate(0);

  return true;
}

bool FIFO_Put(TFIFO * const fifo, const uint8_t data)
{
  //wait for space to become available
  OS_SemaphoreWait(fifo->SpaceAvailable,0);

  //Disable interrupts
  OS_DisableInterrupts();

  // If not full add new data to the buffer
  fifo->Buffer[fifo->Start]=data;
  fifo->Start++; // Increment start position

  // Check if start position needs to be looped
  // >= because the array starts from 0
  if (fifo->Start >= FIFO_SIZE)
    fifo->Start = 0;

  //Enable interrupts
  OS_EnableInterrupts();

  //Increment the number of items in the buffer
  OS_SemaphoreSignal(fifo->ItemsAvailable);
  return true;
}

bool FIFO_Get(TFIFO * const fifo, uint8_t * const dataPtr)
{
  //wait for byte to become available
  OS_SemaphoreWait(fifo->ItemsAvailable,0);

  //Disable interrupts (context switch will be disabled)
  OS_DisableInterrupts();

  // If not empty read data
  (*dataPtr) = fifo->Buffer[fifo->End];
  fifo->End++; // Increment end position

  // Check if End position needs to be looped
  // >= because the array starts from 0
  if (fifo->End >= FIFO_SIZE)
    fifo->End = 0;

  //Enable interrupts
  OS_EnableInterrupts();

  //Increment the number of space in the buffer
  OS_SemaphoreSignal(fifo->SpaceAvailable);
  return true;
}

/*!
 * @}
 */
