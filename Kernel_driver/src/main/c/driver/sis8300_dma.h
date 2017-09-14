/*
 * SIS8300 dma memory allocation/deallocation and dma transfer handling.
 */

#ifndef SIS8300_DMA_H_
#define SIS8300_DMA_H_

#define SIS8300_DMA_ALLIGN  16

void sis8300_dma_free(sis8300_dev *sisdevice, sis8300_buf *sisblock);
int sis8300_dma_alloc(sis8300_dev *sisdevice, sis8300_buf *sisblock);
int sis8300_dma_read(sis8300_dev *device, sis8300_buf *sisbuf, uint32_t offset, uint32_t count);
int sis8300_dma_write(sis8300_dev *device, sis8300_buf *sisbuf, uint32_t offset, uint32_t count);

#endif /* SIS8300_DMA_H_ */
