#ifndef QUERY_IOCTL_H
#define QUERY_IOCTL_H
#include <linux/ioctl.h>
 


typedef struct
{
    size_t size;
    uint64_t SWAddr;
} parameters_t;

typedef struct
{
    size_t size;
    char DevMemName;
    uint64_t AllocAddr;
} MallocParams_t;
 
#define QUERY_GET_DATA _IOR('q', 1, void *)
#define QUERY_CALL_DEVICE 	_IOR('q', 2, uint8_t *)
#define QUERY_SET_DATA 		_IOW('q', 3, void *)
#define QUERY_SET_PARAMETERS 	_IOW('p', 4, parameters_t *)
#define QUERY_WAIT _IOR('q', 5, uint8_t *)
#define DMA_FROM_DEVICE_START _IO('q', 6)
#define DMA_FROM_DEVICE_WAIT _IOR('q', 7, uint8_t *)
#define DMA_TO_DEVICE_WAIT _IOR('q', 8, uint8_t *)
#define QUERY_MALLOC 	_IOWR('p', 9, MallocParams_t *)
#define QUERY_FREE 	_IOW('p', 10, uint64_t *)

#endif