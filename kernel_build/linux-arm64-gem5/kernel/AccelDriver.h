#include <stdio.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>

#include <stdlib.h>
#include <stdint.h>

#define SystemCMemcpyHostToDevice 0
#define SystemCMemcpyDeviceToHost 1

typedef uint64_t DevMemAddr;

void AccelInitialization();

DevMemAddr AccelMalloc(size_t _size, const char _DevMemName);
void AccelFree(DevMemAddr SWAddr);
void AccelMemcpy(DevMemAddr SWAddr, void * data, size_t size, uint8_t TransferType);
void AccelCallDevice();


void AccelFinalization();