/*
 * Copyright (c) 2018 The Regents of The Technical University of Crete
 * All rights reserved.
 * 
 * ----------------------------------------------------------------------------
 * Accelerator User Space Driver
 * Copyright (c) 2018, H2020 ECOSCALE.
 * Copyright (c) 2018, Telecommunications Systems Institute.
 * ----------------------------------------------------------------------------
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Tampouratzis Nikolaos, ntampouratzis@isc.tuc.gr
 */

/* @file
 * Accelerator UserSpace Driver. You can referee in this paper 
 * https://ieeexplore.ieee.org/document/7927071/
 */
#include "AccelDriver.h"
 
#include "SystemC_driver.h"

#include <sys/stat.h>

void wait_function();
void dma_from_device_start();
void dma_from_device_wait();
void dma_to_device_wait();
void CreateDevice(char * dev);
int doesDeviceExist(const char *filename);

int fd;

int doesDeviceExist(const char *filename) {
    struct stat st;
    int result = stat(filename, &st);
    return result == 0;
}

void CreateDevice(char * dev){
  //! Check if the Device already exists !//
  char file_exist[32] = "/dev/";
  strcat(file_exist,dev);
  if(doesDeviceExist(file_exist)){
    return;
  }
  
  char cmd[32] = "cat sys/class/char/";
  strcat(cmd,dev);
  strcat(cmd,"/dev");
  
  FILE *fp;
  char path[32];
  
  //! Open the command for reading. !//
  fp = popen(cmd, "r");
  if (fp == NULL) {
    printf("Failed to run cat sys/class/char command\n" );
    exit(1);
  }

  //! Read the output to find the MAJOR & MINOR NUMBER OF DEVICE and then allocate the device using mknod !//
  if (fgets(path, sizeof(path)-1, fp) != NULL) {
    //printf("%s", path);
    char mknod_str[32] = "mknod /dev/";
    strcat(mknod_str,dev);
    strcat(mknod_str," c ");
    strcat(mknod_str,strtok (path,":"));
    strcat(mknod_str," ");
    strcat(mknod_str,strtok (NULL,":"));
    system(mknod_str);
  }
  else{
   printf("The Kernel Driver does not exists!\n"); 
  }

  //! close !//
  pclose(fp);
}


 

void set_parameters(size_t _size, DevMemAddr _SWAddr)
{
    parameters_t p;
    p.size = _size;
    p.SWAddr = _SWAddr;
    
    if (ioctl(fd, QUERY_SET_PARAMETERS, &p) == -1){
        perror("query_apps ioctl set_parameters");
    }
}

DevMemAddr AccelMalloc(size_t _size, const char _DevMemName){
  
  MallocParams_t params;
  params.size = _size;
  params.DevMemName = _DevMemName;
  
  if (ioctl(fd, QUERY_MALLOC, &params) == -1){
        perror("query_apps ioctl malloc_params");
  }
  
  return params.AllocAddr;
}


void AccelFree(DevMemAddr SWAddr){
  if (ioctl(fd, QUERY_FREE, &SWAddr) == -1){
        perror("query_apps ioctl query_free");
    }
}

void AccelMemcpy(DevMemAddr SWAddr, void * data, size_t size, uint8_t TransferType)
{
    if(TransferType == SystemCMemcpyHostToDevice){
      
      dma_to_device_wait(); //! Check if previous DMA transactions are pending !//
      
      set_parameters(size, SWAddr);
      
      if (ioctl(fd, QUERY_SET_DATA, (void *) data) == -1){
	  perror("query_apps ioctl SystemCMemcpyHostToDevice");
      }
    }
    else if(TransferType == SystemCMemcpyDeviceToHost){
      dma_from_device_wait(); //! Check if previous DMA transactions are pending !//
      
      wait_function();
  
      set_parameters(size, SWAddr);
      
      dma_from_device_start(); //! Set the DMA QUERY !//
      
      dma_from_device_wait(); //! Wait to complete the DMA transaction !//
      
      if (ioctl(fd, QUERY_GET_DATA, (void *) data) == -1){
	  perror("query_apps ioctl SystemCMemcpyDeviceToHost");
      }
    }
}

void dma_to_device_wait()
{
    uint8_t ret;
    if (ioctl(fd, DMA_TO_DEVICE_WAIT, (uint8_t *) &ret) == -1){
        perror("query_apps ioctl dma_to_device_wait");
    }
    
    if(ret == 1){
      dma_to_device_wait();
    }
}

void dma_from_device_start()
{
    if (ioctl(fd, DMA_FROM_DEVICE_START) == -1){
        perror("query_apps ioctl dma_from_device_start");
    }
}

void dma_from_device_wait()
{
    uint8_t ret;
    if (ioctl(fd, DMA_FROM_DEVICE_WAIT, (uint8_t *) &ret) == -1){
        perror("query_apps ioctl dma_from_device_wait");
    }
    
    if(ret == 1){
      dma_from_device_wait();
    }
}


void wait_function()
{
    uint8_t ret;
    if (ioctl(fd, QUERY_WAIT, (uint8_t *) &ret) == -1){
        perror("query_apps ioctl wait_function");
    }
    
    if(ret == 1){
      wait_function();
    }
}
 
void AccelCallDevice()
{
    uint8_t ret;
    if (ioctl(fd, QUERY_CALL_DEVICE, (uint8_t *) &ret) == -1){
        perror("query_apps ioctl AccelCallDevice");
    }
    if(ret == 1){
      AccelCallDevice();
    }
}

 
void AccelInitialization(){
  char * dev = "SystemC";
  CreateDevice(dev);
  char file_name[16] = "/dev/";
  strcat(file_name,dev);
  fd = open(file_name, O_RDWR);
  if (fd == -1){
    perror("Acceleration Device open");
    return;
  }  
}

void AccelFinalization(){
  close (fd);
}
 
