
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
 
