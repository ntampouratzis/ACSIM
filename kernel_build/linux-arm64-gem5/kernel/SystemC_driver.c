#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <linux/types.h>

#include<linux/slab.h>

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <linux/hash.h>
#include <linux/list.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
 
#include "SystemC_driver.h"

#include <linux/ioport.h>
#include <linux/interrupt.h> //# Interrupt Implementation #
#include <asm/io.h>

#define IRQ 54 //# Interrupt Implementation #
 
#define FIRST_MINOR 0
#define MINOR_CNT 1



#define BASE 0x1c061000
#define ADDR_SIZE 0xfff

#define DMA_DATA_SIZE             0x00000
#define DMA_TRANSFER_TO_DEVICE    0x00008
#define DMA_TRANSFER_FROM_DEVICE  0x00010
#define CALL_DEVICE     	  0x00018
#define DMA_NUMBER_OF_CHUNCK      0x00020
#define DMA_SWADDR                0x00028
#define MALLOC_SIZE               0x00030
#define MALLOC_NAME               0x00038
#define MALLOC_ADDR               0x00040
#define FREE_ADDR                 0x00048

#define DEVICE_MEMORY_SIZE 536870912 //in bytes

#define DEVICE_MEMORY_ENTRY_SIZE (64*65536)
#define DEVICE_MEMORY_ENTRIES (DEVICE_MEMORY_SIZE/DEVICE_MEMORY_ENTRY_SIZE)
 
static dev_t dev;
static struct cdev c_dev;
static struct class *cl;
struct device *dev_ret;

static int size = 0;
static u64 SWAddr = 0;

static u8 * dev_data_dma_alloc_coherent_to_device;
static u8 * dev_data_dma_alloc_coherent_from_device;

static void __iomem* ioremap_res; 

static u8 * dev_data[DEVICE_MEMORY_ENTRIES];
static u32 NumberOfServeChuncks = 0;
static u32 NumberOfServeChuncksFromDevice = 0;
static u32 NumberOfRemainderChuncks = 0;
static u32 NumberOfRemainderChuncksFromDevice = 0;
static u8 ret;

static int i = 0;

bool device_busy;

static bool dma_to_device_active;
static bool dma_from_device_active;

dma_addr_t bus_addr_to_device;
dma_addr_t bus_addr_from_device;
 
static int my_open(struct inode *i, struct file *f)
{
    return 0;
}
static int my_close(struct inode *i, struct file *f)
{
    return 0;
}


void ramdevice_init(void)
{
    dma_to_device_active = 0;
    dma_from_device_active = 0;
    ioremap_res = ioremap(BASE, ADDR_SIZE);
    dev_data_dma_alloc_coherent_to_device = dma_alloc_coherent(dev_ret, DEVICE_MEMORY_ENTRY_SIZE, &bus_addr_to_device, GFP_KERNEL);
    if (dma_mapping_error(dev_ret, bus_addr_to_device)){
      printk(KERN_ALERT "\n\n\n---------- dma_MAP_ERROR (bus_addr_to_device): %pa\n\n\n\n",&bus_addr_to_device);
    }
    
    dev_data_dma_alloc_coherent_from_device = dma_alloc_coherent(dev_ret, DEVICE_MEMORY_ENTRY_SIZE, &bus_addr_from_device, GFP_KERNEL);
    if (dma_mapping_error(dev_ret, bus_addr_from_device)){
      printk(KERN_ALERT "\n\n\n---------- dma_MAP_ERROR (bus_addr_from_device): %pa\n\n\n\n",&bus_addr_from_device);
    }
  
}


void ramdevice_cleanup(void)
{
    dma_free_coherent(dev_ret, DEVICE_MEMORY_ENTRY_SIZE, dev_data_dma_alloc_coherent_to_device, bus_addr_to_device);
    dma_free_coherent(dev_ret, DEVICE_MEMORY_ENTRY_SIZE, dev_data_dma_alloc_coherent_from_device, bus_addr_from_device);
}


//# Interrupt Implementation #
irq_handler_t mydev_isr(int irq, void *device_id, struct pt_regs *regs)
{
  u32 ChunckSize;
  device_busy = 0;
  if(dma_to_device_active != 0){
    if(NumberOfRemainderChuncks > 0){
	  if((NumberOfRemainderChuncks == 1)&&(size%DEVICE_MEMORY_ENTRY_SIZE > 0)) //! The last Chunck !//
	    ChunckSize = size%DEVICE_MEMORY_ENTRY_SIZE;
	  else
	    ChunckSize = DEVICE_MEMORY_ENTRY_SIZE;
	  
	  NumberOfRemainderChuncks--; //! Serve the Chunck !//
	  NumberOfServeChuncks++;
	  
	  dma_sync_single_for_cpu(dev_ret, bus_addr_to_device, DEVICE_MEMORY_ENTRY_SIZE, DMA_TO_DEVICE);
	  
	  if(copy_from_user((u8 *)dev_data_dma_alloc_coherent_to_device, (u8 *)dev_data[NumberOfServeChuncks], ChunckSize*sizeof(u8)))
	    return (irq_handler_t) IRQ_HANDLED;
	  
	  dma_sync_single_for_device(dev_ret, bus_addr_to_device, DEVICE_MEMORY_ENTRY_SIZE, DMA_TO_DEVICE);
	      
	  //! Start DMA to set the Data to Device !//
	  iowrite32((unsigned long) ChunckSize, ioremap_res + DMA_DATA_SIZE);
	  iowrite32((unsigned long) SWAddr, ioremap_res + DMA_SWADDR);
	  iowrite32((unsigned long) NumberOfServeChuncks, ioremap_res + DMA_NUMBER_OF_CHUNCK);
	  iowrite32(cpu_to_le32(bus_addr_to_device), ioremap_res + DMA_TRANSFER_TO_DEVICE);
    }
    else{
      dma_to_device_active = 0;
    }
  }
  
  
  if(dma_from_device_active != 0){
    if(NumberOfRemainderChuncksFromDevice == 0) //! The last Chunck !//
      ChunckSize = size%DEVICE_MEMORY_ENTRY_SIZE;
    else
      ChunckSize = DEVICE_MEMORY_ENTRY_SIZE;
    
    if (copy_to_user((u8 *)dev_data[NumberOfServeChuncksFromDevice],(u8 *)dev_data_dma_alloc_coherent_from_device, ChunckSize*sizeof(u8)))
      return (irq_handler_t) IRQ_HANDLED;
    if(NumberOfRemainderChuncksFromDevice > 0){
	  if((NumberOfRemainderChuncksFromDevice == 1)&&(size%DEVICE_MEMORY_ENTRY_SIZE > 0)) //! The last Chunck !//
	    ChunckSize = size%DEVICE_MEMORY_ENTRY_SIZE;
	  else
	    ChunckSize = DEVICE_MEMORY_ENTRY_SIZE;
	  
	  NumberOfRemainderChuncksFromDevice--; //! Serve the Chunck !//
	  NumberOfServeChuncksFromDevice++;
	      	      
	  //! Start DMA to set the Data from Device !//
	  iowrite32((unsigned long) ChunckSize, ioremap_res + DMA_DATA_SIZE);
	  iowrite32((unsigned long) SWAddr, ioremap_res + DMA_SWADDR);
	  iowrite32((unsigned long) NumberOfServeChuncksFromDevice, ioremap_res + DMA_NUMBER_OF_CHUNCK);
	  iowrite32(cpu_to_le32(bus_addr_from_device), ioremap_res + DMA_TRANSFER_FROM_DEVICE);
    }
    else{
      dma_from_device_active = 0;
    }
  }
  return (irq_handler_t) IRQ_HANDLED;
}


#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static int my_ioctl(struct inode *i, struct file *f, unsigned int cmd, unsigned long arg)
#else
static long my_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
#endif
{
    parameters_t p;
    MallocParams_t MallocParams;
    u32 FirstChunckSize;
    u64 FreeAddr;
 
    switch (cmd)
    {
        case QUERY_GET_DATA:
	    //! Copy the Data to UserSpace !//
	    for(i=0;i<size/DEVICE_MEMORY_ENTRY_SIZE;i++){
	      dev_data[i] = (u8 *)(arg +i*DEVICE_MEMORY_ENTRY_SIZE); //copy the userspace pointer
	    }
	    if(size%DEVICE_MEMORY_ENTRY_SIZE > 0){ //! Copy the last chunck of Data !//
	      dev_data[i] = (u8 *)(arg +i*DEVICE_MEMORY_ENTRY_SIZE); //copy the userspace pointer
	    }
            break;
	    
	case DMA_FROM_DEVICE_START:
	      //! Set the FirstChunckSize Parameter !//
	      if(size/DEVICE_MEMORY_ENTRY_SIZE > 0)
		FirstChunckSize = DEVICE_MEMORY_ENTRY_SIZE;
	      else
		FirstChunckSize = size%DEVICE_MEMORY_ENTRY_SIZE;
	  
	      //! Set the RemainderChuncksFromDevice Parameter !//
	      NumberOfRemainderChuncksFromDevice = size/DEVICE_MEMORY_ENTRY_SIZE;
	      if(size%DEVICE_MEMORY_ENTRY_SIZE > 0)
		NumberOfRemainderChuncksFromDevice++;
	  
	      NumberOfRemainderChuncksFromDevice--; //! Serve the first Chunck !//
	      NumberOfServeChuncksFromDevice = 0;
	      
	      dma_from_device_active = 1;
	      	      
	      //! Start DMA to set the Data to Host !//
	      iowrite32((unsigned long) FirstChunckSize, ioremap_res + DMA_DATA_SIZE);
	      iowrite32((unsigned long) SWAddr, ioremap_res + DMA_SWADDR);
	      iowrite32((unsigned long) NumberOfServeChuncksFromDevice, ioremap_res + DMA_NUMBER_OF_CHUNCK);
	      iowrite32(cpu_to_le32(bus_addr_from_device), ioremap_res + DMA_TRANSFER_FROM_DEVICE);
	    
            break;
	    
	case DMA_FROM_DEVICE_WAIT:
	    if(dma_from_device_active != 0)
	      ret = 1;
	    else
	      ret = 0;
	    if (copy_to_user((u8 *)arg, (u8 *)&ret, sizeof(u8)))
                return -EACCES;
            break;     
	    
	case QUERY_WAIT:
	    if(device_busy == 1)
	      ret = 1;
	    else
	      ret = 0;
	    if (copy_to_user((u8 *)arg, (u8 *)&ret, sizeof(u8)))
                return -EACCES;
            break;    
	    
	    
        case QUERY_CALL_DEVICE:
	    if(dma_to_device_active == 0){
	      //! OK! DMA Transaction is completed !//
	      ret = 0;
	      device_busy = 1;
	      iowrite32((unsigned long)0, ioremap(BASE, ADDR_SIZE)+ CALL_DEVICE);
	    }
	    else{
	      //! Wait! DMA Transaction is not completed !//
	      ret = 1;
	    }
	    if (copy_to_user((u8 *)arg, (u8 *)&ret, sizeof(u8))){
                return -EACCES;
	    }
            break;
	    
	case DMA_TO_DEVICE_WAIT: //! Check if previous DMA transactions are pending !//
	    if(dma_to_device_active != 0)
	      ret = 1;
	    else
	      ret = 0;
            
	    if (copy_to_user((u8 *)arg, (u8 *)&ret, sizeof(u8)))
                return -EACCES;
	    
            break;         
        case QUERY_SET_DATA:
	    //! Copy the Data to KernelSpace !//
	    //! Copy the firtst chuncks of Data !//
	    FirstChunckSize = size%DEVICE_MEMORY_ENTRY_SIZE;
	    for(i=0;i<size/DEVICE_MEMORY_ENTRY_SIZE;i++){
	      FirstChunckSize = DEVICE_MEMORY_ENTRY_SIZE;
	      dev_data[i] = (u8 *)(arg +i*DEVICE_MEMORY_ENTRY_SIZE); //copy the userspace pointer
	      
	    }
	    NumberOfRemainderChuncks = i;
	    if(size%DEVICE_MEMORY_ENTRY_SIZE > 0){ //! Copy the last chunck of Data !//
	      NumberOfRemainderChuncks++;
	      dev_data[i] = (u8 *)(arg +i*DEVICE_MEMORY_ENTRY_SIZE); //copy the userspace pointer
	      
	    }
	    
	   if(copy_from_user((u8 *)dev_data_dma_alloc_coherent_to_device, (u8 *)dev_data[0], FirstChunckSize*sizeof(u8)))
	     return -EACCES;
	     
	   NumberOfRemainderChuncks--; //! Serve the first Chunck !//
	   NumberOfServeChuncks = 0;
	   
	   dma_to_device_active = 1;
	      
	   //! Start DMA to set the Data to Device !//
	   iowrite32((unsigned long) FirstChunckSize, ioremap_res + DMA_DATA_SIZE);
	   iowrite32((unsigned long) SWAddr, ioremap_res + DMA_SWADDR);
	   iowrite32((unsigned long) NumberOfServeChuncks, ioremap_res + DMA_NUMBER_OF_CHUNCK);
	   iowrite32(cpu_to_le32(bus_addr_to_device), ioremap_res + DMA_TRANSFER_TO_DEVICE);
        break;
	    
	    
	case QUERY_SET_PARAMETERS:
            if (copy_from_user(&p, (parameters_t *)arg, sizeof(parameters_t))){
                return -EACCES;
            }            
            size   = p.size;
	    SWAddr = p.SWAddr;
            break;
	    
	case QUERY_MALLOC:
            if (copy_from_user(&MallocParams, (MallocParams_t *)arg, sizeof(MallocParams_t))){
                return -EACCES;
            }            

	    iowrite32((unsigned long) MallocParams.size, ioremap(BASE, ADDR_SIZE)+ MALLOC_SIZE);
	    iowrite8((uint8_t) MallocParams.DevMemName, ioremap(BASE, ADDR_SIZE)+ MALLOC_NAME);
	    MallocParams.AllocAddr = (unsigned long) ioread32(ioremap(BASE, ADDR_SIZE)+ MALLOC_ADDR);
	    
	    if (copy_to_user((MallocParams_t *)arg, &MallocParams, sizeof(MallocParams_t))){
                return -EACCES;
	    }
            break;
	    case QUERY_FREE:
            if (copy_from_user(&FreeAddr, (uint64_t *)arg, sizeof(uint64_t))){
                return -EACCES;
            }            
            iowrite32((unsigned long) FreeAddr, ioremap(BASE, ADDR_SIZE)+ FREE_ADDR);
            break;
        default:
            return -EINVAL;
    }
 
    return 0;
}
 
 
 
 
 
 
static struct file_operations query_fops =
{
    .owner = THIS_MODULE,
    .open = my_open,
    .release = my_close,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
    .ioctl = my_ioctl
#else
    .unlocked_ioctl = my_ioctl
#endif
};



 
static int __init SystemC_driver_init(void)
{
  
    int ret;
 
    
    
    if ( ! request_mem_region(BASE, ADDR_SIZE, "SystemC") ) {
	      printk( KERN_ALERT "Unable to get io port at 0x%8X\n", BASE );
	      return -ENODEV;
    }
    
    
    if ( request_irq( IRQ, (irq_handler_t) mydev_isr, IRQF_DISABLED, "SystemC", NULL ) ) {
      printk( KERN_INFO "unable to register IRQ %d\n", IRQ );
      release_mem_region(BASE, ADDR_SIZE);
      return -ENODEV;
    }
    
    
    
    printk( KERN_ALERT "\n\nPHYSICAL DEVICE: REQUEST REGION OK\n\n");
 
    if ((ret = alloc_chrdev_region(&dev, FIRST_MINOR, MINOR_CNT, "SystemC_driver")) < 0)
    {
        return ret;
    }
 
    cdev_init(&c_dev, &query_fops);
 
    if ((ret = cdev_add(&c_dev, dev, MINOR_CNT)) < 0)
    {
        return ret;
    }
     
    if (IS_ERR(cl = class_create(THIS_MODULE, "char")))
    {
        cdev_del(&c_dev);
        unregister_chrdev_region(dev, MINOR_CNT);
        return PTR_ERR(cl);
    }
    if (IS_ERR(dev_ret = device_create(cl, NULL, dev, NULL, "SystemC")))
    {
        class_destroy(cl);
        cdev_del(&c_dev);
        unregister_chrdev_region(dev, MINOR_CNT);
        return PTR_ERR(dev_ret);
    }
    
    ramdevice_init();
    
    
    return 0;
}
 
static void __exit SystemC_driver_exit(void)
{
    ramdevice_cleanup();
    device_destroy(cl, dev);
    class_destroy(cl);
    cdev_del(&c_dev);
    unregister_chrdev_region(dev, MINOR_CNT);
    release_mem_region(BASE, ADDR_SIZE);
}
 
module_init(SystemC_driver_init);
module_exit(SystemC_driver_exit);
 
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tampouratzis Nikolaos <ntampouratzis_at_isc_dot_tuc_dot_gr>");
MODULE_DESCRIPTION("SystemC ioctl() Char Driver");