# ARM gem5-fiendly Kernels
The OS can be represented as a layered structure, as in the following Figure. It contains the User Space with the user applications and all the appropriate libraries, and the Kernel Space (in our case a Linux Kernel), which is strictly reserved for running
a privileged operating system kernel and most of the device drivers. In order to incorporate efficiently our accelerator module we have developed a set of device drivers and we have integrated them in two ARM gem5-configured 3.x kernels (ARM-32 and ARM-64).

The accelerator is activated through programmed I/Os that provide the start address and the size of the array used for the descriptors of the accelerator. The CPU can then sleep until an interprocessor interrupt from the accelerator is delivered to indicate task completion; this approach allows for full overlap of the two sub-simulations. In addition, an ioctl function was developed in order to achieve efficient user-kernel space communication; this function can mainly perform the following tasks (additional helper commands were developed):

### QUERY SET DATA 
Initialise the Direct Memory Access (DMA) copy transaction from Host (Kernel Space) to Accelerator Wrapper. 

### QUERY GET DATA 
Initialise the DMA copy transaction from Accelerator Wrapper to Host (Kernel Space).

### QUERY CALL DEVICE 
Call the SystemC accelerator (executing the specified application). Finally, an interrupt handler was implemented in order to
receive appropriate interrupts from the Accelerator Wrapper, such as the SystemC accelerator finish signal, the memcpy finish signal, etc.

Please refer to [Kernel Driver](linux-arm32-gem5/kernel/SystemC_driver.c) and [UserSpace Driver](linux-arm32-gem5/kernel/AccelDriver.c) for ARM-32.

Please refer to [Kernel Driver](linux-arm64-gem5/kernel/SystemC_driver.c) and [UserSpace Driver](linux-arm64-gem5/kernel/AccelDriver.c) for ARM-64.

Finally, we have implement a reference [User Application](Application/TestApp.c) in order to call our Accelerator.

## Build the ARM-32 Kernel
The following script compliles the kernel with [User Application](Application/TestApp.c) and mount it inside the 32bit .img
```
./build32.sh
```

## Build the ARM-64 Kernel
The following script compliles the kernel with [User Application](Application/TestApp.c) and mount it inside the 64bit .img
```
./build64.sh
```
