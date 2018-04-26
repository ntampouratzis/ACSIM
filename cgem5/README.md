# cgem5 - ACSIM-modified gem5 version. 

The cgem5 repository includes the modified gem5 simulator that ipmlements the ACSIM simulator. The ACSIM can be executed either standalone or within COSSIM project. For this reason, cgem5 supports (optionally) interconnection with IEEE HLA interfaces and modifies the network interface so that it can communicate with other cgem5 nodes through a network simulator. It should be noted that cgem5 can be used independently of ACSIM as a standalone package incorporating all the changes that have been integrated to the official GEM5 October 2017 release.

## Differences between cgem5 and official gem5 October 2017 version
The following subsections describes in tandem the modifications and extensions that have been implemented for ACSIM. In addition, this repository contains all the COSSIM-modifications as described in [COSSIM_cgem5](https://github.com/H2020-COSSIM/cgem5) repository.

### Accelerator Wrapper
Our [Accelerator Wrapper](src/dev/arm/SystemC_Accelerator/dev0/AccelDevice0.cc) device was developed in order to achieve efficient communication and synchronisation of GEM5 with the SystemC accelerator. It inherits all GEM5 DMA device characteristics so that full DMA transactions utilizing the full operating system can be performed. In addition, it contains a large Device Memory to store the data from the OS memcpy, simulating the DDR memory found in most of the real systems incorporating PCI-connected FPGA and/or GPU boards. Subsequently, a core containing mixed C++ and SystemC code was implemented for the connection of the GEM5 C++ functions and the accelerator’s SystemC threads, as illustrated in the following Figure.


<p align="center">
  <img src="https://github.com/ntampouratzis/ACSIM/blob/master/cgem5/ACSIM_Figure.png" />
</p>

The Accelerator Wrapper consists of, in total, eight C++ and SystemC-thread modules as described below:

#### 1) Dynamic Memory Allocator C++ Module 
The Buddy dynamic memory allocation algorithm scheme is implemented in the Accelerator Wrapper to allocate and free Device Memory segments through the GEM5 operating system, similar to the cudaMalloc of NVIDIA GPUs.

#### 2) DMA Write/Read C++ Modules 
Two Direct Memory Access engines were developed so that the GEM5 dma device can efficiently transfer high data volumes from the Linux driver to the Accelerator Wrapper and vise versa, similar to the cudaMemcpy of NVIDIA GPUs. The user can define, through
one parameter, the delay of the DMA data transfer in order to achieve a realistic latency.

#### 3) Synchronisation Event C++ Module 
A GEM5 synchronisation event function is implemented and it is triggered at every SystemC accelerator device cycle; the DeviceClock option (e.g. −−DeviceClock=500MHz) is also added to declare the device clock during the initialisation of GEM5.
This function checks whether the SystemC accelerator has reached the next cycle. Finally, in case GEM5 is faster than the SystemC accelerator 1 , it reschedules the synchronisation event function in order to wait for the SystemC accelerator.

#### 4) Init SystemC Thread 
The initialisation thread is implemented in SystemC so as to generate the reset and start signals both of which are essential for SystemC’s module execution. 

#### 5) Clock SystemC Thread 
The clock thread is implemented in SystemC in order to generate the actual clock signal of the accelerator when called by GEM5’s OS; the DeviceClock option is used in order to define the clock frequency. Moreover, at every SystemC cycle, the full   synchronisation with GEM5 is achieved by checking whether the GEM5 has completed its tasks within this time frame.

#### 6) MemCpy ToDevice/ToHost SystemC Threads 
Two MemCpy SystemC threads developed pass the data from the Wrapper Device Memory to the corresponding synthesisable I/O ports, which depend upon the data type, such as int, double, etc., so that they eventually arrive at the SystemC accelerator. The
user can define through one parameter the amount of data to be read/written in one SystemC cycle.


### SystemC Accelerator
A reference [SystemC accelerator](src/dev/arm/SystemC_Accelerator/dev0/SystemCDevice0.cc) has been developed in order to evaluate the Accelerator Wrapper and the Linux Kernel Drivers; this is also a helpful guideline for designers/users that expect to develop their own SystemC accelerators. Subsequently, the Accelera open-source libraries have been incorporated with the GEM5 SCons construction tool in order to allow for the compilation and execution of complete system applications. 

The SystemC accelerator consists of a main SystemC thread and two SystemC functions described below:

#### 1) Controller/Scheduler SystemC Thread 
This is the main SystemC thread which is called by GEM5’s OS while the user has the ability to create as many individual cores as required so as to best serve his/her application, as represented in Figure 3 by the dashed-line threads. These threads are scheduled by the Controller/Scheduler SystemC Thread.

#### 2) MemCpy ToDevice/ToHost SystemC functions 
Two SystemC memcpy functions were implemented in order to allow for the efficient communication with the Accelerator Wrapper MemCpy SystemC Threads so as to transfer data from the Wrapper Device Memory to the accelerator’s memories.
