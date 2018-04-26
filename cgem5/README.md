# cgem5 - ACSIM-modified gem5 version. 

The cgem5 repository includes the modified gem5 simulator that ipmlements the ACSIM simulator. The ACSIM can be executed either standalone or within COSSIM project. For this reason, cgem5 supports (optionally) interconnection with IEEE HLA interfaces and modifies the network interface so that it can communicate with other cgem5 nodes through a network simulator. It should be noted that cgem5 can be used independently of ACSIM as a standalone package incorporating all the changes that have been integrated to the official GEM5 October 2017 release.

## Differences between cgem5 and official gem5 October 2017 version
The following subsections describes in tandem the modifications and extensions that have been implemented for ACSIM. In addition, this repository contains all the COSSIM-modifications as described in [COSSIM_cgem5](https://github.com/H2020-COSSIM/cgem5) repository.

### Accelerator Wrapper
Our Accelerator Wrapper device was developed in order to achieve efficient communication and synchronisation of GEM5 with the SystemC accelerator. It inherits all GEM5 DMA device characteristics so that full DMA transactions utilizing the full operating system can be performed. In addition, it contains a large Device Memory to store the data from the OS memcpy, simulating the DDR memory found in most of the real systems incorporating PCI-connected FPGA and/or GPU boards. Subsequently, a core containing mixed C++ and SystemC code was implemented for the connection of the GEM5 C++ functions and the accelerator’s SystemC threads.
