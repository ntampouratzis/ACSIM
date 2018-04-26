/*
 * Copyright (c) 2018 The Regents of The Technical University of Crete
 * All rights reserved.
 * 
 * ----------------------------------------------------------------------------
 * Reference Accelerator Module
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
 * Reference accelerator module has been developed, in SystemC, in order to evaluate 
 * the Accelerator Wrapper and the Linux Kernel Drivers; this module, which 
 * is also provided in open-source, can also act as a reference for the designers/users 
 * that will develop their own SystemC accelerators. You can referee in this paper 
 * https://ieeexplore.ieee.org/document/7927071/
 */

#include "SystemCDevice0.hh"

//! MemCpy From Host To Device !//
void SystemCDevice0::memcpyToDevice(void * MEM, char label,uint64_t SWOffset, uint64_t size, uint8_t ElementType){
  
  char * CharPtr; int * IntPtr;float * FloatPtr;double * DoublePtr;
  uint8_t * UINT8Ptr;uint16_t * UINT16Ptr;uint32_t * UINT32Ptr;uint64_t * UINT64Ptr;
  
  //Check if SWMEM serving another transaction
  while(deactivateSystemCMemCpyToDevice.read() == 1){wait();}  
  
  SWMemNameToDevice.write(label);
  MemCpyAddrToDevice.write(SWOffset); //Start Copy from this Address byte
  MemCpySizeToDevice.write(size);   //Copy MemCpySizeToDevice bytes
  MemCpyTypeToDevice.write(ElementType);
  
  activateSystemCMemCpyToDevice.write(1);
  while(deactivateSystemCMemCpyToDevice.read() == 0){wait();}
  for(uint64_t i=0;i< MemCpySizeToDevice;i++){
    switch (ElementType) {
	case ACC_CHAR:
	    CharPtr = (char *)MEM;
	    CharPtr[i] = CHARdata_in.read(); 
	  break;
	case ACC_INT:
	    IntPtr = (int *)MEM;
	    IntPtr[i] = INTdata_in.read(); 
	  break;
	  case ACC_FLOAT:
	    FloatPtr = (float *)MEM;
	    FloatPtr[i] = FLOATdata_in.read(); 
	  break;
	  case ACC_DOUBLE:
	    DoublePtr = (double *)MEM;
	    DoublePtr[i] = DOUBLEdata_in.read(); 
	  break;
	  case ACC_UINT8_T:
	    UINT8Ptr = (uint8_t *) MEM;
	    UINT8Ptr[i] = UINT8data_in.read(); 
	  break;
	  case ACC_UINT16_T:
	    UINT16Ptr = (uint16_t *) MEM;
	    UINT16Ptr[i] = UINT16data_in.read();
	  break;
	  case ACC_UINT32_T:
	    UINT32Ptr = (uint32_t *) MEM;
	    UINT32Ptr[i] = UINT32data_in.read();
	  break;
	  case ACC_UINT64_T:
	    UINT64Ptr = (uint64_t *) MEM;
	    UINT64Ptr[i] = UINT64data_in.read();
	  break;
      }
    wait();
  }
  activateSystemCMemCpyToDevice.write(0);
  while(deactivateSystemCMemCpyToDevice.read() == 1){wait();}
}


//! MemCpy From Device To Host !//
void SystemCDevice0::memcpyToHost(void * MEM, char label,uint64_t SWOffset, uint64_t size, uint8_t ElementType){
  
  char * CharPtr; int * IntPtr;float * FloatPtr;double * DoublePtr;
  uint8_t * UINT8Ptr;uint16_t * UINT16Ptr;uint32_t * UINT32Ptr;uint64_t * UINT64Ptr;
  
  //Check if SWMEM serving another transaction
  while(deactivateSystemCMemCpyToHost.read() == 1){wait();}  
  
  SWMemNameToHost.write(label);
  MemCpyAddrToHost.write(SWOffset); //Start Copy from this Address byte
  MemCpySizeToHost.write(size);   //Copy MemCpySizeToDevice bytes
  MemCpyTypeToHost.write(ElementType);
  activateSystemCMemCpyToHost.write(1);
  while(deactivateSystemCMemCpyToHost.read() == 0){wait();}
  for(uint64_t i=0;i< MemCpySizeToHost;i++){
    switch (ElementType) {
	case ACC_CHAR:
	    CharPtr = (char *)MEM;
	    CHARdata_out.write(CharPtr[i]);
	  break;
	case ACC_INT:
	    IntPtr = (int *)MEM;
	    INTdata_out.write(IntPtr[i]);
	  break;
	  case ACC_FLOAT:
	    FloatPtr = (float *)MEM;
	    FLOATdata_out.write(FloatPtr[i]);
	  break;
	  case ACC_DOUBLE:
	    DoublePtr = (double *)MEM;
	    DOUBLEdata_out.write(DoublePtr[i]);
	  break;
	  case ACC_UINT8_T:
	    UINT8Ptr = (uint8_t *) MEM;
	    UINT8data_out.write(UINT8Ptr[i]);
	  break;
	  case ACC_UINT16_T:
	    UINT16Ptr = (uint16_t *) MEM;
	    UINT16data_out.write(UINT16Ptr[i]);
	  break;
	  case ACC_UINT32_T:
	    UINT32Ptr = (uint32_t *) MEM;
	    UINT32data_out.write(UINT32Ptr[i]);
	  break;
	  case ACC_UINT64_T:
	    UINT64Ptr = (uint64_t *) MEM;
	    UINT64data_out.write(UINT64Ptr[i]);
	  break;
      }
    wait();
  }
  activateSystemCMemCpyToHost.write(0);
  while(deactivateSystemCMemCpyToHost.read() == 1){wait();}
}


void SystemCDevice0::main_thread(){
  //! Reset the signals driven by the thread. 
  deactivateSystemCDevice.write(0);
  
  
  wait();
 
  //!Main Thread Loop
  while(1){

    while(activateSystemCDevice.read() == 0){
      wait();
    }
    deactivateSystemCDevice.write(1);

    //! SystemC Implementation !//
    
    cout<<"SystemC Device processing in time: "<< sc_time_stamp() <<endl;
    
    uint64_t size   = 18; //Copy 18 elements
    memcpyToDevice(DEVICE_MEMORY,'A', 0, size, ACC_INT);
    
    for(uint32_t i=0;i<size;i++){
      cout<<"DEVICE_MEMORY["<<i<<"]: "<<(int)DEVICE_MEMORY[i] <<" in time: "<< sc_time_stamp() <<endl;
      wait();
    }
    
    int sum = 0;
    for(uint32_t i=0;i<size;i++){
      sum = sum + DEVICE_MEMORY[i];
      wait();
    }
    
    cout<<"sum: "<<(int)sum <<" in time: "<< sc_time_stamp() <<endl;
    
    DEVICE_MEMORY[0] = sum;
    
    wait(1000000);
    
    memcpyToHost(DEVICE_MEMORY,'A', 0, size, ACC_INT);
    
    
    //! END SystemC Implementation !//
    
    while(activateSystemCDevice.read() == 1){
      wait();
    }
    deactivateSystemCDevice.write(0);
    
    cout<<"SystemC Device processing completed in time: "<< sc_time_stamp() <<endl;

    wait();
  }
  
}
