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


#include <systemc.h>

#ifndef _SYSTEMC_DEVICE_HH
#define _SYSTEMC_DEVICE_HH


#define MEM_SIZE 1024


#define ACC_CHAR 1
#define ACC_INT 2
#define ACC_FLOAT 3
#define ACC_DOUBLE 4
#define ACC_UINT8_T 5
#define ACC_UINT16_T 6
#define ACC_UINT32_T 7
#define ACC_UINT64_T 8

SC_MODULE(SystemCDevice1) {
  
  //! Create one DEVICE_MEMORY
  int DEVICE_MEMORY[MEM_SIZE];
  
  //Inputs

  //! Global Clock
  sc_in<bool>     clk;

  //! Global Reset
  sc_in<bool>     reset;

  
  sc_in<char>     CHARdata_in;
  sc_in<int>      INTdata_in;
  sc_in<float>    FLOATdata_in;
  sc_in<double>   DOUBLEdata_in;
  sc_in<uint8_t>  UINT8data_in;
  sc_in<uint16_t> UINT16data_in;
  sc_in<uint32_t> UINT32data_in;
  sc_in<uint64_t> UINT64data_in;
  
  //! The request for a new device call
  sc_in<bool> activateSystemCDevice;
  
  
  //!The acknowledge of a new memcpy to Device
  sc_in<bool>deactivateSystemCMemCpyToDevice;
  
  //!The acknowledge of a new memcpy to Host
  sc_in<bool>deactivateSystemCMemCpyToHost;
  
  
  //Outputs
  
  //!The acknowledge of an accomplished device call
  sc_out<bool>deactivateSystemCDevice;
  
  //! The request for a new memcpy to Device
  sc_out<bool> activateSystemCMemCpyToDevice;
  
  //! The request for a new memcpy to Host
  sc_out<bool> activateSystemCMemCpyToHost;
  
  sc_out<char>     CHARdata_out;
  sc_out<int>      INTdata_out;
  sc_out<float>    FLOATdata_out;
  sc_out<double>   DOUBLEdata_out;
  sc_out<uint8_t>  UINT8data_out;
  sc_out<uint16_t> UINT16data_out;
  sc_out<uint32_t> UINT32data_out;
  sc_out<uint64_t> UINT64data_out;
  
  sc_out<char> SWMemNameToDevice; /* Name of SWMemory Label for HostToDevice transfer*/
  sc_out<char> SWMemNameToHost; /* Name of SWMemory Label for DeviceToHost transfer*/
  
  //Size of MemCpy
  sc_out<uint64_t> MemCpySizeToDevice;
  sc_out<uint64_t> MemCpySizeToHost;
  
  //! Start Copy from this Address byte
  sc_out<uint64_t> MemCpyAddrToDevice;
  sc_out<uint64_t> MemCpyAddrToHost;
  
  //Type of MemCpy
  sc_out<uint8_t> MemCpyTypeToDevice;
  sc_out<uint8_t> MemCpyTypeToHost;
  
  SC_CTOR(SystemCDevice1)
  {

    //!Process Specification
    SC_CTHREAD(main_thread, clk.pos());
    //!Reset Explicitely Declared
    /*!
     *The reset signal should be explicitely declared. It should also
     *be declared as active high (true) or active low (false).
     */
    async_reset_signal_is(reset, true);

  }
  
  //!Process declaration
  void main_thread();
  
  
  //! SWOffset is in the ElementType Offset !//
  void memcpyToDevice(void * MEM, char label,uint64_t SWOffset, uint64_t size, uint8_t ElementType);
  
  //! SWOffset is in the ElementType Offset !//
  void memcpyToHost(void * MEM, char label,uint64_t SWOffset, uint64_t size, uint8_t ElementType);
  
    

}; 
#endif
