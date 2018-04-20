/*
 * Copyright (c) 2018 The Regents of The Technical University of Crete
 * All rights reserved.
 * 
 * ----------------------------------------------------------------------------
 * Buddy Dynamic Memory Allocator
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
 * Accelerator Wrapper device is responsible for
 * the efficient communication and synchronisation of cGEM5
 * with the SystemC accelerator. You can referee in this paper 
 * https://ieeexplore.ieee.org/document/7927071/
 */

#ifndef __DEV_ARM_MY_DEVICE2_HH__
#define __DEV_ARM_MY_DEVICE2_HH__

#include <string>

#include <systemc.h>


#include "SystemCDevice1.hh"

#include "dev/arm/MemoryAllocator.hh"
#include "dev/arm/amba_device.hh"
#include "mem/packet.hh"
#include "params/AccelDevice1.hh"


#include "sim/eventq.hh"

#define DEVICE_MEMORY_SIZE 536870912 //in bytes
#define DEVICE_MEMORY_ENTRY_SIZE 65536


#define DELAY_PER_DMA_MEMORY_ENTRY 0

typedef struct MallocElements{
  char     Name;
  uint32_t Size;
  uint32_t AllocAddr;
}MallocElement;


// Registers used by Device
const uint32_t DMA_DATA_SIZE            = 0x00000;
const uint32_t DMA_TRANSFER_TO_DEVICE   = 0x00008;
const uint32_t DMA_TRANSFER_FROM_DEVICE = 0x00010;
const uint32_t CALL_DEVICE     	        = 0x00018;
const uint32_t DMA_NUMBER_OF_CHUNCK     = 0x00020;
const uint32_t DMA_SWADDR               = 0x00028;
const uint32_t MALLOC_SIZE              = 0x00030;
const uint32_t MALLOC_NAME              = 0x00038;
const uint32_t MALLOC_ADDR              = 0x00040;
const uint32_t FREE_ADDR                = 0x00048;

void * AccelThread1(void * arg);

void SystemCInitialization1();

class AccelDevice1 : public AmbaDmaDevice
{
  protected:
    uint8_t retData8;
    uint16_t retData16;
    uint32_t retData32;
    uint64_t retData64;
  public:
    
    MemoryAllocator *allocator;
    
    uint32_t DMACurrSize;
    uint32_t DMAChunckNumber;
    uint32_t DMASWAddr;
    
    uint32_t MallocSize;
    uint8_t MallocName;
    uint32_t AllocAddr;
    
    double SystemClockTicks;    //! SystemClockTicks is the conversion of System frequency (picosecond Granularity)!//
    
    
    void Synch(); //! Global Synchronization Function !//
    typedef EventWrapper<AccelDevice1, &AccelDevice1::Synch> SynchEvent;
    friend void SynchEvent::process();
    SynchEvent synchEvent;
    
    
    void DMARcvPktComplete();
    EventWrapper<AccelDevice1, &AccelDevice1::DMARcvPktComplete> DMARcvPktEvent;
    
    void DMASendPktComplete();
    EventWrapper<AccelDevice1, &AccelDevice1::DMASendPktComplete> DMASendPktEvent;
    
    typedef AccelDevice1Params Params;
    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    AccelDevice1(const Params *p);
    
    void TimeConversion(const Params *p);

    virtual Tick read(PacketPtr pkt);
    virtual Tick write(PacketPtr pkt);
    
    
    /**
     * Determine the address ranges that this device responds to.
     *
     * @return a list of non-overlapping address ranges
     */
    AddrRangeList getAddrRanges() const;

};


SC_MODULE (ConnectToSystemC1) {
  
  sc_out<bool> clk;
  sc_out<bool> reset;
  
  //Outputs
  //! The request for a new device call
  sc_out<bool> activateSystemCDevice;
  
  //!The acknowledge of a new memcpy to Device
  sc_out<bool>deactivateSystemCMemCpyToDevice;
  
  //!The acknowledge of a new memcpy to Host
  sc_out<bool>deactivateSystemCMemCpyToHost;
    
  sc_out<char>     CHARdata_in;
  sc_out<int>      INTdata_in;
  sc_out<float>    FLOATdata_in;
  sc_out<double>   DOUBLEdata_in;
  sc_out<uint8_t>  UINT8data_in;
  sc_out<uint16_t> UINT16data_in;
  sc_out<uint32_t> UINT32data_in;
  sc_out<uint64_t> UINT64data_in;
  
  
  //Inputs
  
  //!The acknowledge of an accomplished device call
  sc_in<bool>deactivateSystemCDevice;
  
  //! The request for a new memcpy to Device
  sc_in<bool> activateSystemCMemCpyToDevice;
  
  //! The request for a new memcpy to Host
  sc_in<bool> activateSystemCMemCpyToHost;
  
  
  sc_in<char>     CHARdata_out;
  sc_in<int>      INTdata_out;
  sc_in<float>    FLOATdata_out;
  sc_in<double>   DOUBLEdata_out;
  sc_in<uint8_t>  UINT8data_out;
  sc_in<uint16_t> UINT16data_out;
  sc_in<uint32_t> UINT32data_out;
  sc_in<uint64_t> UINT64data_out;
  
  sc_in<char> SWMemNameToDevice; /* Name of SWMemory Label for HostToDevice transfer*/
  sc_in<char> SWMemNameToHost; /* Name of SWMemory Label for DeviceToHost transfer*/
  
  //Size of MemCpy
  sc_in<uint64_t> MemCpySizeToDevice;
  sc_in<uint64_t> MemCpySizeToHost;
  
  //! Start Copy from this Address byte
  sc_in<uint64_t> MemCpyAddrToDevice;
  sc_in<uint64_t> MemCpyAddrToHost;
  
  //Type of MemCpy
  sc_in<uint8_t> MemCpyTypeToDevice;
  sc_in<uint8_t> MemCpyTypeToHost;
  
  
  SC_CTOR (ConnectToSystemC1) {
    SC_THREAD(clk_thread);
    
    SC_THREAD(init_thread);
    sensitive << clk.pos();
    
    //!Process Specification
    SC_CTHREAD(memcpyToDevice_thread, clk.pos());
    async_reset_signal_is(reset, true);
    
    //!Process Specification
    SC_CTHREAD(memcpyToHost_thread, clk.pos());
    async_reset_signal_is(reset, true);
    
  }
  
  void clk_thread();
  
  void init_thread();
  
  void memcpyToDevice_thread();
  
  void memcpyToHost_thread();
    
  
};

#endif // __DEV_ARM_MY_DEVICE2_HH__
