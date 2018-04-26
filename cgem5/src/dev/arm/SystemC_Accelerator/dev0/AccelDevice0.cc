/*
 * Copyright (c) 2018 The Regents of The Technical University of Crete
 * All rights reserved.
 * 
 * ----------------------------------------------------------------------------
 * Accelerator Wrapper
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

#include "base/trace.hh"
#include "dev/arm/amba_device.hh"
#include "dev/arm/SystemC_Accelerator/dev0/AccelDevice0.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

#include "sim/system.hh"

#include <unistd.h>



using namespace std;

pthread_t tid0;

uint8_t * DeviceMemory0;

uint64_t SynchCounterSystemC0;
uint64_t SynchCounter0;
bool SystemCActivateSim0;

double DEVICE_CLK_PERIOD0;      //! DEVICE_CLK_PERIOD0 is the ticks of GEM5 and SystemC Synchronization (picosecond Granularity) !//
double DEVICE_CLK_PERIOD_FS0;   //! DEVICE_CLK_PERIOD_FS0 is the ticks of Device Clock (femtosecond Granularity) !//

std::deque<MallocElement> aDeque0;

void ConnectToSystemC0::clk_thread() {
  SynchCounterSystemC0 = 0;
  uint64_t counter;
  while (true) {
    counter = 0;
    clk.write(0);
    //cout<<"CLK-\n";
    wait(DEVICE_CLK_PERIOD_FS0/2, SC_FS);
    clk.write(1);
    //cout<<"CLK+\n";
    
    while(SynchCounter0 < SynchCounterSystemC0){
      wait(1, SC_FS);
      counter = counter + 1;
    }
    assert(counter < (DEVICE_CLK_PERIOD_FS0/2));
    wait((DEVICE_CLK_PERIOD_FS0/2) - counter, SC_FS);
    SynchCounterSystemC0 = SynchCounterSystemC0 + 1;
  }
}


void ConnectToSystemC0::memcpyToDevice_thread(){
  //! Reset the signals driven by the thread. 
  deactivateSystemCMemCpyToDevice.write(0);
  
  char tmp1;int tmp2;float tmp3;double tmp4;
  uint8_t tmp5;uint16_t tmp6;uint32_t tmp7;uint64_t tmp8;
  
  int i;
  MallocElement elem;
  elem.AllocAddr = 0;
  elem.Name      = 0;
  elem.Size      = 0;
  
  wait();
 
  //!Main Thread Loop
  while(1){

    while(activateSystemCMemCpyToDevice.read() == 0){
      wait();
    }
    deactivateSystemCMemCpyToDevice.write(1);
    
    for(i=0;i<aDeque0.size();i++){
      elem = aDeque0[i];
      if(SWMemNameToDevice.read() == elem.Name)
	break;
    }
    if(i==aDeque0.size()){
      panic("ERROR(ConnectToSystemC0::memcpyToDevice_thread()): Memory: %c is not declared by SW Application",SWMemNameToDevice.read());
    }
    
    for(uint64_t i=0;i<MemCpySizeToDevice.read();i++){

      switch (MemCpyTypeToDevice.read()) {
	case ACC_CHAR:
	    
	    memcpy(&tmp1, DeviceMemory0+(elem.AllocAddr+((MemCpyAddrToDevice.read()+i)*sizeof(char))), sizeof(char));
	    CHARdata_in.write(tmp1);
	  break;
	case ACC_INT:
	    memcpy(&tmp2, DeviceMemory0+(elem.AllocAddr+((MemCpyAddrToDevice.read()+i)*sizeof(int))), sizeof(int));
	    INTdata_in.write(tmp2);
	  break;
	  case ACC_FLOAT:
	    memcpy(&tmp3, DeviceMemory0+(elem.AllocAddr+((MemCpyAddrToDevice.read()+i)*sizeof(float))), sizeof(float));
	    FLOATdata_in.write(tmp3);
	  break;
	  case ACC_DOUBLE:
	    memcpy(&tmp4, DeviceMemory0+(elem.AllocAddr+((MemCpyAddrToDevice.read()+i)*sizeof(double))), sizeof(double));
	    DOUBLEdata_in.write(tmp4);
	  break;
	  case ACC_UINT8_T:
	    memcpy(&tmp5, DeviceMemory0+(elem.AllocAddr+((MemCpyAddrToDevice.read()+i)*sizeof(uint8_t))), sizeof(uint8_t));
	    UINT8data_in.write(tmp5);
	  break;
	  case ACC_UINT16_T:
	    memcpy(&tmp6, DeviceMemory0+(elem.AllocAddr+((MemCpyAddrToDevice.read()+i)*sizeof(uint16_t))), sizeof(uint16_t));
	    UINT16data_in.write(tmp6);
	  break;
	  case ACC_UINT32_T:
	    memcpy(&tmp7, DeviceMemory0+(elem.AllocAddr+((MemCpyAddrToDevice.read()+i)*sizeof(uint32_t))), sizeof(uint32_t));
	    UINT32data_in.write(tmp7);
	  break;
	  case ACC_UINT64_T:
	    memcpy(&tmp8, DeviceMemory0+(elem.AllocAddr+((MemCpyAddrToDevice.read()+i)*sizeof(uint64_t))), sizeof(uint64_t));
	    UINT64data_in.write(tmp8);
	  break;
	default:  
	  panic("ConnectToSystemC0::memcpyToDevice_thread(): Unknown Element Type!");
      }
      wait();
    }    

    while(activateSystemCMemCpyToDevice.read() == 1){
      wait();
    }
    deactivateSystemCMemCpyToDevice.write(0);
    
    //cout<<"SystemC Device MemCpyToDevice completed in time: "<< sc_time_stamp() <<endl;

    wait();
  }
  
}



void ConnectToSystemC0::memcpyToHost_thread(){
  //! Reset the signals driven by the thread. 
  deactivateSystemCMemCpyToHost.write(0);
  
  char tmp1;int tmp2;float tmp3;double tmp4;
  uint8_t tmp5;uint16_t tmp6;uint32_t tmp7;uint64_t tmp8;
  
  int i;
  MallocElement elem;
  elem.AllocAddr = 0;
  elem.Name      = 0;
  elem.Size      = 0;
  
  wait();
 
  //!Main Thread Loop
  while(1){

    while(activateSystemCMemCpyToHost.read() == 0){
      wait();
    }
    deactivateSystemCMemCpyToHost.write(1);
    wait(2);
    
    for(i=0;i<aDeque0.size();i++){
      elem = aDeque0[i];
      if(SWMemNameToHost.read() == elem.Name)
	break;
    }
    if(i==aDeque0.size()){
      panic("ERROR(ConnectToSystemC0::memcpyToHost_thread()): Memory: %c is not declared by SW Application",SWMemNameToHost.read());
    }
    
    for(uint64_t i=0;i<MemCpySizeToHost.read();i++){
      
      switch (MemCpyTypeToHost.read()) {
	case ACC_CHAR:
	    tmp1 = CHARdata_out.read();
	    memcpy(DeviceMemory0+(elem.AllocAddr+((MemCpyAddrToHost.read()+i)*sizeof(char))),&tmp1,  sizeof(char));
	  break;
	case ACC_INT:
	    tmp2 = INTdata_out.read();
	    memcpy(DeviceMemory0+(elem.AllocAddr+((MemCpyAddrToHost.read()+i)*sizeof(int))),&tmp2,  sizeof(int));
	  break;
	  case ACC_FLOAT:
	    tmp3 = FLOATdata_out.read();
	    memcpy(DeviceMemory0+(elem.AllocAddr+((MemCpyAddrToHost.read()+i)*sizeof(float))),&tmp3,  sizeof(float));
	  break;
	  case ACC_DOUBLE:
	    tmp4 = DOUBLEdata_out.read();
	    memcpy(DeviceMemory0+(elem.AllocAddr+((MemCpyAddrToHost.read()+i)*sizeof(double))),&tmp4,  sizeof(double));
	  break;
	  case ACC_UINT8_T:
	    tmp5 = UINT8data_out.read();
	    memcpy(DeviceMemory0+(elem.AllocAddr+((MemCpyAddrToHost.read()+i)*sizeof(uint8_t))),&tmp5,  sizeof(uint8_t));
	  break;
	  case ACC_UINT16_T:
	    tmp6 = UINT16data_out.read();
	    memcpy(DeviceMemory0+(elem.AllocAddr+((MemCpyAddrToHost.read()+i)*sizeof(uint16_t))),&tmp6,  sizeof(uint16_t));
	  break;
	  case ACC_UINT32_T:
	    tmp7 = UINT32data_out.read();
	    memcpy(DeviceMemory0+(elem.AllocAddr+((MemCpyAddrToHost.read()+i)*sizeof(uint32_t))),&tmp7,  sizeof(uint32_t));
	  break;
	  case ACC_UINT64_T:
	    tmp8 = UINT64data_out.read();
	    memcpy(DeviceMemory0+(elem.AllocAddr+((MemCpyAddrToHost.read()+i)*sizeof(uint64_t))),&tmp8,  sizeof(uint64_t));
	  break;
	default:  
	  panic("ConnectToSystemC0::memcpyToHost_thread(): Unknown Element Type!");
      }
      wait();
    }   
    
    while(activateSystemCMemCpyToHost.read() == 1){
      wait();
    }
    deactivateSystemCMemCpyToHost.write(0);
    
    //cout<<"SystemC Device MemCpyToHost completed in time: "<< sc_time_stamp() <<endl;

    wait();
  }
  
}


void ConnectToSystemC0::init_thread() {
  while(1){
    reset.write(0);
    wait();
    reset.write(1);
    wait();
    reset.write(0);
    wait();
    
    //! Call the Device !//
    activateSystemCDevice.write(1);
    while(deactivateSystemCDevice.read() == 0){wait();}
    activateSystemCDevice.write(0);
    while(deactivateSystemCDevice.read() == 1){wait();}
    
    sc_pause();
    SystemCActivateSim0 = 0;
  }
}


sc_signal<bool> activateSystemCDevice0;
sc_signal<bool> deactivateSystemCDevice0;

sc_signal<bool> activateSystemCMemCpyToDevice0;
sc_signal<bool> deactivateSystemCMemCpyToDevice0;

sc_signal<bool> activateSystemCMemCpyToHost0;
sc_signal<bool> deactivateSystemCMemCpyToHost0;


sc_signal<char>     CHARdata_in0;
sc_signal<int>      INTdata_in0;
sc_signal<float>    FLOATdata_in0;
sc_signal<double>   DOUBLEdata_in0;
sc_signal<uint8_t>  UINT8data_in0;
sc_signal<uint16_t> UINT16data_in0;
sc_signal<uint32_t> UINT32data_in0;
sc_signal<uint64_t> UINT64data_in0;

sc_signal<char>     CHARdata_out0;
sc_signal<int>      INTdata_out0;
sc_signal<float>    FLOATdata_out0;
sc_signal<double>   DOUBLEdata_out0;
sc_signal<uint8_t>  UINT8data_out0;
sc_signal<uint16_t> UINT16data_out0;
sc_signal<uint32_t> UINT32data_out0;
sc_signal<uint64_t> UINT64data_out0;

sc_signal<char> SWMemNameToDevice0; /* Name of SWMemory Label for HostToDevice transfer*/
sc_signal<char> SWMemNameToHost0; /* Name of SWMemory Label for DeviceToHost transfer*/


sc_signal<uint64_t> MemCpyAddrToDevice0;
sc_signal<uint64_t> MemCpySizeToDevice0;
  
sc_signal<uint64_t> MemCpyAddrToHost0;
sc_signal<uint64_t> MemCpySizeToHost0;

sc_signal<uint8_t> MemCpyTypeToDevice0;
sc_signal<uint8_t> MemCpyTypeToHost0;

    
sc_signal<bool> clk0;
sc_signal<bool> reset0;

ConnectToSystemC0 *wrapper0;
SystemCDevice0 *dev0;

void SystemCInitialization0(){
  
  wrapper0 = new ConnectToSystemC0("SYSTEMC_WRAPPER");
  
  wrapper0->clk(clk0);
  wrapper0->reset(reset0);
  wrapper0->activateSystemCDevice(activateSystemCDevice0);
  wrapper0->deactivateSystemCDevice(deactivateSystemCDevice0);
  wrapper0->activateSystemCMemCpyToDevice(activateSystemCMemCpyToDevice0);
  wrapper0->deactivateSystemCMemCpyToDevice(deactivateSystemCMemCpyToDevice0);
  wrapper0->activateSystemCMemCpyToHost(activateSystemCMemCpyToHost0);
  wrapper0->deactivateSystemCMemCpyToHost(deactivateSystemCMemCpyToHost0);
  
  wrapper0->CHARdata_in(CHARdata_in0);
  wrapper0->INTdata_in(INTdata_in0);
  wrapper0->FLOATdata_in(FLOATdata_in0);
  wrapper0->DOUBLEdata_in(DOUBLEdata_in0);
  wrapper0->UINT8data_in(UINT8data_in0);
  wrapper0->UINT16data_in(UINT16data_in0);
  wrapper0->UINT32data_in(UINT32data_in0);
  wrapper0->UINT64data_in(UINT64data_in0);
  
  wrapper0->CHARdata_out(CHARdata_out0);
  wrapper0->INTdata_out(INTdata_out0);
  wrapper0->FLOATdata_out(FLOATdata_out0);
  wrapper0->DOUBLEdata_out(DOUBLEdata_out0);
  wrapper0->UINT8data_out(UINT8data_out0);
  wrapper0->UINT16data_out(UINT16data_out0);
  wrapper0->UINT32data_out(UINT32data_out0);
  wrapper0->UINT64data_out(UINT64data_out0);

  wrapper0->SWMemNameToDevice(SWMemNameToDevice0);
  wrapper0->SWMemNameToHost(SWMemNameToHost0);
  
  wrapper0->MemCpyAddrToDevice(MemCpyAddrToDevice0);
  wrapper0->MemCpySizeToDevice(MemCpySizeToDevice0);
  wrapper0->MemCpyAddrToHost(MemCpyAddrToHost0);
  wrapper0->MemCpySizeToHost(MemCpySizeToHost0);
  wrapper0->MemCpyTypeToDevice(MemCpyTypeToDevice0);
  wrapper0->MemCpyTypeToHost(MemCpyTypeToHost0);
  
  
  dev0 = new SystemCDevice0("SYSTEMC_DEVICE");
  
  dev0->clk(clk0);
  dev0->reset(reset0);
  dev0->activateSystemCDevice(activateSystemCDevice0);
  dev0->deactivateSystemCDevice(deactivateSystemCDevice0);
  dev0->activateSystemCMemCpyToDevice(activateSystemCMemCpyToDevice0);
  dev0->deactivateSystemCMemCpyToDevice(deactivateSystemCMemCpyToDevice0);
  dev0->activateSystemCMemCpyToHost(activateSystemCMemCpyToHost0);
  dev0->deactivateSystemCMemCpyToHost(deactivateSystemCMemCpyToHost0);
  
  dev0->CHARdata_in(CHARdata_in0);
  dev0->INTdata_in(INTdata_in0);
  dev0->FLOATdata_in(FLOATdata_in0);
  dev0->DOUBLEdata_in(DOUBLEdata_in0);
  dev0->UINT8data_in(UINT8data_in0);
  dev0->UINT16data_in(UINT16data_in0);
  dev0->UINT32data_in(UINT32data_in0);
  dev0->UINT64data_in(UINT64data_in0);
  
  dev0->CHARdata_out(CHARdata_out0);
  dev0->INTdata_out(INTdata_out0);
  dev0->FLOATdata_out(FLOATdata_out0);
  dev0->DOUBLEdata_out(DOUBLEdata_out0);
  dev0->UINT8data_out(UINT8data_out0);
  dev0->UINT16data_out(UINT16data_out0);
  dev0->UINT32data_out(UINT32data_out0);
  dev0->UINT64data_out(UINT64data_out0);
  
  dev0->SWMemNameToDevice(SWMemNameToDevice0);
  dev0->SWMemNameToHost(SWMemNameToHost0);
  
  dev0->MemCpyAddrToDevice(MemCpyAddrToDevice0);
  dev0->MemCpySizeToDevice(MemCpySizeToDevice0);
  dev0->MemCpyAddrToHost(MemCpyAddrToHost0);
  dev0->MemCpySizeToHost(MemCpySizeToHost0);
  dev0->MemCpyTypeToDevice(MemCpyTypeToDevice0);
  dev0->MemCpyTypeToHost(MemCpyTypeToHost0);
  
}




void * AccelThread0(void * arg){   
    sc_start();
    return 0;
}

AccelDevice0::AccelDevice0(const Params *p)
    : AmbaDmaDevice(p), synchEvent(this), DMARcvPktEvent(this), DMASendPktEvent(this)
{
    
    if (p->dev_clk.compare("None") != 0){
      TimeConversion(p);
      pioSize = p->pio_size;
      DeviceMemory0 = (uint8_t*) malloc (DEVICE_MEMORY_SIZE);
      DMACurrSize     = 0;
      DMAChunckNumber = 0;
      MallocSize      = 0;
      MallocName      = 0;
      
      SystemCInitialization0();
      
      allocator = new MemoryAllocator(); //! Initialize the MemoryAllocator !//
      
      SynchCounter0 = 0;
      SystemCActivateSim0 = 0;
    }
            
}


void
AccelDevice0::Synch()
{
  if(SystemCActivateSim0){
    if(SynchCounter0 > SynchCounterSystemC0){ //! Check if GEM5 clock clycle is greater than SystemC clock Cycle !//
      schedule(synchEvent, curTick());
    }
    else{ //! Check if SystemC clock clycle is greater than GEM5 clock Cycle !//
      //printf("Synch with tick: %ld and counter: %ld and SystemCCounter: %ld\n",curTick(),SynchCounter0,SynchCounterSystemC0);
      schedule(synchEvent, curTick() + DEVICE_CLK_PERIOD0);
      SynchCounter0++;
    }
  }
  else{
   gic->sendInt(intNum); //# Send Interrupt when the SystemC Device is completed #
  }
}


Tick
AccelDevice0::read(PacketPtr pkt)
{
    pkt->makeAtomicResponse();
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);   
    
    switch (pkt->getAddr() - pioAddr) {
        case MALLOC_ADDR:
	  pkt->set(AllocAddr);
	  break;
        default:
          panic("Device: Unrecognized Device Option!\n");
    }
    return pioDelay;
}


void
AccelDevice0::DMARcvPktComplete()
{
  /*printf("\n----- DMA Read Completed!!! -----");
  printf("The INPUT data:\n");
  for(int i=0;i<DMACurrSize;i++){
      printf("%d ",DeviceMemory[DMASWAddr + (DMAChunckNumber*DEVICE_MEMORY_ENTRY_SIZE)+i]);
  }
  printf("\n");*/
    
  gic->sendInt(intNum); //# Send Interrupt if the DMA Read Completed #
}

void
AccelDevice0::DMASendPktComplete()
{
  /*printf("\n----- DMA Write Completed!!! -----");
  printf("The OUTPUT data:\n");
  for(int i=0;i<DMACurrSize;i++){
    printf("%d ",DeviceMemory[DMASWAddr + (DMAChunckNumber*DEVICE_MEMORY_ENTRY_SIZE)+i]);
  }
  printf("\n");*/
    
  gic->sendInt(intNum); //# Send Interrupt if the DMA Write Completed #
}

Tick
AccelDevice0::write(PacketPtr pkt)
{
    int err;
    uint32_t bus_addr;
    MallocElement elem;
    uint32_t FreeAddr;
    
    pkt->makeAtomicResponse();    
	    switch (pkt->getAddr() - pioAddr) {
		case CALL_DEVICE:
		    if(!synchEvent.scheduled()){
		      SystemCActivateSim0 = 1;
		      schedule(synchEvent, curTick());
		      err = pthread_create(&tid0, NULL, &AccelThread0, NULL);
		      if (err != 0)
			printf("\ncan't create thread :[%s]", strerror(err));
		    }
		    else{
		      printf("The device is already in use!!\n");
		    }
		    break;
		case MALLOC_SIZE:
		     MallocSize = pkt->get<uint32_t>();
		    break;
		case MALLOC_NAME:
		     MallocName = pkt->get<uint8_t>();
		     assert(MallocSize>0);
		     AllocAddr       = (uint32_t) allocator->alloc_thread(MallocSize);
		     elem.AllocAddr  = AllocAddr;
		     elem.Name       = (char) MallocName;
		     elem.Size       = (uint32_t) MallocSize;
		     aDeque0.push_back(elem);
		    break;
		case FREE_ADDR:
		     FreeAddr = pkt->get<uint32_t>();
		     for(int i=0;i<aDeque0.size();i++){
			elem = aDeque0[i];
			if(FreeAddr == elem.AllocAddr){
			  aDeque0.erase (aDeque0.begin()+i);
			  break;
			}
		      }
		      allocator->free_thread((unsigned long) elem.Size, (unsigned long) FreeAddr);
		    break;
		case DMA_DATA_SIZE:
		     DMACurrSize = pkt->get<uint32_t>();
		    break;
		case DMA_NUMBER_OF_CHUNCK:
		     DMAChunckNumber = pkt->get<uint32_t>();
		    break;
		case DMA_SWADDR:
		     DMASWAddr = pkt->get<uint32_t>();
		    break;    
		case DMA_TRANSFER_TO_DEVICE:
		    bus_addr = pkt->get<uint32_t>();		    
		    dmaRead((Addr) bus_addr, DMACurrSize, &DMARcvPktEvent, DeviceMemory0 + DMASWAddr + (DMAChunckNumber*DEVICE_MEMORY_ENTRY_SIZE), DELAY_PER_DMA_MEMORY_ENTRY);
		    break;
		case DMA_TRANSFER_FROM_DEVICE:
		    bus_addr = pkt->get<uint32_t>();		    
		    dmaWrite((Addr) bus_addr, DMACurrSize, &DMASendPktEvent, DeviceMemory0 + DMASWAddr + (DMAChunckNumber*DEVICE_MEMORY_ENTRY_SIZE), DELAY_PER_DMA_MEMORY_ENTRY);
		    break;
		default:  
		    panic("Device: Unrecognized Device Option!\n");
	    }
           
    return pioDelay;
}


void 
AccelDevice0::TimeConversion(const Params *p){
    
    //! Convert SystemClockTicks to Double //!  
    SystemClockTicks = 0.0;
    const char * sys_clock_str = p->sys_clk.c_str();
    int len = strlen(sys_clock_str);
    const char *last_three = &sys_clock_str[len-3];
    char * first_characters = strndup (sys_clock_str, len-3);
    if(strcmp("GHz",last_three) == 0){
      SystemClockTicks = (double) atoi(first_characters)* (double)1000000000 * (double)p->ticksPerNanoSecond;
    }
    else if(strcmp("MHz",last_three) == 0){
      SystemClockTicks = (double) atoi(first_characters)* (double)1000000 * (double)p->ticksPerNanoSecond;
    }
    
    
    
    double DeviceNanoSec = 0.0;
    const char * device_clock_str = p->dev_clk.c_str();
    int len2 = strlen(device_clock_str);
    const char *last_three2 = &device_clock_str[len2-3];
    char * first_characters2 = strndup (device_clock_str, len2-3);
    if(strcmp("GHz",last_three2) == 0){
      DeviceNanoSec = (1/(double) atoi(first_characters2));
    }
    else if(strcmp("MHz",last_three2) == 0){
      DeviceNanoSec = (1/(double) atoi(first_characters2))*1000;
    }
    else{
      panic("ERROR! Time units cannot be recognized. Please select <MHz> or <GHz> in --DeviceClock (i.e. --DeviceClock=200MHz).\n");
    }
    double DeviceClockTicks = DeviceNanoSec / (double)1000000000;
    DEVICE_CLK_PERIOD0 = DeviceClockTicks * SystemClockTicks;
    DEVICE_CLK_PERIOD_FS0 = DEVICE_CLK_PERIOD0*1000;
}

AddrRangeList
AccelDevice0::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(RangeSize(pioAddr, pioSize));
    return ranges;
}


AccelDevice0 *
AccelDevice0Params::create()
{
    return new AccelDevice0(this);
}
