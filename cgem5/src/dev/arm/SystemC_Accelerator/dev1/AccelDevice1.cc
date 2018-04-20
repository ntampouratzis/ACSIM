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

#include "base/trace.hh"
#include "dev/arm/amba_device.hh"
#include "dev/arm/SystemC_Accelerator/dev1/AccelDevice1.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

#include "sim/system.hh"

#include <unistd.h>



using namespace std;

pthread_t tid1;

uint8_t * DeviceMemory1;

uint64_t SynchCounterSystemC1;
uint64_t SynchCounter1;
bool SystemCActivateSim1;

double DEVICE_CLK_PERIOD1;      //! DEVICE_CLK_PERIOD1 is the ticks of GEM5 and SystemC Synchronization (picosecond Granularity) !//
double DEVICE_CLK_PERIOD_FS1;   //! DEVICE_CLK_PERIOD_FS1 is the ticks of Device Clock (femtosecond Granularity) !//

std::deque<MallocElement> aDeque1;

void ConnectToSystemC1::clk_thread() {
  SynchCounterSystemC1 = 0;
  uint64_t counter;
  while (true) {
    counter = 0;
    clk.write(0);
    //cout<<"CLK-\n";
    wait(DEVICE_CLK_PERIOD_FS1/2, SC_FS);
    clk.write(1);
    //cout<<"CLK+\n";
    
    while(SynchCounter1 < SynchCounterSystemC1){
      wait(1, SC_FS);
      counter = counter + 1;
    }
    assert(counter < (DEVICE_CLK_PERIOD_FS1/2));
    wait((DEVICE_CLK_PERIOD_FS1/2) - counter, SC_FS);
    SynchCounterSystemC1 = SynchCounterSystemC1 + 1;
  }
}


void ConnectToSystemC1::memcpyToDevice_thread(){
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
    
    for(i=0;i<aDeque1.size();i++){
      elem = aDeque1[i];
      if(SWMemNameToDevice.read() == elem.Name)
	break;
    }
    if(i==aDeque1.size()){
      panic("ERROR(ConnectToSystemC1::memcpyToDevice_thread()): Memory: %c is not declared by SW Application",SWMemNameToDevice.read());
    }
    
    for(uint64_t i=0;i<MemCpySizeToDevice.read();i++){

      switch (MemCpyTypeToDevice.read()) {
	case ACC_CHAR:
	    
	    memcpy(&tmp1, DeviceMemory1+(elem.AllocAddr+((MemCpyAddrToDevice.read()+i)*sizeof(char))), sizeof(char));
	    CHARdata_in.write(tmp1);
	  break;
	case ACC_INT:
	    memcpy(&tmp2, DeviceMemory1+(elem.AllocAddr+((MemCpyAddrToDevice.read()+i)*sizeof(int))), sizeof(int));
	    INTdata_in.write(tmp2);
	  break;
	  case ACC_FLOAT:
	    memcpy(&tmp3, DeviceMemory1+(elem.AllocAddr+((MemCpyAddrToDevice.read()+i)*sizeof(float))), sizeof(float));
	    FLOATdata_in.write(tmp3);
	  break;
	  case ACC_DOUBLE:
	    memcpy(&tmp4, DeviceMemory1+(elem.AllocAddr+((MemCpyAddrToDevice.read()+i)*sizeof(double))), sizeof(double));
	    DOUBLEdata_in.write(tmp4);
	  break;
	  case ACC_UINT8_T:
	    memcpy(&tmp5, DeviceMemory1+(elem.AllocAddr+((MemCpyAddrToDevice.read()+i)*sizeof(uint8_t))), sizeof(uint8_t));
	    UINT8data_in.write(tmp5);
	  break;
	  case ACC_UINT16_T:
	    memcpy(&tmp6, DeviceMemory1+(elem.AllocAddr+((MemCpyAddrToDevice.read()+i)*sizeof(uint16_t))), sizeof(uint16_t));
	    UINT16data_in.write(tmp6);
	  break;
	  case ACC_UINT32_T:
	    memcpy(&tmp7, DeviceMemory1+(elem.AllocAddr+((MemCpyAddrToDevice.read()+i)*sizeof(uint32_t))), sizeof(uint32_t));
	    UINT32data_in.write(tmp7);
	  break;
	  case ACC_UINT64_T:
	    memcpy(&tmp8, DeviceMemory1+(elem.AllocAddr+((MemCpyAddrToDevice.read()+i)*sizeof(uint64_t))), sizeof(uint64_t));
	    UINT64data_in.write(tmp8);
	  break;
	default:  
	  panic("ConnectToSystemC1::memcpyToDevice_thread(): Unknown Element Type!");
      }
      wait();
    }    

    while(activateSystemCMemCpyToDevice.read() == 1){
      wait();
    }
    deactivateSystemCMemCpyToDevice.write(0);
    
    cout<<"SystemC Device MemCpyToDevice completed in time: "<< sc_time_stamp() <<endl;

    wait();
  }
  
}



void ConnectToSystemC1::memcpyToHost_thread(){
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
    
    for(i=0;i<aDeque1.size();i++){
      elem = aDeque1[i];
      if(SWMemNameToHost.read() == elem.Name)
	break;
    }
    if(i==aDeque1.size()){
      panic("ERROR(ConnectToSystemC1::memcpyToHost_thread()): Memory: %c is not declared by SW Application",SWMemNameToHost.read());
    }
    
    for(uint64_t i=0;i<MemCpySizeToHost.read();i++){
      
      switch (MemCpyTypeToHost.read()) {
	case ACC_CHAR:
	    tmp1 = CHARdata_out.read();
	    memcpy(DeviceMemory1+(elem.AllocAddr+((MemCpyAddrToHost.read()+i)*sizeof(char))),&tmp1,  sizeof(char));
	  break;
	case ACC_INT:
	    tmp2 = INTdata_out.read();
	    memcpy(DeviceMemory1+(elem.AllocAddr+((MemCpyAddrToHost.read()+i)*sizeof(int))),&tmp2,  sizeof(int));
	  break;
	  case ACC_FLOAT:
	    tmp3 = FLOATdata_out.read();
	    memcpy(DeviceMemory1+(elem.AllocAddr+((MemCpyAddrToHost.read()+i)*sizeof(float))),&tmp3,  sizeof(float));
	  break;
	  case ACC_DOUBLE:
	    tmp4 = DOUBLEdata_out.read();
	    memcpy(DeviceMemory1+(elem.AllocAddr+((MemCpyAddrToHost.read()+i)*sizeof(double))),&tmp4,  sizeof(double));
	  break;
	  case ACC_UINT8_T:
	    tmp5 = UINT8data_out.read();
	    memcpy(DeviceMemory1+(elem.AllocAddr+((MemCpyAddrToHost.read()+i)*sizeof(uint8_t))),&tmp5,  sizeof(uint8_t));
	  break;
	  case ACC_UINT16_T:
	    tmp6 = UINT16data_out.read();
	    memcpy(DeviceMemory1+(elem.AllocAddr+((MemCpyAddrToHost.read()+i)*sizeof(uint16_t))),&tmp6,  sizeof(uint16_t));
	  break;
	  case ACC_UINT32_T:
	    tmp7 = UINT32data_out.read();
	    memcpy(DeviceMemory1+(elem.AllocAddr+((MemCpyAddrToHost.read()+i)*sizeof(uint32_t))),&tmp7,  sizeof(uint32_t));
	  break;
	  case ACC_UINT64_T:
	    tmp8 = UINT64data_out.read();
	    memcpy(DeviceMemory1+(elem.AllocAddr+((MemCpyAddrToHost.read()+i)*sizeof(uint64_t))),&tmp8,  sizeof(uint64_t));
	  break;
	default:  
	  panic("ConnectToSystemC1::memcpyToHost_thread(): Unknown Element Type!");
      }
      wait();
    }   
    
    while(activateSystemCMemCpyToHost.read() == 1){
      wait();
    }
    deactivateSystemCMemCpyToHost.write(0);
    
    cout<<"SystemC Device MemCpyToHost completed in time: "<< sc_time_stamp() <<endl;

    wait();
  }
  
}


void ConnectToSystemC1::init_thread() {
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
    SystemCActivateSim1 = 0;
  }
}


sc_signal<bool> activateSystemCDevice1;
sc_signal<bool> deactivateSystemCDevice1;

sc_signal<bool> activateSystemCMemCpyToDevice1;
sc_signal<bool> deactivateSystemCMemCpyToDevice1;

sc_signal<bool> activateSystemCMemCpyToHost1;
sc_signal<bool> deactivateSystemCMemCpyToHost1;


sc_signal<char>     CHARdata_in1;
sc_signal<int>      INTdata_in1;
sc_signal<float>    FLOATdata_in1;
sc_signal<double>   DOUBLEdata_in1;
sc_signal<uint8_t>  UINT8data_in1;
sc_signal<uint16_t> UINT16data_in1;
sc_signal<uint32_t> UINT32data_in1;
sc_signal<uint64_t> UINT64data_in1;

sc_signal<char>     CHARdata_out1;
sc_signal<int>      INTdata_out1;
sc_signal<float>    FLOATdata_out1;
sc_signal<double>   DOUBLEdata_out1;
sc_signal<uint8_t>  UINT8data_out1;
sc_signal<uint16_t> UINT16data_out1;
sc_signal<uint32_t> UINT32data_out1;
sc_signal<uint64_t> UINT64data_out1;

sc_signal<char> SWMemNameToDevice1; /* Name of SWMemory Label for HostToDevice transfer*/
sc_signal<char> SWMemNameToHost1; /* Name of SWMemory Label for DeviceToHost transfer*/


sc_signal<uint64_t> MemCpyAddrToDevice1;
sc_signal<uint64_t> MemCpySizeToDevice1;
  
sc_signal<uint64_t> MemCpyAddrToHost1;
sc_signal<uint64_t> MemCpySizeToHost1;

sc_signal<uint8_t> MemCpyTypeToDevice1;
sc_signal<uint8_t> MemCpyTypeToHost1;

    
sc_signal<bool> clk1;
sc_signal<bool> reset1;

ConnectToSystemC1 *wrapper1;
SystemCDevice1 *dev1;

void SystemCInitialization1(){
  
  wrapper1 = new ConnectToSystemC1("SYSTEMC_WRAPPER");
  
  wrapper1->clk(clk1);
  wrapper1->reset(reset1);
  wrapper1->activateSystemCDevice(activateSystemCDevice1);
  wrapper1->deactivateSystemCDevice(deactivateSystemCDevice1);
  wrapper1->activateSystemCMemCpyToDevice(activateSystemCMemCpyToDevice1);
  wrapper1->deactivateSystemCMemCpyToDevice(deactivateSystemCMemCpyToDevice1);
  wrapper1->activateSystemCMemCpyToHost(activateSystemCMemCpyToHost1);
  wrapper1->deactivateSystemCMemCpyToHost(deactivateSystemCMemCpyToHost1);
  
  wrapper1->CHARdata_in(CHARdata_in1);
  wrapper1->INTdata_in(INTdata_in1);
  wrapper1->FLOATdata_in(FLOATdata_in1);
  wrapper1->DOUBLEdata_in(DOUBLEdata_in1);
  wrapper1->UINT8data_in(UINT8data_in1);
  wrapper1->UINT16data_in(UINT16data_in1);
  wrapper1->UINT32data_in(UINT32data_in1);
  wrapper1->UINT64data_in(UINT64data_in1);
  
  wrapper1->CHARdata_out(CHARdata_out1);
  wrapper1->INTdata_out(INTdata_out1);
  wrapper1->FLOATdata_out(FLOATdata_out1);
  wrapper1->DOUBLEdata_out(DOUBLEdata_out1);
  wrapper1->UINT8data_out(UINT8data_out1);
  wrapper1->UINT16data_out(UINT16data_out1);
  wrapper1->UINT32data_out(UINT32data_out1);
  wrapper1->UINT64data_out(UINT64data_out1);

  wrapper1->SWMemNameToDevice(SWMemNameToDevice1);
  wrapper1->SWMemNameToHost(SWMemNameToHost1);
  
  wrapper1->MemCpyAddrToDevice(MemCpyAddrToDevice1);
  wrapper1->MemCpySizeToDevice(MemCpySizeToDevice1);
  wrapper1->MemCpyAddrToHost(MemCpyAddrToHost1);
  wrapper1->MemCpySizeToHost(MemCpySizeToHost1);
  wrapper1->MemCpyTypeToDevice(MemCpyTypeToDevice1);
  wrapper1->MemCpyTypeToHost(MemCpyTypeToHost1);
  
  
  dev1 = new SystemCDevice1("SYSTEMC_DEVICE");
  
  dev1->clk(clk1);
  dev1->reset(reset1);
  dev1->activateSystemCDevice(activateSystemCDevice1);
  dev1->deactivateSystemCDevice(deactivateSystemCDevice1);
  dev1->activateSystemCMemCpyToDevice(activateSystemCMemCpyToDevice1);
  dev1->deactivateSystemCMemCpyToDevice(deactivateSystemCMemCpyToDevice1);
  dev1->activateSystemCMemCpyToHost(activateSystemCMemCpyToHost1);
  dev1->deactivateSystemCMemCpyToHost(deactivateSystemCMemCpyToHost1);
  
  dev1->CHARdata_in(CHARdata_in1);
  dev1->INTdata_in(INTdata_in1);
  dev1->FLOATdata_in(FLOATdata_in1);
  dev1->DOUBLEdata_in(DOUBLEdata_in1);
  dev1->UINT8data_in(UINT8data_in1);
  dev1->UINT16data_in(UINT16data_in1);
  dev1->UINT32data_in(UINT32data_in1);
  dev1->UINT64data_in(UINT64data_in1);
  
  dev1->CHARdata_out(CHARdata_out1);
  dev1->INTdata_out(INTdata_out1);
  dev1->FLOATdata_out(FLOATdata_out1);
  dev1->DOUBLEdata_out(DOUBLEdata_out1);
  dev1->UINT8data_out(UINT8data_out1);
  dev1->UINT16data_out(UINT16data_out1);
  dev1->UINT32data_out(UINT32data_out1);
  dev1->UINT64data_out(UINT64data_out1);
  
  dev1->SWMemNameToDevice(SWMemNameToDevice1);
  dev1->SWMemNameToHost(SWMemNameToHost1);
  
  dev1->MemCpyAddrToDevice(MemCpyAddrToDevice1);
  dev1->MemCpySizeToDevice(MemCpySizeToDevice1);
  dev1->MemCpyAddrToHost(MemCpyAddrToHost1);
  dev1->MemCpySizeToHost(MemCpySizeToHost1);
  dev1->MemCpyTypeToDevice(MemCpyTypeToDevice1);
  dev1->MemCpyTypeToHost(MemCpyTypeToHost1);
  
}




void * AccelThread1(void * arg){   
    sc_start();
    return 0;
}

AccelDevice1::AccelDevice1(const Params *p)
    : AmbaDmaDevice(p), synchEvent(this), DMARcvPktEvent(this), DMASendPktEvent(this)
{
    
    if (p->dev_clk.compare("None") != 0){
      TimeConversion(p);
      pioSize = p->pio_size;
      DeviceMemory1 = (uint8_t*) malloc (DEVICE_MEMORY_SIZE);
      DMACurrSize     = 0;
      DMAChunckNumber = 0;
      MallocSize      = 0;
      MallocName      = 0;
      
      SystemCInitialization1();
      
      allocator = new MemoryAllocator(); //! Initialize the MemoryAllocator !//
      
      SynchCounter1 = 0;
      SystemCActivateSim1 = 0;
    }
            
}


void
AccelDevice1::Synch()
{
  if(SystemCActivateSim1){
    if(SynchCounter1 > SynchCounterSystemC1){ //! Check if GEM5 clock clycle is greater than SystemC clock Cycle !//
      schedule(synchEvent, curTick());
    }
    else{ //! Check if SystemC clock clycle is greater than GEM5 clock Cycle !//
      //printf("Synch with tick: %ld and counter: %ld and SystemCCounter: %ld\n",curTick(),SynchCounter1,SynchCounterSystemC1);
      schedule(synchEvent, curTick() + DEVICE_CLK_PERIOD1);
      SynchCounter1++;
    }
  }
  else{
   gic->sendInt(intNum); //# Send Interrupt when the SystemC Device is completed #
  }
}


Tick
AccelDevice1::read(PacketPtr pkt)
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
AccelDevice1::DMARcvPktComplete()
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
AccelDevice1::DMASendPktComplete()
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
AccelDevice1::write(PacketPtr pkt)
{
    int err;
    uint32_t bus_addr;
    MallocElement elem;
    uint32_t FreeAddr;
    
    pkt->makeAtomicResponse();    
	    switch (pkt->getAddr() - pioAddr) {
		case CALL_DEVICE:
		    if(!synchEvent.scheduled()){
		      SystemCActivateSim1 = 1;
		      schedule(synchEvent, curTick());
		      err = pthread_create(&tid1, NULL, &AccelThread1, NULL);
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
		     aDeque1.push_back(elem);
		    break;
		case FREE_ADDR:
		     FreeAddr = pkt->get<uint32_t>();
		     for(int i=0;i<aDeque1.size();i++){
			elem = aDeque1[i];
			if(FreeAddr == elem.AllocAddr){
			  aDeque1.erase (aDeque1.begin()+i);
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
		    dmaRead((Addr) bus_addr, DMACurrSize, &DMARcvPktEvent, DeviceMemory1 + DMASWAddr + (DMAChunckNumber*DEVICE_MEMORY_ENTRY_SIZE), DELAY_PER_DMA_MEMORY_ENTRY);
		    break;
		case DMA_TRANSFER_FROM_DEVICE:
		    bus_addr = pkt->get<uint32_t>();		    
		    dmaWrite((Addr) bus_addr, DMACurrSize, &DMASendPktEvent, DeviceMemory1 + DMASWAddr + (DMAChunckNumber*DEVICE_MEMORY_ENTRY_SIZE), DELAY_PER_DMA_MEMORY_ENTRY);
		    break;
		default:  
		    panic("Device: Unrecognized Device Option!\n");
	    }
           
    return pioDelay;
}


void 
AccelDevice1::TimeConversion(const Params *p){
    
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
    DEVICE_CLK_PERIOD1 = DeviceClockTicks * SystemClockTicks;
    DEVICE_CLK_PERIOD_FS1 = DEVICE_CLK_PERIOD1*1000;
}

AddrRangeList
AccelDevice1::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(RangeSize(pioAddr, pioSize));
    return ranges;
}


AccelDevice1 *
AccelDevice1Params::create()
{
    return new AccelDevice1(this);
}
