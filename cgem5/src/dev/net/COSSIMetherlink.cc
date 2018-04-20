/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * All rights reserved.
 * 
 * ----------------------------------------------------------------------------
 * COSSIM - Etherlink
 * Copyright (c) 2018, H2020 COSSIM.
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
 * Author: Nathan Binkert
 * Author: Tampouratzis Nikolaos, ntampouratzis@isc.tuc.gr
 */

/* @file
 * Device module for modelling a fixed bandwidth full duplex ethernet link
 * and send the Ethernet packets to OMNET++ through HLA
 */

#include <dlfcn.h>

#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <iostream>
#include <string>
#include <unistd.h>

#include "base/callback.hh"
#include "base/random.hh"
#include "base/trace.hh"
#include "debug/Ethernet.hh"
#include "debug/EthernetData.hh"
#include "dev/net/etherdump.hh"
#include "dev/net/etherint.hh"
#include "dev/net/COSSIMetherlink.hh"
#include "dev/net/etherpkt.hh"
#include "params/COSSIMEtherLink.hh"
#include "sim/core.hh"
#include "sim/serialize.hh"
#include "sim/system.hh"



using namespace std;

using std::string ;
using std::cout ;
using std::endl ;


COSSIMEtherLink::COSSIMEtherLink(const Params *p)
    : EtherObject(p), nodeNumber(p->nodeNum), TotalNodes(p->TotalNodes)
{

    TimeConversion(p); 
  
    link = new Link(name() + ".link", this, 0, p->speed, p->delay, p->delay_var, p->dump);
   
    interface = new Interface(name() + ".interface", link, link);
           
    char str_name[30];
    
    /* 1. PROCESSING TO NETWORK HLA INITIALIZATION */
    sprintf(str_name,"COSSIM_PROC_NET_NODE%d",nodeNumber);
    string Sendfederation(str_name); /* PACKETS FROM PROCESSING TO NETWORK */
    printf("\n1. %s\n",Sendfederation.c_str());
    string federate = "GEM5" ;
    string fedfile = "Federation.fed";
    
    
    NodeHLA = new HLA_GEM5(federate,nodeNumber, TotalNodes);
    NodeHLA->HLASendInitialization(Sendfederation,fedfile, false, false);
    /* END PROCESSING TO NETWORK HLA INITIALIZATION */
    
    
    /* HLA GLOBAL SYNCRONIZATION */
    string Synchfederation = "GLOBAL_SYNCHRONIZATION" ;
    
    sprintf(str_name,"GEM5_NODE%d",nodeNumber);
    string Synchfederate(str_name);
    printf("\n2. %s\n",Synchfederate.c_str());
    
    HLAGlobalSynch = new HLA_GEM5(Synchfederate,nodeNumber, TotalNodes);
    HLAGlobalSynch->HLASendInitialization(Synchfederation,fedfile, true, true);
    /* END HLA GLOBAL SYNCRONIZATION */
    
    if(nodeNumber ==0){
      //! Remove HLA Initialization Data !//
      HLAInitializationRequest tmp;
      tmp.type = REMOVE;
      
      strcpy(tmp.name, "OmnetToGem5Signal");
      NodeHLA->RequestFunction(tmp);
      
      strcpy(tmp.name, "Gem5ToOmnetSignal");
      NodeHLA->RequestFunction(tmp);
      
      strcpy(tmp.name, "GlobalSynchSignal");
      NodeHLA->RequestFunction(tmp);

    }
    
    // Register a callback to compensate for the destructor not
    // being called. The callback forces to close the HLA Connection
        Callback* cb = new MakeCallback<COSSIMEtherLink,
            &COSSIMEtherLink::closeHLA>(this);
        registerExitCallback(cb);
          
}


COSSIMEtherLink::~COSSIMEtherLink()
{
    NodeHLA->resign();
    HLAGlobalSynch->resign();
  
    delete link;
    delete interface;
}

void
COSSIMEtherLink::closeHLA()
{
    //! Send Empty message to notify the GEM5 termination !//
    NodeHLA->sendInteraction(NULL,(uint32_t)0);
    NodeHLA->step();
    
    NodeHLA->resign();
    HLAGlobalSynch->resign();

}


EtherInt*
COSSIMEtherLink::getEthPort(const std::string &if_name, int idx)
{
    Interface *i;
    if (if_name == "interface")
        i = interface;
    else
        return NULL;
    if (i->getPeer())
        panic("interface already connected to\n");

    return i;
}


COSSIMEtherLink::Interface::Interface(const string &name, Link *tx, Link *rx)
    : EtherInt(name), txlink(tx)
{
    
    tx->setTxInt(this);
    rx->setRxInt(this);
}

COSSIMEtherLink::Link::Link(const string &name, COSSIMEtherLink *p, int num,
                      double rate, Tick delay, Tick delay_var, EtherDump *d)
    : objName(name), parent(p), number(num), txint(NULL), rxint(NULL),
      ticksPerByte(rate), linkDelay(delay), delayVar(delay_var), dump(d),
      doneEvent(this), RxdoneEvent(this), synchEvent(this)
{ 
  
  if (!synchEvent.scheduled())
    parent->schedule(synchEvent, curTick() + parent->SynchTimeTicks);
  
  if (!RxdoneEvent.scheduled())
      parent->schedule(RxdoneEvent, curTick() + parent->ReceivePacketTicks);
   
}



void
COSSIMEtherLink::Link::Synch()
{
  
  parent->HLAGlobalSynch->step(); //! GLOBAL SYNCHRONIZATION !//
  
  parent->schedule(synchEvent, curTick() + parent->SynchTimeTicks);
}

void
COSSIMEtherLink::Link::rxDone()
{
  
  parent->NodeHLA->step();
  if(parent->NodeHLA->BufferPacketEmpty()){
    parent->schedule(RxdoneEvent, curTick() + parent->ReceivePacketTicks);
    return;
  }
  
  //! **** Receive REAL Packet **** !//
  
  EthPacketPtr Rcvpacket = parent->NodeHLA->getPacket();
  
  if(Rcvpacket->length > 0){ //! Receive Real Packet !//
    
    if (!rxint->sendPacket(Rcvpacket)) {
      printf("Bus busy...Retransmission...\n");
      parent->schedule(RxdoneEvent, curTick() + parent->ReceivePacketTicks);
      return;
    } else {
	if (dump)
	  dump->dump(Rcvpacket);
	parent->NodeHLA->clearRcvPacket();
    }
  }
  else{ //! Receive Empty Packet -- Terminate the HLA Connection !//
    parent->NodeHLA->clearRcvPacket();
    
    parent->NodeHLA->resign();
    parent->HLAGlobalSynch->resign();
    
    panic("GEM5 simulator node %d is stopped by OMNET++ corresponding Node before normal execution\n",parent->nodeNumber);
    
  }
  
  parent->schedule(RxdoneEvent, curTick() + parent->ReceivePacketTicks);
}


void
COSSIMEtherLink::Link::txDone()
{
    if (dump)
        dump->dump(packet);
       
    parent->NodeHLA->sendInteraction(packet->data,(uint32_t)packet->length);
    parent->NodeHLA->step();

    packet = 0;
    txint->sendDone();
}


bool
COSSIMEtherLink::Link::transmit(EthPacketPtr pkt) //TRANSMIT THE PACKET TO OTHER SYSTEM
{
  
    DPRINTF(Ethernet, "packet sent (scheduling...): len=%d\n", pkt->length);
    DDUMP(EthernetData, pkt->data, pkt->length);
    
    packet = pkt;
    
    Tick delay = (Tick)ceil(((double)pkt->length * ticksPerByte) + 1.0);
    if (delayVar != 0)
        delay += random_mt.random<Tick>(0, delayVar);

    
    DPRINTF(Ethernet, "scheduling packet: delay=%d, (rate=%f)\n",
            delay, ticksPerByte);
    parent->schedule(doneEvent, curTick() /*+ delay*/); //delay is calculated through OMNET++

    return true;
}



COSSIMEtherLink *
COSSIMEtherLinkParams::create()
{
    return new COSSIMEtherLink(this);
}


void 
COSSIMEtherLink::TimeConversion(const Params *p){
    
  
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
    
    //! Convert SynchTime to Double //!
    double SynchTime = 0.0;
    const char * SynchTime_str   = p->SynchTime.c_str();
    len = strlen(SynchTime_str);
    const char *last_three2 = &SynchTime_str[len-2];
    char * first_characters2 = strndup (SynchTime_str, len-2);
    if(strcmp("ms",last_three2) == 0){
      SynchTime = (double) atoi(first_characters2) / (double)1000;
    }
    else if(strcmp("us",last_three2) == 0){
      SynchTime = (double) atoi(first_characters2) / (double)1000000;
    }
    else{
      printf("ERROR! Time units cannot be recognized. Please select <ms> or <us> in --SynchTime (i.e. --SynchTime=10ms).\n");
    }
    
    //! SynchTimeTicks are the ticks in which the simulator node will be synchronized !//
    SynchTimeTicks = SynchTime * SystemClockTicks; 
    
    //! Convert RxPacketTime to Double //!
    double RxPacketTime = 0.0;
    const char * RxPacketTime_str   = p->RxPacketTime.c_str();
    len = strlen(RxPacketTime_str);
    const char *last_three3 = &RxPacketTime_str[len-2];
    char * first_characters3 = strndup (RxPacketTime_str, len-2);
    if(strcmp("ms",last_three3) == 0){
      RxPacketTime = (double) atoi(first_characters3) / (double)1000;
    }
    else if(strcmp("us",last_three3) == 0){
      RxPacketTime = (double) atoi(first_characters3) / (double)1000000;
    }
    else{
      printf("ERROR! Time units cannot be recognized. Please select <ms> or <us> in --RxPacketTime (i.e. --RxPacketTime=2ms).\n");
    }
    
    //! ReceivePacketTicks are the ticks in which the simulator node can receive packets !//
    ReceivePacketTicks = RxPacketTime * SystemClockTicks;
    
    
}
