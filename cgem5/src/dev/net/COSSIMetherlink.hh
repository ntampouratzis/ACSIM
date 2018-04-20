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

#ifndef __DEV_COSSIM_ETHERLINK_HH__
#define __DEV_COSSIM_ETHERLINK_HH__

#include "HLA_GEM5.hh"

#include "base/types.hh"
#include "dev/net/etherint.hh"
#include "dev/net/etherobject.hh"
#include "dev/net/etherpkt.hh"
#include "params/COSSIMEtherLink.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"


class EtherDump;
class Checkpoint;
/*
 * Model for a fixed bandwidth full duplex ethernet link
 */
class COSSIMEtherLink : public EtherObject
{
  
  HLA_GEM5 * NodeHLA;
    
  HLA_GEM5 * HLAGlobalSynch;
  
  
  protected:
    class Interface;
    
    int nodeNumber;
    
    double SystemClockTicks;    //! SystemClockTicks is the conversion of System frequency !//
    double SynchTimeTicks; 	//! SynchTimeTicks are the ticks in which the simulator node will be synchronized !//
    double ReceivePacketTicks;	//! ReceivePacketTicks are the ticks in which the simulator node can receive packets !//
        
    int TotalNodes;
    
     /*
      * Model for a single uni-directional link
      */
     public:
    class Link
    {
      public:
        std::string objName;
	
        COSSIMEtherLink *parent;
        int number;
	
        Interface *txint;
        Interface *rxint;

        double ticksPerByte;
        Tick linkDelay;
        Tick delayVar;
        EtherDump *dump;
	
	EthPacketPtr packet;
	

      public:

        void txDone(); //! Send Packet Function !//
        typedef EventWrapper<Link, &Link::txDone> DoneEvent;
        friend void DoneEvent::process();
        DoneEvent doneEvent;
	
	void rxDone(); //! Receive Packet Function !//
        typedef EventWrapper<Link, &Link::rxDone> RxDoneEvent;
        friend void RxDoneEvent::process();
        RxDoneEvent RxdoneEvent;
	
	void Synch(); //! Global Synchronization Function !//
        typedef EventWrapper<Link, &Link::Synch> SynchEvent;
        friend void SynchEvent::process();
        SynchEvent synchEvent;
	

      public:
        Link(const std::string &name, COSSIMEtherLink *p, int num,
             double rate, Tick delay, Tick delay_var, EtherDump *dump);
        ~Link() {}

        const std::string name() const { return objName; }

        bool transmit(EthPacketPtr packet);
	
        void setTxInt(Interface *i) { assert(!txint); txint = i; }
        void setRxInt(Interface *i) { assert(!rxint); rxint = i; }

       
    };

    protected:
    /*
     * Interface at each end of the link
     */
    class Interface : public EtherInt
    {
      private:
        Link *txlink;

      public:
        Interface(const std::string &name, Link *txlink, Link *rxlink);
        bool recvPacket(EthPacketPtr packet) {   
	  return txlink->transmit(packet); } //receive the packet from GEM5 to transmit it
        void sendDone() { peer->sendDone(); }
    };
    
    

    Link *link;
    Interface *interface;

  public:
    typedef COSSIMEtherLinkParams Params;
    COSSIMEtherLink(const Params *p);
    virtual ~COSSIMEtherLink();

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    virtual EtherInt *getEthPort(const std::string &if_name, int idx);
    
    void TimeConversion(const Params *p);
        
    void closeHLA();
        
};

#endif // __COSSIM_ETHERLINK_HH__