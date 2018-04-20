// ----------------------------------------------------------------------------
// COSSIM - GEM5 HLA RunTime Infrastructure
// Copyright (c) 2018, H2020 COSSIM.
// Copyright (c) 2018, Telecommunications Systems Institute.
// Author: Tampouratzis Nikolaos, ntampouratzis@isc.tuc.gr
// ----------------------------------------------------------------------------

#include <config.h>

#include "HLA_GEM5.hh"
#include "PrettyDebug.hh"
#include "MessageBuffer.hh"

#ifndef _MSC_VER
#include <unistd.h>
#endif


#include <memory>
#include <iostream>
#include <cstdlib>
#include <cassert>

using std::auto_ptr ;
using std::string ;
using std::endl ;
using std::cout ;
using std::vector ;

static PrettyDebug D("HLA_GEM5", __FILE__);

// ----------------------------------------------------------------------------
/** Constructor
 */
HLA_GEM5::HLA_GEM5(std::string federate_name, int node, int _TotalNodes)
    : rtiamb(federate_name, node, _TotalNodes),
      federateName(federate_name),
      Node(node),
      TotalNodes(_TotalNodes),
      handle(0),
      creator(false),
      nbTicks(0),
      regulating(false),
      constrained(false),
      localTime(0.0),
      TIME_STEP(1.0)
{
}

// ----------------------------------------------------------------------------
/** Destructor
 */
HLA_GEM5::~HLA_GEM5()
    throw (RTI::FederateInternalError)
{
}

// ============================================================================
//          ------------ INITIALIZATION FUNCTIONS ------------
// ============================================================================

void 
HLA_GEM5::HLASendInitialization(std::string federation, std::string fedfile, bool start_constrained, bool start_regulating){
  
  HLAInitializationRequest tmp;
  
  //! Wait until OMNET federation will be joined !//
  while(1){
    tmp.type = READ;
    strcpy(tmp.name, "OmnetToGem5Signal");
    tmp.node = Node;
    bool ret = RequestFunction(tmp);
    if(ret){ break;}
  }
  
  // Joins federation
  this->join(federation, fedfile);
  
  // Continue initialisation...
  this->pause();
  this->publishAndSubscribeSend();
  this->publishAndSubscribeReceive();
  
  this->setTimeRegulation(start_constrained, start_regulating);
  this->tick();
  
  /* Write "1" in Node+1 line of Gem5ToOmnetSignal Array */
  tmp.type = WRITE;
  strcpy(tmp.name, "Gem5ToOmnetSignal");
  tmp.node = Node;
  RequestFunction(tmp);
  
  if (federation.compare("GLOBAL_SYNCHRONIZATION") == 0){
      /* Write "1" in Node+1 line of GlobalSynchSignal Array */
      tmp.type = WRITE;
      strcpy(tmp.name, "GlobalSynchSignal");
      tmp.node = Node;
      RequestFunction(tmp);

    }
  
  
  this->synchronize();
  
}


/** Get the federate handle
 */
RTI::FederateHandle
HLA_GEM5::getHandle() const
{
    return handle ;
}

// ----------------------------------------------------------------------------
/** Join function is responsible for creating a Federation (if one does not exists) and join the Federate to this Federation. 
    \param federation_name Federation name
    \param fdd_name Federation designator (.fed file)
 */
void
HLA_GEM5::join(std::string federation_name, std::string fdd_name)
{
    federationName = federation_name ;

    // create federation
    try {
        rtiamb.createFederationExecution(federation_name.c_str(),
                                         fdd_name.c_str());
        D.Out(pdInit, "Federation execution created.");
        creator = true ;
    }
    catch (RTI::FederationExecutionAlreadyExists& e) {
        printf("HLA_GEM5 Note : %s Reason is : %s. OK I can join it\n",e._name,e._reason);
        D.Out(pdInit, "Federation execution already created.");
    }
    catch (RTI::CouldNotOpenFED& e) {
        printf("HLA_GEM5 ERROR : %s Reason is : %s\n",e._name,e._reason);
        D.Out(pdExcept, "HLA_GEM5 : Could not use FED file.");
        delete &rtiamb ;
        exit(0);
    }

    // join federation
    bool joined = false ;
    int nb = 5 ;

    while (!joined && nb > 0) {
        nb-- ;
        try {
            handle = rtiamb.joinFederationExecution(federateName.c_str(),
                                                    federation_name.c_str(),
                                                    this);
            joined = true ;
            break ;
        }
        catch (RTI::FederateAlreadyExecutionMember& e) {
            Debug(D, pdExcept) << "Federate " << federateName.c_str()
                        << "already exists." << endl ;

            throw ;
        }
        catch (RTI::FederationExecutionDoesNotExist& e) {
            D.Out(pdExcept, "Federate %s : FederationExecutionDoesNotExist.",
                  federateName.c_str());
        }
        catch (RTI::Exception& e) {
            D.Out(pdExcept,
                  "Federate %s :Join Federation Execution failed : %d .",
                  federateName.c_str(), &e);
            throw ;
        }
    }
}



// ----------------------------------------------------------------------------
/** Resign is related to some services of Declaration management and Federation 
 * management corresponding to correct termination (e.g. resignation) of federates 
 * and the destruction of a federation. 
 */
void
HLA_GEM5::resign()
{
  
    setTimeRegulation(false, false);

    try {
        rtiamb.resignFederationExecution(
            RTI::DELETE_OBJECTS_AND_RELEASE_ATTRIBUTES);
        D.Out(pdTerm, "Just resigned from federation");
    }
    catch (RTI::Exception &e) {
        D.Out(pdExcept,
              "** Exception during resignFederationExecution by federate");
    }
    
    // Destruction of the federation
    if (creator) {
        for (;;) {
            tick();
            try {
                D.Out(pdTerm, "Asking from federation destruction...");
                rtiamb.destroyFederationExecution(federationName.c_str());

                D.Out(pdTerm, "Federation destruction granted.");
                break ;
            }
            catch (RTI::FederatesCurrentlyJoined) {
                sleep(5);
            }
        }
    }
    D.Out(pdTerm, "Destroying RTIAmbassador and FedAmbassador.");
    D.Out(pdTerm, "Federation terminated.");
}


// ----------------------------------------------------------------------------
/** Carry out publications and subscriptions
 */
void
HLA_GEM5::publishAndSubscribeReceive()
{
    
    NODE_TO_GEM5_ID = rtiamb.getInteractionClassHandle("NODE_TO_GEM5");
    PacketDataToGem5ID = rtiamb.getParameterHandle("PacketDataToGem5", NODE_TO_GEM5_ID);
    PacketLengthToGem5ID = rtiamb.getParameterHandle("PacketLengthToGem5", NODE_TO_GEM5_ID);
    
    rtiamb.subscribeInteractionClass(NODE_TO_GEM5_ID, RTI::RTI_TRUE);

    D.Out(pdInit, "publishAndSubscribeReceive Objects and Interactions published and subscribed.");
}

// ----------------------------------------------------------------------------
/** Carry out publications and subscriptions
 */
void
HLA_GEM5::publishAndSubscribeSend()
{
  
   
    // Interactions 
    NODE_TO_OMNET_ID = rtiamb.getInteractionClassHandle("NODE_TO_OMNET");
    PacketDataToOmnetID = rtiamb.getParameterHandle("PacketDataToOmnet", NODE_TO_OMNET_ID);
    PacketLengthToOmnetID = rtiamb.getParameterHandle("PacketLengthToOmnet", NODE_TO_OMNET_ID);

    rtiamb.publishInteractionClass(NODE_TO_OMNET_ID);
    
    D.Out(pdInit, "publishAndSubscribeSend Objects and Interactions published and subscribed.");
}

// ============================================================================
//          ------------ END INITIALIZATION FUNCTIONS ------------
// ============================================================================


// ============================================================================
//          ------------ INTERACTION FUNCTIONS ------------
// ============================================================================

/* 
 * SendInteraction function is implemented to send Data Packets from Federate to HLA. 
 * Any other federate which join in the same federation can receive the DataPacket 
 * through receiveInteraction HLA service 
 */

void
HLA_GEM5::sendInteraction(uint8_t* PacketDataToOmnet, uint32_t n)
{
  
    libhla::MessageBuffer buffer;
    RTI::ParameterHandleValuePairSet *parameterSet=NULL ;
    
    if(n>0){
      parameterSet = RTI::ParameterSetFactory::create(2);
      
      //! Send the Packet Data !//    
      buffer.reset();
      buffer.write_uint8s(PacketDataToOmnet,n);
      buffer.updateReservedBytes();
      parameterSet->add(PacketDataToOmnetID, static_cast<char*>(buffer(0)), buffer.size());
    }
    else{
      parameterSet = RTI::ParameterSetFactory::create(1);
    }
    
    //! Send the Packet Length !// 
    buffer.reset();
    buffer.write_uint32(n);
    buffer.updateReservedBytes();
    parameterSet->add(PacketLengthToOmnetID, static_cast<char*>(buffer(0)), buffer.size());
      
    
    try {
      rtiamb.sendInteraction(NODE_TO_OMNET_ID, *parameterSet, "");
            
    }
    catch (RTI::Exception& e) {
        std::cout<<"sendInteraction raise exception "<<e._name<<"("<<e._reason<<")"<<std::endl;
        D.Out(pdExcept, "**** Exception sending interaction : %d", &e);
    }

    delete parameterSet ;
}



// ----------------------------------------------------------------------------
/** Callback : receive interaction
 *  ReceiveInteraction function is implemented which inherits the receiveInteraction 
 * callback to decode and read the DataPacket whenever is triggered.
 */

void 
HLA_GEM5::receiveInteraction(RTI::InteractionClassHandle theInteraction, 
			      const RTI::ParameterHandleValuePairSet & theParameters, 
			      const char */*theTag*/) 
	throw (RTI::InteractionClassNotKnown, RTI::InteractionParameterNotKnown, 
	       RTI::FederateInternalError) 
{
    libhla::MessageBuffer buffer;
    RTI::ULong valueLength ;
    uint32_t PacketLengthFromOMNET = 0;
    
    D.Out(pdTrace, "Fed : receiveInteraction");
    if (theInteraction != NODE_TO_GEM5_ID) {
        printf("CALLBACK receiveInteraction : Unknown Interaction received");
        exit(-1);
    }

    D.Out(pdDebug, "receiveInteraction - nb attributs= %d", theParameters.size());
    
    
    EthPacketPtr RcvPacketPtr;
    
    for (unsigned int j = 0 ; j < theParameters.size(); ++j) {
        RTI::ParameterHandle parmHandle = theParameters.getHandle(j);

        valueLength = theParameters.getValueLength(j);
        assert(valueLength>0);
        buffer.resize(valueLength);
        buffer.reset();
        theParameters.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
        buffer.assumeSizeFromReservedBytes();

        if (parmHandle == PacketLengthToGem5ID) {            
	    PacketLengthFromOMNET = buffer.read_uint32();  
	    if(PacketLengthFromOMNET == 0){
              RcvPacketPtr = std::make_shared<EthPacketData>();
            }
        }
        else if (parmHandle == PacketDataToGem5ID) { 
	  if(PacketLengthFromOMNET > 0){
	    RcvPacketPtr = std::make_shared<EthPacketData>(PacketLengthFromOMNET);
	    buffer.read_uint8s(RcvPacketPtr->data, PacketLengthFromOMNET);
	  }
        }
        else {
	    D.Out(pdError, "Unrecognized parameter handle");
        }
    }
    
    RcvPacketPtr->length = PacketLengthFromOMNET;
    
    packetBuffer.push(RcvPacketPtr);
   
}


bool 
HLA_GEM5::BufferPacketEmpty(){
  return packetBuffer.empty();
}


EthPacketPtr 
HLA_GEM5::getPacket(){
  EthPacketPtr packet = packetBuffer.front();
  return packet;
}

void 
HLA_GEM5::clearRcvPacket(){
  packetBuffer.front() = NULL;
  packetBuffer.pop();
}

// ============================================================================
//          ------------ END INTERACTION FUNCTIONS ------------
// ============================================================================




// ============================================================================
//          ------------ SYNCHRONIZATION FUNCTIONS ------------
// ============================================================================

/** Pause function initiates the establishment of a named checkpoint that serves 
 *  to synchronize all federates according to federation-defined semantics. 
 */
void
HLA_GEM5::pause()
{
    if (creator) {
        D.Out(pdInit, "Pause requested");
        try {
            rtiamb.registerFederationSynchronizationPoint("Init", "Waiting all federations.");
        }
        catch (RTI::Exception& e) {
            Debug(D, pdExcept) << "Federate " << federateName
                        << " : Register Synchronization Point failed : %d"
                        << endl ;
        }
    }
}

// ----------------------------------------------------------------------------
/** Creator put federation in pause for synchronization with a friend
 */
void
HLA_GEM5::pause_friend()
{
    if (creator) {
        D.Out(pdInit, "Pause requested for friend");
        try {
             RTI::FederateHandle numfed(0) ;
             RTI::FederateHandleSet *federateSet = RTI::FederateHandleSetFactory::create(1) ;
             cout << "Now we test Register Federation Synchronisation Point on some federates" << endl ;
             cout << "Please enter a federate handle (zero means none)" << endl ;
             cout << "This federate will be synchronized with the creator and not the others" << endl;


             if (numfed != 0)
                 {
                 // We store numfed into the federate set
                 federateSet->add(numfed) ;
                 rtiamb.registerFederationSynchronizationPoint("Friend","Synchro with a friend",
                                                          *federateSet) ;
                 }
        }
        catch (RTI::Exception& e) {
            Debug(D, pdExcept) << "Federate " << federateName
                        << " : Register Synchronization Point failed : %d"
                        << endl ;
        }
    }
}

/** tick the RTI
 */
void
HLA_GEM5::tick()
{
    usleep( 0 ) ;
    rtiamb.tick();
    nbTicks++ ;
}
void
HLA_GEM5::tick2()
{
    rtiamb.tick2();
    nbTicks++ ;
}

// ----------------------------------------------------------------------------
/** Set time regulation (time regulating and time constrained)
    @param start_constrained boolean, if true federate is constrained
    @param start_regulating boolean, if true federate is regulating
    
 *  A regulating federate participates actively in the decisions for the progress 
    of time, while a constrained federate follows the time progress imposed by other 
    federates. 
 */
void
HLA_GEM5::setTimeRegulation(bool start_constrained, bool start_regulating)
{
    D.Out(pdInit, "Time Regulation setup");

    if (start_constrained) {
        if (!constrained) {
            // change from no constrained to constrained
            rtiamb.enableTimeConstrained();
            constrained = true ;
            D.Out(pdInit, "Time Constrained enabled.");
        }
    }
    else {
        if (constrained) {
            // change from constrained to no constrained
            rtiamb.disableTimeConstrained();
            constrained = false ;
            D.Out(pdInit, "Time Constrained disabled.");
        }
    }

    if (start_regulating) {
        if (!regulating) {
            // change from no regulating to regulating
            for (;;) {
                rtiamb.queryFederateTime(localTime);

                try {
                    rtiamb.enableTimeRegulation(localTime, TIME_STEP);
                    regulating = true ;
                    break ;
                }
                catch (RTI::FederationTimeAlreadyPassed) {
                    rtiamb.queryFederateTime(localTime);

                    RTIfedTime requestTime(((RTIfedTime&)localTime).getTime());
                    requestTime += TIME_STEP ;

		    granted = false ;
                    rtiamb.timeAdvanceRequest(requestTime);		    
                    while (!granted) {
                        try {
                            tick();
                        }
                        catch (RTI::RTIinternalError) {
                            printf("RTIinternalError Raised in tick.\n");
                            exit(-1);
                        }
                    }
                }
                catch (RTI::RTIinternalError) {
                    printf("RTIinternalError Raised in setTimeRegulating.\n");
                    exit(-1);
                }
            }
        }
    }
    else {
        if (regulating) {
            // change from regulating to no regulating
            rtiamb.disableTimeRegulation();
            regulating = false ;
        }
    }
}

// ----------------------------------------------------------------------------
/** Synchronize with other federates
 */
void
HLA_GEM5::synchronize()
{
    D.Out(pdInit, "Synchronize");

    if (creator) {
        D.Out(pdInit, "Creator can resume execution...");
        while (!paused)
            try {
		
                D.Out(pdInit, "not paused");
                tick();
            }
            catch (RTI::Exception& e) {
                D.Out(pdExcept, "******** Exception ticking the RTI : %d ", &e);
                throw ;
            }
        D.Out(pdDebug, "paused");

        try {
            rtiamb.synchronizationPointAchieved("Init");
        }
        catch (RTI::Exception& e) {
            D.Out(pdExcept, "**** Exception achieving a synchronization "
                  "point by creator : %d", &e);
        }
       
        while (paused)
            try {
                tick();
            }
            catch (RTI::Exception& e) {
                D.Out(pdExcept, "**** Exception ticking the RTI : %d.", &e);
                throw ;
            }
    }
    else {
        if (!paused) {
            D.Out(pdInit,
                  "Federate not paused: too early");
            while (!paused) {
                try {
                    tick();
                }
                catch (RTI::Exception& e) {
                    D.Out(pdExcept,
                          "******** Exception ticking the RTI : %d.", &e);
                    throw ;
                }
            }
        }
        D.Out(pdInit, "Federate paused");

        try {
            // Federate ends its synchronization.
            rtiamb.synchronizationPointAchieved("Init");
            D.Out(pdInit, "Pause achieved.");
        }
        catch (RTI::Exception& e) {
            D.Out(pdExcept,
                  "**** Exception achieving a synchronization point : %d",
                  &e);
        }

        D.Out(pdInit,
              "Federate waiting end of pause...");
        while (paused) {
            try {
                tick();
            }
            catch (RTI::Exception& e) {
                D.Out(pdExcept, "******** Exception ticking the RTI : %d.", &e);
                throw ;
            }
        }
        D.Out(pdInit, "End of pause");
    }

    D.Out(pdInit, "Federation is synchronized.");
    

}




// ----------------------------------------------------------------------------
/** one simulation step advance
 */
void
HLA_GEM5::step()
{
    granted = false ;

    try {
        rtiamb.queryFederateTime(localTime);
    }
    catch (RTI::Exception& e) {
        D.Out(pdExcept,
              "**** Exception asking for federate local time : ", &e);
    }

    try {
        RTIfedTime time_aux(localTime.getTime()+TIME_STEP.getTime());

        D.Out(pdDebug, "time_aux : %.2f - localtime : %.2f - "
              "timestep : %.2f", time_aux.getTime(),
              ((RTIfedTime&)localTime).getTime(),
              ((RTIfedTime&)TIME_STEP).getTime());
        granted = false ;
        rtiamb.timeAdvanceRequest(time_aux);
    }
    catch (RTI::Exception& e) {
        D.Out(pdExcept, "******* Exception sur timeAdvanceRequest.");
    }

    while (!granted) {
        try {
            tick2();
        }
        catch (RTI::Exception& e) {
            D.Out(pdExcept, "******** Exception ticking the RTI : %d.", &e);
            throw ;
        }
    }

    next_step = (localTime + TIME_STEP);
    
}

// ----------------------------------------------------------------------------
/** Callback announce synchronization point
 */
void
HLA_GEM5::announceSynchronizationPoint(const char *label, const char */*tag*/)
    throw (RTI::FederateInternalError)
{
    if (strcmp(label, "Init") == 0) {
        paused = true ;
        printf("announceSynchronizationPoint\n");
    }
    else if (strcmp(label, "Friend") == 0) {
        std::cout<<"**** I am happy : I have a friend ****"<<std::endl;
        paused = true ;
        D.Out(pdProtocol, "announceSynchronizationPoint (friend).");
    } 
    else {
        cout << "Unexpected synchronization label" << endl ;
        exit(1);
    }
}

// ----------------------------------------------------------------------------
/** Callback : federation synchronized
 */
void
HLA_GEM5::federationSynchronized(const char *label)
    throw (RTI::FederateInternalError)
{
    if (strcmp(label, "Init") == 0) {
        paused = false ;
        D.Out(pdProtocol,
              "CALLBACK : federationSynchronized with label %s", label);
    }
}


// ----------------------------------------------------------------------------
/** Callback : time advance granted
 */
void
HLA_GEM5::timeAdvanceGrant(const RTI::FedTime& theTime)
    throw (RTI::InvalidFederationTime, RTI::TimeAdvanceWasNotInProgress, 
	   RTI::FederateInternalError)
{    
    granted = true ;
    localTime = theTime ;
    D.Out(pdTrace, "Time advanced, local time is now %.2f.",
          localTime.getTime());
}

// ============================================================================
//          ------------ END SYNCHRONIZATION FUNCTIONS ------------
// ============================================================================





// ============================================================================
//          ------------ ATTRIBUTES FUNCTIONS ------------
// ============================================================================


// ----------------------------------------------------------------------------
/** Callback : reflect attribute values with time
 */
void 
HLA_GEM5::reflectAttributeValues(RTI::ObjectHandle theObject, 
			    const RTI::AttributeHandleValuePairSet & theAttributes, 
			    const char * /*theTag*/) 
	throw (RTI::ObjectNotKnown, RTI::AttributeNotKnown, RTI::FederateOwnsAttributes,
	       RTI::FederateInternalError)
{
  
}

// ----------------------------------------------------------------------------
/** Callback : remove object instance
 */
void
HLA_GEM5::removeObjectInstance(RTI::ObjectHandle theObject,
			      const RTI::FedTime &,
			      const char *,
			      RTI::EventRetractionHandle)
    throw (RTI::ObjectNotKnown, RTI::InvalidFederationTime, RTI::FederateInternalError)
{
 
}

// ----------------------------------------------------------------------------
/** Callback : discover object instance
 */
void
HLA_GEM5::discoverObjectInstance(RTI::ObjectHandle theObject,
				RTI::ObjectClassHandle theObjectClass,
				const char */*theObjectName*/)
    throw (RTI::CouldNotDiscover, RTI::ObjectClassNotKnown, 
	   RTI::FederateInternalError)
{
   
}

// ============================================================================
//          ------------ END ATTRIBUTES FUNCTIONS ------------
// ============================================================================


//! --- CERTI INITIALIZATION IP --- !//
bool 
HLA_GEM5::RequestFunction(HLAInitializationRequest rqst){
  int sockfd = 0, n = 0;
  bool ret = false;
  
  struct sockaddr_in serv_addr; 

  if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
      printf("\n Error : Could not create socket \n");
      exit(-1);
  } 

  memset(&serv_addr, '0', sizeof(serv_addr)); 

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(60401); 

  char* pPath = getenv ("CERTI_HOST");
  if (pPath==NULL)
    pPath = (char *) "127.0.0.1";
  
  if(inet_pton(AF_INET, pPath, &serv_addr.sin_addr)<=0)
  {
      printf("\n inet_pton error occured\n");
      exit(-1);
  }

  if( connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
  {
     if(!FirstConnectionWithHLAInitialization){
       printf("\n Error : Connection with HLA Initialization Server (IP:%s) Failed \n",pPath);
       exit(-1);
     }
     else{
       printf("\n Warning : Connection with HLA Initialization Server (IP:%s) Failed.. Retry in 30 secs\n",pPath);
       sleep(30);
       return RequestFunction(rqst);
     }
  }
  else{
    FirstConnectionWithHLAInitialization = true;
  }
    
  n = write(sockfd, (const void *) &rqst, sizeof(rqst)); 
    
  n = read(sockfd, (void *) &ret, sizeof(ret));
  if(n < 0)
    printf("\n Reply error \n");
  
  close(sockfd);
  //! Set the appropriate delay if the server is in localhost or not !//
  if((strcmp(pPath,(char *)"127.0.0.1")!=0)&&((rqst.type == READ)||(rqst.type == READ_GLOBAL))){
    usleep(200000);
  }
  else{
    usleep(10000);
  }
  return ret;
}
//! --- END CERTI INITIALIZATION IP --- !//