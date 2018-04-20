// ----------------------------------------------------------------------------
// COSSIM - GEM5 HLA RunTime Infrastructure
// Copyright (c) 2018, H2020 COSSIM.
// Copyright (c) 2018, Telecommunications Systems Institute.
// Author: Tampouratzis Nikolaos, ntampouratzis@isc.tuc.gr
// ----------------------------------------------------------------------------

#ifndef CERTI_HLA_GEM5_HH
#define CERTI_HLA_GEM5_HH

#include "RTI.hh"
#include "NullFederateAmbassador.hh"
#include "fedtime.hh"
#include "certi.hh"

#include "etherpkt.hh"

#define GetCurrentDir getcwd

#include <vector>

#include <queue>

#include <fstream>
#include <limits>
#include <algorithm>

//! --- CERTI INITIALIZATION IP --- !//
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h> 

//! Server Functions !//
#define CREATE 0
#define REMOVE 1
#define WRITE 2
#define READ 3
#define READ_GLOBAL 4
#define CLOSE_SERVER 5

typedef struct HLAInitializationRequests{
 int type;
 char name[30];
 int  node;
}HLAInitializationRequest;
//! --- END CERTI INITIALIZATION IP --- !//

class HLA_GEM5 : public NullFederateAmbassador
{
public:

    HLA_GEM5(std::string, int, int);
    virtual ~HLA_GEM5() throw (RTI::FederateInternalError);

    void join(std::string, std::string);
    void pause();
    void pause_friend();
    virtual void publishAndSubscribeSend();
    virtual void publishAndSubscribeReceive();
    
    void resign();
    void setTimeRegulation(bool constrained, bool regulating);
    void step();
    void synchronize();
    void tick();
    void tick2();
    void HLASendInitialization(std::string federation, std::string fedfile, bool start_constrained, bool start_regulating);
    virtual EthPacketPtr getPacket();
    virtual void clearRcvPacket();
    virtual bool BufferPacketEmpty();
       
    //! --- CERTI INITIALIZATION IP --- !//
    bool RequestFunction(HLAInitializationRequest rqst);
    bool FirstConnectionWithHLAInitialization;
    //! --- END CERTI INITIALIZATION IP --- !//

    unsigned long ID ; // object handle
    
    std::queue<EthPacketPtr> packetBuffer;
    
    
    RTI::FederateHandle getHandle() const ;

    // Callbacks
    void announceSynchronizationPoint(const char *label, const char *tag)
        throw (RTI::FederateInternalError);

    void federationSynchronized(const char *label)
        throw (RTI::FederateInternalError);

    void timeAdvanceGrant(const RTI::FedTime& theTime)
        throw (RTI::FederateInternalError, RTI::TimeAdvanceWasNotInProgress, 
	       RTI::InvalidFederationTime);

    void discoverObjectInstance(RTI::ObjectHandle theObject,
                                RTI::ObjectClassHandle theObjectClass,
                                const char *theObjectName)
        throw (RTI::FederateInternalError, RTI::ObjectClassNotKnown, RTI::CouldNotDiscover);
	
	
    void reflectAttributeValues(RTI::ObjectHandle, const RTI::AttributeHandleValuePairSet &, const char *) 
	throw (RTI::ObjectNotKnown, RTI::AttributeNotKnown, RTI::FederateOwnsAttributes,
	       RTI::FederateInternalError);


    void receiveInteraction(RTI::InteractionClassHandle theInteraction,
                            const RTI::ParameterHandleValuePairSet& theParameters,
                            const RTI::FedTime& theTime, const char *theTag,
                            RTI::EventRetractionHandle theHandle)
        throw (RTI::InteractionClassNotKnown, RTI::InteractionParameterNotKnown,
               RTI::InvalidFederationTime, RTI::FederateInternalError){ };

    void receiveInteraction(RTI::InteractionClassHandle,
                            const RTI::ParameterHandleValuePairSet &,
                            const char *)
        throw (RTI::InteractionClassNotKnown, RTI::InteractionParameterNotKnown,
	       RTI::FederateInternalError);

    void removeObjectInstance(RTI::ObjectHandle theObject, const RTI::FedTime& theTime,
			      const char *theTag,
			      RTI::EventRetractionHandle theHandle)
	throw (RTI::ObjectNotKnown, RTI::InvalidFederationTime, RTI::FederateInternalError);

    void removeObjectInstance(RTI::ObjectHandle, const char *)
	throw (RTI::ObjectNotKnown, RTI::FederateInternalError) { };

    
    void sendInteraction(uint8_t* , uint32_t);

    void sendUpdate(double, double, int, RTI::ObjectHandle);

    bool getCreator(){return creator;};
    
    

protected:
 
    RTI::RTIambassador rtiamb ;

    std::string federateName ;
    std::string federationName ;
    
    int Node;
    int TotalNodes;
    
    RTI::FederateHandle handle ;
    bool creator ;
    long nbTicks ;

    bool regulating ;
    bool constrained ;
    RTIfedTime localTime ;
    const RTIfedTime TIME_STEP ;
    
    RTIfedTime next_step;
        
    
    bool paused ;
    bool granted ;
   
    
    //Node to OMNET++
    RTI::InteractionClassHandle NODE_TO_OMNET_ID ;
    RTI::ParameterHandle PacketDataToOmnetID ;
    RTI::ParameterHandle PacketLengthToOmnetID ;
    
    //Node to GEM5++
    RTI::InteractionClassHandle NODE_TO_GEM5_ID ;
    RTI::ParameterHandle PacketDataToGem5ID ;
    RTI::ParameterHandle PacketLengthToGem5ID ;

};

#endif // CERTI_HLA_GEM5_HH