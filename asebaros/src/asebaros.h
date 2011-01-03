#ifndef __ASEBA_ROS_H
#define __ASEBA_ROS_H

#include <dashel/dashel.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <msg/msg.h>
#include <msg/descriptions-manager.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "asebaros/LoadScripts.h"
#include "asebaros/GetNodeList.h"
#include "asebaros/GetNodeId.h"
#include "asebaros/GetNodeName.h"
#include "asebaros/GetVariableList.h"
#include "asebaros/SetVariable.h"
#include "asebaros/GetVariable.h"
#include "asebaros/GetEventId.h"
#include "asebaros/GetEventName.h"

#include "asebaros/AsebaEvent.h"
#include "asebaros/AsebaAnonymousEvent.h"

#include <vector>


class AsebaROS;

class AsebaDashelHub: public Dashel::Hub
{
private:
	boost::thread* thread; //! thread for the hub
	AsebaROS* asebaROS; //!< pointer to aseba ROS
	bool forward; //!< should we only forward messages instead of transmit them back to the sender
	
public:
	/*! Creates the hub, listen to TCP on port, and creates a DBus interace.
		@param port port on which to listen for incoming connections
		@param forward should we only forward messages instead of transmit them back to the sender
	*/
	AsebaDashelHub(AsebaROS* asebaROS, unsigned port, bool forward);
	
	/*! Sends a message to Dashel peers.
		Does not delete the message, should be called by the main thread.
		@param message aseba message to send
		@param sourceStream originate of the message, if from Dashel.
	*/
	void sendMessage(Aseba::Message *message, bool doLock, Dashel::Stream* sourceStream = 0);

	//! run the hub
	void operator()();
	//! start the hub thread
	void startThread();
	//! stop the hub thread and wait for its termination
	void stopThread();
	
protected:
	virtual void connectionCreated(Dashel::Stream *stream);
	virtual void incomingData(Dashel::Stream *stream);
	virtual void connectionClosed(Dashel::Stream *stream, bool abnormal);
};

using namespace asebaros;

typedef std::vector<ros::ServiceServer> ServiceServers;
typedef std::vector<ros::Publisher> Publishers;
typedef std::vector<ros::Subscriber> Subscribers;

class AsebaROS: public Aseba::DescriptionsManager
{
protected:
	typedef std::map<std::string, unsigned> NodesNamesMap;
	typedef std::map<std::string, Aseba::Compiler::VariablesMap> UserDefinedVariablesMap;
	class GetVariableQueryKey
	{
	public:
		GetVariableQueryKey(unsigned nodeId, unsigned pos) : nodeId(nodeId), pos(pos) { }
		bool operator<(const GetVariableQueryKey &that) const {
			return (nodeId < that.nodeId && pos < that.pos);
		}
		unsigned nodeId;
		unsigned pos;
		
	};
	struct GetVariableQueryValue
	{
		typedef std::vector<sint16> DataVector;
		DataVector data;
		boost::condition_variable cond;
	};
	typedef std::map<GetVariableQueryKey, GetVariableQueryValue*> GetVariableQueryMap;
	
	ros::NodeHandle n; //!< node handler of this class 
	ServiceServers s; //!< all services of this class
	
	ros::Publisher anonPub; //!< anonymous publisher, for aseba events with no associated name
	ros::Subscriber anonSub; //!< anonymous subscriber, for aseba events with no associated name
	Publishers pubs; //!< publishers for known events
	Subscribers subs; //!< subscribers for known events

	AsebaDashelHub hub; //!< hub is the network interface for dashel peers
	boost::mutex mutex; //!< mutex for protecting accesses from hub
	
	Aseba::CommonDefinitions commonDefinitions; //!< description of aseba constants and events
	NodesNamesMap nodesNames; //!< the name of all nodes
	UserDefinedVariablesMap userDefinedVariablesMap; //!< the name of the user-defined variables
	GetVariableQueryMap getVariableQueries; //!< all get variable queries
	
protected:
	bool loadScript(LoadScripts::Request& req, LoadScripts::Response& res);
	
	bool getNodeList(GetNodeList::Request& req, GetNodeList::Response& res);
	bool getNodeId(GetNodeId::Request& req, GetNodeId::Response& res);
	bool getNodeName(GetNodeName::Request& req, GetNodeName::Response& res);
	
	bool getVariableList(GetVariableList::Request& req, GetVariableList::Response& res);
	bool setVariable(SetVariable::Request& req, SetVariable::Response& res);
	bool getVariable(GetVariable::Request& req, GetVariable::Response& res);
	
	bool getEventId(GetEventId::Request& req, GetEventId::Response& res);
	bool getEventName(GetEventName::Request& req, GetEventName::Response& res);
	
	// utility
	bool getNodePosFromNames(const std::string& nodeName, const std::string& variableName, unsigned& nodeId, unsigned& pos) const;
	void sendEventOnROS(const Aseba::UserMessage* asebaMessage);
	
	// callbacks
	void nodeDescriptionReceived(unsigned nodeId);
	void eventReceived(const AsebaAnonymousEventConstPtr& event);
	void knownEventReceived(const uint16 id, const AsebaEventConstPtr& event);
	
public:
	AsebaROS(unsigned port, bool forward);
	~AsebaROS();
	
	void run();
	
	void processAsebaMessage(Aseba::Message *message);
	
	void connectTarget(const std::string& target) { hub.connect(target); }
};



#endif // __ASEBA_ROS_H
