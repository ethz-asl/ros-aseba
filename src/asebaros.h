#ifndef __ASEBA_ROS_H
#define __ASEBA_ROS_H

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "asebaros/LoadScripts.h"
#include "asebaros/GetNodeList.h"
#include "asebaros/GetNodeId.h"
#include "asebaros/GetNodeName.h"
#include "asebaros/GetVariableList.h"
#include "asebaros/SetVariable.h"
#include "asebaros/GetVariable.h"
#include "asebaros/SendEventById.h"
#include "asebaros/SendEventByName.h"
#include "asebaros/CreateEventFilter.h"

#include "asebaros/ListenEventById.h"
#include "asebaros/ListenEventByName.h"
#include "asebaros/IgnoreEventById.h"
#include "asebaros/IgnoreEventByName.h"
#include "asebaros/FreeEventFilter.h"

#include "asebaros/AsebaEvent.h"

#include <vector>

using namespace asebaros;

typedef std::vector<ros::ServiceServer> ServiceServers;

class EventFilter
{
protected:
	ros::NodeHandle n;
	ServiceServers s;

protected:
	bool listenEventById(ListenEventById::Request& req, ListenEventById::Response& res);
	bool listenEventByName(ListenEventByName::Request& req, ListenEventByName::Response& res);
	bool ignoreEventById(IgnoreEventById::Request& req, IgnoreEventById::Response& res);
	bool ignoreEventByName(IgnoreEventByName::Request& req, IgnoreEventByName::Response& res);
	bool freeEventFilter(FreeEventFilter::Request& req, FreeEventFilter::Response& res);
	
public:
	EventFilter(unsigned number);
};

class AsebaROS
{
protected:
	typedef std::vector<EventFilter> EventFilters;
	
	ros::NodeHandle n;
	ServiceServers s;
	
	EventFilters eventFilters;
	
protected:
	bool loadScript(LoadScripts::Request& req, LoadScripts::Response& res);
	
	bool getNodeList(GetNodeList::Request& req, GetNodeList::Response& res);
	bool getNodeId(GetNodeId::Request& req, GetNodeId::Response& res);
	bool getNodeName(GetNodeName::Request& req, GetNodeName::Response& res);
	
	bool getVariableList(GetVariableList::Request& req, GetVariableList::Response& res);
	bool setVariable(SetVariable::Request& req, SetVariable::Response& res);
	bool getVariable(GetVariable::Request& req, GetVariable::Response& res);
	
	bool sendEventById(SendEventById::Request& req, SendEventById::Response& res);
	bool sendEventByName(SendEventByName::Request& req, SendEventByName::Response& res);
	bool createEventFilter(CreateEventFilter::Request& req, CreateEventFilter::Response& res);
	
public:
	AsebaROS();
};

#endif // __ASEBA_ROS_H
