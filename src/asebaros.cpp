#include "asebaros.h"

#include <vector>
#include <boost/format.hpp>

using namespace asebaros;
using namespace std;
using namespace boost;


bool EventFilter::listenEventById(ListenEventById::Request& req, ListenEventById::Response& res)
{
	return true;
}

bool EventFilter::listenEventByName(ListenEventByName::Request& req, ListenEventByName::Response& res)
{
	return true;
}

bool EventFilter::ignoreEventById(IgnoreEventById::Request& req, IgnoreEventById::Response& res)
{
	return true;
}

bool EventFilter::ignoreEventByName(IgnoreEventByName::Request& req, IgnoreEventByName::Response& res)
{
	return true;
}

bool EventFilter::freeEventFilter(FreeEventFilter::Request& req, FreeEventFilter::Response& res)
{
	return true;
}

EventFilter::EventFilter(unsigned number):
	n(str(format("event_filter_%1%") % number))
{
	s.push_back(n.advertiseService("listen_event_by_id", &EventFilter::listenEventById, this));
	s.push_back(n.advertiseService("listen_event_by_name", &EventFilter::listenEventByName, this));
	s.push_back(n.advertiseService("ignore_event_by_id", &EventFilter::ignoreEventById, this));
	s.push_back(n.advertiseService("ignore_event_by_name", &EventFilter::ignoreEventByName, this));
	s.push_back(n.advertiseService("free", &EventFilter::freeEventFilter, this));
}



bool AsebaROS::loadScript(LoadScripts::Request& req, LoadScripts::Response& res)
{
	return true;
}

bool AsebaROS::getNodeList(GetNodeList::Request& req, GetNodeList::Response& res)
{
	return true;
}

bool AsebaROS::getNodeId(GetNodeId::Request& req, GetNodeId::Response& res)
{
	return true;
}

bool AsebaROS::getNodeName(GetNodeName::Request& req, GetNodeName::Response& res)
{
	return true;
}

bool AsebaROS::getVariableList(GetVariableList::Request& req, GetVariableList::Response& res)
{
	return true;
}

bool AsebaROS::setVariable(SetVariable::Request& req, SetVariable::Response& res)
{
	return true;
}

bool AsebaROS::getVariable(GetVariable::Request& req, GetVariable::Response& res)
{
	return true;
}

bool AsebaROS::sendEventById(SendEventById::Request& req, SendEventById::Response& res)
{
	return true;
}

bool AsebaROS::sendEventByName(SendEventByName::Request& req, SendEventByName::Response& res)
{
	return true;
}

bool AsebaROS::createEventFilter(CreateEventFilter::Request& req, CreateEventFilter::Response& res)
{
	return true;
}

AsebaROS::AsebaROS()
{
	// script
	s.push_back(n.advertiseService("load_script", &AsebaROS::loadScript, this));
	
	// nodes
	s.push_back(n.advertiseService("get_node_list", &AsebaROS::getNodeList, this));
	s.push_back(n.advertiseService("get_node_id", &AsebaROS::getNodeId, this));
	s.push_back(n.advertiseService("get_node_name", &AsebaROS::getNodeName, this));
	
	// variables
	s.push_back(n.advertiseService("get_variable_list", &AsebaROS::getVariableList, this));
	s.push_back(n.advertiseService("set_variable", &AsebaROS::setVariable, this));
	s.push_back(n.advertiseService("get_variable", &AsebaROS::getVariable, this));
	
	// events
	s.push_back(n.advertiseService("send_event_by_id", &AsebaROS::sendEventById, this));
	s.push_back(n.advertiseService("send_event_by_name", &AsebaROS::sendEventByName, this));
	s.push_back(n.advertiseService("create_event_filter", &AsebaROS::createEventFilter, this));
	
	eventFilters.push_back(EventFilter(34));
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "aseba");
	AsebaROS asebaROS;	
	ros::spin();

	return 0;
}



