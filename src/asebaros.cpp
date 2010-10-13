#include "asebaros.h"

#include <compiler/compiler.h>

#include <ros/console.h>

#include <vector>
#include <sstream>
#include <boost/format.hpp>

#include <libxml/parser.h>
#include <libxml/tree.h>


using namespace asebaros;
using namespace std;
using namespace boost;
using namespace Dashel;
using namespace Aseba;

// AsebaDashelHub

AsebaDashelHub::AsebaDashelHub(AsebaROS* asebaROS, unsigned port, bool forward):
	Dashel::Hub(),
	asebaROS(asebaROS),
	forward(forward)
{
	ostringstream oss;
	oss << "tcpin:port=" << port;
	Dashel::Hub::connect(oss.str());
}

void AsebaDashelHub::sendMessage(Message *message, bool doLock, Stream* sourceStream)
{
	// dump if requested
	// TODO: check verbosity
	// if (
	ostringstream oss;
	message->dump(oss);
	ROS_DEBUG_STREAM(oss.str());
	
	// Might be called from the ROS thread, not the Hub thread, need to lock
	if (doLock)
		lock();

	// write on all connected streams
	for (StreamsSet::iterator it = dataStreams.begin(); it != dataStreams.end();++it)
	{
		Stream* destStream(*it);
		
		if ((forward) && (destStream == sourceStream))
			continue;
		
		try
		{
			message->serialize(destStream);
			destStream->flush();
		}
		catch (DashelException e)
		{
			// if this stream has a problem, ignore it for now, and let Hub call connectionClosed later.
			ROS_ERROR("error while writing message");
		}
	}

	if (doLock)
		unlock();
}

void AsebaDashelHub::operator()()
{
	Hub::run();
	//cerr << "hub returned" << endl;
	ros::shutdown();
}

void AsebaDashelHub::startThread()
{
	thread = new boost::thread(ref(*this));
}

void AsebaDashelHub::stopThread()
{
	Hub::stop();
	thread->join();
	delete thread;
	thread = 0;
}

// the following method run in the blocking reception thread
	
void AsebaDashelHub::incomingData(Stream *stream)
{
	// receive message
	Message *message = 0;
	try
	{
		message = Message::receive(stream);
	}
	catch (DashelException e)
	{
		// if this stream has a problem, ignore it for now, and let Hub call connectionClosed later.
		ROS_ERROR("error while writing message");
	}
	
	// send message to Dashel peers
	sendMessage(message, false, stream);
	
	// process message for ROS peers, the receiver will delete it
	asebaROS->processAsebaMessage(message);
	
	// free the message
	delete message;
}

void AsebaDashelHub::connectionCreated(Stream *stream)
{
	ROS_INFO_STREAM("Incoming connection from " << stream->getTargetName());
	
	if (dataStreams.size() == 1)
	{
		// Note: on some robot such as the marXbot, because of hardware
		// constraints this might not work. In this case, an external
		// hack is required
		GetDescription getDescription;
		sendMessage(&getDescription, false);
	}
}

void AsebaDashelHub::connectionClosed(Stream* stream, bool abnormal)
{
	if (abnormal)
		ROS_INFO_STREAM("Abnormal connection closed to " << stream->getTargetName() << " : " << stream->getFailReason());
	else
		ROS_INFO_STREAM("Normal connection closed to " << stream->getTargetName());
}


// AsebaROS

bool AsebaROS::loadScript(LoadScripts::Request& req, LoadScripts::Response& res)
{
	// locking: in this method, we lock access to the object's members
	
	// open document
	const string& fileName(req.fileName);
	xmlDoc *doc = xmlReadFile(fileName.c_str(), NULL, 0);
	if (!doc)
	{
		ROS_ERROR_STREAM("Cannot read XML from file " << fileName);
		return false;
	}
    xmlNode *domRoot = xmlDocGetRootElement(doc);
	
	// clear existing data
	mutex.lock();
	commonDefinitions.events.clear();
	commonDefinitions.constants.clear();
	userDefinedVariablesMap.clear();
	pubs.clear();
	subs.clear();
	mutex.unlock();
	
	// load new data
	int noNodeCount = 0;
	bool wasError = false;
	if (!xmlStrEqual(domRoot->name, BAD_CAST("network")))
	{
		ROS_ERROR("root node is not \"network\", XML considered as invalid");
		wasError = true;
	}
	else for (xmlNode *domNode = xmlFirstElementChild(domRoot); domNode; domNode = domNode->next)
	{
		//cerr << "node " << domNode->name << endl;
		if (domNode->type == XML_ELEMENT_NODE)
		{
			if (xmlStrEqual(domNode->name, BAD_CAST("node")))
			{
				// get attributes, child and content
				xmlChar *name = xmlGetProp(domNode, BAD_CAST("name"));
				if (!name)
					ROS_WARN("missing \"name\" attribute in \"node\" entry");
				else
				{
					const string _name((const char *)name);
					xmlChar * text = xmlNodeGetContent(domNode);
					if (!text)
						ROS_WARN("missing text in \"node\" entry");
					else
					{
						//cerr << text << endl;
						mutex.lock();
						bool ok;
						unsigned nodeId(DescriptionsManager::getNodeId(_name, &ok));
						mutex.unlock();
						if (ok)
						{
							mutex.lock();
							// compile code
							std::istringstream is((const char *)text);
							Error error;
							BytecodeVector bytecode;
							unsigned allocatedVariablesCount;
							
							Compiler compiler;
							compiler.setTargetDescription(getDescription(nodeId));
							compiler.setCommonDefinitions(&commonDefinitions);
							bool result = compiler.compile(is, bytecode, allocatedVariablesCount, error);
							mutex.unlock();
							
							if (result)
							{
								typedef std::vector<Message*> MessageVector;
								MessageVector messages;
								sendBytecode(messages, nodeId, std::vector<uint16>(bytecode.begin(), bytecode.end()));
								for (MessageVector::const_iterator it = messages.begin(); it != messages.end(); ++it)
								{
									hub.sendMessage(*it, true);
									delete *it;
								}
								Run msg(nodeId);
								hub.sendMessage(&msg, true);
								// retrieve user-defined variables for use in get/set
								mutex.lock();
								userDefinedVariablesMap[_name] = *compiler.getVariablesMap();
								mutex.unlock();
							}
							else
							{
								ROS_ERROR_STREAM("compilation of " << fileName << ", node " << _name << " failed: " << error.toString());
								wasError = true;
							}
						}
						else
							noNodeCount++;
						
						// free attribute and content
						xmlFree(text);
					}
					xmlFree(name);
				}
			}
			else if (xmlStrEqual(domNode->name, BAD_CAST("event")))
			{
				// get attributes
				xmlChar *name = xmlGetProp(domNode, BAD_CAST("name"));
				if (!name)
					ROS_WARN("missing \"name\" attribute in \"event\" entry");
				xmlChar *size = xmlGetProp(domNode, BAD_CAST("size")); 
				if (!size)
					ROS_WARN("missing \"size\" attribute in \"event\" entry");
				// add event
				if (name && size)
				{
					int eventSize(atoi((const char *)size));
					if (eventSize > ASEBA_MAX_EVENT_ARG_SIZE)
					{
						ROS_ERROR("Event %s has a length %d larger than maximum %d", name, eventSize, ASEBA_MAX_EVENT_ARG_SIZE);
						wasError = true;
						break;
					}
					else
					{
						lock_guard<boost::mutex> lock(mutex);
						commonDefinitions.events.push_back(NamedValue(string((const char *)name), eventSize));
					}
				}
				// free attributes
				if (name)
					xmlFree(name);
				if (size)
					xmlFree(size);
			}
			else if (xmlStrEqual(domNode->name, BAD_CAST("constant")))
			{
				// get attributes
				xmlChar *name = xmlGetProp(domNode, BAD_CAST("name"));
				if (!name)
					ROS_WARN("missing \"name\" attribute in \"constant\" entry");
				xmlChar *value = xmlGetProp(domNode, BAD_CAST("value")); 
				if (!value)
					ROS_WARN("missing \"value\" attribute in \"constant\" entry");
				// add constant if attributes are valid
				if (name && value)
				{
					lock_guard<boost::mutex> lock(mutex);
					commonDefinitions.constants.push_back(NamedValue(string((const char *)name), atoi((const char *)value)));
				}
				// free attributes
				if (name)
					xmlFree(name);
				if (value)
					xmlFree(value);
			}
			else
				ROS_WARN_STREAM("Unknown XML node seen in .aesl file: " << domNode->name);
		}
	}

	// release memory
	xmlFreeDoc(doc);
	
	// check if there was an error
	if (wasError)
	{
		ROS_ERROR_STREAM("There was an error while loading script " << fileName);
		mutex.lock();
		commonDefinitions.events.clear();
		commonDefinitions.constants.clear();
		userDefinedVariablesMap.clear();
		mutex.unlock();
	}
	
	// check if there was some matching problem
	if (noNodeCount)
	{
		ROS_WARN_STREAM(noNodeCount << " scripts have no corresponding nodes in the current network and have not been loaded.");
	}
	
	// recreate publishers and subscribers
	mutex.lock();
	typedef EventsDescriptionsVector::const_iterator EventsDescriptionsConstIt;
	for (EventsDescriptionsConstIt it(commonDefinitions.events.begin()); it != commonDefinitions.events.end(); ++it)
	{
		pubs.push_back(n.advertise<AsebaEvent>("events/"+it->name, 100));
		subs.push_back(n.subscribe<AsebaEvent>("events/"+it->name, 100, bind(&AsebaROS::eventReceived, this, it->value, _1)));
	}
	mutex.unlock();
	
	return true;
}

bool AsebaROS::getNodeList(GetNodeList::Request& req, GetNodeList::Response& res)
{
	lock_guard<boost::mutex> lock(mutex);
	
	transform(nodesNames.begin(), nodesNames.end(), back_inserter(res.nodeList), bind(&NodesNamesMap::value_type::first,_1));
	return true;
}

bool AsebaROS::getNodeId(GetNodeId::Request& req, GetNodeId::Response& res)
{
	lock_guard<boost::mutex> lock(mutex);
	
	NodesNamesMap::const_iterator nodeIt(nodesNames.find(req.nodeName));
	if (nodeIt != nodesNames.end())
	{
		res.nodeId = nodeIt->second;
		return true;
	}
	else
	{
		ROS_ERROR_STREAM("node " << req.nodeName << " does not exists");
		return false;
	}
}

bool AsebaROS::getNodeName(GetNodeName::Request& req, GetNodeName::Response& res)
{
	lock_guard<boost::mutex> lock(mutex);
	
	NodesDescriptionsMap::const_iterator nodeIt(nodesDescriptions.find(req.nodeId));
	if (nodeIt != nodesDescriptions.end())
	{
		res.nodeName = nodeIt->second.name;
		return true;
	}
	else
	{
		ROS_ERROR_STREAM("node " << req.nodeId << " does not exists");
		return false;
	}
}

struct ExtractName
{
	string operator()(const TargetDescription::NamedVariable& nv) const { return nv.name; }
};

bool AsebaROS::getVariableList(GetVariableList::Request& req, GetVariableList::Response& res)
{
	lock_guard<boost::mutex> lock(mutex);
	
	NodesNamesMap::const_iterator nodeIt(nodesNames.find(req.nodeName));
	if (nodeIt != nodesNames.end())
	{
		// search if we have a user-defined variable map?
		const UserDefinedVariablesMap::const_iterator userVarMapIt(userDefinedVariablesMap.find(req.nodeName));
		if (userVarMapIt != userDefinedVariablesMap.end())
		{
			// yes, us it 
			const Compiler::VariablesMap& variablesMap(userVarMapIt->second);
			transform(variablesMap.begin(), variablesMap.end(),
					  back_inserter(res.variableList),  bind(&Compiler::VariablesMap::value_type::first,_1));
		}
		else
		{
			// no, then only show node-defined variables
			const unsigned nodeId(nodeIt->second);
			const NodesDescriptionsMap::const_iterator descIt(nodesDescriptions.find(nodeId));
			const NodeDescription& description(descIt->second);
			transform(description.namedVariables.begin(), description.namedVariables.end(),
					  back_inserter(res.variableList), ExtractName());
		}
		return true;
	}
	else
	{
		ROS_ERROR_STREAM("node " << req.nodeName << " does not exists");
		return false;
	}
}



bool AsebaROS::setVariable(SetVariable::Request& req, SetVariable::Response& res)
{
	// lock the access to the member methods
	unsigned nodeId, pos;
	
	mutex.lock();
	bool exists = getNodePosFromNames(req.nodeName, req.variableName, nodeId, pos);
	mutex.unlock();
	
	if (!exists)
		return false;
	
	SetVariables msg(nodeId, pos, req.data);
	hub.sendMessage(&msg, true);
	return true;
}

bool AsebaROS::getVariable(GetVariable::Request& req, GetVariable::Response& res)
{
	unsigned nodeId, pos;
	
	// lock the access to the member methods, wait will unlock the underlying mutex
	unique_lock<boost::mutex> lock(mutex);
	
	// get information about variable
	bool exists = getNodePosFromNames(req.nodeName, req.variableName, nodeId, pos);
	if (!exists)
		return false;
	bool ok;
	unsigned length = getVariableSize(nodeId, req.variableName, &ok);
	if (!ok)
		return false;
	
	// create query
	const GetVariableQueryKey key(nodeId, pos);
	GetVariableQueryValue query;
	getVariableQueries[key] = &query;
	lock.unlock();
	
	// send message, outside lock to avoid deadlocks
	GetVariables msg(nodeId, pos, length);
	hub.sendMessage(&msg, true);
	system_time const timeout(get_system_time()+posix_time::milliseconds(100));
	
	// wait 100 ms, considering the possibility of spurious wakes
	bool result;
	lock.lock();
	while (query.data.empty())
	{
		result = query.cond.timed_wait(lock, timeout);
		if (!result)
			break;
	}
	
	// remove key and return answer
	getVariableQueries.erase(key);
	if (result)
	{
		res.data = query.data;
		return true;
	}
	else
	{
		ROS_ERROR_STREAM("read of node " << req.nodeName << ", variable " << req.variableName << " did not return a valid answer within 100ms");
		return false;
	}
}

bool AsebaROS::getEventId(GetEventId::Request& req, GetEventId::Response& res)
{
	// needs locking, called by ROS's service thread
	lock_guard<boost::mutex> lock(mutex);
	size_t id;
	if (commonDefinitions.events.contains(req.name, &id))
	{
		res.id = id;
		return true;
	}
	return false;
}

bool AsebaROS::getEventName(GetEventName::Request& req, GetEventName::Response& res)
{
	// needs locking, called by ROS's service thread
	lock_guard<boost::mutex> lock(mutex);
	if (req.id < commonDefinitions.events.size())
	{
		res.name = commonDefinitions.events[req.id].name;
		return true;
	}
	return false;
}

bool AsebaROS::getNodePosFromNames(const string& nodeName, const string& variableName, unsigned& nodeId, unsigned& pos) const
{
	// does not need locking, called by other member function already within the lock
	
	// make sure the node exists
	NodesNamesMap::const_iterator nodeIt(nodesNames.find(nodeName));
	if (nodeIt == nodesNames.end())
	{
		ROS_ERROR_STREAM("node " << nodeName << " does not exists");
		return false;
	}
	nodeId = nodeIt->second;
	pos = unsigned(-1);
	
	// check whether variable is user-defined
	const UserDefinedVariablesMap::const_iterator userVarMapIt(userDefinedVariablesMap.find(nodeName));
	if (userVarMapIt != userDefinedVariablesMap.end())
	{
		const Compiler::VariablesMap& userVarMap(userVarMapIt->second);
		const Compiler::VariablesMap::const_iterator userVarIt(userVarMap.find(variableName));
		if (userVarIt != userVarMap.end())
		{
			pos = userVarIt->second.first;
		}
	}
	
	// if variable is not user-defined, check whether it is provided by this node
	if (pos == unsigned(-1))
	{
		bool ok;
		pos = getVariablePos(nodeId, variableName, &ok);
		if (!ok)
		{
			ROS_ERROR_STREAM("variable " << variableName << " does not exists in node " << nodeName);
			return false;
		}
	}
	return true;
}

void AsebaROS::sendEventOnROS(const UserMessage* asebaMessage)
{
	// does not need locking, called by other member function already within lock
	assert(pubs.size() == commonDefinitions.events.size());
	if (asebaMessage->type < commonDefinitions.events.size())
	{
		// known, send on a named channel
		shared_ptr<AsebaEvent> event(new AsebaEvent);
		event->source = asebaMessage->source;
		event->data = asebaMessage->data;
		pubs[asebaMessage->type].publish(event);
	}
	else
	{
		// unknown, send on the anonymous channel
		shared_ptr<AsebaAnonymousEvent> event(new AsebaAnonymousEvent);
		event->source = asebaMessage->source;
		event->type = asebaMessage->type;
		event->data = asebaMessage->data;
		anonPub.publish(event);
	}
}

void AsebaROS::nodeDescriptionReceived(unsigned nodeId)
{
	// does not need locking, called by parent object
	nodesNames[nodesDescriptions.at(nodeId).name] = nodeId;
}

void AsebaROS::eventReceived(const AsebaAnonymousEventConstPtr& event)
{
	// does not need locking, does not touch object's members
	UserMessage userMessage(event->type, event->data);
	hub.sendMessage(&userMessage, true);
}

void AsebaROS::eventReceived(const uint16 id, const AsebaEventConstPtr& event)
{
	// does not need locking, does not touch object's members
	UserMessage userMessage(id, event->data);
	hub.sendMessage(&userMessage, true);
}

AsebaROS::AsebaROS(unsigned port, bool forward):
	anonPub(n.advertise<AsebaAnonymousEvent>("anonymous_events", 100)),
	anonSub(n.subscribe("anonymous_events", 100, &AsebaROS::eventReceived, this)),
	hub(this, port, forward) // hub for dashel 
{
	// does not need locking, called by main
	
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
	s.push_back(n.advertiseService("get_event_id", &AsebaROS::getEventId, this));
	s.push_back(n.advertiseService("get_event_name", &AsebaROS::getEventName, this));
}

AsebaROS::~AsebaROS()
{
	// does not need locking, called by main
	xmlCleanupParser();
}

void AsebaROS::run()
{
	// does not need locking, called by main
	hub.startThread();
	ros::spin();
	//cerr << "ros returned" << endl;
	hub.stopThread();
}

void AsebaROS::processAsebaMessage(Message *message)
{
	// needs locking, called by Dashel hub
	lock_guard<boost::mutex> lock(mutex);
	
	// scan this message for nodes descriptions
	DescriptionsManager::processMessage(message);
	
	// if user message, send to D-Bus as well
	UserMessage *userMessage = dynamic_cast<UserMessage *>(message);
	if (userMessage)
		sendEventOnROS(userMessage);
	
	// if variables, check for pending answers
	Variables *variables = dynamic_cast<Variables *>(message);
	if (variables)
	{
		const GetVariableQueryKey queryKey(variables->source, variables->start);
		GetVariableQueryMap::const_iterator queryIt(getVariableQueries.find(queryKey));
		if (queryIt != getVariableQueries.end())
		{
			queryIt->second->data = variables->variables;
			queryIt->second->cond.notify_one();
		}
		else
			ROS_WARN_STREAM("received Variables from node " << variables->source << ", pos " << variables->start << ", but no corresponding query was found");
	}
}

//! Show usage
void dumpHelp(std::ostream &stream, const char *programName)
{
	stream << "AsebaROS, connects aseba components together and with ROS, usage:\n";
	stream << programName << " [options] [additional targets]*\n";
	stream << "Options:\n";
	stream << "-l, --loop      : makes the switch transmit messages back to the send, not only forward them.\n";
	stream << "-p port         : listens to incoming connection on this port\n";
	stream << "-h, --help      : shows this help\n";
	stream << "ROS_OPTIONS     : see ROS documentation\n";
	stream << "Additional targets are any valid Dashel targets." << std::endl;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "aseba");
	
	unsigned port = ASEBA_DEFAULT_PORT;
	bool forward = true;
	std::vector<std::string> additionalTargets;
	
	int argCounter = 1;
	
	while (argCounter < argc)
	{
		const char *arg = argv[argCounter];
		
		if ((strcmp(arg, "-l") == 0) || (strcmp(arg, "--loop") == 0))
		{
			forward = false;
		}
		else if (strcmp(arg, "-p") == 0)
		{
			arg = argv[++argCounter];
			port = atoi(arg);
		}
		else if ((strcmp(arg, "-h") == 0) || (strcmp(arg, "--help") == 0))
		{
			dumpHelp(std::cout, argv[0]);
			return 0;
		}
		else
		{
			additionalTargets.push_back(argv[argCounter]);
		}
		argCounter++;
	}
	
	AsebaROS asebaROS(port, forward);
	
	try
	{
		for (size_t i = 0; i < additionalTargets.size(); i++)
			asebaROS.connectTarget(additionalTargets[i]);
	}
	catch(Dashel::DashelException e)
	{
		std::cerr << e.what() << std::endl;
	}
	
	asebaROS.run();

	return 0;
}



