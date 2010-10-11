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

void AsebaDashelHub::sendMessage(Message *message, Stream* sourceStream)
{
	// dump if requested
	// TODO: check verbosity
	// if (
	ostringstream oss;
	message->dump(oss);
	ROS_DEBUG_STREAM(oss.str());
	
	// Might be called from the ROS thread, not the Hub thread, need to lock	
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

	unlock();
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
	sendMessage(message, stream);
	
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
		// TODO: do somthing
		//emit firstConnectionCreated();
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
	commonDefinitions.events.clear();
	commonDefinitions.constants.clear();
	userDefinedVariablesMap.clear();
	pubs.clear();
	subs.clear();
	
	// load new data
	int noNodeCount = 0;
	bool wasError = false;
	for (xmlNode *domNode = domRoot; domNode; domNode = domNode->next)
	{
		if (domNode->type == XML_ELEMENT_NODE)
		{
			if (xmlStrEqual(domNode->name, BAD_CAST("node")))
			{
				// get attributes, child and content
				xmlChar *name = xmlGetProp(domNode, BAD_CAST("name"));
				if (!name)
					ROS_WARN("missing \"name\" attribute in \"node\" entry");
				xmlNode* child = xmlFirstElementChild(domNode);
				if (!child)
					ROS_WARN("missing child in \"node\" entry");
				else if (name)
				{
					const string _name((const char *)name);
					xmlChar * text = xmlNodeGetContent(child);
					if (!text)
						ROS_WARN("missing text in child of \"node\" entry");
					else
					{
						bool ok;
						unsigned nodeId(DescriptionsManager::getNodeId(_name, &ok));
						if (ok)
						{
							// compile code
							std::istringstream is((const char *)text);
							Error error;
							BytecodeVector bytecode;
							unsigned allocatedVariablesCount;
							
							Compiler compiler;
							compiler.setTargetDescription(getDescription(nodeId));
							compiler.setCommonDefinitions(&commonDefinitions);
							bool result = compiler.compile(is, bytecode, allocatedVariablesCount, error);
							
							if (result)
							{
								typedef std::vector<Message*> MessageVector;
								MessageVector messages;
								sendBytecode(messages, nodeId, std::vector<uint16>(bytecode.begin(), bytecode.end()));
								for (MessageVector::const_iterator it = messages.begin(); it != messages.end(); ++it)
								{
									hub.sendMessage(*it);
									delete *it;
								}
								Run msg(nodeId);
								hub.sendMessage(&msg);
								// retrieve user-defined variables for use in get/set
								userDefinedVariablesMap[_name] = *compiler.getVariablesMap();
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
				}
				if (name)
					xmlFree(name);
				
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
						commonDefinitions.events.push_back(NamedValue(string((const char *)name), eventSize));
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
					commonDefinitions.constants.push_back(NamedValue(string((const char *)name), atoi((const char *)value)));
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
		commonDefinitions.events.clear();
		commonDefinitions.constants.clear();
		userDefinedVariablesMap.clear();
	}
	
	// check if there was some matching problem
	if (noNodeCount)
	{
		ROS_WARN_STREAM(noNodeCount << " scripts have no corresponding nodes in the current network and have not been loaded.");
	}
	
	// recreate publishers and subscribers
	typedef EventsDescriptionsVector::const_iterator EventsDescriptionsConstIt;
	for (EventsDescriptionsConstIt it(commonDefinitions.events.begin()); it != commonDefinitions.events.end(); ++it)
	{
		pubs.push_back(n.advertise<AsebaEvent>("events/"+it->name, 100));
		subs.push_back(n.subscribe<AsebaEvent>("events/"+it->name, 100, bind(&AsebaROS::eventReceived, this, it->value, _1)));
	}
	
	return true;
}

bool AsebaROS::getNodeList(GetNodeList::Request& req, GetNodeList::Response& res)
{
	//res.
	// TODO
	return true;
}

bool AsebaROS::getNodeId(GetNodeId::Request& req, GetNodeId::Response& res)
{
	// TODO
	return true;
}

bool AsebaROS::getNodeName(GetNodeName::Request& req, GetNodeName::Response& res)
{
	// TODO
	return true;
}

bool AsebaROS::getVariableList(GetVariableList::Request& req, GetVariableList::Response& res)
{
	// TODO
	return true;
}

bool AsebaROS::setVariable(SetVariable::Request& req, SetVariable::Response& res)
{
	// TODO
	return true;
}

bool AsebaROS::getVariable(GetVariable::Request& req, GetVariable::Response& res)
{
	// TODO
	return true;
}

bool AsebaROS::getEventId(GetEventId::Request& req, GetEventId::Response& res)
{
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
	if (req.id < commonDefinitions.events.size())
	{
		res.name = commonDefinitions.events[req.id].name;
		return true;
	}
	return false;
}

void AsebaROS::sendEventOnROS(const UserMessage* asebaMessage)
{
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
	nodesNames.at(nodesDescriptions.at(nodeId).name) = nodeId;
}

void AsebaROS::eventReceived(const AsebaAnonymousEventConstPtr& event)
{
	UserMessage userMessage(event->type, event->data);
	hub.sendMessage(&userMessage);
}

void AsebaROS::eventReceived(const uint16 id, const AsebaEventConstPtr& event)
{
	UserMessage userMessage(id, event->data);
	hub.sendMessage(&userMessage);
}

AsebaROS::AsebaROS(unsigned port, bool forward):
	anonPub(n.advertise<AsebaAnonymousEvent>("anonymous_events", 100)),
	anonSub(n.subscribe("anonymous_events", 100, &AsebaROS::eventReceived, this)),
	spinner(8), // number of threads for ROS
	hub(this, port, forward) // hub for dashel 
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
	s.push_back(n.advertiseService("get_event_id", &AsebaROS::getEventId, this));
	s.push_back(n.advertiseService("get_event_name", &AsebaROS::getEventName, this));
	
	spinner.start();
}

AsebaROS::~AsebaROS()
{
	xmlCleanupParser();
}

void AsebaROS::run()
{
	hub.run();
}

void AsebaROS::processAsebaMessage(Message *message)
{
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
		const unsigned nodeId(variables->source);
		const unsigned pos(variables->start);
		// TODO
		/*for (RequestsList::iterator it = pendingReads.begin(); it != pendingReads.end(); ++it)
		{
			RequestData* request(*it);
			if (request->nodeId == nodeId && request->pos == pos)
			{
				QDBusMessage &reply(request->reply);
				Values values(fromAsebaVector(variables->variables));
				reply << QVariant::fromValue(values);
				DBusConnectionBus().send(reply);
				delete request;
				pendingReads.erase(it);
				break;
			}
		}*/
	}
}

// AsebaROS::sendEvent(const AsebaEvent& asebaEvent)
// {
// 	// event
// 	// TODO
// }

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



