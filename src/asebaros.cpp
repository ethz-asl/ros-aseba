#include "ros/ros.h"
#include "std_msgs/String.h"
#include "asebaros/LoadScripts.h"
#include "asebaros/GetNodeList.h"

using namespace asebaros;

class AsebaROS
{
protected:
	ros::NodeHandle n;
	
protected:
	bool loadScript(LoadScripts::Request& req, LoadScripts::Response& res)
	{
		return true;
	}
	
	bool getNodeList(GetNodeList::Request& req, GetNodeList::Response& res)
	{
		return true;
	}
	
public:
	AsebaROS()
	{
		n.advertiseService("load_script", &AsebaROS::loadScript, this);
		n.advertiseService("get_node_list", &AsebaROS::getNodeList, this);
	}
	
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "aseba");
	AsebaROS asebaROS;	
	ros::spin();

	return 0;
}



