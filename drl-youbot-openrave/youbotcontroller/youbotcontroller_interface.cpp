#include "ros/ros.h"

#include "plugindefs.h"
#include <openrave/plugin.h>

ControllerBasePtr CreateYoubotController(EnvironmentBasePtr penv, std::istream& sinput);

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    switch(type) {
    case PT_Controller:
        if( interfacename == "youbotcontroller") {
            if (!ros::isInitialized()) {
                int argc = 0;
                std::string node_name = "youbotcontroller";
                ros::init(argc, NULL, node_name, ros::init_options::AnonymousName);
                RAVELOG_INFO("Starting ROS node '%s'.\n", node_name.c_str());
            } else {
                RAVELOG_INFO("Using existing ROS node '%s'\n", ros::this_node::getName().c_str());
            }
            return CreateYoubotController(penv,sinput);
        }
        break;
    default:
        break;
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Controller].push_back("YoubotController");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
