#include <openrave/config.h>
#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <ros/ros.h>

#include "tfplugin.h"


OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr env) {
    switch(type) {
        case OpenRAVE::PT_Sensor:
            if( interfacename == "tfplugin") {
                std::string node_name, openrave_tf_frame;
                sinput >> node_name >> openrave_tf_frame;

                RAVELOG_INFO("node_name = %s , openrave_tf_frame = %s\n",
                        node_name.c_str(), openrave_tf_frame.c_str()
                        );

                if (sinput.fail()) {
                    RAVELOG_ERROR("tfplugin is missing the node_name, and/or openrave_tf_frame parameter(s).\n");
                    return OpenRAVE::InterfaceBasePtr();
                }

                if (!ros::isInitialized()) {
                    int argc = 0;
                    ros::init(argc, NULL, node_name, ros::init_options::AnonymousName);
                    RAVELOG_INFO("Starting ROS node '%s'.\n", node_name.c_str());
                } else {
                    RAVELOG_INFO("Using existing ROS node '%s'\n", ros::this_node::getName().c_str());
                }
                OpenRAVE::SensorBasePtr tfp = OpenRAVE::SensorBasePtr(new TfPlugin(env,openrave_tf_frame));
                tfp->SetName(node_name);
                return tfp;
            }
            break;
        default:
            break;
    }
    return OpenRAVE::InterfaceBasePtr();
}

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info) {
    info.interfacenames[OpenRAVE::PT_Sensor].push_back("tfplugin");
}

OPENRAVE_PLUGIN_API void DestroyPlugin() {
}


