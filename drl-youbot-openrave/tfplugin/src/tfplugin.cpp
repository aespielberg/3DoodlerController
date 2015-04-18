#include <sstream>
#include <boost/foreach.hpp>                                   
#include <boost/typeof/typeof.hpp>
#include <boost/thread/mutex.hpp>
#include <math.h>
#include <cmath>

#include <tf/transform_listener.h>

#include "tfplugin.h"

TfPlugin::TfPlugin(OpenRAVE::EnvironmentBasePtr env, std::string const &openrave_tf_frame)
    : OpenRAVE::SensorBase(env)
    , openrave_tf_frame_(openrave_tf_frame)
    , tf_(nh_)
    , paused_(false)
{
    RegisterCommand("RegisterBody", boost::bind(&TfPlugin::RegisterBody, this, _1, _2),
                    "Register a body with a tf frame.");
    RegisterCommand("RegisterRobotHand", boost::bind(&TfPlugin::RegisterRobotHand, this, _1, _2),
                    "Register a robot's hand frame with a tf frame.");
    RegisterCommand("UnregisterBody", boost::bind(&TfPlugin::UnregisterBody, this, _1, _2),
                    "Unregister a body from a tf frame.");
    RegisterCommand("Pause", boost::bind(&TfPlugin::Pause, this, _1, _2),
                    "Pause the plugin. Leaves registered objects in their current state.");
    RegisterCommand("Resume", boost::bind(&TfPlugin::Resume, this, _1, _2),
                    "Resumes the plugin.");
    RegisterCommand("Clear", boost::bind(&TfPlugin::Clear, this, _1, _2),
                    "Reset the plugin. Disassociates all objects.");
}

TfPlugin::~TfPlugin(void) {
    RAVELOG_INFO("TfPlugin::~TfPlugin\n");
}

void TfPlugin::Reset(void) {
    bodies_.clear();
    offsets_.clear();
    planar_.clear();
    hands_.clear();
}

void TfPlugin::Destroy() {
    Reset();
    RAVELOG_INFO("module unloaded from environment\n");
}

OpenRAVE::Transform TfPlugin::GetPlanarPose(const OpenRAVE::Transform & pose) {
    OpenRAVE::TransformMatrix m(pose);
    double yaw = atan2(m.rot(1,0), m.rot(0,0)); 
    OpenRAVE::Vector quat(quatFromAxisAngle(OpenRAVE::Vector(0.,0.,1.), yaw)); // setting roll and pitch to zero.
    OpenRAVE::Vector translation(m.trans[0],m.trans[1],0.0); // Setting z to zero
    OpenRAVE::Transform planar(quat, translation);
    return planar;
}

bool TfPlugin::SimulationStep(OpenRAVE::dReal fTimeElapsed) {
    boost::mutex::scoped_lock lock(mutex_);
    if (!paused_)
    {
        std::map<std::string,std::string>::iterator it;
        for(it = bodies_.begin(); it != bodies_.end(); it++) {
            OpenRAVE::KinBodyPtr body = GetEnv()->GetKinBody(it->first);
            if (!body) { 
                RAVELOG_WARN("Body %s is not in the environment. \n",it->first.c_str());
                //UnregisterBodyHelper(it->first);
                continue;
            }

            tf::StampedTransform transform;
            try{
                tf_.lookupTransform(openrave_tf_frame_, it->second, ros::Time(0), transform);
            }
            catch (tf::LookupException ex){
                ROS_WARN("Cannot find the transform between tf frames: %s and %s.",openrave_tf_frame_.c_str(),it->second.c_str());
                continue;
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                continue;
            }
            OpenRAVE::Transform pose = GetOrTransform(transform)*offsets_[it->first];
            if (planar_[it->first]) {
                pose = GetPlanarPose(pose);
            }
            {
                OpenRAVE::EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex()); 
                body->SetTransform(pose);
            }
        }
        for(it = hands_.begin(); it != hands_.end(); it++) {
            OpenRAVE::RobotBasePtr robot = GetEnv()->GetRobot(it->first);
            if (!robot) { 
                RAVELOG_WARN("Robot %s is not in the environment. \n",it->first.c_str());
                //UnregisterBodyHelper(it->first);
                continue;
            }

            tf::StampedTransform transform;
            try{
                tf_.lookupTransform(openrave_tf_frame_, it->second, ros::Time(0), transform);
            }
            catch (tf::LookupException ex){
                ROS_WARN("Cannot find the transform between tf frames: %s and %s.",openrave_tf_frame_.c_str(),it->second.c_str());
                continue;
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                continue;
            }
            {
                OpenRAVE::EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex()); 
                OpenRAVE::Transform current_robot_in_world = robot->GetTransform();
                OpenRAVE::Transform current_hand_in_world = robot->GetManipulators()[0]->GetEndEffectorTransform();
                OpenRAVE::Transform robot_in_hand = current_hand_in_world.inverse()*current_robot_in_world;
                OpenRAVE::Transform new_hand_in_world = GetOrTransform(transform);
                OpenRAVE::Transform new_robot_in_world = new_hand_in_world*robot_in_hand; 
                //robot->SetTransform(new_robot_in_world);
                // Trying snapping
                OpenRAVE::Transform snapped = GetPlanarPose(new_robot_in_world);
                robot->SetTransform(snapped);
            }
        }

    }
    return true;
}

bool TfPlugin::Clear(std::ostream& sout, std::istream& sinput) {
    boost::mutex::scoped_lock lock(mutex_);
    Reset();
    return true;
}

bool TfPlugin::RegisterBody(std::ostream& sout, std::istream& sinput) {
    boost::mutex::scoped_lock lock(mutex_);
    std::string body_name, tf_frame;
    sinput >> body_name >> tf_frame;
    if (sinput.fail()) {
        RAVELOG_ERROR("RegisterBody is missing body_name and/or tf_frame parameter(s).\n");
        return false;
    }
    if (!GetEnv()->GetKinBody(body_name)) {
        RAVELOG_ERROR("RegisterBody: Body name %s does not exist.\n",body_name.c_str());
        return false;
    }
    std::string cmd;
    sinput >> cmd;
    double x=0.,y=0.,z=0.,qw=1.,qx=0.,qy=0.,qz=0.;
    if (!sinput.fail()) {
        if (cmd == "openrave_frame_in_tf_frame") {
            RAVELOG_INFO("RegisterBody will use an offset for %s.\n",body_name.c_str());
            sinput >> x >> y >> z >> qw >> qx >> qy >> qz;
            if (sinput.fail()){
                RAVELOG_ERROR("RegisterBody openrave_tf_frame requires 7 values: x y z qw qx qy qz.\n");
                return false;
            }
        } else {
            RAVELOG_ERROR("RegisterBody unknown command:%s.\n",cmd.c_str());
            return false;
        }
    } 
    sinput >> cmd;
    bool planar_tracking = false;
    if (!sinput.fail()) {
        if (cmd == "planar_tracking") {
            planar_tracking = true;
        } else {
            RAVELOG_ERROR("RegisterBody unknown command:%s.\n",cmd.c_str());
            return false;
        }
    }

    bodies_[body_name] = tf_frame;
    offsets_[body_name] = OpenRAVE::Transform(OpenRAVE::Vector(qw,qx,qy,qz),OpenRAVE::Vector(x,y,z)); 
    planar_[body_name] = planar_tracking;
    return true;
}

bool TfPlugin::RegisterRobotHand(std::ostream& sout, std::istream& sinput) {
    boost::mutex::scoped_lock lock(mutex_);
    std::string body_name, tf_frame;
    sinput >> body_name >> tf_frame;
    if (sinput.fail()) {
        RAVELOG_ERROR("RegisterRobotHand is missing robot name and/or tf_frame parameter(s).\n");
        return false;
    }
    if (!GetEnv()->GetKinBody(body_name)) {
        RAVELOG_ERROR("RegisterRobotHand: Robot name %s does not exist.\n",body_name.c_str());
        return false;
    }
    hands_[body_name] = tf_frame;
    return true;
}

bool TfPlugin::UnregisterBody(std::ostream& sout, std::istream& sinput) {
    boost::mutex::scoped_lock lock(mutex_);
    std::string body_name;
    sinput >> body_name;
    if (sinput.fail()) {
        RAVELOG_ERROR("UnregisterBody is missing body_name parameter.\n");
        return false;
    }
    return UnregisterBodyHelper(body_name);
}

bool TfPlugin::UnregisterBodyHelper(std::string const &body_name) {
    if (IsBodyRegistered(body_name)) {
        bodies_.erase(body_name);
        offsets_.erase(body_name);
        planar_.erase(body_name);
        return true;
    }
    // TODO If it exists in both, this just removes from teh body list.
    if (IsHandRegistered(body_name)) {
        hands_.erase(body_name);
        return true;
    }
    RAVELOG_WARN("UnregisterBody: Body name %s does not exist. Ignoring unregister request.\n",body_name.c_str());
    return true;
}

bool TfPlugin::Pause(std::ostream& sout, std::istream& sinput) {
    boost::mutex::scoped_lock lock(mutex_);
    paused_ = true;
    return true;
}

bool TfPlugin::Resume(std::ostream& sout, std::istream& sinput) {
    boost::mutex::scoped_lock lock(mutex_);
    paused_ = false;
    return true;
}

bool TfPlugin::IsBodyRegistered(std::string const &body_name) {
    if(bodies_.find(body_name) == bodies_.end()) {
        return false;
    }
    return true;
}

bool TfPlugin::IsHandRegistered(std::string const &body_name) {
    if(hands_.find(body_name) == hands_.end()) {
        return false;
    }
    return true;
}

OpenRAVE::Transform TfPlugin::GetOrTransform(tf::StampedTransform const &transform) {
    OpenRAVE::Vector quat(transform.getRotation().getW(),
                          transform.getRotation().getX(),
                          transform.getRotation().getY(),
                          transform.getRotation().getZ());
    OpenRAVE::Vector translation(transform.getOrigin().x(), 
                                 transform.getOrigin().y(), 
                                 transform.getOrigin().z());
    return OpenRAVE::Transform(quat, translation);
}

bool TfPlugin::Init(const std::string& cmd) {
    return true;
}

bool TfPlugin::Connect(std::ostream &output, std::istream &input) {
    return true;
}

void TfPlugin::Reset(int options) {
    Reset();
}

int TfPlugin::Configure(OpenRAVE::SensorBase::ConfigureCommand command, bool blocking) {
    return 0;
}

OpenRAVE::SensorBase::SensorGeometryPtr TfPlugin::GetSensorGeometry(OpenRAVE::SensorBase::SensorType type) {
    return OpenRAVE::SensorBase::SensorGeometryPtr();
}

OpenRAVE::SensorBase::SensorDataPtr TfPlugin::CreateSensorData(OpenRAVE::SensorBase::SensorType type) {
    return OpenRAVE::SensorBase::SensorDataPtr();
}

bool TfPlugin::GetSensorData(OpenRAVE::SensorBase::SensorDataPtr psensordata) {
    return false;
}

bool TfPlugin::Supports(OpenRAVE::SensorBase::SensorType type) {
    return false;
}

OpenRAVE::Transform TfPlugin::GetTransform() {
    return OpenRAVE::Transform();
}

void TfPlugin::SetTransform(OpenRAVE::Transform const &transform) {
    return;
}

