#include <openrave/openrave.h>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"

#include "brics_actuator/JointValue.h"
#include "brics_actuator/JointPositions.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <boost/algorithm/string.hpp>
#include <cstddef>
#include <math.h>
#include <cmath>
#include <ctime>
#include <time.h>
#include <algorithm>
#include <exception>

static const double     _PI= 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;
static const double _TWO_PI= 6.2831853071795864769252867665590057683943387987502116419498891846156328125724179972560696;

using namespace std;
using namespace OpenRAVE;


class YoubotController : public ControllerBase
{
    public:
        YoubotController(EnvironmentBasePtr penv, std::istream& sinput) : ControllerBase(penv) //, cmdid(0), _bPause(false), _bIsDone(true), _bCheckCollision(false), _bThrowExceptions(false)
    {
        __description = ":Interface Author: Dalitso Banda";

        RegisterCommand("MoveArm", boost::bind(&YoubotController::MoveArm, this, _1, _2), "moves the arm ");
        RegisterCommand("MoveGripper", boost::bind(&YoubotController::MoveGripper, this, _1, _2), "moves the gripper ");
        RegisterCommand("Pause", boost::bind(&YoubotController::Pause, this, _1, _2), "Pauses the controller.");
        RegisterCommand("Resume", boost::bind(&YoubotController::Resume, this, _1, _2), "Resumes the controller.");
        RegisterCommand("SetHighPrecision", boost::bind(&YoubotController::SetHighPrecision, this, _1, _2), "Change goal precision.");

        //RegisterCommand("SetThrowExceptions",boost::bind(&YoubotController::_SetThrowExceptions,this,_1,_2),
        //                "If set, will throw exceptions instead of print warnings. Format is:\n\n  [0/1]");

        _nControlTransformation = 0;
        _penv = penv;
        _paused = false;
        _initialized = false;
        _high_precision_goal = true;
    }

        bool SetHighPrecision(ostream& sout, istream& sinput)
        {

            string cmd;
            sinput >> cmd;
            if (sinput.fail()) {
                RAVELOG_ERROR("SetHighPrecision is missing 1/0.\n");
                return false;
            }
            if (cmd == "1") {
                RAVELOG_INFO("SetHighPrecision is called to 1.\n");
                _high_precision_goal = true;
            } else if(cmd == "0") {
                RAVELOG_INFO("SetHighPrecision is called to 0.\n");
                _high_precision_goal = false;
            } else {
                RAVELOG_ERROR("SetHighPrecision unknown argument %s.\n",cmd.c_str());
                return false;
            }
            return true;
        }

        bool Pause(ostream& sout, istream& sinput)
        {
            _paused = true;
            sout << "True";
            return true;
        }

        bool Resume(ostream& sout, istream& sinput)
        {
            _paused = false;
            sout << "True";
            return true;
        }

        bool MoveArm(ostream& sout, istream& sinput){
            std::string valuesString;
            sinput >> valuesString;

            ROS_INFO("input %s", valuesString.c_str());

            vector<std::string> valuestokens;
            boost::split(valuestokens, valuesString, boost::is_any_of("\t,"));

            vector<double> values;

            for (size_t i = 0; i < valuestokens.size(); ++i)
            {
                values.push_back(boost::lexical_cast<double>(valuestokens[i]));
            }

            bool ret_val = MoveArm(values);

            if (ret_val){
                sout << "True";
                return true;
            }
            sout << "False";
            return false;
        }

        bool MoveArm(vector<double> values)
        {
            if (ros::ok() && !_paused){

                for (size_t i = 0; i < _offset.size(); ++i)
                {
                    values[i] += _offset.at(i);
                }


                brics_actuator::JointPositions command;
                vector <brics_actuator::JointValue> armJointPositions(5);
                vector <std::string> armJointNames(5);
                armJointNames[0] = "arm_joint_1";
                armJointNames[1] = "arm_joint_2";
                armJointNames[2] = "arm_joint_3";
                armJointNames[3] = "arm_joint_4";
                armJointNames[4] = "arm_joint_5";
                for (int i = 0; i < 5; ++i)
                {
                    armJointPositions[i].joint_uri = armJointNames[i].c_str();
                    armJointPositions[i].value = values[i];
                    armJointPositions[i].unit = std::string("rad");
                }

                command.positions = armJointPositions;
                ROS_INFO("the values are %f %f %f %f %f", values[0], values[1], values[2], values[3], values[4]);
                _move_arm_pub.publish(command);

                // Store last values for later.
                _last_arm_command = values;

                return true;
            }
            return false;
        }


        bool MoveGripper(ostream& sout, istream& sinput)
        {
            if (ros::ok() && !_paused)
            {
                std::string valuesString;
                sinput >> valuesString;
                ROS_INFO("input %s", valuesString.c_str());

                vector<std::string> valuestokens;
                boost::split(valuestokens, valuesString, boost::is_any_of("\t,"));

                vector<double> values;
                for (size_t i = 0; i < valuestokens.size(); ++i)
                {
                    values.push_back(boost::lexical_cast<double>(valuestokens[i]));
                }

                brics_actuator::JointPositions command;
                vector <brics_actuator::JointValue> gripperJointPositions;

                vector <std::string> gripperJointNames;

                gripperJointPositions.resize(2);
                gripperJointNames.resize(2);

                //ROS_INFO("initializing gripper");
                gripperJointNames[0] = "gripper_finger_joint_l";
                gripperJointNames[1] = "gripper_finger_joint_r";

                for (unsigned int i = 0; i < 2; ++i)
                {
                    gripperJointPositions[i].joint_uri = gripperJointNames[i].c_str();
                    gripperJointPositions[i].value = values[i];
                    gripperJointPositions[i].unit = std::string("m");
                }

                // Store last values for later.
                _last_gripper_command = values;

                command.positions = gripperJointPositions;
                ROS_INFO("the values are %f %f ", values[0], values[1]);
                _move_gripper_pub.publish(command);
                sout << "True";
                return true;
            }
            sout << "True";
            return false;
        }


        bool MoveBaseTowards(Transform &goal, bool high_precision)
        {
            TransformMatrix target_in_frame = _probot->GetTransform().inverse() * goal;
            double x_diff = target_in_frame.trans[0];
            double y_diff = target_in_frame.trans[1];
            double yaw_diff = atan2(target_in_frame.rot(1,0), target_in_frame.rot(0,0)); 
            // atan2 already returns in the range -pi.+pi. So no need to wrap. Otherwise we would need to.

            double speed_x = 0.03; //std::fabs(waypoint[vel_start + 3]);
            double speed_y = 0.03; // std::fabs(waypoint[vel_start + 4]);
            double speed_yaw = 0.02; //std::fabs(waypoint[vel_start + 5]);
            if (high_precision) {
                speed_x = 0.010; //std::fabs(waypoint[vel_start + 3]);
                speed_y = 0.010; // std::fabs(waypoint[vel_start + 4]);
                //speed_x = 0.015; //std::fabs(waypoint[vel_start + 3]);
                //speed_y = 0.015; // std::fabs(waypoint[vel_start + 4]);
                speed_yaw = 0.010; //std::fabs(waypoint[vel_start + 5]);
                //speed_yaw = 0.010; //std::fabs(waypoint[vel_start + 5]);
            } 

            double dist_thresh = std::sqrt(speed_x * speed_x + speed_y * speed_y) / 5.0;
            double yaw_thresh = speed_yaw / 5.0;              
            if (!high_precision) {
                dist_thresh *= 20.0;
                yaw_thresh *= 100.0;
            }
            double dist2 = std::sqrt(x_diff * x_diff + y_diff * y_diff);

            double x_vel = 0.0;
            double y_vel = 0.0;
            double z_vel = 0.0;
            double roll_vel = 0.0;
            double pitch_vel = 0.0;
            double yaw_vel = 0.0;

            //std::cout << "x_diff is " << x_diff << std::endl;
            //std::cout << "y_diff is " << y_diff << std::endl;
            //std::cout << "distance is " << dist2 << std::endl;
            //std::cout << "dist thresh is " << dist_thresh << std::endl;
            //std::cout << "yaw_diff is " << yaw_diff << std::endl;
            //std::cout << "yaw thresh is " << yaw_thresh << std::endl;

            float wait_to_stop_window = 2.0;
            if (dist2 < dist_thresh && std::fabs(yaw_diff) < yaw_thresh){
                //stop the base
                geometry_msgs::Twist command;
                command.linear.x = 0.0;
                command.linear.y = 0.0;
                command.linear.z = 0.0;
                command.angular.x = 0.0;
                command.angular.y = 0.0;
                command.angular.z = 0.0;
                _move_base_pub.publish(command);
                if (!high_precision) {
                    return true; // We are at the goal
                }
                if (!_stop_commanded) {
                    _time_stop_commanded = clock();
                    _stop_commanded = true;
                }
                if ((float(clock() - _time_stop_commanded)/CLOCKS_PER_SEC) > wait_to_stop_window) {
                    return true; // We are at the goal
                }
            }else{
                _stop_commanded = false;
                if (std::fabs(yaw_diff) < yaw_thresh){ // XXX Hack to first zero yaw, and then x-y.
                    if (dist2 < dist_thresh){
                        x_vel = 0;
                        y_vel = 0;
                    }else{
                        x_vel = x_diff / dist2 * speed_x;
                        y_vel = y_diff / dist2 * speed_y;
                        if (std::fabs(x_diff) > 0.08) // Add a step function here to make it fast when far enough.
                        {
                            x_vel *= 10.0;
                        }
                        //else
                        //{
                        //    x_vel *= 0.8;
                        //}
                        if (std::fabs(y_diff) > 0.08) // Add a step function here to make it fast when far enough.
                        {
                            y_vel *= 10.0;
                        }
                        //else
                        //{
                        //    y_vel *= 0.8;
                        //}
                        double x_sign = copysign(1.0, x_vel);
                        double y_sign = copysign(1.0, y_vel);
                        double max_speed = 0.2;
                        x_vel = std::fabs(x_vel) < max_speed ? x_vel : max_speed * x_sign;
                        y_vel = std::fabs(y_vel) < max_speed ? y_vel : max_speed * y_sign;
                        std::cout << "x_vel is " << x_vel << std::endl;
                        std::cout << "y_vel is " << y_vel << std::endl;
                    }
                }
                if (std::fabs(yaw_diff) < yaw_thresh){
                    yaw_vel = 0;
                }else{
                    yaw_vel = speed_yaw * copysign(1.0, yaw_diff);
                    if (std::fabs(yaw_diff) > 0.2) // Add a step function here to make it fast when far enough.
                    {
                        yaw_vel *= 15.0;
                    }
                }
                geometry_msgs::Twist command;
                command.linear.x = x_vel;
                command.linear.y = y_vel;
                command.linear.z = z_vel;
                command.angular.x = roll_vel;
                command.angular.y = pitch_vel;
                command.angular.z = yaw_vel;
                _move_base_pub.publish(command);
            }
            return false;
        }

        bool MoveBaseTowardsWithArmFeedbackGains(Transform &goal, bool high_precision)
        {
            double translation_x_gain = 0.5;
            double translation_y_gain = 0.5;
            double rotational_gain = 0.5;
            double max_translational_speed = 0.25;
            double max_rotational_speed = 0.6;
            double translational_threshold = 0.0025;
            double translational_threshold_sqr = translational_threshold*translational_threshold;
            double rotational_threshold = 0.04;

            double low_precision_multiplier = 10.0;
            if (!high_precision) {
                translational_threshold = translational_threshold * low_precision_multiplier;
                translational_threshold_sqr = translational_threshold*translational_threshold;
                rotational_threshold = rotational_threshold * low_precision_multiplier;
            }

            TransformMatrix target_in_frame = _probot->GetTransform().inverse() * goal;
            double x_diff = target_in_frame.trans[0];
            double y_diff = target_in_frame.trans[1];
            double yaw_diff = atan2(target_in_frame.rot(1,0), target_in_frame.rot(0,0)); 
            // atan2 already returns in the range -pi.+pi. So no need to wrap. Otherwise we would need to.

            double vel_x = x_diff * translation_x_gain;
            double vel_y = y_diff * translation_y_gain;
            double vel_yaw = yaw_diff * rotational_gain;

            if (std::fabs(vel_x) > max_translational_speed) {
               vel_x = copysign(1.0,vel_x) * max_translational_speed;
            }
            if (std::fabs(vel_y) > max_translational_speed) {
               vel_y = copysign(1.0,vel_y) * max_translational_speed;
            }
            if (std::fabs(vel_yaw) > max_rotational_speed) {
               vel_yaw = copysign(1.0,vel_yaw) * max_rotational_speed;
            }

            if (x_diff*x_diff + y_diff*y_diff < translational_threshold_sqr && yaw_diff < rotational_threshold) {
                // STOP
                geometry_msgs::Twist command;
                command.linear.x = 0.0;
                command.linear.y = 0.0;
                command.linear.z = 0.0;
                command.angular.x = 0.0;
                command.angular.y = 0.0;
                command.angular.z = 0.0;
                _move_base_pub.publish(command);
                if (!high_precision) {
                    return true; // We are at the goal
                }
                if (!_stop_commanded) {
                    _time_stop_commanded = clock();
                    _stop_commanded = true;
                }
                float wait_to_stop_window = 0.5;
                if ((float(clock() - _time_stop_commanded)/CLOCKS_PER_SEC) > wait_to_stop_window) {
                    return true; // We are at the goal
                }
            } else {
                _stop_commanded = false;
                geometry_msgs::Twist command;
                command.linear.x = vel_x;
                command.linear.y = vel_y;
                command.linear.z = 0.0;
                command.angular.x = 0.0;
                command.angular.y = 0.0;
                command.angular.z = vel_yaw;
                _move_base_pub.publish(command);
            }
            return false;
        }

        double IsSameArmConfig(vector<double> &config1, vector<double> &config2)
        {
            for (unsigned int i = 0; i < config1.size(); i++)
            {
                if (std::fabs( config1[i] - config2[i] ) > 0.01)
                {
                    return false;
                }
            }
            return true;
        }

        bool IsArmAtConfig(vector<double> &config)
        {
            std::vector<double> current_arm_config;
            static const int arr[] = {0, 1, 2, 3, 4};
            vector<int> dofindices (arr, arr + sizeof(arr) / sizeof(arr[0]) );
            _probot->GetDOFValues(current_arm_config, dofindices);
            return IsSameArmConfig(current_arm_config,config);
        }

        bool MoveArmTowards(vector<double> &config)
        {
            if (IsArmAtConfig(config))
            {
                return true; // already there.
            }
            if (_last_arm_command.empty() || !IsSameArmConfig(_last_arm_command,config)) 
            {
                MoveArm(config);
                _last_arm_command = config; // TODO this is also set in MoveArm. Fine for only traj executions. If we wnat to use MoveArm(ostream,istream) we need to set a different flag to indicate that special call. 
            }
            return false;
        }

        virtual ~YoubotController()
        {
        }

        void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
        {
            if (_paused)
            {
                return;
            }
            if (msg->name.at(0) != string("arm_joint_1"))   // wheel joints are published to the same topic. Ignore them.
            {
                return;
            }

            std::vector <double> joint_angles;
            std::vector <double> joint_torques;

            // subtract the offset from the read values
            for (unsigned int i = 0; i < (msg->position).size(); i++)
            {
                joint_angles.push_back((msg->position).at(i) - _offset.at(i));
                joint_torques.push_back((msg->effort).at(i));
            }

            {
                OpenRAVE::EnvironmentMutex::scoped_lock lockenv(_penv->GetMutex()); 
                //_probot->SetDOFValues(joint_angles, 0); // The 0 is to not check limits.
                _probot->SetDOFValues(joint_angles,KinBody::CLA_CheckLimitsSilent); // To clamp the joint values to accepted range.
                _probot->SetDOFTorqueLimits(joint_torques); // Hijacking the limits array to carry the actual values.
            }
        }

        virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
        {
            _probot = robot;

            if ( !!_probot )
            {
                _dofindices = dofindices;
                _nControlTransformation = nControlTransformation;
            }
            // intialize ros and the ros node handle
            _pn = new ros::NodeHandle();

            // create ros subscriber for joint messages.
            _joint_angles_sub = _pn->subscribe(("/" + string(_probot->GetName()) + "/joint_states").c_str(), 1, &YoubotController::JointStateCallback, this);

            _move_arm_pub = _pn->advertise<brics_actuator::JointPositions>(("/" + string(_probot->GetName()) + "/arm_1/arm_controller/position_command").c_str(), 1);
            _move_gripper_pub = _pn->advertise<brics_actuator::JointPositions>(("/" + string(_probot->GetName()) + "/arm_1/gripper_controller/position_command").c_str(), 1);
            _move_base_pub = _pn->advertise<geometry_msgs::Twist>(("/" + string(_probot->GetName()) + "/cmd_vel").c_str(), 1);

            _offset.clear();
            double offsetvals[] = {2.950, 1.1345, -2.5482, 1.7890, 2.9234, 0.0, 0.0 , 0.0};
            for (int i = 0 ; i < 8; i++)
            {
                _offset.push_back(offsetvals[i]);
            }

            _paused = false;

            _traj = RaveCreateTrajectory(_penv,"");
            _traj->Init(robot->GetActiveConfigurationSpecification());

            _initialized = true;

            return true;
        }

        virtual void Reset(int options)
        {
        }

        virtual const std::vector<int>& GetControlDOFIndices() const
        {
            return _dofindices;
        }

        virtual int IsControlTransformation() const
        {
            return _nControlTransformation;
        }

        virtual bool SetDesired(const std::vector<dReal>& values, TransformConstPtr trans)
        {
            return true;
        }

        virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
        {
            if (ptraj != NULL)
            {
                _traj->Clone(ptraj,Clone_Bodies);
            }
            return true;
        }

        virtual void SimulationStep(dReal fTimeElapsed)
        {
            if (!_initialized)
            {
                return;
            }

            if (_traj->GetNumWaypoints() > 0) {
                // Get the next trajectory waypoint.
                std::vector< dReal > waypoint;
                _traj->GetWaypoint(0, waypoint);

                // Handle base
                Transform base_goal_transform;
                bool base_at_waypoint = true;
                if (_traj->GetConfigurationSpecification().ExtractTransform(base_goal_transform, waypoint.begin(), _probot) )
                {
                    bool high_precision = false;
                    if (_traj->GetNumWaypoints() == 1 && _high_precision_goal ) {
                        high_precision = true;
                    }
                    base_at_waypoint = MoveBaseTowardsWithArmFeedbackGains(base_goal_transform, high_precision);
                }

                // Handle arm
                static const int arr[] = {0,1,2,3,4};
                std::vector< int > arm_indices(arr,arr+sizeof(arr)/sizeof(arr[0]));
                std::vector< dReal > arm_goal(5);
                bool arm_at_waypoint = true;
                if( _traj->GetConfigurationSpecification().ExtractJointValues(arm_goal.begin(),waypoint.begin(),_probot,arm_indices) )
                {
                    arm_at_waypoint = MoveArmTowards(arm_goal);
                }

                if (base_at_waypoint && arm_at_waypoint)
                {
                    _traj->Remove(0,1); // Remove the reached (first) waypoint. Now the next waypoint is the first waypoint.
                }
            }

            ros::spinOnce();
        }

        virtual bool IsDone()
        {
            return _traj->GetNumWaypoints() == 0;
        }

        virtual dReal GetTime() const
        {
            return 0;
        }
        virtual RobotBasePtr GetRobot() const
        {
            return _probot;
        }


    private:

        bool _initialized;
        RobotBasePtr _probot;               ///< controlled body
        std::vector<int> _dofindices;
        int _nControlTransformation;
        ros::Subscriber _joint_angles_sub;
        ros::Publisher _move_arm_pub;
        ros::Publisher _move_gripper_pub;
        ros::Publisher _move_base_pub;
        ros::NodeHandle* _pn;
        ros::ServiceServer service;
        ros::ServiceClient client;
        EnvironmentBasePtr _penv;
        std::vector<double> _offset;
        bool _paused;
        bool _stop_commanded;
        clock_t _time_stop_commanded;
        bool _high_precision_goal;

        std::vector<double> _last_arm_command;
        std::vector<double> _last_gripper_command;

        TrajectoryBasePtr _traj;
};

ControllerBasePtr CreateYoubotController(EnvironmentBasePtr penv, std::istream& sinput)
{
    return ControllerBasePtr(new YoubotController(penv, sinput));
}

