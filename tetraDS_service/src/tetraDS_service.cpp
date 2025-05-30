// TETRA_DS Service ROS Package_Ver 0.1 //
// 240315 Last Updated main function C++ //
#include <ros/ros.h>
#include <ros/master.h> // add_move_base die check
#include <ros/this_node.h> // add_move_base die check
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h> //teb_poses...
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h> //teb markers..
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h> //move_base check...
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h> //add...cygbot Local costmap toggle_enabled
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h> //bumper
#include <sensor_msgs/Joy.h> //add 
#include <teb_local_planner/TrajectoryMsg.h>
#include <teb_local_planner/TrajectoryPointMsg.h>
#include <teb_local_planner/FeedbackMsg.h>
#include <actionlib_msgs/GoalID.h>

#include <sstream>
#include <vector>
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <time.h>
#include <thread> //thread add...
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <sys/types.h>
#include <dirent.h>
#include <error.h>
#include <cstdlib> //std::system call
#include <algorithm>
#include <signal.h>

//JSON add////////////////////////////////////
#include <fstream>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/writer.h>
using namespace rapidjson;
//////////////////////////////////////////////

//Service
#include "tetraDS_service/gotolocation.h" //SRV
#include "tetraDS_service/gotolocation2.h" //SRV
#include "tetraDS_service/getlocation.h" //SRV
#include "tetraDS_service/setlocation.h" //SRV
#include "tetraDS_service/setsavemap.h" //SRV
#include "tetraDS_service/getinformation.h" //SRV
#include "tetraDS_service/dockingcontrol.h" //SRV
#include "tetraDS_service/getlocationlist.h" //SRV
#include "tetraDS_service/getlandmarklist.h" //SRV
#include "tetraDS_service/deletelocation.h" //SRV
#include "tetraDS_service/deletelandmark.h" //SRV
#include "tetraDS_service/runmapping.h" //SRV
#include "tetraDS_service/runnavigation.h" //SRV
#include "tetraDS_service/rosnodekill.h" //SRV
#include "tetraDS_service/getmaplist.h" //SRV
#include "tetraDS_service/deletemap.h" //SRV
#include "tetraDS_service/ledcontrol.h" //SRV
#include "tetraDS_service/ledtogglecontrol.h" //SRV
#include "tetraDS_service/toggleon.h" //SRV
#include "tetraDS_service/gotocancel.h" //SRV
#include "tetraDS_service/setmaxspeed.h" //SRV
#include "tetraDS_service/accelerationslop.h" //SRV
#include "tetraDS_service/servo.h" //SRV
#include "ar_track_alvar_msgs/AlvarMarkers.h" //MSG AR_TAG
#include "tetraDS_service/setinitpose.h" //SRV
#include "tetraDS_service/virtual_obstacle.h" //SRV
#include "tetraDS_service/pose_estimate.h" //SRV
//Conveyor Service
#include "tetraDS_service/gotoconveyor.h" //SRV
#include "tetraDS_service/loadingcheck.h" //SRV
#include "tetraDS_service/unloadingcheck.h" //SRV
//add patrol service//
#include "tetraDS_service/patrol.h" //SRV
//#include "tetraDS_service/patrol_conveyor.h" //SRV
#include "tetraDS_service/conveyor_auto_movement.h" //SRV
//add delete data all service//
#include "tetraDS_service/deletedataall.h" //SRV
//dynamic reconfigure Service//
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

//virtual costmap msg//
#include <virtual_costmap_layer/Obstacles.h>
#include <virtual_costmap_layer/Zone.h>
#include <virtual_costmap_layer2/Obstacles2.h>
#include <virtual_costmap_layer2/Zone2.h>

//IMU Service//
#include "tetraDS_service/all_data_reset.h"
#include "tetraDS_service/euler_angle_init.h"
#include "tetraDS_service/euler_angle_reset.h"
#include "tetraDS_service/pose_velocity_reset.h"
#include "tetraDS_service/reboot_sensor.h"
//robot_localization//
#include "tetraDS_service/SetPose.h"
//Set EKF & IMU Reset Service//
#include "tetraDS_service/setekf.h"

#include "tetraDS_service/setweightobstacle.h"

#define HIGH_BATTERY 95
#define LOW_BATTERY 15
#define MAX_RETRY_CNT 99
#define BUF_LEN 4096
using namespace std;
std::string tf_prefix_;
string m_strRobotIP = "";
int m_iRotation_Mode = 0; //Docking Rotation Mode Select
int m_iReset_flag = 0;
FILE *fp;
int status;
char Textbuffer[BUF_LEN];

//test... Marker Tracking & Front docking...
int EX_MARKER_ID = 0;
int m_iParticleCloud_size = 0;
pthread_t p_docking_thread;
pthread_t p_auto_thread;
int  m_iRetry_cnt = 0;
//Docking Command //
int ex_iDocking_CommandMode = 0;
int m_iDocking_timeout_cnt = 0;
int m_iNoMarker_cnt = 0;
bool ex_bMissionDocking_Flag = false;
//Teb Pose Info
double m_dTeb_Pose_head_Angle[1024] = {0.0, };
//patrol...//
int  m_patrol_location_cnt = 0;
string arr_patrol_location[255] = {"", };
//reset Timer count../
int  m_iTimer_cnt = 0;
// Active map Check //
bool m_bActive_map_check = false;
//add...220608
double m_dTF_Yaw = 0.0;
double m_dTF_New_Pose_X = 0.0;
double m_dTF_New_Pose_Y = 0.0;
int m_iList_Count = 0;
int m_iList_Count2 = 0;
int m_iMode_Count = 0;
int m_iMode_Count2 = 0;
//Teb Via Point...
int m_iViaPoint_Index = 0;
bool m_bFlag_nomotion_call = false;
//clesr_costmap service call...
bool m_flag_clesr_costmap_call = false;
//Dynamic_reconfigure call flag//
bool m_flag_Dynamic_reconfigure_call = false;
bool m_flag_Dynamic_TebMarkers_major_update = false;
bool m_flag_Dynamic_Teblocalplan_major_update = false;
bool m_flag_Dynamic_Teblocalplan_minor_update = false;
bool m_flag_Dynamic_Linear_velocity_major_update = false;
bool m_flag_Dynamic_Linear_velocity_minor_update = false;
//Set Goal flag//
bool m_flag_setgoal = false;
//flag
bool m_flag_PREEMPTED = false;
//Speed Zone Enable
bool m_flag_SpeedZone = false;
bool m_flag_major_update[255] = {false, };
bool m_flag_minor_update[255] = {false, };
//virtualWallCheck ... 231115 add
bool m_bFlag_virtualWallCheck = true;

double Set_Weight_obstacle = 20.0;

//Speed Zone Point...
typedef struct TAGPOINT
{
    double x;
    double y;
}POINT, *PPOINT;

typedef struct TAGLIST
{
    POINT _pTagPoint[4] = {0.0, };
    double dSpeed_value = 0.0;

}TAGLIST;
TAGLIST _pTagList[255];

//Vitual Wall Data Point...
typedef struct WALL_DATA_POINT
{
    int m_ilist_count_size = 0;
    double m_dform_x[4] = {0.0, };
    double m_dform_y[4] = {0.0, };
    double m_dform_z[4] = {0.0, };

}WALL_DATA_POINT;
WALL_DATA_POINT _pWallDataPoint;

//Speed Zone Data Point...
typedef struct ZONE_DATA_POINT
{
    int m_iZone_count = 0;
    double m_dform_x2[2] = {0.0, };
    double m_dform_y2[2] = {0.0, };
    double m_dform_z2[2] = {0.0, };

}ZONE_DATA_POINT;
ZONE_DATA_POINT _pZoneDataPoint;

typedef struct HOME_POSE
{
    string HOME_strLOCATION = "HOME";
    double HOME_dPOSITION_X = -0.6;
    double HOME_dPOSITION_Y = 0.0;
    double HOME_dPOSITION_Z = 0.0;
    double HOME_dQUATERNION_X = 0.0;
    double HOME_dQUATERNION_Y = 0.0;
    double HOME_dQUATERNION_Z = 0.0;
    double HOME_dQUATERNION_W = 1.0;

}HOME_POSE;
HOME_POSE _pHomePose;

typedef struct LANDMARK_POSE
{
    string header_frame_id;
    string ns;
    int mark_id;
    double pose_position_x;
    double pose_position_y;
    double pose_position_z;
    double pose_orientation_x;
    double pose_orientation_y;
    double pose_orientation_z;
    double pose_orientation_w;

}LANDMARK_POSE;

typedef struct FALG_VALUE
{
    bool m_bflag_NextStep = false;
    bool m_bflag_ComebackHome = false;
    bool m_bfalg_DockingExit = false;
    bool m_bflag_Conveyor_docking = false;
    bool m_bFlag_Disable_bumper = false;
    int m_Onetime_reset_cnt = 0;
    //Tracking obstacle Check//
    bool m_bFlag_Obstacle_Right = false;
    bool m_bFlag_Obstacle_Center = false;
    bool m_bFlag_Obstacle_Left = false;
    //PCL obstacle Check//
    bool m_bFlag_Obstacle_PCL1 = false;
    bool m_bFlag_Obstacle_PCL2 = false;
    //Cygbot Check//
    bool m_bFlag_Obstacle_cygbot = false;
    //Error Flag//
    bool m_bumperhit_flag = false;
    bool m_emgpush_flag = false;
    bool BUMPER_BT = false;
    //Dynamic reconfigure flag//
    bool m_bTebMarker_reconfigure_flag = false;
    bool m_bflag_patrol = false;
    bool m_bflag_patrol2 = false;
    bool m_bflag_goto_cancel = false;
    bool m_bflagGo = false;
    bool m_bflagGo2 = false;
    bool m_bCorneringFlag = true;
    //no motion service call flag//
    bool m_bFlag_nomotion = true;
    bool m_bflag_auto_patrol = false;
    bool m_bFlag_Initialpose = false;
    //Setgoal_pub flag//
    bool m_bFlag_pub = false;


}FALG_VALUE;
FALG_VALUE _pFlag_Value;

//landmark...
typedef struct LAND_MARK_POSE
{
    float init_position_x = 0.0;
    float init_position_y = 0.0;
    float init_position_z = 0.0;
    float init_orientation_z = 0.0;
    float init_orientation_w = 1.0;

}LAND_MARK_POSE;
LAND_MARK_POSE _pLandMarkPose;

//amcl_Pose...
typedef struct AMCL_POSE
{
    double poseAMCLx = 0.0;
    double poseAMCLy = 0.0;
    double poseAMCLz = 0.0;
    double poseAMCLqx = 0.0;
    double poseAMCLqy = 0.0;
    double poseAMCLqz = 0.0;
    double poseAMCLqw = 1.0;

}AMCL_POSE;
AMCL_POSE _pAMCL_pose;

//tf_Pose.(map->base_footprint TF)..
typedef struct TF_POSE
{
    double poseTFx = 0.0;
    double poseTFy = 0.0;
    double poseTFz = 0.0;
    double poseTFqx = 0.0;
    double poseTFqy = 0.0;
    double poseTFqz = 0.0;
    double poseTFqw = 0.0;

}TF_POSE;
TF_POSE _pTF_pose;

//tf_Pose.(map->odom TF)..
typedef struct TF_POSE2
{
    double poseTFx2 = 0.0;
    double poseTFy2 = 0.0;
    double poseTFz2 = 0.0;
    double poseTFqx2 = 0.0;
    double poseTFqy2 = 0.0;
    double poseTFqz2 = 0.0;
    double poseTFqw2 = 0.0;

}TF_POSE2;
TF_POSE2 _pTF_pose2;

//goal_pose...
typedef struct GOAL_POSE
{
    float goal_positionX = 0.0;
    float goal_positionY = 0.0;
    float goal_positionZ = 0.0;
    float goal_quarterX = 0.0;
    float goal_quarterY = 0.0;
    float goal_quarterZ = 0.0;
    float goal_quarterW = 0.0;

}GOAL_POSE;
GOAL_POSE _pGoal_pose;

//Callback Value
typedef struct ROBOT_STATUS
{
    int m_iCallback_Battery = 0;
    int m_iCallback_ErrorCode = 0;
    int m_iCallback_EMG = 0;
    int m_iCallback_Bumper = 0;
    int m_iCallback_Charging_status = 0;
    //Bumper Collision Behavior//
    int m_iBumperCollisionBehavor_cnt = 0;
    //auto test
    int m_iMovebase_Result = 0;
    //Conveyor Info..(Option)
    double m_dLoadcell_weight = 0.0;
    int m_iConveyor_Sensor_info = 0;
    int HOME_ID = 0; //Docking ID Param Read//
    int CONVEYOR_ID = 0;
    int CONVEYOR_MOVEMENT = 0; // 0: nomal , 1: Loading , 2: Unloading
    //add..Total Distance ... 240129 mwcha
    double m_cmd_vel = 0.0;
    double m_backmove_cmd = 0.0;
    double m_dTotal_Distance = 0.0;
    double m_dGoal_Distance = 0.0;

}ROBOT_STATUS;
ROBOT_STATUS _pRobot_Status;

//AR_TAG Pose
typedef struct AR_TAG_POSE
{
    int m_iSelect_AR_tag_id = 0;
    int m_iAR_tag_id_Index = 0;
    int m_iAR_tag_id = -1;
    float m_fAR_tag_pose_x = 0.0;
    float m_fAR_tag_pose_y = 0.0;
    float m_fAR_tag_pose_z = 0.0;
    float m_fAR_tag_orientation_x = 0.0;
    float m_fAR_tag_orientation_y = 0.0;
    float m_fAR_tag_orientation_z = 0.0;
    float m_fAR_tag_orientation_w = 0.0;
    double m_fAR_tag_roll = 0.0;
    double m_fAR_tag_pitch = 0.0;
    double m_fAR_tag_yaw = 0.0;
    //Transform AR Tag Axis -> Robot Axis
    float m_transform_pose_x = 0.0;
    float m_transform_pose_y = 0.0;
    float m_fPositioning_Angle = 0.0;
    //Calc Odom to AR_Marker TF
    double m_target_yaw = 0.0;

}AR_TAG_POSE;
AR_TAG_POSE _pAR_tag_pose;

//roslaunch mode check//
int ex_ilaunchMode = 0;

//dynamic parameter//
typedef struct DYNAMIC_PARAM
{
    double MAX_Linear_velocity_origin = 0.0;
    double MAX_Linear_velocity = 0.0;
    double m_linear_vel = 0.0;
    double m_angular_vel = 0.0;
    double m_dweight_kinematics_forward_drive_default = 600.0;
    double m_dweight_kinematics_forward_drive_backward = 10.0;
}DYNAMIC_PARAM;
DYNAMIC_PARAM _pDynamic_param;

//SetEKF_Command Service 
typedef struct RESET_SRV
{
    bool   bflag_reset = false;
    double init_position_x = 0.0;
    double init_position_y = 0.0;
    double init_position_z = 0.0;
    double init_orientation_x = 0.0;
    double init_orientation_y = 0.0;
    double init_orientation_z = 0.0;
    double init_orientation_w = 1.0;

}RESET_SRV;
RESET_SRV _pReset_srv;

//Publisher & msg define....
ros::Publisher cmdpub_;
ros::Publisher service_pub; 
ros::Publisher PoseReset_pub;
ros::Publisher GotoCancel_pub;
ros::Publisher Accel_pub;
ros::Publisher initialpose_pub;
ros::Publisher amcl_publisher;
ros::Publisher servo_pub;
geometry_msgs::PoseWithCovarianceStamped initPose_;
std_msgs::Int32 accel_vel;
std_msgs::Int32 servo_request;
geometry_msgs::Point goal_position;
geometry_msgs::Quaternion goal_quarter;
move_base_msgs::MoveBaseActionGoal goal;
//Docking_progress
ros::Publisher docking_progress_pub;
std_msgs::Int32 docking_progress;
// Docking positioning//
ros::Publisher positioning_pub;
geometry_msgs::Pose2D positioning_pose;

//Virtual Costmap//
ros::Publisher virtual_obstacle_pub;
//custom_msgs::Obstacles virtual_obstacle;
virtual_costmap_layer::Obstacles virtual_obstacle;
//Virtual Costmap2//
ros::Publisher virtual_obstacle2_pub;
//custom_msgs::Obstacles virtual_obstacle2;
virtual_costmap_layer2::Obstacles2 virtual_obstacle2;

//**Command srv _ Service Server************************/
tetraDS_service::getlocation getlocation_cmd;
ros::ServiceServer getlocation_service;
tetraDS_service::gotolocation goto_cmd;
ros::ServiceServer goto_service;
tetraDS_service::gotolocation2 goto_cmd2;
ros::ServiceServer goto_service2;
tetraDS_service::setlocation setlocation_cmd;
ros::ServiceServer setlocation_service;
tetraDS_service::setsavemap savemap_cmd;
ros::ServiceServer save_map_service;
tetraDS_service::getinformation getinfo_cmd;
ros::ServiceServer getinfo_service;
tetraDS_service::dockingcontrol docking_cmd;
ros::ServiceServer docking_service;
tetraDS_service::getlocationlist locationlist_cmd;
ros::ServiceServer locationlist_service;
tetraDS_service::getlandmarklist landmarklist_cmd;
ros::ServiceServer landmarklist_service;
tetraDS_service::deletelocation delete_location_cmd;
ros::ServiceServer delete_location_service;
tetraDS_service::deletelandmark delete_landmark_cmd;
ros::ServiceServer delete_landmark_service;
tetraDS_service::deletemap delete_map_cmd;
ros::ServiceServer delete_map_service;
tetraDS_service::runmapping mapping_cmd;
ros::ServiceServer mapping_service;
tetraDS_service::runnavigation navigation_cmd;
ros::ServiceServer navigation_service;
tetraDS_service::rosnodekill nodekill_cmd;
ros::ServiceServer nodekill_service;
tetraDS_service::getmaplist maplist_cmd;
ros::ServiceServer maplist_service;
tetraDS_service::gotocancel gotocancel_cmd;
ros::ServiceServer gotocancel_service;
tetraDS_service::setmaxspeed setspeed_cmd;
ros::ServiceServer setspeed_service;
tetraDS_service::accelerationslop sloptime_cmd;
ros::ServiceServer sloptime_service;
tetraDS_service::servo servo_cmd;
ros::ServiceServer servo_service;
//virtual_costmap//
tetraDS_service::virtual_obstacle virtual_obstacle_cmd;
ros::ServiceServer virtual_obstacle_service;

//Set InitPose//
tetraDS_service::setinitpose setinitpose_cmd;
ros::ServiceServer setinitpose_service;
//2D_Pose_Estimate//
tetraDS_service::pose_estimate pose_estimate_cmd;
ros::ServiceServer pose_Estimate_service;
//Docking Exit Service//
ros::ServiceServer docking_exit;
std_srvs::Empty m_request_dockiong_exit;
//Conveyor Service//
tetraDS_service::gotoconveyor gotoconveyor_cmd;
ros::ServiceServer gotoconveyor_service;
tetraDS_service::loadingcheck loadingcheck_cmd;
ros::ServiceServer loadingcheck_service;
tetraDS_service::unloadingcheck unloadingcheck_cmd;
ros::ServiceServer unloadingcheck_service;
//patrol On / Off Service//
tetraDS_service::patrol patrol_cmd;
ros::ServiceServer patrol_service;
//tetraDS_service::patrol_conveyor patrol_conveyor_cmd;
//ros::ServiceServer patrol_conveyor_service;
//conveyor movement Service Client
//ros::ServiceClient Conveyor_cmd_client;
//tetraDS_service::conveyor_auto_movement conveyor_srv;
//delete data all Service//
tetraDS_service::deletedataall deletedataall_cmd;
ros::ServiceServer deletedataall_service;
//Set EKF & IMU Reset Service//
tetraDS_service::setekf set_ekf_cmd;
ros::ServiceServer set_ekf_service;

tetraDS_service::setweightobstacle setweightobstacle_cmd;
ros::ServiceServer set_weight_obstacle_service; 

//**Command srv _ Service Client************************/
//Usb_cam Service Client//
ros::ServiceClient usb_cam_On_client;
ros::ServiceClient usb_cam_Off_client;
std_srvs::Empty m_request;
//Charging Port on/off Service Client//
ros::ServiceClient charging_port_On_client;
ros::ServiceClient charging_port_Off_client;
std_srvs::Empty m_request2;
//request_nomotion_update 
ros::ServiceClient request_nomotion_update_client;
std_srvs::Empty m_request3;
//Robot Pose Reset msgs
std_msgs::Int32 tetra_PoseRest;
//LED Control Service Client// (tetraDS_interface package)
ros::ServiceClient led_cmd_client;
tetraDS_service::ledcontrol led_srv;
ros::ServiceClient ledtoggle_cmd_client;
tetraDS_service::ledtogglecontrol ledtoggle_srv;
ros::ServiceClient turnon_cmd_client;
tetraDS_service::toggleon turnon_srv;
//goto goal id//
actionlib_msgs::GoalID goto_goal_id;
//Clear costmap Service Client//
ros::ServiceClient clear_costmap_client;

//robot_localization Service Client//
ros::ServiceClient SetPose_cmd_client;
tetraDS_service::SetPose setpose_srv;
geometry_msgs::PoseWithCovarianceStamped set_pose;

//local_costmap toggle_enabled Service Client//
ros::ServiceClient toggle_enabled_client;
std_srvs::SetBool toggle_enabled;

//Bumper_data to Pointcloud2_data//
ros::Publisher  pointcloud_pub_;
sensor_msgs::PointCloud2 pointcloud_;
float P_INF_X = 0.1;  // somewhere out of reach from the robot (positive x)
float P_INF_Y = + P_INF_X*cos(0.34906585);  // somewhere out of reach from the robot (positive y)
float N_INF_Y = - P_INF_X*cos(0.34906585);  // somewhere out of reach from the robot (negative y)
float ZERO = 0.0;
float pc_radius_ = 0.05;
float pc_height_ = 0.0;
float p_side_x_ = 0.05;
float p_side_y_ = + p_side_x_*cos(0.34906585);
float n_side_y_ = - p_side_x_*cos(0.34906585);

//conveyor_test...
int  m_iLoading_ID = 0;
int  m_iUnloading_ID = 0;
string m_strLoading_loacation_name = "LOADING";
string m_strUnloading_loacation_name = "UNLOADING";

//Ignition point landmark add..
ros::Publisher landmark_pub;
visualization_msgs::Marker node;
visualization_msgs::Marker node_name;
ros::Publisher landmark_pub2;
visualization_msgs::Marker node2;
visualization_msgs::Marker node_name2;


//IMU Service Client//
ros::ServiceClient all_data_reset_cmd_client;
tetraDS_service::all_data_reset all_data_reset_srv;
ros::ServiceClient euler_angle_init_cmd_client;
tetraDS_service::euler_angle_init euler_angle_init_srv;
ros::ServiceClient euler_angle_reset_cmd_client;
tetraDS_service::euler_angle_reset euler_angle_reset_srv;
ros::ServiceClient pose_velocity_reset_cmd_client;
tetraDS_service::pose_velocity_reset pose_velocity_reset_srv;
ros::ServiceClient reboot_sensor_cmd_client;
tetraDS_service::reboot_sensor reboot_sensor_srv;

//Cygbot local costmap toggle_enabled flag//
bool m_bToggle_enabled_flag1 = false;
bool m_bToggle_enabled_flag2 = false;

//SpeedZone
vector<POINT> vV[255];


//Read virtual_wall JSON **************************************************************************************************************//
bool Read_virtual_wall_JSON()
{
    bool bResult = false;
    int iInt_count = 0;
    int iNext_count = 0;

    //read file path
    std::ifstream ifs("/home/tetra/virtualwall.json");

    //View _Ignition Point to Text...
    node_name.header.frame_id = "/map";
    node_name.header.stamp = ros::Time(0);
    node_name.text = "";
    node_name.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    node_name.action = visualization_msgs::Marker::DELETEALL;
    landmark_pub.publish(node_name);

    if(ifs)
    {
        printf("[success]: virtualwall json file read OK !! \n"); 
        IStreamWrapper isw(ifs);
        Document doc;
        doc.ParseStream(isw);

        virtual_obstacle.list.clear();
        //Global_costmap Loop//
        virtual_obstacle.list.resize(doc["list_count"].Size());
        for(rapidjson::SizeType i=0; i < doc["list_count"].Size(); i++)
        {
            virtual_obstacle.list[i].form.resize(doc["list_count"][i].GetInt());
            iInt_count = doc["list_count"][i].GetInt();
            for(int j=0; j<doc["list_count"][i].GetInt(); j++)
            {
                virtual_obstacle.list[i].form[j].x = floor(doc["form_x"][iNext_count + j].GetDouble() * 1000.f + 0.5) / 1000.f;
                virtual_obstacle.list[i].form[j].y = floor(doc["form_y"][iNext_count + j].GetDouble() * 1000.f + 0.5) / 1000.f;
                virtual_obstacle.list[i].form[j].z = floor(doc["form_z"][iNext_count + j].GetDouble() * 1000.f + 0.5) / 1000.f;
                
            }
            iNext_count += iInt_count;

            //View _Ignition Point to Text...
            node_name.header.frame_id = "/map";
            node_name.header.stamp = ros::Time(0);
            node_name.text = std::to_string(i+1);
            node_name.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            node_name.id = i+1;
            node_name.action = visualization_msgs::Marker::ADD; 
            node_name.pose.position.x = (virtual_obstacle.list[i].form[0].x + virtual_obstacle.list[i].form[2].x) / 2.0;
            node_name.pose.position.y = (virtual_obstacle.list[i].form[0].y + virtual_obstacle.list[i].form[2].y) / 2.0;
            node_name.pose.orientation.w = 1.0; 
            node_name.color.a = 1.0; 
            node_name.color.r = 0.0;
            node_name.color.g = 0.5;
            node_name.color.b = 0.0; 
            node_name.scale.z = 0.6;
            //Publish_maker_text
            landmark_pub.publish(node_name);

            usleep(10000); //10ms

        }
        virtual_obstacle_pub.publish(virtual_obstacle);

        _pWallDataPoint.m_ilist_count_size = doc["list_count"].Size(); //index Update

        bResult = true;          
    }
    else
    {
        printf("[fail]: virtualwall json file read fail !! \n"); 
        bResult = false;
    }

    return bResult;
}

//Read speed zone JSON ***************************************************************************************************************//
bool Read_speed_zone_JSON()
{
    bool bResult = false;
    int iInt_count = 0;
    int iNext_count = 0;

    //read file path
    std::ifstream ifs("/home/tetra/speedzone.json");

    //View _Ignition Point to Text...
    node_name2.header.frame_id = "/map";
    node_name2.header.stamp = ros::Time(0);
    node_name2.text = "";
    node_name2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    node_name2.action = visualization_msgs::Marker::DELETEALL;
    landmark_pub2.publish(node_name2);

    if(ifs)
    {
        printf("[success]: speedzone json file read OK !! \n"); 
        IStreamWrapper isw(ifs);
        Document doc;
        doc.ParseStream(isw);


        for(rapidjson::SizeType i=0; i < doc["tag_list"].Size(); i++)
        {
            iInt_count = doc["tag_list"][i].GetInt();
            for(int j=0; j<doc["tag_list"][i].GetInt(); j++)
            {
                _pTagList[i]._pTagPoint[j] = {floor(doc["form_x"][iNext_count + j].GetDouble() * 1000.f + 0.5) / 1000.f,
                                              floor(doc["form_y"][iNext_count + j].GetDouble() * 1000.f + 0.5) / 1000.f};

                vV[i].push_back(_pTagList[i]._pTagPoint[j]);

                //printf("_pTagList[%d]._pTagPoint[%d].x: %.3f , _pTagList[%d]._pTagPoint[%d].y: %.3f \n",i,j,_pTagList[i]._pTagPoint[j].x, i,j,_pTagList[i]._pTagPoint[j].y);
            
            }
            iNext_count += iInt_count;

            // Set Speed
            _pTagList[i].dSpeed_value = doc["speed"][i].GetDouble();

            //View _Ignition Point...
            node2.header.frame_id = "/map";
            node2.header.stamp = ros::Time(0); //ros::Time::now(); 
            node2.type = visualization_msgs::Marker::CUBE;
            node2.id = i;
            node2.ns = "Ignition_cube_" + std::to_string(node2.id);
            node2.action = visualization_msgs::Marker::ADD; 
            node2.scale.x = abs((_pTagList[i]._pTagPoint[2].x) - (_pTagList[i]._pTagPoint[0].x));
            node2.scale.y = abs((_pTagList[i]._pTagPoint[2].y) - (_pTagList[i]._pTagPoint[0].y));
            node2.scale.z = 0.1;
            //printf("node.scale.x: %.3f , node.scale.y: %.3f \n", node.scale.x, node.scale.y);
            node2.pose.position.x = ((_pTagList[i]._pTagPoint[0].x) + (_pTagList[i]._pTagPoint[2].x)) / 2.0;
            node2.pose.position.y = ((_pTagList[i]._pTagPoint[0].y) + (_pTagList[i]._pTagPoint[2].y)) / 2.0;
            node2.pose.position.z = 0.0;
            node2.pose.orientation.x = 0.0;
            node2.pose.orientation.y = 0.0;
            node2.pose.orientation.z = 0.0;
            node2.pose.orientation.w = 1.0;
            // Points are blue 
            node2.color.a = 0.5;
            node2.color.r = 0.8;
            node2.color.g = 0.8;
            node2.color.b = 0.0;
            node2.lifetime = ros::Duration(0.0);
            //Publish
            landmark_pub2.publish(node2);


            //View _Ignition Point to Text...
            node_name2.header.frame_id = "/map";
            node_name2.header.stamp = ros::Time(0);
            node_name2.text = std::to_string(i);
            node_name2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            node_name2.id = i;
            node_name2.action = visualization_msgs::Marker::ADD; 
            node_name2.pose.position.x = ((_pTagList[i]._pTagPoint[0].x) + (_pTagList[i]._pTagPoint[2].x)) / 2.0;
            node_name2.pose.position.y = ((_pTagList[i]._pTagPoint[0].y) + (_pTagList[i]._pTagPoint[2].y)) / 2.0;
            node_name2.pose.orientation.w = 1.0; 
            node_name2.color.a = 1.0; 
            node_name2.color.r = 0.0;
            node_name2.color.g = 0.0;
            node_name2.color.b = 0.5; 
            node_name2.scale.z = 0.5;
            //Publish_maker_text
            landmark_pub2.publish(node_name2);

            usleep(10000); //10ms

        }
        
        _pZoneDataPoint.m_iZone_count = doc["tag_list"].Size(); //index Update
        //printf("_pZoneDataPoint.m_iZone_count: %d \n", _pZoneDataPoint.m_iZone_count);

        if(_pZoneDataPoint.m_iZone_count > 0)
            m_flag_SpeedZone = true;

        bResult = true;          
    }
    else
    {
        printf("[fail]: speedzone json file read fail !! \n"); 
        bResult = false;
    }


    return bResult;
}

//************************************************************************************************************************************//
//Polygon Inside or Outside Check algorithm//
bool isinner(POINT _CurrentPos, vector<POINT> _vVertex)
{
    int iSize = _vVertex.size();
    int iCount = 0;

    for(int i=0; i < iSize; i++)
    {
        int j = (i + 1) % iSize;
        bool b_first_rule = _CurrentPos.y > _vVertex[i].y != _CurrentPos.y > _vVertex[j].y;
        if(b_first_rule)
        {
            double S[3] = {0, };
            S[0] = _vVertex[j].x - _vVertex[i].x;
            S[1] = _vVertex[j].y - _vVertex[i].y;
            S[2] = _CurrentPos.y - _vVertex[i].y;

            double itsPosX = S[0] * S[2] / S[1] + _vVertex[i].x;
            if(_CurrentPos.x < itsPosX)
            {
                ++iCount;
            }
        }
    }

    return (iCount % 2) != 0;
}

//************************************************************************************************************************//
void my_handler(sig_atomic_t s)
{
    printf("Caught signal %d\n",s);
    exit(1); 
}

/** reduce angle to between 0 and 360 degrees. */

double reduceHeading(double a) 
{
    return remainder(a-180, 360)+180;
}

///////Logitech F710 Joypad//////////////////////////////////////
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(joy->buttons[6]) //LT button
	{	
        //Docking Start//
		ex_iDocking_CommandMode = 1;
        _pFlag_Value.m_bfalg_DockingExit = false;
	}
    if(joy->buttons[7]) //RT button
	{	
		//Docking Stop//
		ex_iDocking_CommandMode = 0;
        _pFlag_Value.m_bfalg_DockingExit = true;

	}
	
}

///////SICK TIM 571  Range Check//////////////////////////////////////
void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    int size = msg->ranges.size();

    //Right Check//
    int R_minIndex = 10;
    int R_maxIndex = 185;
    int R_closestIndex = -1;
    double R_minVal = 0.2;

    for (int i = R_minIndex; i < R_maxIndex; i++)
    {
        if ((msg->ranges[i] <= R_minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
        {
            R_minVal = msg->ranges[i];
            R_closestIndex = i;
        }
    }
    //printf("R_closestIndex: %d || check: %f \n" , R_closestIndex, msg->ranges[R_closestIndex]);
    if(R_closestIndex > 0)
        _pFlag_Value.m_bFlag_Obstacle_Right = true;
    else
        _pFlag_Value.m_bFlag_Obstacle_Right = false;

    /**************************************************************************************************************************/
    //Center Check//
    int C_minIndex = 320; //180;
    int C_maxIndex = 420; //550;
    int C_closestIndex = -1;
    double C_minVal = 0.8; //0.3

    for (int i = C_minIndex; i < C_maxIndex; i++)
    {
        if ((msg->ranges[i] <= C_minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
        {
            C_minVal = msg->ranges[i];
            C_closestIndex = i;
        }
    }
    //printf("C_closestIndex: %d || check: %f \n" , C_closestIndex, msg->ranges[C_closestIndex]);
    if(C_closestIndex > 0)
        _pFlag_Value.m_bFlag_Obstacle_Center = true;
    else
        _pFlag_Value.m_bFlag_Obstacle_Center = false;

    /**************************************************************************************************************************/
    //Left Check//
    int L_minIndex = 555;
    int L_maxIndex = 730;
    int L_closestIndex = -1;
    double L_minVal = 0.2;

    for (int i = L_minIndex; i < L_maxIndex; i++)
    {
        if ((msg->ranges[i] <= L_minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
        {
            L_minVal = msg->ranges[i];
            L_closestIndex = i;
        }
    }
    //printf("L_closestIndex: %d || check: %f \n" , L_closestIndex, msg->ranges[L_closestIndex]);
    if(L_closestIndex > 0)
        _pFlag_Value.m_bFlag_Obstacle_Left = true;
    else
        _pFlag_Value.m_bFlag_Obstacle_Left = false;

}

void PCL1_Callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    int size = msg->ranges.size();
    //printf("PCL1_size: %d \n",size); //848
    int PCL1_minIndex = 0;
    int PCL1_maxIndex = 848;
    int PCL1_closestIndex = -1;
    double PCL1_minVal = 0.6;

    for (int i = PCL1_minIndex; i < PCL1_maxIndex; i++)
    {
        if ((msg->ranges[i] <= PCL1_minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
        {
            PCL1_minVal = msg->ranges[i];
            PCL1_closestIndex = i;
        }
    }
    //printf("PCL1_closestIndex: %d || check: %f \n" , PCL1_closestIndex, msg->ranges[PCL1_closestIndex]);
    if(PCL1_closestIndex > 0)
        _pFlag_Value.m_bFlag_Obstacle_PCL1 = true;
    else
        _pFlag_Value.m_bFlag_Obstacle_PCL1 = false;

}

void PCL2_Callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    int size = msg->ranges.size();
    //printf("PCL2_size: %d \n",size);
    int PCL2_minIndex = 0;
    int PCL2_maxIndex = 848;
    int PCL2_closestIndex = -1;
    double PCL2_minVal = 0.6;

    for (int i = PCL2_minIndex; i < PCL2_maxIndex; i++)
    {
        if ((msg->ranges[i] <= PCL2_minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
        {
            PCL2_minVal = msg->ranges[i];
            PCL2_closestIndex = i;
        }
    }
    //printf("PCL1_closestIndex: %d || check: %f \n" , PCL1_closestIndex, msg->ranges[PCL1_closestIndex]);
    if(PCL2_closestIndex > 0)
        _pFlag_Value.m_bFlag_Obstacle_PCL2 = true;
    else
        _pFlag_Value.m_bFlag_Obstacle_PCL2 = false;
}

void Cygbot_Callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    int size = msg->ranges.size();
    //printf("cygbot_size: %d \n",size);
    int cygbot_minIndex = 1;
    int cygbot_maxIndex = 160;
    int cygbot_closestIndex = -1;
    double cygbot_minVal = 0.5;

    for (int i = cygbot_minIndex; i < cygbot_maxIndex; i++)
    {
        if ((msg->ranges[i] <= cygbot_minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
        {
            cygbot_minVal = msg->ranges[i];
            cygbot_closestIndex = i;
        }
    }
    //printf("cygbot_closestIndex: %d || check: %f \n" , cygbot_closestIndex, msg->ranges[cygbot_closestIndex]);
    if(cygbot_closestIndex > 0)
        _pFlag_Value.m_bFlag_Obstacle_cygbot = true;
    else
        _pFlag_Value.m_bFlag_Obstacle_cygbot = false;
    
}

double Quaternion2Yaw(double Quaternion_W, double Quaternion_X, double Quaternion_Y, double Quaternion_Z)
{
    double m_dYaw_deg = 0.0;
    double m_dRoll = 0.0;
    double m_dPitch = 0.0;
    double m_dYaw = 0.0;

    //< Declaration of quaternion
    tf::Quaternion q;
    q.setW(Quaternion_W);
    q.setX(Quaternion_X);
    q.setY(Quaternion_Y);
    q.setZ(Quaternion_Z);
    //< quaternion -> rotation Matrix
    tf::Matrix3x3 m(q);
    //< rotation Matrix - > quaternion
    m.getRotation(q);
    //< rotation Matrix -> rpy
    m.getRPY(m_dRoll, m_dPitch, m_dYaw);
    m_dYaw_deg = m_dYaw * (180.0/M_PI);
    if(m_dYaw_deg < 0)
    {
        m_dYaw_deg = -1.0 * m_dYaw_deg;
    }
    
    return m_dYaw_deg;
}

double Quaternion2Yaw_rad(double Quaternion_W, double Quaternion_X, double Quaternion_Y, double Quaternion_Z)
{
    double m_dYaw_rad = 0.0;
    double m_dRoll = 0.0;
    double m_dPitch = 0.0;
    double m_dYaw = 0.0;

    //< Declaration of quaternion
    tf::Quaternion q;
    q.setW(Quaternion_W);
    q.setX(Quaternion_X);
    q.setY(Quaternion_Y);
    q.setZ(Quaternion_Z);

    //< quaternion -> rotation Matrix
    tf::Matrix3x3 m(q);
    //< rotation Matrix - > quaternion
    m.getRotation(q);
    //< rotation Matrix -> rpy
    m.getRPY(m_dRoll, m_dPitch, m_dYaw);
    m_dYaw_rad = m_dYaw; 
    
    //m_dYaw_rad = reduceHeading(m_dYaw);
    
    return m_dYaw_rad;
}


string GetWIFI_IPAddress()
{
    string str_ip;
    int fd;
    struct ifreq ifr;
    char myEth0_addr[20] = {0,};
 
    fd = socket(AF_INET, SOCK_DGRAM, 0);
     
    ifr.ifr_addr.sa_family = AF_INET;
    strncpy(ifr.ifr_name, "enp3s0", IFNAMSIZ -1); //Ehernet device name Check
    
    ioctl(fd, SIOCGIFADDR, &ifr);
    close(fd);
     
    sprintf(myEth0_addr, "%s", inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
    

    str_ip = myEth0_addr;
    return str_ip;
}

int old_slopTime = 0;
bool AccelerationControl(int slopTime)
{
    if(slopTime < 0) return false;

    if(old_slopTime != slopTime)
    {
        accel_vel.data = slopTime;
        Accel_pub.publish(accel_vel);
        printf(" AccelerationControl %d : %d \n", old_slopTime, slopTime);
    }

    old_slopTime = slopTime;
    return true;
}

void Docking_EXIT()
{
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
    cmd->linear.x =  0.0; 
    cmd->angular.z = 0.0;
    ex_iDocking_CommandMode = 0;
    if(!_pFlag_Value.m_bumperhit_flag)
    {
        ex_bMissionDocking_Flag = false;
    }
    printf("[EXIT]: Docking Exit !! \n");
}

bool DockingStop_Command(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    Docking_EXIT();
	return true;
}

bool SlopTime_Command(tetraDS_service::accelerationslop::Request &req, 
				      tetraDS_service::accelerationslop::Response &res)
{
	bool bResult = false;

    AccelerationControl(req.slop_time);
	/*
	int32 slop_time
    ---
    bool command_Result
	*/
    bResult = true;
	res.command_Result = bResult;
	return true;
}

bool Servo_Command(tetraDS_service::servo::Request &req, 
				   tetraDS_service::servo::Response &res)
{
	bool bResult = false;

    servo_request.data = req.data; 
    servo_pub.publish(servo_request);
    bResult = true;
	/*
	int32 data
    ---
    bool command_Result
	*/
	res.command_Result = bResult;
	return true;
}

string DRD_old_strname = "";
double DRD_old_dValue = 0.0;
void Dynamic_reconfigure_Teb_Set_DoubleParam(string strname, double dValue)
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config reconf;

    if((DRD_old_strname != strname) || (DRD_old_dValue != dValue))
    {
        m_flag_Dynamic_reconfigure_call = true;

        double_param.name = strname;
        double_param.value = dValue;
        reconf.doubles.push_back(double_param);

        srv_req.config = reconf;

        ros::service::call("move_base/TebLocalPlannerROS/set_parameters", srv_req, srv_resp);

        m_flag_Dynamic_reconfigure_call = false;
    }

    DRD_old_strname = strname;
    DRD_old_dValue = dValue;

}

string DRI_old_strname = "";
int DRI_old_iValue = 0;
void Dynamic_reconfigure_Costmap_Set_IntParam(string strname, int iValue)
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::IntParameter int_param;
    dynamic_reconfigure::Config reconf2;

    if((DRI_old_strname != strname) || (DRI_old_iValue != iValue))
    {
        int_param.name = strname;
        int_param.value = iValue;
        reconf2.ints.push_back(int_param);

        srv_req.config = reconf2;

        ros::service::call("move_base/local_costmap/set_parameters", srv_req, srv_resp);
        
    }

    DRI_old_strname = strname;
    DRI_old_iValue = iValue;

}

string DRB_old_strname = "";
bool DRB_old_bValue = false;
void Dynamic_reconfigure_Costmap_Set_BoolParam(string strname, bool bValue)
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::BoolParameter bool_param;
    dynamic_reconfigure::Config reconf3;

    if((DRB_old_strname != strname) || (DRB_old_bValue != bValue))
    {
        bool_param.name = strname;
        bool_param.value = bValue;
        reconf3.bools.push_back(bool_param);

        srv_req.config = reconf3;

        ros::service::call("move_base/global_costmap/virtual_layer", srv_req, srv_resp);
        ros::service::call("move_base/local_costmap/virtual_layer", srv_req, srv_resp);

        
    }

    DRB_old_strname = strname;
    DRB_old_bValue = bValue;

}

void LED_Turn_On(int id)
{
    turnon_srv.request.ID = id;
    turnon_cmd_client.call(turnon_srv);
}

void LED_Control(int id, int led_brightness)
{
    led_srv.request.ID = id;
    led_srv.request.led_brightness = led_brightness;
    led_cmd_client.call(led_srv);
}

void LED_Toggle_Control(int de_index, int light_acc, int High_brightness, int light_dec, int Low_brightness)
{
    ledtoggle_srv.request.de_index = de_index;
    ledtoggle_srv.request.light_accel = light_acc;
    ledtoggle_srv.request.led_High_brightness = High_brightness;
    ledtoggle_srv.request.light_decel = light_dec;
    ledtoggle_srv.request.led_Low_brightness = Low_brightness;
    ledtoggle_cmd_client.call(ledtoggle_srv);
}

bool Setspeed_Command(tetraDS_service::setmaxspeed::Request &req, 
				      tetraDS_service::setmaxspeed::Response &res)
{
	bool bResult = false;

    Dynamic_reconfigure_Teb_Set_DoubleParam("max_vel_x", (double)req.speed);
    _pDynamic_param.MAX_Linear_velocity_origin = _pDynamic_param.MAX_Linear_velocity = (double)req.speed;

	/*
	float32 speed
    ---
    float32 set_vel
    bool command_Result
	*/
    res.set_vel = req.speed;
    bResult = true;
	res.command_Result = bResult;
	return true;
}

void cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    _pDynamic_param.m_linear_vel  = msg->linear.x;
    _pDynamic_param.m_angular_vel = msg->angular.z;

    if(_pDynamic_param.m_linear_vel < 0)
    {
        if(!m_bToggle_enabled_flag1)
        {
            toggle_enabled.request.data = true;
            toggle_enabled_client.call(toggle_enabled);
            printf("toggle_Enable !\n");
            m_bToggle_enabled_flag1 = true;
            m_bToggle_enabled_flag2 = false;
        }
        
    }
    else
    {
        if(!m_bToggle_enabled_flag2)
        {
            toggle_enabled.request.data = false;
            toggle_enabled_client.call(toggle_enabled);
            printf("toggle_Disable !\n");
            m_bToggle_enabled_flag1 = false;
            m_bToggle_enabled_flag2 = true;
        }
    }

}

void Particle_Callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{

    m_iParticleCloud_size = msg->poses.size();
    //if(m_iParticleCloud_size > 501 && _pDynamic_param.m_linear_vel == 0.0 && _pDynamic_param.m_angular_vel == 0.0 && _pRobot_Status.m_iCallback_Charging_status != 1)
    if(m_iParticleCloud_size > 501 && _pDynamic_param.m_linear_vel == 0.0 && _pDynamic_param.m_angular_vel == 0.0 && _pFlag_Value.m_bFlag_Initialpose)
    {
        if(_pFlag_Value.m_bFlag_nomotion)
        {
	        m_bFlag_nomotion_call = true;
            request_nomotion_update_client.call(m_request3);
        }   
    }
    else
    {
            m_bFlag_nomotion_call = false;
            _pFlag_Value.m_bFlag_Initialpose = false;
    }
    
}

void TebMarkers_Callback(const visualization_msgs::Marker::ConstPtr& msg)
{
    float m_point_vector = 0.0;
    if(msg->ns == "ViaPoints")
    {
        m_iViaPoint_Index = msg->points.size();
        //printf("!!!m_iIndex: %d \n", m_iIndex);
        if(m_iViaPoint_Index <= 1)
        {
            if(!_pFlag_Value.m_bTebMarker_reconfigure_flag)
            {
                if(_pDynamic_param.m_linear_vel >= 0.3)
                {
                    if(!m_flag_Dynamic_TebMarkers_major_update)
                    {
                        Dynamic_reconfigure_Teb_Set_DoubleParam("max_vel_x", 0.3);
                        m_flag_Dynamic_TebMarkers_major_update = true;
                        _pFlag_Value.m_bTebMarker_reconfigure_flag = true;
                    }
                }
            }
        }
        else
        {
            _pFlag_Value.m_bTebMarker_reconfigure_flag = false;
            m_flag_Dynamic_TebMarkers_major_update = false;
        }
        _pFlag_Value.m_bflagGo = true;
    }
    else
    {
        _pFlag_Value.m_bflagGo = false;
    }
}

void Teblocalplan_Callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    double m_dDelta_Value = 0.0;
    int m_iPoseIndex = msg->poses.size();
    for(int i=0; i<m_iPoseIndex; i++)
    {
        m_dTeb_Pose_head_Angle[i] = Quaternion2Yaw(msg->poses[i].orientation.w, 
                                                    msg->poses[i].orientation.x, 
                                                    msg->poses[i].orientation.y, 
                                                    msg->poses[i].orientation.z);

    }

    m_dDelta_Value = m_dTeb_Pose_head_Angle[1] - m_dTeb_Pose_head_Angle[0];
    if(m_dDelta_Value < 0)
    {
        m_dDelta_Value = -1.0 * m_dDelta_Value;
    }

    if(_pFlag_Value.m_bCorneringFlag)
    {
        if(m_dDelta_Value >= 3.5)
        {
            if(!m_flag_Dynamic_Teblocalplan_major_update)
            {
                Dynamic_reconfigure_Teb_Set_DoubleParam("max_vel_theta", 0.35); 
                m_flag_Dynamic_Teblocalplan_major_update = true;
                m_flag_Dynamic_Teblocalplan_minor_update = false;
                _pFlag_Value.m_bTebMarker_reconfigure_flag = true;
            }
            else
            {
                _pFlag_Value.m_bTebMarker_reconfigure_flag = false;
            }
            
        }
        else
        {
            if(!m_flag_Dynamic_Teblocalplan_minor_update)
            {
                Dynamic_reconfigure_Teb_Set_DoubleParam("max_vel_theta", 0.05);
                m_flag_Dynamic_Teblocalplan_minor_update = true;
                m_flag_Dynamic_Teblocalplan_major_update = false;
                _pFlag_Value.m_bTebMarker_reconfigure_flag = true;
            }
            else
            {
                _pFlag_Value.m_bTebMarker_reconfigure_flag = false;
            }
        }
    }
}

void setGoal(move_base_msgs::MoveBaseActionGoal& goal)
{
    ros::Time now = ros::Time::now(); //ros::Time(0);

    goal.header.frame_id="map";
    goal.header.stamp=now;
    goal.goal_id.stamp = now;
    goal.goal.target_pose.header.stamp = now;
    goal.goal.target_pose.header.frame_id = "map";
    goal.goal.target_pose.pose.position.x = _pGoal_pose.goal_positionX;
    goal.goal.target_pose.pose.position.y = _pGoal_pose.goal_positionY;
    goal.goal.target_pose.pose.position.z = _pGoal_pose.goal_positionZ;
    goal.goal.target_pose.pose.orientation.x = _pGoal_pose.goal_quarterX;
    goal.goal.target_pose.pose.orientation.y = _pGoal_pose.goal_quarterY;
    goal.goal.target_pose.pose.orientation.z = _pGoal_pose.goal_quarterZ;
    goal.goal.target_pose.pose.orientation.w = _pGoal_pose.goal_quarterW;

    service_pub.publish(goal);
    printf("setGoal call: %.5f, %.5f !!\n", _pGoal_pose.goal_positionX, _pGoal_pose.goal_positionY);
    _pFlag_Value.m_bFlag_pub = true;
}

bool SaveLocation(string str_location, 
                  float m_fposeAMCLx, float m_fposeAMCLy,
                  float m_fposeAMCLqx, float m_fposeAMCLqy, float m_fposeAMCLqz, float m_fposeAMCLqw)
{
    bool bResult = false;

    string m_strFilePathName;
    m_strFilePathName = "/home/tetra/DATA/" + str_location + ".txt";    
    fp = fopen(m_strFilePathName.c_str(), "w");
    if(fp == NULL)
    { 
        ROS_INFO("file is null");
        bResult = false;
    }
    else
    {
        fprintf(fp, "0,%lf,%lf,%lf,%lf,%lf,%lf \n",
                m_fposeAMCLx, m_fposeAMCLy, m_fposeAMCLqx, m_fposeAMCLqy, m_fposeAMCLqz, m_fposeAMCLqw);
        fclose(fp);
        bResult = true;
    }

    return bResult;
}

bool OpenLocationFile(string str_location)
{
    bool bResult = false;
    string m_strFilePathName;

    if(str_location == "HOME")
    {
        _pGoal_pose.goal_positionX = _pHomePose.HOME_dPOSITION_X;
        _pGoal_pose.goal_positionY = _pHomePose.HOME_dPOSITION_Y;
        _pGoal_pose.goal_positionZ = _pHomePose.HOME_dPOSITION_Z;
        _pGoal_pose.goal_quarterX = _pHomePose.HOME_dQUATERNION_X;
        _pGoal_pose.goal_quarterY = _pHomePose.HOME_dQUATERNION_Y;
        _pGoal_pose.goal_quarterZ = _pHomePose.HOME_dQUATERNION_Z;
        _pGoal_pose.goal_quarterW = _pHomePose.HOME_dQUATERNION_W;
   
        bResult = true; 
    }
    else
    {
        m_strFilePathName = "/home/tetra/DATA/" + str_location + ".txt";  
        fp = fopen(m_strFilePathName.c_str(), "r");  

        if(fp != NULL) //File Open Check
        {
            while(!feof(fp))
            {
                if(fgets(Textbuffer, sizeof(Textbuffer), fp) != NULL)
                {
                    char* ptr = strtok(Textbuffer, ",");
                    int icnt = 0;

                    while(ptr != NULL)
                    {   
                        ptr = strtok(NULL, ",");
                        switch(icnt)
                        {
                            case 0:
                                if(ptr != NULL)
                                {
                                    _pGoal_pose.goal_positionX = atof(ptr);
                                }
                                break;
                            case 1:
                                _pGoal_pose.goal_positionY = atof(ptr);
                                break;
                            case 2:
                                _pGoal_pose.goal_quarterX = atof(ptr);
                                break;
                            case 3:
                                _pGoal_pose.goal_quarterY = atof(ptr);
                                break;
                            case 4:
                                _pGoal_pose.goal_quarterZ = atof(ptr);  
                                break;
                            case 5:
                                _pGoal_pose.goal_quarterW = atof(ptr);
                                break;
                        }
                        icnt++;
                    }
                    bResult = true; 
                }
                else
                {
                    bResult = false; 
                }
            }                
            fclose(fp);
        }
        else
        {
            ROS_INFO("File Open Fail: %s", str_location.c_str());
            bResult = false;
        }
    }

    return bResult;
}

bool GetLocation_Command(tetraDS_service::getlocation::Request  &req, 
					     tetraDS_service::getlocation::Response &res)
{
	bool bResult = false;

    //Get POSE
    // res.poseAMCLx = _pAMCL_pose.poseAMCLx;
    // res.poseAMCLy = _pAMCL_pose.poseAMCLy;
    // res.poseAMCLqx = _pAMCL_pose.poseAMCLqx;
    // res.poseAMCLqy = _pAMCL_pose.poseAMCLqy;
    // res.poseAMCLqz = _pAMCL_pose.poseAMCLqz;
    // res.poseAMCLqw = _pAMCL_pose.poseAMCLqw;

    res.poseAMCLx = _pTF_pose.poseTFx;
    res.poseAMCLy = _pTF_pose.poseTFy;
    res.poseAMCLqx = _pTF_pose.poseTFqx;
    res.poseAMCLqy = _pTF_pose.poseTFqy;
    res.poseAMCLqz = _pTF_pose.poseTFqz;
    res.poseAMCLqw = _pTF_pose.poseTFqw;
	/*
    ---
    bool   command_Result
    float  poseAMCLx
    float  poseAMCLy
    float  poseAMCLqx
    float  poseAMCLqy
    float  poseAMCLqz
    float  poseAMCLqw
	*/
	bResult = res.command_Result = true;
	return true;
}


bool Depart_Station2Move()
{
    bool bResult = false;
    printf("Depart_station2Move ... docking exit!! \n"); 

    float m_fdistance = 0.0;
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
    m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);
    printf("Depart_fdistance: %.5f \n", m_fdistance);
    if(m_fdistance < 0.2 && _pRobot_Status.m_iCallback_Charging_status < 2)
    {
        printf("Cant move because robot has docked but docking_station doesn`t work !! \n");
        cmd->linear.x = 0.0;           
    }
    else
    {
        if(_pAR_tag_pose.m_transform_pose_x <= 0.6) //600mm depart move
        {
            if(_pFlag_Value.m_bFlag_Obstacle_cygbot)
            {
                cmd->linear.x =  0.0; 
                cmd->angular.z = 0.0;
                //cmdpub_.publish(cmd);
                bResult = false;
            }
            else
            {
                cmd->linear.x =  -0.05; 
                cmd->angular.z = 0.0;
                //cmdpub_.publish(cmd);
                bResult = false;
            }    
        }
        else
        {
            cmd->linear.x =  0.0; 
            cmd->angular.z = 0.0;
            //cmdpub_.publish(cmd);

            //add goto cmd call//
            setGoal(goal);

            ex_iDocking_CommandMode = 0;

            bResult = true;
        }
    }

    cmdpub_.publish(cmd);
    return bResult;
}

bool Goto_Command(tetraDS_service::gotolocation::Request &req, 
				  tetraDS_service::gotolocation::Response &res)
{
	bool bResult = false;

    m_flag_setgoal = true;

    //costmap clear call//
	m_flag_clesr_costmap_call = clear_costmap_client.call(m_request);
	while(!m_flag_clesr_costmap_call)
	{
		sleep(1);
	}
	m_flag_clesr_costmap_call = false;

    LED_Toggle_Control(1,3,100,3,1);
    if(_pFlag_Value.m_bflag_patrol)
        LED_Turn_On(100); //blue led
    else
        LED_Turn_On(63); //White led


    bResult = OpenLocationFile(req.Location);
    printf("Goto bResult: %d \n", bResult);
    res.goal_positionX = _pGoal_pose.goal_positionX;
    res.goal_positionY = _pGoal_pose.goal_positionY;
    res.goal_quarterX  = _pGoal_pose.goal_quarterX;
    res.goal_quarterY  = _pGoal_pose.goal_quarterY;
    res.goal_quarterZ  = _pGoal_pose.goal_quarterZ;
    res.goal_quarterW  = _pGoal_pose.goal_quarterW;
    goto_goal_id.id = req.Location;

    ROS_INFO("goto_id.id: %s", goto_goal_id.id.c_str());

    if((_pRobot_Status.m_iCallback_Charging_status <= 1 && (!ex_bMissionDocking_Flag && !_pFlag_Value.m_bumperhit_flag)) && (_pAR_tag_pose.m_iAR_tag_id == -1 || ((_pAR_tag_pose.m_iAR_tag_id != _pRobot_Status.HOME_ID && _pAR_tag_pose.m_transform_pose_x >= 0.25) || _pAR_tag_pose.m_transform_pose_x >= 0.6))) //Nomal
    {
        ROS_INFO("Goto Nomal Loop !");
        setGoal(goal);
        bResult = true;
        
    }
    else if((_pFlag_Value.m_bumperhit_flag || ex_bMissionDocking_Flag) || (((_pAR_tag_pose.m_iAR_tag_id != -1) && (_pAR_tag_pose.m_iAR_tag_id != _pRobot_Status.HOME_ID)) && _pAR_tag_pose.m_transform_pose_x < 0.25))
    {
        ex_iDocking_CommandMode = 310;
        bResult = true;
    }
    else //docking
    {
        if(_pAR_tag_pose.m_iAR_tag_id == 0)
        {
            ex_iDocking_CommandMode = 10; //Depart Move
        }
        else
        {
            ex_iDocking_CommandMode = 310;
        }
        bResult = true;
    }

    if(req.Location == "HOME") //HOME Point Check
    {
        _pFlag_Value.m_bflag_ComebackHome = true;
    }

	/*
	string Location
    int32 mark_id
    int32 movement
    ---
    float64 goal_positionX
    float64 goal_positionY
    float64 goal_quarterX
    float64 goal_quarterY
    float64 goal_quarterZ
    float64 goal_quarterW
    bool command_Result
	*/
	res.command_Result = bResult;

    //reset flag...
    _pFlag_Value.m_bFlag_Disable_bumper = false;
    _pFlag_Value.m_bFlag_Initialpose = false;

    m_flag_setgoal = false;

	return true;
}

bool Goto_Command2(tetraDS_service::gotolocation2::Request &req, 
				   tetraDS_service::gotolocation2::Response &res)
{
	bool bResult = false;

    m_flag_setgoal = true;

    //costmap clear call//
	m_flag_clesr_costmap_call = clear_costmap_client.call(m_request);
	while(!m_flag_clesr_costmap_call)
	{
		sleep(1);
	}
	m_flag_clesr_costmap_call = false;

    _pGoal_pose.goal_positionX = req.goal_positionX;
    _pGoal_pose.goal_positionY = req.goal_positionY;
    _pGoal_pose.goal_quarterX = req.goal_quarterX;
    _pGoal_pose.goal_quarterY = req.goal_quarterY;
    _pGoal_pose.goal_quarterZ = req.goal_quarterZ;
    _pGoal_pose.goal_quarterW = req.goal_quarterW;
    //goto_goal_id.id = "1";
    //ros::Time(0);
    printf("Goto_Command2 !! \n");

    LED_Toggle_Control(1, 3,100,3,1);
    LED_Turn_On(63);

    if((_pRobot_Status.m_iCallback_Charging_status <= 1 && (!ex_bMissionDocking_Flag && !_pFlag_Value.m_bumperhit_flag)) && (_pAR_tag_pose.m_iAR_tag_id == -1 || ((_pAR_tag_pose.m_iAR_tag_id != _pRobot_Status.HOME_ID && _pAR_tag_pose.m_transform_pose_x >= 0.35) || _pAR_tag_pose.m_transform_pose_x >= 0.6))) //Nomal
    {
        setGoal(goal);
        bResult = true;
    }
    else if((_pFlag_Value.m_bumperhit_flag || ex_bMissionDocking_Flag) || (((_pAR_tag_pose.m_iAR_tag_id != -1) && (_pAR_tag_pose.m_iAR_tag_id != _pRobot_Status.HOME_ID)) && _pAR_tag_pose.m_transform_pose_x < 0.25))
    {
        ex_iDocking_CommandMode = 310;
        bResult = true;
    }
    else //Docking...
    {
        if(_pAR_tag_pose.m_iAR_tag_id == 0)
        {
            ex_iDocking_CommandMode = 10; //Depart Move
        }
        else
        {
            ex_iDocking_CommandMode = 310;
        }
        bResult = true;//false;
    }
	/*
    float goal_positionX
    float goal_positionY
    float goal_quarterX
    float goal_quarterY
    float goal_quarterZ
    float goal_quarterW
    ---
    bool command_Result
	*/
	res.command_Result = bResult;
    //reset flag...
    _pFlag_Value.m_bFlag_Disable_bumper = false;
    _pFlag_Value.m_bFlag_Initialpose = false;

    m_flag_setgoal = false;

	return true;
}

bool GotoCancel_Command(tetraDS_service::gotocancel::Request &req, 
				        tetraDS_service::gotocancel::Response &res)
{
	bool bResult = false;

    m_flag_setgoal = true;

    goto_goal_id.id = "";
    ROS_INFO("Goto Cancel call");
    GotoCancel_pub.publish(goto_goal_id);
	/*
	string Location_id
    ---
    bool command_Result
	*/
    bResult = true;
	res.command_Result = bResult;
    ex_iDocking_CommandMode = 0;

    m_flag_setgoal = false;
    _pFlag_Value.m_bFlag_pub = false;
	return true;
}

bool SetLocation_Command(tetraDS_service::setlocation::Request  &req, 
					     tetraDS_service::setlocation::Response &res)
{
	bool bResult = false;

    if(req.Location == "HOME")
    {
        res.command_Result = false;
        printf("[ERROR]: HOME location cannot be saved! \n");
    }
    else
    {
        /*
        res.command_Result = SaveLocation(req.Location, 
                                        _pAMCL_pose.poseAMCLx, _pAMCL_pose.poseAMCLy,
                                        _pAMCL_pose.poseAMCLqx, _pAMCL_pose.poseAMCLqy, _pAMCL_pose.poseAMCLqz, _pAMCL_pose.poseAMCLqw);
        
        res.goal_positionX = _pAMCL_pose.poseAMCLx;
        res.goal_positionY = _pAMCL_pose.poseAMCLy;
        res.goal_quarterX  = _pAMCL_pose.poseAMCLqx;
        res.goal_quarterY  = _pAMCL_pose.poseAMCLqy;
        res.goal_quarterZ  = _pAMCL_pose.poseAMCLqz;
        res.goal_quarterW  = _pAMCL_pose.poseAMCLqw;
        */

        res.command_Result = SaveLocation(req.Location, 
                                _pTF_pose.poseTFx,_pTF_pose.poseTFy,
                                _pTF_pose.poseTFqx,_pTF_pose.poseTFqy,_pTF_pose.poseTFqz,_pTF_pose.poseTFqw);
        
        res.goal_positionX = _pTF_pose.poseTFx;
        res.goal_positionY = _pTF_pose.poseTFy;
        res.goal_quarterX  = _pTF_pose.poseTFqx;
        res.goal_quarterY  = _pTF_pose.poseTFqy;
        res.goal_quarterZ  = _pTF_pose.poseTFqz;
        res.goal_quarterW  = _pTF_pose.poseTFqw;
    }

    /*
	string Location
    ---
    float goal_positionX
    float goal_positionY
    float goal_quarterX
    float goal_quarterY
    float goal_quarterZ
    float goal_quarterW
    bool command_Result
	*/
	bResult = res.command_Result;

	return true;
}

void CMD_SaveMap(string strMapname)
{
    //call rosrun command//
    string str_command = "gnome-terminal -- rosrun map_server map_saver --occ 55 --free 45 -f ";
    string str_command2 = str_command + strMapname;
    std::vector<char> writable1(str_command2.begin(), str_command2.end());
    writable1.push_back('\0');
    char* ptr1 = &writable1[0];

    int iResult = std::system(ptr1);
}


bool SetSavemap_Command(tetraDS_service::setsavemap::Request &req, 
					    tetraDS_service::setsavemap::Response &res)
{
	bool bResult = false;
    
    ROS_INFO("Save Map Call _ %s", req.map_name.c_str());

    //call rosrun command//
    string str_command = "gnome-terminal -- /home/tetra/mapsave.sh ";
    string str_command2 = str_command + req.map_name.c_str();

    std::vector<char> writable1(str_command2.begin(), str_command2.end());
    writable1.push_back('\0');
    char* ptr1 = &writable1[0];

    int iResult = std::system(ptr1);
    /*
    string map_name
    ---
    bool command_Result
    */
	bResult = res.command_Result = true;
    //bResult = true;
	return true;
}

bool GetInformation_Command(tetraDS_service::getinformation::Request  &req, 
					        tetraDS_service::getinformation::Response &res)
{
	bool bResult = false;
    res.command_Result = false;

    //Get Data
    res.battery = _pRobot_Status.m_iCallback_Battery;
    res.Error_Code = _pRobot_Status.m_iCallback_ErrorCode;
    res.EMG = _pRobot_Status.m_iCallback_EMG;
    res.bumper = _pRobot_Status.m_iCallback_Bumper;
    res.charging = _pRobot_Status.m_iCallback_Charging_status;
    res.running_mode = ex_ilaunchMode; //0:nomal, 1:mapping, 2:navigation
    /*
    ---
    bool command_Result
    int32 battery
    int32 Error_Code
    bool EMG
    bool bumper
    bool charging
    */

	bResult = res.command_Result = true;
	return true;
}

bool Docking_Command(tetraDS_service::dockingcontrol::Request  &req, 
					 tetraDS_service::dockingcontrol::Response &res)
{
	bool bResult = false;
    _pAR_tag_pose.m_iSelect_AR_tag_id = req.id;
    //_pRobot_Status.HOME_ID = _pAR_tag_pose.m_iSelect_AR_tag_id = req.id;
    ex_iDocking_CommandMode = req.mode;

    /*
    int32 id
    int32 mode
    ---
    bool command_Result
    */

	bResult = res.command_Result = true;
	return true;
}

void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
    _pAMCL_pose.poseAMCLx = msgAMCL->pose.pose.position.x;
    _pAMCL_pose.poseAMCLy = msgAMCL->pose.pose.position.y;
    _pAMCL_pose.poseAMCLz = msgAMCL->pose.pose.position.z;
    _pAMCL_pose.poseAMCLqx = msgAMCL->pose.pose.orientation.x;
    _pAMCL_pose.poseAMCLqy = msgAMCL->pose.pose.orientation.y;
    _pAMCL_pose.poseAMCLqz = msgAMCL->pose.pose.orientation.z;
    _pAMCL_pose.poseAMCLqw = msgAMCL->pose.pose.orientation.w;

}


bool LocationList_Command(tetraDS_service::getlocationlist::Request  &req, 
					      tetraDS_service::getlocationlist::Response &res)
{
    bool bResult = false;

    int m_icnt =0;
    DIR *dir;
    struct dirent *ent;
    dir = opendir ("/home/tetra/DATA/"); //file path
    if (dir != NULL) 
    {
        res.command_Result = true;
        //print all the files and directories within directory
        while ((ent = readdir (dir)) != NULL)
        {
            if(strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) 
            { 
                continue; 
            }
            m_icnt++;
            char *listname = strchr(ent->d_name, '.');
            if(listname != NULL){
                *listname = 0;
            }
            //printf ("%s\n", ent->d_name);
            res.location_name.push_back(ent->d_name);

        }

        //Total List number
        res.list_num = m_icnt;
        closedir (dir);
    } 
    else 
    {
        //could not open directory
        perror ("");
        res.command_Result = false;
    }

    /*
    ---
    int32 list_num
    string[] location_name
    bool command_Result
    */

    bResult = res.command_Result;
    return true;
}

bool LandmarkList_Command(tetraDS_service::getlandmarklist::Request  &req, 
					      tetraDS_service::getlandmarklist::Response &res)
{
    bool bResult = false;

    int m_icnt =0;
    DIR *dir;
    struct dirent *ent;
    dir = opendir ("/home/tetra/LANDMARK/"); //file path
    if (dir != NULL) 
    {
        res.command_Result = true;
        //print all the files and directories within directory
        while ((ent = readdir (dir)) != NULL)
        {
            if(strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) 
            { 
                continue; 
            }
            m_icnt++;
            char *listname = strchr(ent->d_name, '.');
            if(listname != NULL){
                *listname = 0;
            }
            //printf ("%s\n", ent->d_name);
            res.landmark_name.push_back(ent->d_name);

        }

        //Total List number
        res.list_num = m_icnt;
        closedir (dir);
    } 
    else 
    {
        //could not open directory
        perror ("");
        res.command_Result = false;
    }

    bResult = res.command_Result;
    return true;
}

bool MapList_Command(tetraDS_service::getmaplist::Request &req, 
					 tetraDS_service::getmaplist::Response &res)
{
    bool bResult = false;

    //Load File List
    int m_icnt =0;
    DIR *dir;
    struct dirent *ent;
    dir = opendir ("/home/tetra/catkin_ws/src/tetraDS_2dnav/maps/"); //file path

    if (dir != NULL) 
    {
        res.command_Result = true;
        //print all the files and directories within directory
        while ((ent = readdir (dir)) != NULL)
        {
            if(strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) 
            { 
                continue; 
            }
            if(strstr(ent->d_name, ".yaml") == 0)
            {
                m_icnt++;
                char *listname = strchr(ent->d_name, '.');
                if(listname != NULL){
                    *listname = 0;
                }
                //printf("%s\n", ent->d_name);
                res.map_name.push_back(ent->d_name);
            }

        }

        //Total List number
        res.list_num = m_icnt;
        closedir (dir);
    } 
    else 
    {
        //could not open directory
        perror ("");
        res.command_Result = false;
    }

    /*
    string IP_address
    ---
    int32 list_num
    string[] map_name
    bool command_Result
    */

    bResult = res.command_Result;
    return true;
}

bool DeleteLocation_Command(tetraDS_service::deletelocation::Request  &req, 
				           tetraDS_service::deletelocation::Response &res)
{
    bool bResult = false;

    string m_strLocationName;
    m_strLocationName = "/home/tetra/DATA/" + req.Location + ".txt";  
    int iResult = remove(m_strLocationName.c_str());
    if(iResult == 0)
    {
        res.command_Result = true;
    }
    else
    {
        res.command_Result = false;
    }
    
    /*
    string Location
    ---
    bool command_Result
    */

    bResult = res.command_Result;
    return true;
}

bool DeleteLandmark_Command(tetraDS_service::deletelandmark::Request  &req, 
				           tetraDS_service::deletelandmark::Response &res)
{
    bool bResult = false;

    string m_strLandmarkName;
    m_strLandmarkName = "/home/tetra/LANDMARK/" + req.Landmark + ".txt";  
    
    int iResult = remove(m_strLandmarkName.c_str());
    if(iResult == 0)
    {
        res.command_Result = true;
    }
    else
    {
        res.command_Result = false;
    }
    

    bResult = res.command_Result;
    return true;
}

bool DeleteMap_Command(tetraDS_service::deletemap::Request &req, 
				      tetraDS_service::deletemap::Response &res)
{
    bool bResult = false;

    string m_strMapName_yaml;
    string m_strMapName_pgm;
    m_strMapName_yaml = "/home/tetra/catkin_ws/src/tetraDS_2dnav/maps/" + req.map_name + ".yaml";  
    m_strMapName_pgm  = "/home/tetra/catkin_ws/src/tetraDS_2dnav/maps/" + req.map_name + ".pgm";

    //yaml file erase
    int iResult1 = remove(m_strMapName_yaml.c_str());
    if(iResult1 == 0)
    {
        res.command_Result = true;
    }
    else
    {
        res.command_Result = false;
    }
    //pgm file erase
    int iResult2 = remove(m_strMapName_pgm.c_str());
    if(iResult2 == 0)
    {
        res.command_Result = true;
    }
    else
    {
        res.command_Result = false;
    }
    
    /*
    string map_name
    ---
    bool command_Result
    */
    bResult = res.command_Result;
    return true;
}



bool Virtual_Obstacle_Command(tetraDS_service::virtual_obstacle::Request &req, 
				              tetraDS_service::virtual_obstacle::Response &res)
{
	bool bResult = false;
    int m_iInt_count = 0;
    int m_iNext_count = 0;

    // msg clear
    virtual_obstacle.list.clear();
    //Global_costmap Loop//
    virtual_obstacle.list.resize(req.list_count.size());
    for(int i=0; i<req.list_count.size(); i++)
    {
        //virtual_obstacle.list[i].form.clear();
        virtual_obstacle.list[i].form.resize(req.list_count[i]);
        m_iInt_count = req.list_count[i];
        for(int j=0; j<req.list_count[i]; j++)
        {
            virtual_obstacle.list[i].form[j].x = floor(req.form_x[m_iNext_count + j] * 1000.f + 0.5) / 1000.f;
            virtual_obstacle.list[i].form[j].y = floor(req.form_y[m_iNext_count + j] * 1000.f + 0.5) / 1000.f;
            virtual_obstacle.list[i].form[j].z = floor(req.form_z[m_iNext_count + j] * 1000.f + 0.5) / 1000.f;
        }
        m_iNext_count += m_iInt_count;
        
    }
    
    if(m_bFlag_nomotion_call || !_pFlag_Value.m_bFlag_nomotion || m_flag_Dynamic_reconfigure_call || m_flag_setgoal || _pFlag_Value.m_bTebMarker_reconfigure_flag)
    {
        bResult = false;
		res.command_Result = bResult;
		return true;
    
	}
	virtual_obstacle_pub.publish(virtual_obstacle);
        // 231115 add
    	if(virtual_obstacle.list.size() != 0 )
    	{
            m_bFlag_virtualWallCheck = true;
    	}
	/*
	int32[]  list_count
	float64[] form_x
	float64[] form_y
	float64[] form_z
	---
	bool command_Result
    */
    bResult = true;
	res.command_Result = bResult;
	return true;
}

bool Patrol_Command(tetraDS_service::patrol::Request &req, 
				    tetraDS_service::patrol::Response &res)
{
	bool bResult = false;


    _pFlag_Value.m_bflag_auto_patrol = _pFlag_Value.m_bflag_patrol = req.on;
    m_patrol_location_cnt = req.list_count;
    printf("# m_bflag_patrol: %d  \n", _pFlag_Value.m_bflag_patrol);
    printf("# m_patrol_location_cnt: %d  \n", m_patrol_location_cnt);

    for(int i=0; i<m_patrol_location_cnt; i++)
    {
        arr_patrol_location[i] = req.location_name[i];
        ROS_INFO("# arr_patrol_location[%d]:%s ", i, arr_patrol_location[i].c_str());
    }
   
	/*
    bool on
    int32  list_count
    string[] location_name
    ---
    bool command_Result
	*/
    bResult = true;
	res.command_Result = bResult;
	return true;
}

// bool Patrol_Conveyor_Command(tetraDS_service::patrol_conveyor::Request &req, 
// 				             tetraDS_service::patrol_conveyor::Response &res)
// {
// 	bool bResult = false;

//     _pFlag_Value.m_bflag_patrol2 = req.on;
//     m_iLoading_ID = req.loading_id;
//     m_iUnloading_ID = req.unloading_id;
//     m_strLoading_loacation_name = req.loading_location_name;
//     m_strUnloading_loacation_name = req.unloading_location_name;

// 	/*
//     bool on
//     int32  loading_id
//     int32  unloading_id
//     string loading_location_name
//     string unloading_location_name
//     ---
//     bool command_Result
// 	*/
//     bResult = true;
// 	res.command_Result = bResult;
// 	return true;
// }

void Reset_EKF_SetPose()
{
	//robot_localization::SetPose ekf_reset;
	setpose_srv.request.pose.header.frame_id = tf_prefix_ + "/odom";
	setpose_srv.request.pose.header.stamp = ros::Time(0); //ros::Time::now();

	setpose_srv.request.pose.pose.pose.position.x = 0.0; //_pTF_pose.poseTFx;
	setpose_srv.request.pose.pose.pose.position.y = 0.0; //_pTF_pose.poseTFy;
	setpose_srv.request.pose.pose.pose.position.z = 0.0; //_pTF_pose.poseTFz;

	setpose_srv.request.pose.pose.pose.orientation.x = 0.0;
	setpose_srv.request.pose.pose.pose.orientation.y = 0.0;
	setpose_srv.request.pose.pose.pose.orientation.z = 0.0; //_pTF_pose.poseTFqz;
	setpose_srv.request.pose.pose.pose.orientation.w = 1.0; //_pTF_pose.poseTFqw;

	setpose_srv.request.pose.pose.covariance[0] = 0.25;
	setpose_srv.request.pose.pose.covariance[6 * 1 + 1] = 0.25;
	setpose_srv.request.pose.pose.covariance[6 * 5 + 5] = 0.06853892326654787;
    
	SetPose_cmd_client.call(setpose_srv); //Set_pose call//
	//printf("##Set_Pose(EKF)! \n");
    usleep(200000);

    initPose_.header.stamp = ros::Time(0); //ros::Time::now(); 
    initPose_.header.frame_id = "map";
    //position
    initPose_.pose.pose.position.x = 0.0;
    initPose_.pose.pose.position.y = 0.0;
    initPose_.pose.pose.position.z = 0.0;
    //orientation
    initPose_.pose.pose.orientation.x = 0.0;
    initPose_.pose.pose.orientation.y = 0.0;
    initPose_.pose.pose.orientation.z = 0.0;
    initPose_.pose.pose.orientation.w = 1.0;

    initPose_.pose.covariance[0] = 0.25;
    initPose_.pose.covariance[6 * 1 + 1] = 0.25;
    initPose_.pose.covariance[6 * 5 + 5] = 0.06853892326654787;

    initialpose_pub.publish(initPose_);
    //printf("##Set_initPose(2D Estimate)! \n");

    usleep(500000);

    //_pFlag_Value.m_bFlag_nomotion = true;
}

bool Marker_Reset_Robot_Pose()
{
    _pFlag_Value.m_bFlag_nomotion = false;

    bool bResult = false;
    string m_strFilePathName;
    string landmark_name = "marker_" + std::to_string(_pAR_tag_pose.m_iAR_tag_id);
    if (_pAR_tag_pose.m_iAR_tag_id == -1) 
    {
        bResult = false;
        return -1;
    }
    else 
    {
        //printf("#Docking Reset Marker_ID: %d \n", _pAR_tag_pose.m_iAR_tag_id);
        bResult = true;
    }

   
    m_strFilePathName = "/home/tetra/LANDMARK/" + landmark_name + ".txt";
    fp = fopen(m_strFilePathName.c_str(), "r");

    if(_pAR_tag_pose.m_iAR_tag_id > 0 && fp == NULL) 
    {
        _pFlag_Value.m_bFlag_nomotion = true;
        return false;
    }


    if (fp != NULL) //File Open Check
    {
        while (!feof(fp))
        {
            if (fgets(Textbuffer, sizeof(Textbuffer), fp) != NULL)
            {
                char* ptr = strtok(Textbuffer, ",");
                int icnt = 0;

                while (ptr != NULL)
                {
                    ptr = strtok(NULL, ",");
                    switch (icnt)
                    {
                    case 6:
                        _pLandMarkPose.init_position_x = atof(ptr);
                        break;
                    case 7:
                        _pLandMarkPose.init_position_y = atof(ptr);
                        break;
                    case 8:
                        _pLandMarkPose.init_position_z = atof(ptr);
                        break;
                    case 9:
                        _pLandMarkPose.init_orientation_z = atof(ptr);
                        break;
                    case 10:
                        _pLandMarkPose.init_orientation_w = atof(ptr);
                        break;
                    }

                    icnt++;
                }
            }
        }
        fclose(fp);
	    initPose_.header.stamp = ros::Time(0); //ros::Time::now(); 
	    initPose_.header.frame_id = "map";
	    //position
	    initPose_.pose.pose.position.x = _pLandMarkPose.init_position_x;
	    initPose_.pose.pose.position.y = _pLandMarkPose.init_position_y;
	    initPose_.pose.pose.position.z = _pLandMarkPose.init_position_z;
	    //orientation
	    initPose_.pose.pose.orientation.x = 0.0;
	    initPose_.pose.pose.orientation.y = 0.0;
	    initPose_.pose.pose.orientation.z = -1.0 * _pLandMarkPose.init_orientation_z;
	    initPose_.pose.pose.orientation.w = _pLandMarkPose.init_orientation_w;

	    initPose_.pose.covariance[0] = 0.25;
	    initPose_.pose.covariance[6 * 1 + 1] = 0.25;
	    initPose_.pose.covariance[6 * 5 + 5] = 0.06853892326654787;

        //publish msg
        initialpose_pub.publish(initPose_);

        printf("$$$$ init_position_x: %f , init_position_y: %f \n", _pLandMarkPose.init_position_x, _pLandMarkPose.init_position_y);
    }
    else
    {
        bResult = false;
    }

    _pFlag_Value.m_bFlag_nomotion = true;

    return bResult;
}


void Reset_Robot_Pose()
{
    if(_pRobot_Status.HOME_ID == _pAR_tag_pose.m_iAR_tag_id) // Same as Home ID... Loop 
    {
        _pFlag_Value.m_bFlag_nomotion = false;

        //IMU reset//
        euler_angle_reset_cmd_client.call(euler_angle_reset_srv);
	    printf("## IMU Reset ! \n");
        //tetra odometry Reset//
        tetra_PoseRest.data = m_iReset_flag;
        PoseReset_pub.publish(tetra_PoseRest);
        usleep(100000);

        Reset_EKF_SetPose();
    }

    Marker_Reset_Robot_Pose();

    //costmap clear call//
    //clear_costmap_client.call(m_request);

    virtual_obstacle2.list.clear();
    virtual_obstacle2_pub.publish(virtual_obstacle2);

    _pFlag_Value.m_bFlag_Initialpose = true;

    LED_Toggle_Control(1, 3,100,3,1);
    LED_Turn_On(63);
    
}

bool SetInitPose_Command(tetraDS_service::setinitpose::Request  &req, 
					     tetraDS_service::setinitpose::Response &res)
{
    _pFlag_Value.m_bFlag_nomotion = false;

    bool bResult = false;

    string landmark_name = "marker_" + std::to_string(_pAR_tag_pose.m_iAR_tag_id);
    if(_pAR_tag_pose.m_iAR_tag_id == -1)
    {
        bResult = false; 
    }
    else
    {
        res.m_iAR_tag_id = _pAR_tag_pose.m_iAR_tag_id;
        bResult = true; 
    }

    string m_strFilePathName;
    m_strFilePathName = "/home/tetra/LANDMARK/" + landmark_name + ".txt";  
    fp = fopen(m_strFilePathName.c_str(), "r");  

    if(fp != NULL) //File Open Check
    {
        while(!feof(fp))
        {
            if(fgets(Textbuffer, sizeof(Textbuffer), fp) != NULL)
            {
                char* ptr = strtok(Textbuffer, ",");
                int icnt = 0;

                while(ptr != NULL)
                {   
                    ptr = strtok(NULL, ",");
                    switch(icnt)
                    {
                        case 6:
                            _pLandMarkPose.init_position_x = atof(ptr);
                            res.init_position_x = _pLandMarkPose.init_position_x;
                            break;
                        case 7:
                            _pLandMarkPose.init_position_y = atof(ptr);
                            res.init_position_y = _pLandMarkPose.init_position_y;
                            break;
                        case 8:
                            _pLandMarkPose.init_position_z = atof(ptr);
                            res.init_position_z = _pLandMarkPose.init_position_z;
                            break;
                        case 9:
                            _pLandMarkPose.init_orientation_z = atof(ptr);
                            res.init_orientation_z = _pLandMarkPose.init_orientation_z;
                            break;
                        case 10:
                            _pLandMarkPose.init_orientation_w = atof(ptr);
                            res.init_orientation_w = _pLandMarkPose.init_orientation_w;
                            break;
                    }
        
                    icnt++;
                }
            }
        }                
        fclose(fp);
    }

    initPose_.header.stamp = ros::Time(0); //ros::Time::now(); 
    initPose_.header.frame_id = "map";
    //position
    initPose_.pose.pose.position.x = _pLandMarkPose.init_position_x;
    initPose_.pose.pose.position.y = _pLandMarkPose.init_position_y;
    initPose_.pose.pose.position.z = _pLandMarkPose.init_position_z;
    //orientation
    initPose_.pose.pose.orientation.x = 0.0;
    initPose_.pose.pose.orientation.y = 0.0;
    initPose_.pose.pose.orientation.z = _pLandMarkPose.init_orientation_z;
    initPose_.pose.pose.orientation.w = _pLandMarkPose.init_orientation_w;
    
    initPose_.pose.covariance[0] = 0.25;
    initPose_.pose.covariance[6 * 1 + 1] = 0.25;
    initPose_.pose.covariance[6 * 5 + 5] = 0.06853892326654787;

    //publish msg
    initialpose_pub.publish(initPose_);
    //costmap clear call//
    clear_costmap_client.call(m_request);
    res.command_Result = bResult;
    _pFlag_Value.m_bFlag_nomotion = true;

    return true;
}

bool Set2D_Pose_Estimate_Command(tetraDS_service::pose_estimate::Request  &req, 
					             tetraDS_service::pose_estimate::Response &res)
{
    _pFlag_Value.m_bFlag_nomotion = false;

    bool bResult = false;
    initPose_.header.stamp = ros::Time(0); //ros::Time::now(); 
    initPose_.header.frame_id = "map";
    //position
    initPose_.pose.pose.position.x = req.estimate_position_x;
    initPose_.pose.pose.position.y = req.estimate_position_y;
    initPose_.pose.pose.position.z = req.estimate_position_z;
    //orientation
    initPose_.pose.pose.orientation.x = req.estimate_orientation_x;
    initPose_.pose.pose.orientation.y = req.estimate_orientation_y;
    initPose_.pose.pose.orientation.z = req.estimate_orientation_z;
    initPose_.pose.pose.orientation.w = req.estimate_orientation_w;
    
    initPose_.pose.covariance[0] = 0.25;
    initPose_.pose.covariance[6 * 1 + 1] = 0.25;
    initPose_.pose.covariance[6 * 5 + 5] = 0.06853892326654787;

    //publish msg
    initialpose_pub.publish(initPose_);
    //costmap clear call//
    clear_costmap_client.call(m_request);
    bResult = true;
    res.command_Result = bResult;
    _pFlag_Value.m_bFlag_nomotion = true;
/*
    float64 estimate_position_x
    float64 estimate_position_y
    float64 estimate_position_z
    float64 estimate_orientation_x
    float64 estimate_orientation_y
    float64 estimate_orientation_z
    float64 estimate_orientation_w
    ---
    bool command_Result
*/
    return true;
}

void reCharging()
{
    _pFlag_Value.m_Onetime_reset_cnt = 0;
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
    ROS_INFO("reCharging...");
    cmd->linear.x = 0.1;
    cmd->angular.z = 0.0;
    cmdpub_.publish(cmd);
    sleep(2);
    cmd->linear.x = -0.1;
    cmd->angular.z = 0.0;
    cmdpub_.publish(cmd);
    sleep(2);
    cmd->linear.x = 0.0;
    cmd->angular.z = 0.0;
    cmdpub_.publish(cmd);
}

void ChargingCallback(const std_msgs::Int32::ConstPtr& msg)
{
    _pRobot_Status.m_iCallback_Charging_status = msg->data; //docking_status
    
    if(_pRobot_Status.m_iCallback_Charging_status == 7)
    {
        _pFlag_Value.m_Onetime_reset_cnt++;
        if(_pFlag_Value.m_Onetime_reset_cnt > 100) reCharging();
    }else{
        _pFlag_Value.m_Onetime_reset_cnt = 0;
    }
}

void BatteryCallback(const std_msgs::Int32::ConstPtr& msg)
{
    _pRobot_Status.m_iCallback_Battery = msg->data;
}

void EMGCallback(const std_msgs::Int32::ConstPtr& msg)
{
    _pRobot_Status.m_iCallback_EMG = msg->data;
    if(_pRobot_Status.m_iCallback_EMG != 0)
    {
        LED_Toggle_Control(1, 10,100,10,1);
        LED_Turn_On(18);
        printf("[EMG] Push EMG button!! _ RED LED On \n");
        _pFlag_Value.m_emgpush_flag = false;
    }
    else
    {
        if(!_pFlag_Value.m_emgpush_flag)
        {
            LED_Toggle_Control(1, 3,100,3,1);
            LED_Turn_On(63);
            _pFlag_Value.m_emgpush_flag = true;
        }
    }
}

void BumperCallback(const std_msgs::Int32::ConstPtr& msg)
{
    bool isChange= _pRobot_Status.m_iCallback_Bumper != msg->data;
    _pRobot_Status.m_iCallback_Bumper = msg->data;

    if(_pRobot_Status.m_iCallback_Bumper == 1)
    {
        memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &p_side_x_, sizeof(float));
        memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &p_side_y_, sizeof(float));
        memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &pc_radius_, sizeof(float));
        memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &p_side_x_, sizeof(float));
        memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &n_side_y_, sizeof(float));
        pointcloud_.header.stamp = ros::Time(0);
        pointcloud_pub_.publish(pointcloud_);

        if(!_pFlag_Value.m_bFlag_Disable_bumper)
        {
            LED_Toggle_Control(1, 10,100,10,1);
            LED_Turn_On(18);
            if(isChange) printf("[Bumper] Push Bumper!! _ RED LED On \n");
       
            if(_pFlag_Value.m_bflagGo)
            {
                goto_goal_id.id = "";
                ROS_INFO("[Bumper On]Goto Cancel call");
                GotoCancel_pub.publish(goto_goal_id);
                _pFlag_Value.m_bflagGo = false;
                _pFlag_Value.m_bflagGo2 = true;
                if(_pFlag_Value.BUMPER_BT)
                    ex_iDocking_CommandMode = 100;
            }
        }
        _pFlag_Value.m_bumperhit_flag = true;
    }
    else if(_pRobot_Status.m_iCallback_Bumper == 2 || _pRobot_Status.m_iCallback_Bumper == 3)
    {
        if(isChange) printf("[Switch] Push Servo Switch Button !! \n");
    }
    else // 230629 ... update loop by mwcha
    { 
        memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
        memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &P_INF_Y, sizeof(float));
        memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
        memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
        memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &N_INF_Y, sizeof(float));
        pointcloud_.header.stamp = ros::Time(0);
        pointcloud_pub_.publish(pointcloud_);

        if(_pFlag_Value.m_bumperhit_flag)
        {
            LED_Toggle_Control(1,3,100,3,1);
            LED_Turn_On(63); //White led
            _pFlag_Value.m_bumperhit_flag = false;
        }

    }

}

//Conveyor function
void LoadcellCallback(const std_msgs::Float64::ConstPtr& msg)
{
    _pRobot_Status.m_dLoadcell_weight = msg->data;
}

void SensorCallback(const std_msgs::Int32::ConstPtr& msg)
{
    _pRobot_Status.m_iConveyor_Sensor_info = msg->data;
}
//add.. Total Distance Callback ... 240129 mwcha
void TotalDistance_Callback(const std_msgs::Float32::ConstPtr& msg)
{
    _pRobot_Status.m_dTotal_Distance = msg->data;
}
/////////LanMark file Write & Read Fuction///////////
bool SaveLandMark(LANDMARK_POSE p)
{
    bool bResult = false;

    string m_strFilePathName;
    m_strFilePathName = "/home/tetra/LANDMARK/" + p.ns + ".txt";    
    fp = fopen(m_strFilePathName.c_str(), "w");
    if(fp == NULL)
    { 
        ROS_INFO("file is null");
        bResult = false;
    }
    else
    {
        fprintf(fp, "0,%s,%s,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf \n",
                p.header_frame_id.c_str(), p.ns.c_str(), p.mark_id,
                p.pose_position_x, p.pose_position_y, p.pose_position_z, 
                p.pose_orientation_x, p.pose_orientation_y, p.pose_orientation_z, p.pose_orientation_w);
        fclose(fp);

        bResult = true;
    }

    return bResult;
}



//AR_tagCallback
void AR_tagCallback(ar_track_alvar_msgs::AlvarMarkers req) 
{
    if (!req.markers.empty()) 
    {
        //add...find ID array Index Loop...
        for (int i = 0; i < req.markers.size(); i++)
        {
            if (_pAR_tag_pose.m_iSelect_AR_tag_id == req.markers[i].id)
            {
                _pAR_tag_pose.m_iAR_tag_id_Index = i;
            }
        }

        //AR_Tag data update...
        _pAR_tag_pose.m_iAR_tag_id = req.markers[_pAR_tag_pose.m_iAR_tag_id_Index].id;
        _pAR_tag_pose.m_fAR_tag_pose_x = req.markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.position.x;
        _pAR_tag_pose.m_fAR_tag_pose_y = req.markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.position.y;
        _pAR_tag_pose.m_fAR_tag_pose_z = req.markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.position.z;
        _pAR_tag_pose.m_fAR_tag_orientation_x = req.markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.orientation.x;
        _pAR_tag_pose.m_fAR_tag_orientation_y = req.markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.orientation.y;
        _pAR_tag_pose.m_fAR_tag_orientation_z = req.markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.orientation.z;
        _pAR_tag_pose.m_fAR_tag_orientation_w = req.markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.orientation.w;
        
        //< Declaration of quaternion
        tf::Quaternion q;
        q.setW(_pAR_tag_pose.m_fAR_tag_orientation_w);
        q.setX(_pAR_tag_pose.m_fAR_tag_orientation_x);
        q.setY(_pAR_tag_pose.m_fAR_tag_orientation_y);
        q.setZ(_pAR_tag_pose.m_fAR_tag_orientation_z);
        //< quaternion -> rotation Matrix
        tf::Matrix3x3 m(q);
        //< rotation Matrix - > quaternion
        m.getRotation(q);
        //< rotation Matrix -> rpy
        m.getRPY(_pAR_tag_pose.m_fAR_tag_roll, _pAR_tag_pose.m_fAR_tag_pitch, _pAR_tag_pose.m_fAR_tag_yaw);
        _pAR_tag_pose.m_fPositioning_Angle = _pAR_tag_pose.m_fAR_tag_pitch * (180.0/M_PI);

        //Transform Axis
        _pAR_tag_pose.m_transform_pose_x = _pAR_tag_pose.m_fAR_tag_pose_z;
        _pAR_tag_pose.m_transform_pose_y = _pAR_tag_pose.m_fAR_tag_pose_x;

    }
    else
    {
        _pAR_tag_pose.m_iAR_tag_id = -1;
    }  
}

// add Move_base Die Check
bool checkNode(std::string& node_name, std::string& node_name2)
{
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
    std::vector<std::string> node_list;
    ros::master::getNodes(node_list);

    bool m_bCheck_flag = false;

    for (unsigned int node_list_idx = 0; node_list_idx < node_list.size(); node_list_idx++)
    {
        if (node_list[node_list_idx] == node_name2) {
            m_bCheck_flag = true;
        }
        if (node_list[node_list_idx] == node_name) {
            return true;
        }
    }

    if(m_bCheck_flag){
        _pGoal_pose.goal_positionX = 0.0;
        _pGoal_pose.goal_positionY = 0.0;
    }else{
        ROS_ERROR("move_base error");
        cmd->linear.x = 0.0;
        cmd->angular.z = 0.0;
        cmdpub_.publish(cmd);
        _pFlag_Value.m_bFlag_pub = true;
    }
    
    return false;
}

void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msgStatus)
{
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
    int status_id = 0;
    //uint8 PENDING         = 0  
    //uint8 ACTIVE          = 1 
    //uint8 PREEMPTED       = 2
    //uint8 SUCCEEDED       = 3
    //uint8 ABORTED         = 4
    //uint8 REJECTED        = 5
    //uint8 PREEMPTING      = 6
    //uint8 RECALLING       = 7
    //uint8 RECALLED        = 8
    //uint8 LOST            = 9
    if(!msgStatus->status_list.empty())
    {
        actionlib_msgs::GoalStatus goalStatus = msgStatus->status_list[0];
        status_id = goalStatus.status;
        //ROS_INFO("[move_base]StatusCallback: %d ",status_id);
    }
    else
    {
        if(_pFlag_Value.m_bFlag_pub)
        {
            ROS_INFO("move_base die Catch !! ");
            //Stop
            cmd->linear.x =  0.0; 
            cmd->angular.z = 0.0;
            cmdpub_.publish(cmd);
            //Retry setGoal..
            if(
                _pGoal_pose.goal_positionX > 0.01 || 
                _pGoal_pose.goal_positionY > 0.01 || 
                _pGoal_pose.goal_positionX < -0.01 || 
                _pGoal_pose.goal_positionY < -0.01
            )
            {
                setGoal(goal);
            }
            _pFlag_Value.m_bFlag_pub = false;
        }
    }
}

constexpr unsigned int HashCode(const char* str)
{
    return str[0] ? static_cast<unsigned int>(str[0]) + 0xEDB8832Full * HashCode(str + 1) : 8603;
}

bool terminal_call()
{
    bool m_Result = false;

    string str_command = "gnome-terminal";
    std::vector<char> writable(str_command.begin(), str_command.end());
    writable.push_back('\0');
    char* ptr = &writable[0];
    int iResult = std::system(ptr);

    m_Result = true;
    return m_Result;
}

bool rosrun_mapping()
{
    bool m_Result = false;

    string str_command = "gnome-terminal -- /home/tetra/mapping.sh ";
    std::vector<char> writable2(str_command.begin(), str_command.end());
    writable2.push_back('\0');
    char* ptr2 = &writable2[0];
    int iResult = std::system(ptr2);

    ex_ilaunchMode = 1;

    m_Result = true;
    return m_Result;
}

bool rosnodekill_all()
{
    bool m_Result = false;

    string str_command = "rosnode list | grep \"/\\w*/move_base\\|/\\w*/amcl\\|/\\w*/tetraDS_landmark\\|/\\w*/map_server\\|/\\w*/ar_track_alvar2\\|/\\w*/world_linker\\|/\\w*/cartographer_\\|/\\w*/rviz\" | xargs rosnode kill";
    std::vector<char> writable3(str_command.begin(), str_command.end());
    writable3.push_back('\0');
    char* ptr3 = &writable3[0];
    int iResult = std::system(ptr3);

    ex_ilaunchMode = 0;

    m_Result = true;
    return m_Result;
}

bool rosrun_navigation(string strMapname)
{
    bool m_Result = false;

    string str_command = "gnome-terminal -- /home/tetra/navigation.sh ";
    string str_command2 = str_command + strMapname.c_str();
    std::vector<char> writable4(str_command2.begin(), str_command2.end());
    writable4.push_back('\0');
    char* ptr4 = &writable4[0];
    int iResult = std::system(ptr4);

    ex_ilaunchMode = 2;

    m_Result = true;
    return m_Result;
}

bool Approach_Station2Move()
{
    bool bResult = false;
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());

    if(_pRobot_Status.m_iCallback_Charging_status <= 1)
    {
        if(m_iDocking_timeout_cnt > 3000)
        {
            cmd->linear.x =  0.0; 
            cmd->angular.z = 0.0;
            cmdpub_.publish(cmd);
            m_iDocking_timeout_cnt = 0;
            ex_iDocking_CommandMode = 9;

        }
        else
        {
            cmd->linear.x =  0.01; 
            cmd->angular.z = 0.0;
            cmdpub_.publish(cmd);
            m_iDocking_timeout_cnt++;
            
        }
    }
    else
    {
        cmd->linear.x =  0.0; 
        cmd->angular.z = 0.0;
        cmdpub_.publish(cmd);
        printf("Approach Loop STOP !! \n");
        ex_iDocking_CommandMode = 6;
    }
    
    bResult = true;
    return bResult;
}

bool Approach_Station2Move2()
{
    bool bResult = false;
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());

    if(_pRobot_Status.m_iCallback_Charging_status <= 1)
    {
        if(m_iDocking_timeout_cnt > 3000)
        {
            cmd->linear.x =  0.0; 
            cmd->angular.z = 0.0;
            cmdpub_.publish(cmd);
            m_iDocking_timeout_cnt = 0;
            ex_iDocking_CommandMode = 9;

        }
        else
        {
            cmd->linear.x =  0.01; 
            cmd->angular.z = 0.0;
            cmdpub_.publish(cmd);
            m_iDocking_timeout_cnt++;
            
        }

    }
    else
    {
        cmd->linear.x =  0.0; 
        cmd->angular.z = 0.0;
        cmdpub_.publish(cmd);
        printf("Approach Loop STOP !! \n");
        ex_iDocking_CommandMode = 16;
    }
    

    bResult = true;
    return bResult;
}

double Rotation_Movement()
{
    double iResult = 0.1;

    switch(m_iRotation_Mode)
    {
        case 0:
            iResult = 0.1;
            break;
        case 1:
            iResult = -0.1;
            break;
        case 2:
            iResult = 0.1;
            break;

    }

    return iResult;
}

bool BumperCollision_Behavior()
{
    bool bResult = false;
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());

    if(_pRobot_Status.m_iBumperCollisionBehavor_cnt > 500)
    {
        cmd->linear.x =  0.0; 
        cmd->angular.z = 0.0;
        cmdpub_.publish(cmd);
        
        if(_pFlag_Value.m_bflagGo2)
        {
            LED_Toggle_Control(1, 3,100,3,1);
            LED_Turn_On(63);
            ROS_INFO("[RETRY Behavior2]: goto_ %s", goal.goal_id.id.c_str());
            setGoal(goal);
            _pFlag_Value.m_bflagGo2 = false;
        }
        
        _pRobot_Status.m_iBumperCollisionBehavor_cnt = 0;
        ex_iDocking_CommandMode = 0;

        bResult = true;
    }
    else
    {
        if(_pFlag_Value.m_bFlag_Obstacle_cygbot)
        {
            cmd->linear.x =  0.0; 
            cmd->angular.z = 0.0;
            cmdpub_.publish(cmd);
        }
        else
        {
            cmd->linear.x =  -0.02; 
            cmd->angular.z = 0.0;
            cmdpub_.publish(cmd);
            _pRobot_Status.m_iBumperCollisionBehavor_cnt++;
        } 
    }

    return bResult;
}

/****************************************************************/
//20240702 CYW//
//Mission Station Docking Function//
bool MissionStation_tracking(bool bOn, int marker_id)
{
    bool bResult = false;
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());

    //Todo.....
    if(bOn)
    {
        float m_fdistance = 0.0;
        if(_pAR_tag_pose.m_iAR_tag_id == marker_id)
        {
            m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);
            //printf("master_distance: %.5f \n", m_fdistance);
            if(m_fdistance > 0.41 && m_fdistance < 1.5)
            {
                cmd->linear.x = 1.0 * (m_fdistance /1.2) * 0.15; 
                //printf("linear velocity: %.2f \n", cmd->linear.x);
                if(cmd->linear.x > 1.0)
                {
                    //Linear Over speed exit loop...
                    cmd->linear.x =  0.0; 
                    cmd->angular.z = 0.0;
                    printf("[Linear Over speed]: follower is closing \n");
                    return false;
                }
                
                cmd->angular.z = -1.0 * atan2(_pAR_tag_pose.m_transform_pose_y, _pAR_tag_pose.m_transform_pose_x) / 1.25;
                //printf("angular velocity: %.2f \n", cmd-angular.z);
                if((cmd->angular.z > 1.0) || (cmd->angular.z < -1.0))
                {
                    //Angular Over speed exit loop......
                    cmd->linear.x =  0.0; 
                    cmd->angular.z = 0.0;
                    printf("[Angular Over speed]: follower is closing \n");
                    return false;
                }
                
                cmdpub_.publish(cmd);
            }
            else
            {
                cmd->linear.x =  0.0; 
                cmdpub_.publish(cmd);
                printf("Tracking STOP !! \n");
                ex_iDocking_CommandMode = 303;
                m_iNoMarker_cnt = 0;
            }
        }
        else
        {
            printf("No Marker, Rotation Movement !! \n");
            cmd->angular.z = Rotation_Movement(); //0.1;
            cmdpub_.publish(cmd);

            if(m_iNoMarker_cnt > 4000) //retry timeout!!
            {
                m_iNoMarker_cnt = 0;
                cmd->linear.x =  0.0; 
                cmd->angular.z = 0.0;
                cmdpub_.publish(cmd);
                printf("DockingStation scan Fail !! \n");
                ex_iDocking_CommandMode = 309;

            }
            else
            {
                m_iNoMarker_cnt++;
            }

        }

    }
    else
    {
        cmd->linear.x =  0.0; 
        cmd->angular.z = 0.0;
        cmdpub_.publish(cmd);
        printf("Docking Loop STOP!_not find Marker!! \n");
        ex_iDocking_CommandMode = 0;
    }
    

    bResult = true;
    return bResult;
}

bool MissionStation_Yaw_tracking()
{ 
    bool bResult = false;
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
    
    int m_iback_cnt = 0;
    float m_fdistance = 0.0;
    m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);

    if(_pAR_tag_pose.m_target_yaw <= 0.0174533 && _pAR_tag_pose.m_target_yaw >= -0.0174533) //+- 1.0deg
    {
        ex_iDocking_CommandMode = 304;
        bResult = true;
        return bResult;
    }

    if(_pAR_tag_pose.m_target_yaw > 0)
    {
        printf("++dir \n");
        cmd->angular.z = -1.0 * _pAR_tag_pose.m_target_yaw * 1.6;
        cmdpub_.publish(cmd);
        sleep(2);

        if(m_fdistance > 1.0 || m_fdistance < -1.0)
        {
            printf("[Error] Marker too far away !! \n");
            cmd->angular.z = 0.0;
            cmd->linear.x = 0.0;
            cmdpub_.publish(cmd);

            ex_iDocking_CommandMode = 309;
            bResult = false;
            return bResult;
        }


        while(m_iback_cnt < 30)
        {
            // if(_pFlag_Value.m_bFlag_Obstacle_cygbot)
            // {
            //     cmd->angular.z = 0.0;
            //     cmd->linear.x = 0.0;
            //     cmdpub_.publish(cmd);
            // }
            // else
            // {
                cmd->angular.z = 0.0;
                cmd->linear.x = -1.0 * m_fdistance * 0.2;
                cmdpub_.publish(cmd);
                m_iback_cnt++;
            // }
            usleep(100000); //100ms
        }

        cmd->angular.z = 0.0;
        cmd->linear.x = 0.0;
        cmdpub_.publish(cmd);
        printf("++dir STOP \n");
        m_iRotation_Mode = 2;
        ex_iDocking_CommandMode = 302;
    }
    else
    {
        printf("--dir \n");
        cmd->angular.z = -1.0 * _pAR_tag_pose.m_target_yaw * 1.6;
        cmdpub_.publish(cmd);
        sleep(2);

        if(m_fdistance > 1.0 || m_fdistance < -1.0)
        {
            printf("[Error] Marker too far away !! \n");
            cmd->angular.z = 0.0;
            cmd->linear.x = 0.0;
            cmdpub_.publish(cmd);

            ex_iDocking_CommandMode = 309;
            bResult = false;
            return bResult;
        }

        while(m_iback_cnt < 30)
        {
            // if(_pFlag_Value.m_bFlag_Obstacle_cygbot)
            // {
            //     cmd->angular.z = 0.0;
            //     cmd->linear.x = 0.0;
            //     cmdpub_.publish(cmd);
            // }
            // else
            // {
                cmd->angular.z = 0.0;
                cmd->linear.x = -1.0 * m_fdistance * 0.2;
                cmdpub_.publish(cmd);
                m_iback_cnt++;
            // }
            usleep(100000); //100ms
            
        }

        cmd->angular.z = 0.0;
        cmd->linear.x = 0.0;
        cmdpub_.publish(cmd);
        printf("--dir STOP \n");
        m_iRotation_Mode = 1;
        ex_iDocking_CommandMode = 302;
    }
    

    bResult = true;
    return bResult;
}

bool MissionStation_tracking2(int marker_id)
{
    bool bResult = false;
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
    
    float m_fdistance = 0.0;
    if(_pAR_tag_pose.m_iAR_tag_id == marker_id)
    {
        m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);
        //printf("master_distance: %.5f \n", m_fdistance);
        if(!_pFlag_Value.m_bumperhit_flag)
        {
            cmd->linear.x = 1.0 * (m_fdistance /1.2) * 0.3; //max speed 0.3m/s
            //printf("linear velocity: %.2f \n", cmd->linear.x);
            if(cmd->linear.x > 1.0)
            {
                //Linear Over speed exit loop......
                cmd->linear.x =  0.0; 
                cmd->angular.z = 0.0;
                printf("[Linear Over speed]: follower is closing \n");
                return false;
            }
            
            cmd->angular.z = -1.0 * atan2(_pAR_tag_pose.m_transform_pose_y, _pAR_tag_pose.m_transform_pose_x) / 1.25;
            //printf("angular velocity: %.2f \n", cmd->angular.z);
            if((cmd->angular.z > 1.0) || (cmd->angular.z < -1.0))
            {
                //Angular Over speed exit loop......
                cmd->linear.x =  0.0; 
                cmd->angular.z = 0.0;
                printf("[Angular Over speed]: follower is closing \n");
                return false;
            }
            
            cmdpub_.publish(cmd);
        }
        else
        {
            cmd->linear.x =  0.0; 
            cmd->angular.z = 0.0;
            cmdpub_.publish(cmd);
            printf("Tracking STOP & Docking Finish !! \n");
            ex_iDocking_CommandMode = 306; //5;
            m_iNoMarker_cnt = 0;
        }
    }
    else
    {
        cmd->linear.x =  0.0; 
        cmd->angular.z = 0.0;
        cmdpub_.publish(cmd);
        printf("No Marker 2! \n");
        if(m_iNoMarker_cnt >= 10)
        {
            m_iNoMarker_cnt = 0;
            ex_iDocking_CommandMode = 305;
            printf("No Marker 2_Timeout! \n");
        }
        else
        {
            m_iNoMarker_cnt++;
        }
    }

    bResult = true;
    return bResult;
}

bool Approach_MissionStation2Move()
{
    bool bResult = false;
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());

    if(!_pFlag_Value.m_bumperhit_flag)
    {
        if(m_iDocking_timeout_cnt > 5000)
        {
            cmd->linear.x =  0.0; 
            cmd->angular.z = 0.0;
            cmdpub_.publish(cmd);
            m_iDocking_timeout_cnt = 0;
            ex_iDocking_CommandMode = 309;
        }
        else
        {
            cmd->linear.x =  0.1; 
            cmd->angular.z = 0.0;
            cmdpub_.publish(cmd);
            m_iDocking_timeout_cnt++;
        }
    }
    else
    {
        cmd->linear.x =  0.0; 
        cmd->angular.z = 0.0;
        cmdpub_.publish(cmd);
        printf("Approach Loop STOP !! \n");
        ex_iDocking_CommandMode = 306;
    }
    
    bResult = true;
    return bResult;
}

bool Depart_MissionStation2Move(int marker_id)
{
    bool bResult = false;
    printf("Depart_MissionStation2Move ... docking exit!! \n"); //240315 mwcha made new func

    float m_fdistance = 0.0;
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
    
    if(_pFlag_Value.m_bumperhit_flag && ex_bMissionDocking_Flag)
    {
        cmd->linear.x =  -0.05; 
        cmd->angular.z = 0.0;
        //cmdpub_.publish(cmd);
        bResult = false;
    }
    else
    {
        if(_pAR_tag_pose.m_iAR_tag_id == -1)
        {
            if(_pFlag_Value.m_bFlag_Obstacle_cygbot)
            {
                cmd->angular.z = 0.0;
                cmd->linear.x = 0.0;

                bResult = false;
            }
            else
            {
                cmd->linear.x =  -0.05; 
                cmd->angular.z = 0.0;
                //cmdpub_.publish(cmd);
                bResult = false;
            }
        }
        else
        {
            m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);
            printf("Depart_fdistance: %.5f \n", m_fdistance);

            if(_pAR_tag_pose.m_transform_pose_x <= 0.35) //350mm depart move
            {
                // if(_pFlag_Value.m_bFlag_Obstacle_cygbot)
                // {
                //     cmd->linear.x =  0.0; 
                //     cmd->angular.z = 0.0;
                //     //cmdpub_.publish(cmd);
                //     bResult = false;
                // }
                // else
                // {
                    cmd->linear.x =  -0.05; 
                    cmd->angular.z = 0.0;
                    //cmdpub_.publish(cmd);
                    bResult = false;
                // }
            }
            else
            {
                cmd->linear.x =  0.0; 
                cmd->angular.z = 0.0;
                //cmdpub_.publish(cmd);

                //add goto cmd call//
                setGoal(goal);

                ex_bMissionDocking_Flag = false;
                ex_iDocking_CommandMode = 0;

                bResult = true;
            }
        }
    }

    cmdpub_.publish(cmd);
    return bResult;
}

/****************************************************************/
//Charging Station Docking Function//
bool ChargingStation_tracking(bool bOn, int marker_id)
{
    bool bResult = false;
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());

    //Todo.....
    if(bOn)
    {
        float m_fdistance = 0.0;
        if(_pAR_tag_pose.m_iAR_tag_id == marker_id)
        {
            m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);
            //printf("master_distance: %.5f \n", m_fdistance);
            if(m_fdistance > 0.41 && m_fdistance < 1.5)
            {
                cmd->linear.x = 1.0 * (m_fdistance /1.2) * 0.15; 
                //printf("linear velocity: %.2f \n", cmd->linear.x);
                if(cmd->linear.x > 1.0)
                {
                    //Linear Over speed exit loop...
                    cmd->linear.x =  0.0; 
                    cmd->angular.z = 0.0;
                    printf("[Linear Over speed]: follower is closing \n");
                    return false;
                }
                
                cmd->angular.z = -1.0 * atan2(_pAR_tag_pose.m_transform_pose_y, _pAR_tag_pose.m_transform_pose_x) / 1.25;
                //printf("angular velocity: %.2f \n", cmd-angular.z);
                if((cmd->angular.z > 1.0) || (cmd->angular.z < -1.0))
                {
                    //Angular Over speed exit loop......
                    cmd->linear.x =  0.0; 
                    cmd->angular.z = 0.0;
                    printf("[Angular Over speed]: follower is closing \n");
                    return false;
                }
                
                cmdpub_.publish(cmd);
            }
            else
            {
                cmd->linear.x =  0.0; 
                cmdpub_.publish(cmd);
                printf("Tracking STOP !! \n");
                ex_iDocking_CommandMode = 3;
                m_iNoMarker_cnt = 0;
            }
        }
        else
        {
            printf("No Marker, Rotation Movement !! \n");
            cmd->angular.z = Rotation_Movement(); //0.1;
            cmdpub_.publish(cmd);

            if(m_iNoMarker_cnt > 4000) //retry timeout!!
            {
                m_iNoMarker_cnt = 0;
                cmd->linear.x =  0.0; 
                cmd->angular.z = 0.0;
                cmdpub_.publish(cmd);
                printf("DockingStation scan Fail !! \n");
                ex_iDocking_CommandMode = 9;

            }
            else
            {
                m_iNoMarker_cnt++;
            }

        }

    }
    else
    {
        cmd->linear.x =  0.0; 
        cmd->angular.z = 0.0;
        cmdpub_.publish(cmd);
        printf("Docking Loop STOP!_not find Marker!! \n");
        ex_iDocking_CommandMode = 0;
    }
    

    bResult = true;
    return bResult;
}

bool ChargingStation_Yaw_tracking()
{ 
    bool bResult = false;
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
    
    int m_iback_cnt = 0;
    float m_fdistance = 0.0;
    m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);

    if(_pAR_tag_pose.m_target_yaw <= 0.0174533 && _pAR_tag_pose.m_target_yaw >= -0.0174533) //+- 1.0deg
    {
        ex_iDocking_CommandMode = 4;
        bResult = true;
        return bResult;
    }

    if(_pAR_tag_pose.m_target_yaw > 0)
    {
        printf("++dir \n");
        cmd->angular.z = -1.0 * _pAR_tag_pose.m_target_yaw * 1.6;
        cmdpub_.publish(cmd);
        sleep(2);

        if(m_fdistance > 1.0 || m_fdistance < -1.0)
        {
            printf("[Error] Marker too far away !! \n");
            cmd->angular.z = 0.0;
            cmd->linear.x = 0.0;
            cmdpub_.publish(cmd);

            ex_iDocking_CommandMode = 9;
            bResult = false;
            return bResult;
        }


        while(m_iback_cnt < 30)
        {
            if(_pFlag_Value.m_bFlag_Obstacle_cygbot)
            {
                cmd->angular.z = 0.0;
                cmd->linear.x = 0.0;
                cmdpub_.publish(cmd);
            }
            else
            {
                cmd->angular.z = 0.0;
                cmd->linear.x = -1.0 * m_fdistance * 0.2;
                cmdpub_.publish(cmd);
                m_iback_cnt++;
            }
            usleep(100000); //100ms
            
        }

        cmd->angular.z = 0.0;
        cmd->linear.x = 0.0;
        cmdpub_.publish(cmd);
        printf("++dir STOP \n");
        m_iRotation_Mode = 2;
        ex_iDocking_CommandMode = 2;
    }
    else
    {
        printf("--dir \n");
        cmd->angular.z = -1.0 * _pAR_tag_pose.m_target_yaw * 1.6;
        cmdpub_.publish(cmd);
        sleep(2);

        if(m_fdistance > 1.0 || m_fdistance < -1.0)
        {
            printf("[Error] Marker too far away !! \n");
            cmd->angular.z = 0.0;
            cmd->linear.x = 0.0;
            cmdpub_.publish(cmd);

            ex_iDocking_CommandMode = 9;
            bResult = false;
            return bResult;
        }

        while(m_iback_cnt < 30)
        {
            if(_pFlag_Value.m_bFlag_Obstacle_cygbot)
            {
                cmd->angular.z = 0.0;
                cmd->linear.x = 0.0;
                cmdpub_.publish(cmd);
            }
            else
            {
                cmd->angular.z = 0.0;
                cmd->linear.x = -1.0 * m_fdistance * 0.2;
                cmdpub_.publish(cmd);
                m_iback_cnt++;
            }
            usleep(100000); //100ms
            
        }

        cmd->angular.z = 0.0;
        cmd->linear.x = 0.0;
        cmdpub_.publish(cmd);
        printf("--dir STOP \n");
        m_iRotation_Mode = 1;
        ex_iDocking_CommandMode = 2;
    }
    

    bResult = true;
    return bResult;
}

bool ChargingStation_tracking2(int marker_id)
{
    bool bResult = false;
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
    
    float m_fdistance = 0.0;
    if(_pAR_tag_pose.m_iAR_tag_id == marker_id)
    {
        m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);
        //printf("master_distance: %.5f \n", m_fdistance);
        if(_pRobot_Status.m_iCallback_Charging_status < 2)
        {
            cmd->linear.x = 1.0 * (m_fdistance /1.2) * 0.1; //max speed 0.1m/s
            //printf("linear velocity: %.2f \n", cmd->linear.x);
            if(cmd->linear.x > 1.0)
            {
                //Linear Over speed exit loop......
                cmd->linear.x =  0.0; 
                cmd->angular.z = 0.0;
                printf("[Linear Over speed]: follower is closing \n");
                return false;
            }
            
            cmd->angular.z = -1.0 * atan2(_pAR_tag_pose.m_transform_pose_y, _pAR_tag_pose.m_transform_pose_x) / 1.25;
            //printf("angular velocity: %.2f \n", cmd->angular.z);
            if((cmd->angular.z > 1.0) || (cmd->angular.z < -1.0))
            {
                //Angular Over speed exit loop......
                cmd->linear.x =  0.0; 
                cmd->angular.z = 0.0;
                printf("[Angular Over speed]: follower is closing \n");
                return false;
            }
            
            cmdpub_.publish(cmd);
        }
        else
        {
            cmd->linear.x =  0.0; 
            cmd->angular.z = 0.0;
            cmdpub_.publish(cmd);
            printf("Tracking STOP & Docking Finish !! \n");
            ex_iDocking_CommandMode = 6; //5;
            m_iNoMarker_cnt = 0;
        }
    }
    else
    {
        cmd->linear.x =  0.0; 
        cmd->angular.z = 0.0;
        cmdpub_.publish(cmd);
        printf("No Marker 2! \n");
        if(m_iNoMarker_cnt >= 10)
        {
            m_iNoMarker_cnt = 0;
             ex_iDocking_CommandMode = 6;
            printf("No Marker 2_Timeout! \n");
        }
        else
        {
            m_iNoMarker_cnt++;
        }
    }

    bResult = true;
    return bResult;
}
/****************************************************************/
//Conveyor Station Docking Function//
bool ConveyorStation_tracking(bool bOn, int marker_id)
{
    bool bResult = false;
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());

    if(bOn)
    {
        printf("Conveyor_ID: %d  \n", marker_id);
        float m_fdistance = 0.0;
        if(_pAR_tag_pose.m_iAR_tag_id == marker_id)
        {
            m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);
            //printf("master_distance: %.5f \n", m_fdistance);
            if(m_fdistance > 0.41 && m_fdistance < 1.5)
            {
                cmd->linear.x = 1.0 * (m_fdistance /1.2) * 0.15; 
                //printf("linear velocity: %.2f \n", cmd->linear.x);
                if(cmd->linear.x > 1.0)
                {
                    //Linear Over speed exit loop...
                    cmd->linear.x =  0.0; 
                    cmd->angular.z = 0.0;
                    printf("[Linear Over speed]: follower is closing \n");
                    return false;
                }
                
                cmd->angular.z = -1.0 * atan2(_pAR_tag_pose.m_transform_pose_y, _pAR_tag_pose.m_transform_pose_x) / 1.25;
                //printf("angular velocity: %.2f \n", cmd->angular.z);
                if((cmd->angular.z > 1.0) || (cmd->angular.z < -1.0))
                {
                    //Angular Over speed exit loop......
                    cmd->linear.x =  0.0; 
                    cmd->angular.z = 0.0;
                    printf("[Angular Over speed]: follower is closing \n");
                    return false;
                }
                
                cmdpub_.publish(cmd);
            }
            else
            {
                cmd->linear.x =  0.0; 
                //cmd->angular.z = 0.0;
                cmdpub_.publish(cmd);
                printf("Conveyor Tracking STOP !! \n");
                ex_iDocking_CommandMode = 13;
                m_iNoMarker_cnt = 0;
            }
        }
        else
        {
            printf("No Marker, Rotation Movement !! \n");
            cmd->angular.z = Rotation_Movement(); //0.1;
            cmdpub_.publish(cmd);
            if(m_iNoMarker_cnt > 4000) //retry timeout!!
            {
                m_iNoMarker_cnt = 0;
                cmd->linear.x =  0.0; 
                cmd->angular.z = 0.0;
                cmdpub_.publish(cmd);
                printf("ConveyorStation scan Fail !! \n");
                ex_iDocking_CommandMode = 9;

            }
            else
            {
                m_iNoMarker_cnt++;
            }

        }

    }
    else
    {
        cmd->linear.x =  0.0; 
        cmd->angular.z = 0.0;
        cmdpub_.publish(cmd);
        printf("Conveyor Docking Loop STOP!_not find Marker!! \n");
        ex_iDocking_CommandMode = 0;
    }
    

    bResult = true;
    return bResult;
}

bool ConveyorStation_Yaw_tracking()
{ 
    bool bResult = false;
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());

    int m_iback_cnt = 0;
    float m_fdistance = 0.0;
    m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);

    if(_pAR_tag_pose.m_target_yaw <= 0.0174533 && _pAR_tag_pose.m_target_yaw >= -0.0174533) //+- 1.0deg
    {
        ex_iDocking_CommandMode = 14;
        bResult = true;
        return bResult;
    }

    if(_pAR_tag_pose.m_target_yaw > 0)
    {
        printf("++dir \n");
        cmd->angular.z = -1.0 * _pAR_tag_pose.m_target_yaw * 1.6;
        cmdpub_.publish(cmd);
        sleep(2);

        if(m_fdistance > 1.0 || m_fdistance < -1.0)
        {
            printf("[Error] Marker too far away !! \n");
            cmd->angular.z = 0.0;
            cmd->linear.x = 0.0;
            cmdpub_.publish(cmd);

            ex_iDocking_CommandMode = 9;
            bResult = false;
            return bResult;
        }

        while(m_iback_cnt < 30)
        {
            if(_pFlag_Value.m_bFlag_Obstacle_cygbot)
            {
                cmd->angular.z = 0.0;
                cmd->linear.x = 0.0;
                cmdpub_.publish(cmd);
            }
            else
            {
                cmd->angular.z = 0.0;
                cmd->linear.x = -1.0 * m_fdistance * 0.2;
                cmdpub_.publish(cmd);
                m_iback_cnt++;
            }
            usleep(100000); //100ms
            
        }
        cmd->angular.z = 0.0;
        cmd->linear.x = 0.0;
        cmdpub_.publish(cmd);
        printf("++dir STOP \n");
        m_iRotation_Mode = 2;
        ex_iDocking_CommandMode = 12;
    }
    else
    {
        printf("--dir \n");
        cmd->angular.z = -1.0 * _pAR_tag_pose.m_target_yaw * 1.6;
        cmdpub_.publish(cmd);
        sleep(2);

        if(m_fdistance > 1.0 || m_fdistance < -1.0)
        {
            printf("[Error] Marker too far away !! \n");
            cmd->angular.z = 0.0;
            cmd->linear.x = 0.0;
            cmdpub_.publish(cmd);

            ex_iDocking_CommandMode = 9;
            bResult = false;
            return bResult;
        }

        while(m_iback_cnt < 30)
        {
            if(_pFlag_Value.m_bFlag_Obstacle_cygbot)
            {
                cmd->angular.z = 0.0;
                cmd->linear.x = 0.0;
                cmdpub_.publish(cmd);
            }
            else
            {
                cmd->angular.z = 0.0;
                cmd->linear.x = -1.0 * m_fdistance * 0.2;
                cmdpub_.publish(cmd);
                m_iback_cnt++;
            }
            usleep(100000); //100ms
            
        }
        cmd->angular.z = 0.0;
        cmd->linear.x = 0.0;
        cmdpub_.publish(cmd);
        printf("--dir STOP \n");
        m_iRotation_Mode = 1;
        ex_iDocking_CommandMode = 12;
    }
    

    bResult = true;
    return bResult;
}

bool ConveyorStation_tracking2(int marker_id)
{
    bool bResult = false;
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
    
    float m_fdistance = 0.0;
    if(_pAR_tag_pose.m_iAR_tag_id == marker_id)
    {
        m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);
        if(_pRobot_Status.m_iCallback_Charging_status < 2)
        {
            cmd->linear.x = 1.0 * (m_fdistance /1.2) * 0.1; //max speed 0.1m/s
            //printf("linear velocity: %.2f \n", cmd->linear.x);
            if(cmd->linear.x > 1.0)
            {
                //Linear Over speed exit loop......
                cmd->linear.x =  0.0; 
                cmd->angular.z = 0.0;
                printf("[Linear Over speed]: follower is closing \n");
                return false;
            }
            
            cmd->angular.z = -1.0 * atan2(_pAR_tag_pose.m_transform_pose_y, _pAR_tag_pose.m_transform_pose_x) / 1.25;
            //printf("angular velocity: %.2f \n", cmd->angular.z);
            if((cmd->angular.z > 1.0) || (cmd->angular.z < -1.0))
            {
                //Angular Over speed exit loop......
                cmd->linear.x =  0.0; 
                cmd->angular.z = 0.0;
                printf("[Angular Over speed]: follower is closing \n");
                return false;
            }
            
            cmdpub_.publish(cmd);
        }
        else
        {
            cmd->linear.x =  0.0; 
            cmd->angular.z = 0.0;
            cmdpub_.publish(cmd);
            printf("Conveyor Tracking STOP & Docking Finish !! \n");
            ex_iDocking_CommandMode = 16;
            m_iNoMarker_cnt = 0;
        }
    }
    else
    {
        cmd->linear.x =  0.0; 
        cmd->angular.z = 0.0;
        cmdpub_.publish(cmd);
        printf("No Marker 2! \n");
        if(m_iNoMarker_cnt >= 10)
        {
            m_iNoMarker_cnt = 0;
             ex_iDocking_CommandMode = 16;
            printf("No Marker 2_Timeout! \n");
        }
        else
        {
            m_iNoMarker_cnt++;
        }
    }

    bResult = true;
    return bResult;
}

bool mapping_Command(tetraDS_service::runmapping::Request &req, 
					 tetraDS_service::runmapping::Response &res)
{
	bool bResult = false;
    bResult = req.flag_mapping;

    rosrun_mapping();
    /*
    bool flag_mapping
    ---
    bool command_Result
    */

	res.command_Result = true;
	return true;
}

bool navigation_Command(tetraDS_service::runnavigation::Request &req, 
					    tetraDS_service::runnavigation::Response &res)
{
	bool bResult = false;

    string str_mapname = req.map_name.c_str();
    rosrun_navigation(str_mapname);
    /*
    string map_name
    ---
    bool command_Result
    */

	res.command_Result = true;
	return true;
}

bool nodekill_Command(tetraDS_service::rosnodekill::Request &req, 
					  tetraDS_service::rosnodekill::Response &res)
{
	bool bResult = false;
    
    //bResult = req.flag_kill;

    bResult = rosnodekill_all();
    printf("rosnodekill_all: %d \n", bResult);
    // sleep(1);

    /*
    ---
    bool command_Result
    */
    //throw std::runtime_error("####this will show up");

	res.command_Result = bResult;
	return true;
}

bool Goto_Conveyor_Command(tetraDS_service::gotoconveyor::Request &req, 
				            tetraDS_service::gotoconveyor::Response &res)
{
	bool bResult = false;

    //costmap clear call//
	m_flag_clesr_costmap_call = clear_costmap_client.call(m_request);
	while(!m_flag_clesr_costmap_call)
	{
		sleep(1);
	}
	m_flag_clesr_costmap_call = false;

    LED_Toggle_Control(1, 3,100,3,1);
    LED_Turn_On(45); //sky_blue

    bResult = OpenLocationFile(req.Location);
    //printf("Goto bResult: %d \n", bResult);

    goto_goal_id.id = req.Location;
    _pRobot_Status.CONVEYOR_ID = req.id;
    _pRobot_Status.CONVEYOR_MOVEMENT = req.movement;
    _pFlag_Value.m_bflag_Conveyor_docking = true;

    ROS_INFO("goto_conveyor_name: %s, id: %d, movement: %d", goto_goal_id.id.c_str(), _pRobot_Status.CONVEYOR_ID, _pRobot_Status.CONVEYOR_MOVEMENT);

    if(_pRobot_Status.m_iCallback_Charging_status <= 1 && (_pAR_tag_pose.m_iAR_tag_id == -1 || _pAR_tag_pose.m_transform_pose_x <= 0.5))  //Nomal
    {
        LED_Toggle_Control(1, 3,100,3,1);
        LED_Turn_On(45); //sky_blue
        setGoal(goal);
    }
    else if(_pFlag_Value.m_bumperhit_flag || ex_bMissionDocking_Flag)
    {
        ex_iDocking_CommandMode = 310;
        bResult = true;
    }
    else //Docking...
    {
        ex_iDocking_CommandMode = 10; //Depart Move
    }

	/*
    string Location
    int32 id
    int32 movement
    ---
    bool command_Result
	*/
	res.command_Result = bResult;
	return true;
}

bool Loading_check_Command(tetraDS_service::loadingcheck::Request &req, tetraDS_service::loadingcheck::Response &res)
{
	bool bResult = false;
    //to do Check Loop...
    if(_pRobot_Status.m_iConveyor_Sensor_info == 0)
    {
        bResult = true;
    }
    else
    {
        bResult = false;
    }
	/*
    ---
    int32 command_Result
	*/
	res.command_Result = bResult;
	return true;
}

bool Unloading_check_Command(tetraDS_service::unloadingcheck::Request &req, tetraDS_service::unloadingcheck::Response &res)
{
	bool bResult = false;
    //to do Check Loop...
    if(_pRobot_Status.m_iConveyor_Sensor_info >= 2)
    {
        bResult = true;
    }
    else
    {
        bResult = false;
    }
	/*
    ---
    int32 command_Result
	*/
	res.command_Result = bResult;
	return true;
}

bool RemoveAll_map_data()
{
    bool bResult = false;

    string m_strLocationName;
    char   filename[1024];
    DIR *dir;
    struct dirent *ent;
    dir = opendir ("/home/tetra/catkin_ws/src/tetraDS_2dnav/maps/"); //file path
    if (dir != NULL) 
    {
        bResult= true;
        //print all the files and directories within directory
        while ((ent = readdir (dir)) != NULL)
        {
            if(strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) 
            { 
                continue; 
            }
            sprintf(filename, "%s", ent->d_name);
            printf ("%s\n", filename);
            string str(filename);
            m_strLocationName = "/home/tetra/catkin_ws/src/tetraDS_2dnav/maps/" + str; 
            remove(m_strLocationName.c_str());
        }
        closedir (dir);
    } 
    else 
    {
        //could not open directory
        perror ("");
        bResult = false;
    }
}

bool RemoveAll_location_data()
{
    bool bResult = false;

    string m_strLocationName;
    char   filename[1024];
    DIR *dir;
    struct dirent *ent;
    dir = opendir ("/home/tetra/DATA/"); //file path
    if (dir != NULL) 
    {
        bResult= true;
        //print all the files and directories within directory
        while ((ent = readdir (dir)) != NULL)
        {
            if(strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) 
            { 
                continue; 
            }
            sprintf(filename, "%s", ent->d_name);
            printf ("%s\n", filename);
            string str(filename);
            m_strLocationName = "/home/tetra/DATA/" + str; 
            remove(m_strLocationName.c_str());
        }
        closedir (dir);
    } 
    else 
    {
        //could not open directory
        perror ("");
        bResult = false;
    }
}

bool RemoveAll_landmark_data()
{
    bool bResult = false;

    string m_strLandmarkName;
    char   filename[1024];
    DIR *dir;
    struct dirent *ent;
    dir = opendir ("/home/tetra/LANDMARK/"); //file path
    if (dir != NULL) 
    {
        bResult= true;
        //print all the files and directories within directory
        while ((ent = readdir (dir)) != NULL)
        {
            if(strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) 
            { 
                continue; 
            }
            sprintf(filename, "%s", ent->d_name);
            printf ("%s\n", filename);
            string str(filename);
            m_strLandmarkName = "/home/tetra/LANDMARK/" + str; 
            remove(m_strLandmarkName.c_str());
        }
        closedir (dir);
    } 
    else 
    {
        //could not open directory
        perror ("");
        bResult = false;
    }
}

bool DeleteData_All_Command(tetraDS_service::deletedataall::Request  &req, 
				            tetraDS_service::deletedataall::Response &res)
{
    bool bResult = false;
    bool bRemove_map = false;
    bool bRemove_location = false;
    bool bRemove_landmark = false;
    
    bRemove_map = RemoveAll_map_data();
    usleep(10000); //10ms
    bRemove_location = RemoveAll_location_data();
    usleep(10000); //10ms
    bRemove_landmark = RemoveAll_landmark_data();
    usleep(10000); //10ms

    if(bRemove_map && bRemove_location && bRemove_landmark)
    {
        bResult = true;
    }
    else
    {
        bResult = false;
    }
    
    /*
    ---
    bool command_Result
    */
    res.command_Result = bResult;
    return true;
}

void backmovement() // add 240129 mwcha
{
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
    printf("Target Distance = %.3f \n", _pRobot_Status.m_dGoal_Distance);
    printf("SUM = %.3f \n", _pRobot_Status.m_dTotal_Distance +_pRobot_Status.m_backmove_cmd);
    printf("_pRobot_Status.m_dTotal_Distance = %.3f \n", _pRobot_Status.m_dTotal_Distance);

    if(_pRobot_Status.m_dTotal_Distance < _pRobot_Status.m_dGoal_Distance)
    {
        if(_pRobot_Status.m_cmd_vel < 0) // - move
        {
            if(_pFlag_Value.m_bFlag_Obstacle_cygbot)
            {
                cmd->linear.x =  0.0; 
                cmd->angular.z = 0.0;
                cmdpub_.publish(cmd);
                        
            }
            else
            {
                cmd->linear.x = _pRobot_Status.m_cmd_vel; 
                cmd->angular.z = 0.0;
                cmdpub_.publish(cmd);
                        
            }
        }
        else // + move
        {
            if(_pFlag_Value.m_bFlag_Obstacle_Center)
            {
                cmd->linear.x =  0.0; 
                cmd->angular.z = 0.0;
                cmdpub_.publish(cmd);
                        
            }
            else
            {
                cmd->linear.x = _pRobot_Status.m_cmd_vel; 
                cmd->angular.z = 0.0;
                cmdpub_.publish(cmd);
                        
            }

        }
        printf("_pRobot_Status.m_cmd_vel; = %.3f \n", _pRobot_Status.m_cmd_vel);
        cmd->linear.x = _pRobot_Status.m_cmd_vel; 
        cmd->angular.z = 0.0;
        cmdpub_.publish(cmd);  
    }
    else
    {
            cmd->linear.x =  0.0; 
            cmd->angular.z = 0.0;
            cmdpub_.publish(cmd);
            ex_iDocking_CommandMode = 0;
    }

}

void *DockingThread_function(void *data)
{
    while(1)
    {
        switch(ex_iDocking_CommandMode)
        {
            case 0:
                
                break;
            /****************************************************************/
            // Station Docking Loop//
            case 1:
            
                LED_Toggle_Control(1, 3,100,3,100);
                LED_Turn_On(63);
                //usb_cam_On_client.call(m_request);
                sleep(2);
                ex_iDocking_CommandMode = 2;
                docking_progress.data = 1;
                docking_progress_pub.publish(docking_progress);
                break;
            case 2:
                ChargingStation_tracking(true, _pAR_tag_pose.m_iSelect_AR_tag_id);
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    ex_iDocking_CommandMode = 0;
                }
                docking_progress.data = 2;
                docking_progress_pub.publish(docking_progress);
                break;
            case 3:
                _pAR_tag_pose.m_target_yaw = _pAR_tag_pose.m_fAR_tag_pitch;
                ChargingStation_Yaw_tracking();
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    ex_iDocking_CommandMode = 0;
                }
                docking_progress.data = 3;
                docking_progress_pub.publish(docking_progress);
                break;
            case 4:
                ChargingStation_tracking2(_pAR_tag_pose.m_iSelect_AR_tag_id);
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    ex_iDocking_CommandMode = 0;
                }
                docking_progress.data = 4;
                docking_progress_pub.publish(docking_progress);
                break;
            case 5:
                //charging_port_On_client.call(m_request2);
                Approach_Station2Move();
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    ex_iDocking_CommandMode = 0;
                }
                docking_progress.data = 5;
                docking_progress_pub.publish(docking_progress);
                break;
            case 6:
                LED_Turn_On(9);
                
                ROS_INFO_STREAM("TETRA POSE Reset!");
                m_iReset_flag = 1;
                docking_progress.data = 6;
                docking_progress_pub.publish(docking_progress);
                ////PoseReset_call
                Reset_Robot_Pose();
                LED_Toggle_Control(1, 5,100,5,1);
                LED_Turn_On(63);

                ex_iDocking_CommandMode = 0;
                break;
            case 9:
                printf("Docking FAIL ! \n");
                LED_Toggle_Control(1, 10,100,10,1);
                LED_Turn_On(18);
                //ex_iDocking_CommandMode = 0;
                ex_iDocking_CommandMode = 119;
                break;
            case 10:
                Depart_Station2Move();
                break;
            /****************************************************************/
            // Conveyor Docking Loop//
            case 11:
                LED_Toggle_Control(1, 3,100,3,100);
                LED_Turn_On(63);
                //usb_cam_On_client.call(m_request);
                sleep(2);
                ex_iDocking_CommandMode = 12;
                docking_progress.data = 1;
                docking_progress_pub.publish(docking_progress);
                break;
            case 12:
                ConveyorStation_tracking(true, _pAR_tag_pose.m_iSelect_AR_tag_id);
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    ex_iDocking_CommandMode = 0;
                }
                docking_progress.data = 2;
                docking_progress_pub.publish(docking_progress);
                break;
            case 13:
                _pAR_tag_pose.m_target_yaw = _pAR_tag_pose.m_fAR_tag_pitch;
                ConveyorStation_Yaw_tracking();
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    ex_iDocking_CommandMode = 0;
                }
                docking_progress.data = 3;
                docking_progress_pub.publish(docking_progress);
                break;
            case 14:
                ConveyorStation_tracking2(_pAR_tag_pose.m_iSelect_AR_tag_id);
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    ex_iDocking_CommandMode = 0;
                }
                docking_progress.data = 4;
                docking_progress_pub.publish(docking_progress);
                break;
            case 15:
                //charging_port_On_client.call(m_request2);
                Approach_Station2Move2();
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    ex_iDocking_CommandMode = 0;
                }
                docking_progress.data = 5;
                docking_progress_pub.publish(docking_progress);
                break;
            case 16:
                LED_Turn_On(9);
                //usb_cam_Off_client.call(m_request);
                //ROS_INFO_STREAM("TETRA POSE Rest!");
                m_iReset_flag = 1;
                docking_progress.data = 6;
                docking_progress_pub.publish(docking_progress);
                //PoseReset_call
                Reset_Robot_Pose();
                
                LED_Toggle_Control(1, 5,100,5,1);
                LED_Turn_On(63);

                ex_iDocking_CommandMode = 0;
                break;
            /****************************************************************/
            case 30:
                _pFlag_Value.m_bCorneringFlag = false;
                ex_iDocking_CommandMode = 0;
                break;
            case 31:
                _pFlag_Value.m_bCorneringFlag = true;
                ex_iDocking_CommandMode = 0;
                break;
            case 100: //Check Bumper
                BumperCollision_Behavior();
                break;
            case 119: //Retry Goto Home...
                printf(" Retry Goto Home ! \n");
                //costmap clear call//
                clear_costmap_client.call(m_request);
                _pGoal_pose.goal_positionX = _pHomePose.HOME_dPOSITION_X;
                _pGoal_pose.goal_positionY = _pHomePose.HOME_dPOSITION_Y;
                _pGoal_pose.goal_positionZ = _pHomePose.HOME_dPOSITION_Z;
                _pGoal_pose.goal_quarterX = _pHomePose.HOME_dQUATERNION_X;
                _pGoal_pose.goal_quarterY = _pHomePose.HOME_dQUATERNION_Y;
                _pGoal_pose.goal_quarterZ = _pHomePose.HOME_dQUATERNION_Z;
                _pGoal_pose.goal_quarterW = _pHomePose.HOME_dQUATERNION_W;

                goto_goal_id.id = "HOME";
                setGoal(goal);
                _pFlag_Value.m_bflag_ComebackHome = true;
                ex_iDocking_CommandMode = 0;

            case 200: //Auto Patrol Loop...
                if(_pRobot_Status.m_iCallback_Battery == HIGH_BATTERY && _pFlag_Value.m_bflag_auto_patrol)
                {
                    _pFlag_Value.m_bflag_patrol = true;
                    ex_iDocking_CommandMode = 0;
                }
                break;
            
            /**********************20240702 CYW************************/
            // Mission Station Docking (Bumper) Loop//
            case 300:
                ex_iDocking_CommandMode = 301;
                docking_progress.data = 300;
                docking_progress_pub.publish(docking_progress);
                break;
            case 301:
                LED_Toggle_Control(1, 3,100,3,100);
                LED_Turn_On(63);
                //usb_cam_On_client.call(m_request);
                sleep(2);
                ex_iDocking_CommandMode = 302;
                docking_progress.data = 301;
                docking_progress_pub.publish(docking_progress);
                break;
            case 302:
                MissionStation_tracking(true, _pAR_tag_pose.m_iSelect_AR_tag_id);
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    ex_iDocking_CommandMode = 0;
                }
                docking_progress.data = 302;
                docking_progress_pub.publish(docking_progress);
                break;
            case 303:
                _pAR_tag_pose.m_target_yaw = _pAR_tag_pose.m_fAR_tag_pitch;
                MissionStation_Yaw_tracking();
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    ex_iDocking_CommandMode = 0;
                }
                docking_progress.data = 303;
                docking_progress_pub.publish(docking_progress);
                break;
            case 304:
                MissionStation_tracking2(_pAR_tag_pose.m_iSelect_AR_tag_id);
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    ex_iDocking_CommandMode = 0;
                }
                docking_progress.data = 304;
                docking_progress_pub.publish(docking_progress);
                break;
            case 305:
                //charging_port_On_client.call(m_request2);
                Approach_MissionStation2Move();
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    ex_iDocking_CommandMode = 0;
                }
                docking_progress.data = 305;
                docking_progress_pub.publish(docking_progress);
                break;
            case 306:
                LED_Turn_On(9);
                
                ROS_INFO_STREAM("TETRA Mission Docking Done!");
                // m_iReset_flag = 1;
                docking_progress.data = 306;
                docking_progress_pub.publish(docking_progress);
                ex_bMissionDocking_Flag = true;
                ////PoseReset_call
                // Reset_Robot_Pose();
                LED_Toggle_Control(1, 5,100,5,1);
                LED_Turn_On(63);

                ex_iDocking_CommandMode = 0;
                break;
            case 309:
                printf("Docking FAIL ! \n");
                LED_Toggle_Control(1, 10,100,10,1);
                LED_Turn_On(18);
                ex_iDocking_CommandMode = 310;
                // ex_iDocking_CommandMode = 119;
                break;
            case 310:
                Depart_MissionStation2Move(_pAR_tag_pose.m_iSelect_AR_tag_id);
                break;
            default:
                break;
        }
        //printf("ex_iDocking_CommandMode: %d \n", ex_iDocking_CommandMode);
        usleep(20000); //20ms
    }

    pthread_cancel(p_docking_thread); //Thread kill

}

/////*******************************************************************************//////
////TEST Thread Loop...
void *AutoThread_function(void *data)
{
    while(1)
    {
        if(_pFlag_Value.m_bflag_patrol)
        {
            for(int i=0; i<m_patrol_location_cnt; i++)
            {
                LED_Toggle_Control(1,3,100,3,1);
                LED_Turn_On(100); //blue led

                OpenLocationFile(arr_patrol_location[i]);
                if(_pRobot_Status.m_iCallback_Charging_status <= 1) //Nomal
                {
                    clear_costmap_client.call(m_request);
                    setGoal(goal);
                }
                else if(_pFlag_Value.m_bumperhit_flag || ex_bMissionDocking_Flag)
                {
                    ex_iDocking_CommandMode = 310;
                    while(ex_iDocking_CommandMode != 0)
                    {
                        sleep(1);
                        ROS_INFO("Depart_MissionStation2Move....");
                    }
                }
                else //Docking check...
                {
                    ex_iDocking_CommandMode = 10; //Depart Move
                    while(ex_iDocking_CommandMode != 0)
                    {
                        sleep(1); //1 sec
                        ROS_INFO("Depart_Station2Move....");
                    }
                }

                ROS_INFO("[patrol]: goto_ %s", arr_patrol_location[i].c_str());
                while(_pRobot_Status.m_iMovebase_Result != 3)
                {
                    if(!_pFlag_Value.m_bflag_patrol && !_pFlag_Value.m_bflag_goto_cancel)
                    {
                        goto_goal_id.id = "";
                        ROS_INFO("Goto Cancel call");
                        GotoCancel_pub.publish(goto_goal_id);
                        _pFlag_Value.m_bflag_goto_cancel = true;
                    }
                    else
                        usleep(1000000); //100ms
                }
                _pRobot_Status.m_iMovebase_Result = 0;
                _pFlag_Value.m_bflag_goto_cancel = false;
            }

            if(_pRobot_Status.m_iCallback_Battery <= LOW_BATTERY)
            {
                goto_goal_id.id = "";
                ROS_INFO("Goto Cancel call");
                GotoCancel_pub.publish(goto_goal_id);
                _pFlag_Value.m_bflag_goto_cancel = true;
                usleep(1000000); //100ms

                OpenLocationFile("HOME");
                if(_pRobot_Status.m_iCallback_Charging_status <= 1) //Nomal
                {
                    setGoal(goal);
                    usleep(1000000); //100ms
                    _pFlag_Value.m_bflag_ComebackHome = true;
                }
                LED_Toggle_Control(1,3,100,3,1);
                LED_Turn_On(100);
                ROS_INFO("[Battery Low]: come back Home~");

                _pFlag_Value.m_bflag_patrol = false;
                
            }
            

        }
        
        sleep(1); //1sec
    }
    pthread_cancel(p_auto_thread); //Thread2 kill
}


//add _ GUI Button callback fuction...
void RVIZ_GUI_Callback(const std_msgs::String::ConstPtr& msg)
{
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());

    if(msg->data == "STOP") //Stop...
    {
        goto_goal_id.id = "";
        ROS_INFO("Goto Cancel call !");
        GotoCancel_pub.publish(goto_goal_id);
        ex_iDocking_CommandMode = 0;

        cmd->linear.x =  0.0; 
        cmd->angular.z = 0.0;
        cmdpub_.publish(cmd);
    }

    if(msg->data == "GETDB") //Read JSON file...
    {
        ROS_INFO("Read Virtual Wall JSON file !");
        Read_virtual_wall_JSON();
        Read_speed_zone_JSON();
    }

    if(msg->data == "SETDB") //Write JSON file...
    {
        ROS_INFO("Write Virtual Wall JSON file !");
        //Read_virtual_wall_JSON();

    }
    
}

void RVIZ_GUI_Str_Callback(const std_msgs::String::ConstPtr& msg)
{
    //msg->data.c_str();
    printf("Set Location Name: %s \n", msg->data.c_str());
    SaveLocation(msg->data.c_str(), _pTF_pose.poseTFx,_pTF_pose.poseTFy,
                                    _pTF_pose.poseTFqx,_pTF_pose.poseTFqy,_pTF_pose.poseTFqz,_pTF_pose.poseTFqw);
    
}

void RVIZ_GUI_Goto_Callback(const std_msgs::String::ConstPtr& msg)
{
    //msg->data.c_str();
    bool m_bGoto = false;
    
    if(msg->data != "") //HOME goto...
    {
        OpenLocationFile(msg->data.c_str());
        goto_goal_id.id = msg->data.c_str();
        m_bGoto = true;
    }
    else
    {

        m_bGoto = false;
    }
    

    if(m_bGoto)
    {
        ROS_INFO("goto_id.id: %s", goto_goal_id.id.c_str());
        //costmap clear call//
        clear_costmap_client.call(m_request);

        if(_pRobot_Status.m_iCallback_Charging_status <= 1) //Nomal
        {
            setGoal(goal);
        }
        else if(_pFlag_Value.m_bumperhit_flag || ex_bMissionDocking_Flag)
        {
            ex_iDocking_CommandMode = 310;
        }
        else //Docking...
        {
            ex_iDocking_CommandMode = 10; //Depart Move
        }

        if(goto_goal_id.id == "HOME") //HOME Point Check
        {
            _pFlag_Value.m_bflag_ComebackHome = true;
            //View _Ignition Point...
            node.header.frame_id = "/map";
            node.header.stamp = ros::Time(0); //ros::Time::now(); 
            node.type = visualization_msgs::Marker::SPHERE;
            node.ns = "Ignition_shapes";
            node.id = 0;
            node.action = visualization_msgs::Marker::ADD; 
            node.pose.position.x = _pGoal_pose.goal_positionX;
            node.pose.position.y = _pGoal_pose.goal_positionY;
            node.pose.position.z = _pGoal_pose.goal_positionZ;
            
            node.pose.orientation.x = 0.0;
            node.pose.orientation.y = 0.0; 
            node.pose.orientation.z = 0.0; 
            node.pose.orientation.w = 1.0; 
            
            // Points are green 
            node.color.a = 0.8; 
            node.color.r = 0.0;
            node.color.g = 0.5;
            node.color.b = 0.0;  
            node.scale.x = 0.3;
            node.scale.y = 0.3;
            node.scale.z = 0.3;

            node.lifetime = ros::Duration();

            //Publish
            landmark_pub.publish(node);
        }
        else
        {
            //View _Ignition Point...
            node.header.frame_id = "/map";
            node.header.stamp = ros::Time(0); //ros::Time::now(); 
            node.type = visualization_msgs::Marker::SPHERE;
            node.ns = "Ignition_shapes";
            node.id = 0;
            node.action = visualization_msgs::Marker::ADD; 
            node.pose.position.x = _pGoal_pose.goal_positionX;
            node.pose.position.y = _pGoal_pose.goal_positionY;
            node.pose.position.z = _pGoal_pose.goal_positionZ;
            
            node.pose.orientation.x = 0.0;
            node.pose.orientation.y = 0.0; 
            node.pose.orientation.z = 0.0; 
            node.pose.orientation.w = 1.0; 
            
            // Points are green 
            node.color.a = 0.8; 
            node.color.r = 1.0;
            node.color.g = 0.0;
            node.color.b = 0.0;  
            node.scale.x = 0.3;
            node.scale.y = 0.3;
            node.scale.z = 0.3;

            node.lifetime = ros::Duration();

            //Publish
            landmark_pub.publish(node);
        }

        m_bGoto = false; //reset
    }
}

int m_iNext_count3 = 0;
void RVIZ_Wall_Callback(const geometry_msgs::Point::ConstPtr& msg)
{
    //read file path
    std::ifstream ifs("/home/tetra/virtualwall.json");
    
    int m_ilist_count = 0;
    int m_ilist_count_size = 0;
    int m_iInt_count2 = 0;
    int m_iNext_count2 = 0;
    int m_iInt_count3 = 0;
    //int m_iNext_count3 = 0;

    m_ilist_count = msg->z;

    switch(m_ilist_count)
    {
        case 0:
            break;
        case 1:
            _pWallDataPoint.m_dform_x[0] = msg->x;
            _pWallDataPoint.m_dform_y[0] = msg->y;
            _pWallDataPoint.m_dform_z[0] = 0.0;

            //View _Ignition Point...
            node.header.frame_id = "/map";
            node.header.stamp = ros::Time(0); //ros::Time::now(); 
            node.type = visualization_msgs::Marker::SPHERE;
            node.ns = "Ignition_shapes";
            node.id = 1;
            node.action = visualization_msgs::Marker::ADD; 
            node.pose.position.x = _pWallDataPoint.m_dform_x[0];
            node.pose.position.y = _pWallDataPoint.m_dform_y[0];
            node.pose.position.z = 0.0;//printf("[RVIZ_Restrict_setting_Callback]: %.3f, %.3f, %.3f \n", msg->x, msg->y, msg->z);
            // Points are blue 
            node.color.a = 0.5; 
            node.color.r = 0.0;
            node.color.g = 0.0;
            node.color.b = 0.2;  
            node.scale.x = 0.2;
            node.scale.y = 0.2;
            node.scale.z = 0.2;

            node.lifetime = ros::Duration(3.0);

            //Publish
            landmark_pub.publish(node);

            break;
        case 2:
            _pWallDataPoint.m_dform_x[1] = msg->x;
            _pWallDataPoint.m_dform_y[1] = msg->y;
            _pWallDataPoint.m_dform_z[1] = 0.0;

            //View _Ignition Point...
            node.header.frame_id = "/map";
            node.header.stamp = ros::Time(0); //ros::Time::now(); 
            node.type = visualization_msgs::Marker::SPHERE;
            node.ns = "Ignition_shapes";
            node.id = 2;
            node.action = visualization_msgs::Marker::ADD; 
            node.pose.position.x = _pWallDataPoint.m_dform_x[1];
            node.pose.position.y = _pWallDataPoint.m_dform_y[1];
            node.pose.position.z = 0.0;
            
            node.pose.orientation.x = 0.0;
            node.pose.orientation.y = 0.0; 
            node.pose.orientation.z = 0.0; 
            node.pose.orientation.w = 1.0; 
            
            // Points are blue 
            node.color.a = 0.5; 
            node.color.r = 0.0;
            node.color.g = 0.0;
            node.color.b = 0.2;  
            node.scale.x = 0.2;
            node.scale.y = 0.2;
            node.scale.z = 0.2;

            node.lifetime = ros::Duration(3.0);

            //Publish
            landmark_pub.publish(node);
            break;
        case 3:
            _pWallDataPoint.m_dform_x[2] = msg->x;
            _pWallDataPoint.m_dform_y[2] = msg->y;
            _pWallDataPoint.m_dform_z[2] = 0.0;

            //View _Ignition Point...
            node.header.frame_id = "/map";
            node.header.stamp = ros::Time(0); //ros::Time::now(); 
            node.type = visualization_msgs::Marker::SPHERE;
            node.ns = "Ignition_shapes";
            node.id = 3;
            node.action = visualization_msgs::Marker::ADD; 
            node.pose.position.x = _pWallDataPoint.m_dform_x[2];
            node.pose.position.y = _pWallDataPoint.m_dform_y[2];
            node.pose.position.z = 0.0;
            
            node.pose.orientation.x = 0.0;
            node.pose.orientation.y = 0.0; 
            node.pose.orientation.z = 0.0; 
            node.pose.orientation.w = 1.0; 
            
            // Points are blue 
            node.color.a = 0.5; 
            node.color.r = 0.0;
            node.color.g = 0.0;
            node.color.b = 0.2;  
            node.scale.x = 0.2;
            node.scale.y = 0.2;
            node.scale.z = 0.2;

            node.lifetime = ros::Duration(3.0);

            //Publish
            landmark_pub.publish(node);
            break;
        case 4:
            _pWallDataPoint.m_dform_x[3] = msg->x;
            _pWallDataPoint.m_dform_y[3] = msg->y;
            _pWallDataPoint.m_dform_z[3] = 0.0;

            //View _Ignition Point...
            node.header.frame_id = "/map";
            node.header.stamp = ros::Time(0); //ros::Time::now(); 
            node.type = visualization_msgs::Marker::SPHERE;
            node.ns = "Ignition_shapes";
            node.id = 4;
            node.action = visualization_msgs::Marker::ADD; 
            node.pose.position.x = _pWallDataPoint.m_dform_x[3];
            node.pose.position.y = _pWallDataPoint.m_dform_y[3];
            node.pose.position.z = 0.0;
            
            node.pose.orientation.x = 0.0;
            node.pose.orientation.y = 0.0; 
            node.pose.orientation.z = 0.0; 
            node.pose.orientation.w = 1.0; 
            
            // Points are blue 
            node.color.a = 0.5; 
            node.color.r = 0.0;
            node.color.g = 0.0;
            node.color.b = 0.2;  
            node.scale.x = 0.2;
            node.scale.y = 0.2;
            node.scale.z = 0.2;

            node.lifetime = ros::Duration(3.0);

            //Publish
            landmark_pub.publish(node);
            
            m_ilist_count_size = _pWallDataPoint.m_ilist_count_size += 1;
            //printf("m_ilist_count_size: %d \n", m_ilist_count_size);

           
            virtual_obstacle.list.resize(m_ilist_count_size);
            for(int i=(m_ilist_count_size-1); i<m_ilist_count_size; i++)
            {
                virtual_obstacle.list[i].form.resize(m_ilist_count);
                m_iInt_count2 = m_ilist_count;
                for(int j=0; j<m_ilist_count; j++)
                {
                    virtual_obstacle.list[i].form[j].x = _pWallDataPoint.m_dform_x[m_iNext_count2 + j];
                    virtual_obstacle.list[i].form[j].y = _pWallDataPoint.m_dform_y[m_iNext_count2 + j];
                    virtual_obstacle.list[i].form[j].z = _pWallDataPoint.m_dform_z[m_iNext_count2 + j];
                }
                m_iNext_count2 += m_iInt_count2;
                
            }
            
            virtual_obstacle_pub.publish(virtual_obstacle);

            //write JSON..
            if(ifs)
            {
                IStreamWrapper isw(ifs);
                Document doc;
                doc.ParseStream(isw);
                Document::AllocatorType& allocator = doc.GetAllocator();

                if(m_ilist_count_size > doc["list_count"].Size())
                {
                    doc["list_count"].PushBack(m_ilist_count,allocator);
                    // change value
                    for(int i=(m_ilist_count_size-1); i<m_ilist_count_size; i++)
                    {
                        m_iInt_count3 = m_ilist_count;
                        for(int j=0; j<m_ilist_count; j++)
                        {
                            doc["form_x"].PushBack(floor(virtual_obstacle.list[i].form[j].x * 1000.f + 0.5) / 1000.f ,allocator);
                            doc["form_y"].PushBack(floor(virtual_obstacle.list[i].form[j].y * 1000.f + 0.5) / 1000.f ,allocator);
                            doc["form_z"].PushBack(floor(virtual_obstacle.list[i].form[j].z * 1000.f + 0.5) / 1000.f ,allocator);

                        }   
                        m_iNext_count3 += m_iInt_count3;
                        //printf("[%d]m_iInt_count3: %d , m_iNext_count3: %d \n",i, m_iInt_count3, m_iNext_count3);
                    }
                }
                else
                {
                    // change value
                    for(int i=(m_ilist_count_size-1); i<m_ilist_count_size; i++)
                    {
                        m_iInt_count3 = m_ilist_count;
                        for(int j=0; j<m_ilist_count; j++)
                        {
                            doc["form_x"][m_iNext_count3 + j].SetDouble(floor(virtual_obstacle.list[i].form[j].x * 1000.f + 0.5) / 1000.f);
                            doc["form_y"][m_iNext_count3 + j].SetDouble(floor(virtual_obstacle.list[i].form[j].y * 1000.f + 0.5) / 1000.f);
                            doc["form_z"][m_iNext_count3 + j].SetDouble(floor(virtual_obstacle.list[i].form[j].z * 1000.f + 0.5) / 1000.f);
                        }   
                        m_iNext_count3 += m_iInt_count3;
                    }
                }
                
                // write file
                std::ofstream ofs("/home/tetra/virtualwall.json");
                OStreamWrapper osw(ofs);
                Writer<OStreamWrapper> writer(osw);
                doc.Accept(writer);
                    
            }
            

            //View _Ignition Point to Text...
            node_name.header.frame_id = "/map";
            node_name.header.stamp = ros::Time(0);
            node_name.text = std::to_string(m_ilist_count_size);
            node_name.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            node_name.id = m_ilist_count_size;
            node_name.action = visualization_msgs::Marker::ADD; 
            node_name.pose.position.x = (_pWallDataPoint.m_dform_x[0] + _pWallDataPoint.m_dform_x[2]) / 2.0;//_pWallDataPoint.m_dform_x[3];
            node_name.pose.position.y = (_pWallDataPoint.m_dform_y[0] + _pWallDataPoint.m_dform_y[2]) / 2.0;//_pWallDataPoint.m_dform_y[3];
            node_name.pose.orientation.w = 1.0; 
            node_name.color.a = 1.0; 
            node_name.color.r = 0.0;
            node_name.color.g = 0.5;
            node_name.color.b = 0.0; 
            node_name.scale.z = 0.6;

            //Publish
            landmark_pub.publish(node_name);

            break;
        default:
            break;
    }
}

void RVIZ_Restrict_setting_Callback(const geometry_msgs::Point::ConstPtr& msg)
{
    //read file path
    std::ifstream ifs("/home/tetra/virtualwall.json");

    int m_ilist_num = msg->x - 1.0;
    int m_iflag = msg->y;

    //printf("[RVIZ_Restrict_setting_Callback]: %d \n", m_ilist_num);
    if(m_iflag)
    {
        // msg clear
        virtual_obstacle.list.resize(_pWallDataPoint.m_ilist_count_size + 1);
        virtual_obstacle.list[m_ilist_num].form.resize(4);
        for(int j=0; j<4; j++)
        {
            virtual_obstacle.list[m_ilist_num].form[j].x = 0.0;
            virtual_obstacle.list[m_ilist_num].form[j].y = 0.0;
            virtual_obstacle.list[m_ilist_num].form[j].z = 0.0;

        }
    
        
        virtual_obstacle_pub.publish(virtual_obstacle);   

        //View _Ignition Point to Text...
        node_name.header.frame_id = "/map";
        node_name.header.stamp = ros::Time(0);
        node_name.text = "";
        node_name.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        node_name.id = msg->x;
        node_name.action = visualization_msgs::Marker::DELETE;
        landmark_pub.publish(node_name);

        //write JSON..
        if(ifs)
        {
            IStreamWrapper isw(ifs);
            Document doc;
            doc.ParseStream(isw);
            Document::AllocatorType& allocator = doc.GetAllocator();

            // change value
            for(int j=0; j<4; j++)
            {
                doc["form_x"][(m_ilist_num * 4) + j].SetDouble(0.0);
                doc["form_y"][(m_ilist_num * 4) + j].SetDouble(0.0);
                doc["form_z"][(m_ilist_num * 4) + j].SetDouble(0.0);
            }   
            
            // write file
            std::ofstream ofs("/home/tetra/virtualwall.json");
            OStreamWrapper osw(ofs);
            Writer<OStreamWrapper> writer(osw);
            doc.Accept(writer);

        } 
    }
}

//add...Zone
int m_iNext_count4 = 0;
void RVIZ_Zone_Callback(const geometry_msgs::Point::ConstPtr& msg)
{
    //read file path
    std::ifstream ifs("/home/tetra/speedzone.json");

    int m_ilist_count2 = 0;
    int m_ilist_count_size2 = 0;

    int m_iInt_count3 = 0;

    m_ilist_count2 = msg->z;
    switch(m_ilist_count2)
    {
        case 0:
            break;
        case 1:
            _pZoneDataPoint.m_dform_x2[0] = msg->x;
            _pZoneDataPoint.m_dform_y2[0] = msg->y;
            break;
        case 2:
            _pZoneDataPoint.m_dform_x2[1] = msg->x;
            _pZoneDataPoint.m_dform_y2[1] = msg->y;
            
            //View _Ignition Point...
            node2.header.frame_id = "/map";
            node2.header.stamp = ros::Time(0); //ros::Time::now(); 
            node2.type = visualization_msgs::Marker::CUBE;
            node2.id = _pZoneDataPoint.m_iZone_count;//_pZoneDataPoint.m_iZone_count += 1;
            node2.ns = "Ignition_cube_" + std::to_string(node2.id);
            node2.action = visualization_msgs::Marker::ADD; 
            node2.scale.x = abs((_pZoneDataPoint.m_dform_x2[1]) - (_pZoneDataPoint.m_dform_x2[0]));
            node2.scale.y = abs((_pZoneDataPoint.m_dform_y2[1]) - (_pZoneDataPoint.m_dform_y2[0]));
            node2.scale.z = 0.1;
            //printf("node.scale.x: %.3f , node.scale.y: %.3f \n", node.scale.x, node.scale.y);
            node2.pose.position.x = ((_pZoneDataPoint.m_dform_x2[0]) + (_pZoneDataPoint.m_dform_x2[1])) / 2.0;
            node2.pose.position.y = ((_pZoneDataPoint.m_dform_y2[0]) + (_pZoneDataPoint.m_dform_y2[1])) / 2.0;
            node2.pose.position.z = 0.0;
            node2.pose.orientation.x = 0.0;
            node2.pose.orientation.y = 0.0;
            node2.pose.orientation.z = 0.0;
            node2.pose.orientation.w = 1.0;
            // Points are blue 
            node2.color.a = 0.5;
            node2.color.r = 0.8;
            node2.color.g = 0.8;
            node2.color.b = 0.0;
            node2.lifetime = ros::Duration(0.0);
            //Publish
            landmark_pub2.publish(node2);

            //4 POINT Calc_CCW rotation ////////////////////////////////////////////////
            _pTagList[_pZoneDataPoint.m_iZone_count]._pTagPoint[0] = {_pZoneDataPoint.m_dform_x2[0], _pZoneDataPoint.m_dform_y2[0]};
            _pTagList[_pZoneDataPoint.m_iZone_count]._pTagPoint[1] = {_pZoneDataPoint.m_dform_x2[0], _pZoneDataPoint.m_dform_y2[1]};
            _pTagList[_pZoneDataPoint.m_iZone_count]._pTagPoint[2] = {_pZoneDataPoint.m_dform_x2[1], _pZoneDataPoint.m_dform_y2[1]};
            _pTagList[_pZoneDataPoint.m_iZone_count]._pTagPoint[3] = {_pZoneDataPoint.m_dform_x2[1], _pZoneDataPoint.m_dform_y2[0]};

            vV[_pZoneDataPoint.m_iZone_count].push_back(_pTagList[_pZoneDataPoint.m_iZone_count]._pTagPoint[0]);
            vV[_pZoneDataPoint.m_iZone_count].push_back(_pTagList[_pZoneDataPoint.m_iZone_count]._pTagPoint[1]);
            vV[_pZoneDataPoint.m_iZone_count].push_back(_pTagList[_pZoneDataPoint.m_iZone_count]._pTagPoint[2]);
            vV[_pZoneDataPoint.m_iZone_count].push_back(_pTagList[_pZoneDataPoint.m_iZone_count]._pTagPoint[3]);
            ////////////////////////////////////////////////////////////////////////////
            //View _Ignition Point to Text...
            node_name2.header.frame_id = "/map";
            node_name2.header.stamp = ros::Time(0);
            node_name2.text = std::to_string(node2.id);
            node_name2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            node_name2.id = node2.id;
            node_name2.action = visualization_msgs::Marker::ADD; 
            node_name2.pose.position.x = ((_pZoneDataPoint.m_dform_x2[0]) + (_pZoneDataPoint.m_dform_x2[1])) / 2.0;
            node_name2.pose.position.y = ((_pZoneDataPoint.m_dform_y2[0]) + (_pZoneDataPoint.m_dform_y2[1])) / 2.0;
            node_name2.pose.orientation.w = 1.0; 
            node_name2.color.a = 1.0; 
            node_name2.color.r = 0.0;
            node_name2.color.g = 0.0;
            node_name2.color.b = 0.5; 
            node_name2.scale.z = 0.5;
            //Publish
            landmark_pub2.publish(node_name2);

            //write JSON file//
            if(ifs)
            {
                IStreamWrapper isw(ifs);
                Document doc;
                doc.ParseStream(isw);
                Document::AllocatorType& allocator = doc.GetAllocator();

                doc["tag_list"].PushBack(4,allocator);

                doc["form_x"].PushBack(floor(_pZoneDataPoint.m_dform_x2[0] * 1000.f + 0.5) / 1000.f ,allocator);
                doc["form_y"].PushBack(floor(_pZoneDataPoint.m_dform_y2[0] * 1000.f + 0.5) / 1000.f ,allocator);

                doc["form_x"].PushBack(floor(_pZoneDataPoint.m_dform_x2[0] * 1000.f + 0.5) / 1000.f ,allocator);
                doc["form_y"].PushBack(floor(_pZoneDataPoint.m_dform_y2[1] * 1000.f + 0.5) / 1000.f ,allocator);

                doc["form_x"].PushBack(floor(_pZoneDataPoint.m_dform_x2[1] * 1000.f + 0.5) / 1000.f ,allocator);
                doc["form_y"].PushBack(floor(_pZoneDataPoint.m_dform_y2[1] * 1000.f + 0.5) / 1000.f ,allocator);

                doc["form_x"].PushBack(floor(_pZoneDataPoint.m_dform_x2[1] * 1000.f + 0.5) / 1000.f ,allocator);
                doc["form_y"].PushBack(floor(_pZoneDataPoint.m_dform_y2[0] * 1000.f + 0.5) / 1000.f ,allocator);

                // write file
                std::ofstream ofs("/home/tetra/speedzone.json");
                OStreamWrapper osw(ofs);
                Writer<OStreamWrapper> writer(osw);
                doc.Accept(writer);

            }

            _pZoneDataPoint.m_iZone_count++;
            if(node_name2.id > 0)
                m_flag_SpeedZone = true;

            break;
        default:
            break;
    }

}

void RVIZ_Restrict_setting2_Callback(const geometry_msgs::Point::ConstPtr& msg)
{
    //read file path
    std::ifstream ifs("/home/tetra/speedzone.json");

    int m_ilist_num2 = msg->x;
    int m_iflag2 = msg->y;

    _pTagList[_pZoneDataPoint.m_iZone_count-1].dSpeed_value = msg->z;
    //printf("_pTagList[%d].dSpeed_value: %.3f \n", _pZoneDataPoint.m_iZone_count, _pTagList[_pZoneDataPoint.m_iZone_count-1].dSpeed_value);

    if(m_iflag2)
    {
        //View _Ignition Point...
        node2.header.frame_id = "/map";
        node2.header.stamp = ros::Time(0); //ros::Time::now(); 
        node2.type = visualization_msgs::Marker::CUBE;
        node2.id = m_ilist_num2;
        node2.ns = "Ignition_cube_" + std::to_string(node2.id);
        node2.action = visualization_msgs::Marker::DELETE; 
        //Publish
        landmark_pub2.publish(node2);

        //View _Ignition Point to Text...
        node_name2.header.frame_id = "/map";
        node_name2.header.stamp = ros::Time(0);
        node_name2.text = "";
        node_name2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        node_name2.id = m_ilist_num2;
        node_name2.action = visualization_msgs::Marker::DELETE;
        //Publish
        landmark_pub2.publish(node_name2);
    }

    //write JSON file//
    if(ifs)
    {
        IStreamWrapper isw(ifs);
        Document doc;
        doc.ParseStream(isw);
        Document::AllocatorType& allocator = doc.GetAllocator();

        doc["speed"].PushBack(floor(_pTagList[_pZoneDataPoint.m_iZone_count-1].dSpeed_value * 1000.f + 0.5) / 1000.f ,allocator);

        // write file
        std::ofstream ofs("/home/tetra/speedzone.json");
        OStreamWrapper osw(ofs);
        Writer<OStreamWrapper> writer(osw);
        doc.Accept(writer);

    }
}

void Reset_Call_service()
{
    _pFlag_Value.m_bFlag_nomotion = false;

    //IMU reset//
    euler_angle_reset_cmd_client.call(euler_angle_reset_srv);
    printf("## IMU Reset ! \n");
    //tetra odometry Reset//
    tetra_PoseRest.data = m_iReset_flag;
    PoseReset_pub.publish(tetra_PoseRest);
    usleep(100000);

    //robot_localization::SetPose ekf_reset;
	setpose_srv.request.pose.header.frame_id = tf_prefix_ + "/odom";
	setpose_srv.request.pose.header.stamp = ros::Time(0); //ros::Time::now();

	setpose_srv.request.pose.pose.pose.position.x = _pReset_srv.init_position_x;
	setpose_srv.request.pose.pose.pose.position.y = _pReset_srv.init_position_y;
	setpose_srv.request.pose.pose.pose.position.z = _pReset_srv.init_position_z;

	setpose_srv.request.pose.pose.pose.orientation.x = _pReset_srv.init_orientation_x;
	setpose_srv.request.pose.pose.pose.orientation.y = _pReset_srv.init_orientation_y;
	setpose_srv.request.pose.pose.pose.orientation.z = _pReset_srv.init_orientation_z;
	setpose_srv.request.pose.pose.pose.orientation.w = _pReset_srv.init_orientation_w;

	setpose_srv.request.pose.pose.covariance[0] = 0.25;
	setpose_srv.request.pose.pose.covariance[6 * 1 + 1] = 0.25;
	setpose_srv.request.pose.pose.covariance[6 * 5 + 5] = 0.06853892326654787;
    
	SetPose_cmd_client.call(setpose_srv); //Set_pose call//
	printf("##Set_Pose(EKF)2! \n");

    initPose_.header.stamp = ros::Time(0); //ros::Time::now(); 
    initPose_.header.frame_id = "map";
    //position
    initPose_.pose.pose.position.x = _pReset_srv.init_position_x;
    initPose_.pose.pose.position.y = _pReset_srv.init_position_y;
    initPose_.pose.pose.position.z = _pReset_srv.init_position_z;
    //orientation
    initPose_.pose.pose.orientation.x = _pReset_srv.init_orientation_x;
    initPose_.pose.pose.orientation.y = _pReset_srv.init_orientation_y;
    initPose_.pose.pose.orientation.z = _pReset_srv.init_orientation_z;
    initPose_.pose.pose.orientation.w = _pReset_srv.init_orientation_w;

    initPose_.pose.covariance[0] = 0.25;
    initPose_.pose.covariance[6 * 1 + 1] = 0.25;
    initPose_.pose.covariance[6 * 5 + 5] = 0.06853892326654787;

    initialpose_pub.publish(initPose_);
    printf("##Set_initPose(2D Estimate)2! \n");

    usleep(500000);

    _pFlag_Value.m_bFlag_nomotion = true;

}

bool SetEKF_Command(tetraDS_service::setekf::Request &req, 
				    tetraDS_service::setekf::Response &res)
{   
    bool bResult = false;

    _pReset_srv.init_position_x = req.init_position_x;
    _pReset_srv.init_position_y = req.init_position_y;
    _pReset_srv.init_position_z = req.init_position_z;

    _pReset_srv.init_orientation_x = req.init_orientation_x;
    _pReset_srv.init_orientation_y = req.init_orientation_y;
    _pReset_srv.init_orientation_z = req.init_orientation_z;
    _pReset_srv.init_orientation_w = req.init_orientation_w;

    _pReset_srv.bflag_reset = true;

    //Reset_Call_service();

    bResult = true;
	res.command_Result = bResult;
	return true;
}

//InitialposeCallback
void InitialposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgInitialpose)
{
    //printf("## InitialposeCallback ## \n");
    _pFlag_Value.m_bFlag_Initialpose = true;


}

bool Set_Weight_obstacle_Command(tetraDS_service::setweightobstacle::Request &req, 
				                tetraDS_service::setweightobstacle::Response &res)
{
    bool bResult = false;

    Dynamic_reconfigure_Teb_Set_DoubleParam("weight_obstacle", (double)req.weight_obstacle);
    Set_Weight_obstacle = (double)req.weight_obstacle;

    /*
    float64 weight_obstacle
    ---
    float64 weight_obstacle
    bool command_Result
    */
    res.weight_obstacle = req.weight_obstacle;
    bResult = true;
    res.command_Result = bResult;
    return true;
}


void resultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msgResult)
{
   uint8_t PENDING    = 0;   // The goal has yet to be processed by the action server
   uint8_t ACTIVE     = 1;   // The goal is currently being processed by the action server
   uint8_t PREEMPTED  = 2;   // The goal received a cancel request after it started executing
                             //   and has since completed its execution (Terminal State)
   uint8_t SUCCEEDED  = 3;   // The goal was achieved successfully by the action server (Terminal State)
   uint8_t ABORTED    = 4;   // The goal was aborted during execution by the action server due
                             //    to some failure (Terminal State)
   uint8_t REJECTED   = 5;   // The goal was rejected by the action server without being processed,
                             //    because the goal was unattainable or invalid (Terminal State)
   uint8_t PREEMPTING = 6;   // The goal received a cancel request after it started executing
                             //    and has not yet completed execution
   uint8_t RECALLING  = 7;   // The goal received a cancel request before it started executing,
                             //    but the action server has not yet confirmed that the goal is canceled
   uint8_t RECALLED   = 8;   // The goal received a cancel request before it started executing
                             //    and was successfully cancelled (Terminal State)
   uint8_t LOST       = 9;   // An action client can determine that a goal is LOST. This should not be
                             //    sent over the wire by an action server
   time_t curr_time;
   struct tm *curr_tm;
   curr_time = time(NULL);
   curr_tm = localtime(&curr_time);

  if(msgResult->status.status == SUCCEEDED)
  { 
    ROS_INFO("[SUCCEEDED]resultCallback: %d ",msgResult->status.status);
    _pFlag_Value.m_bflag_NextStep = true;
    _pRobot_Status.m_iMovebase_Result = 3;
    m_iRetry_cnt = 0;
    //LED_Turn_On(4);
    LED_Control(1, 100);
    usleep(100000); 
    LED_Control(2, 100);
    //costmap clear call//
    //clear_costmap_client.call(m_request);
    // initial
    _pGoal_pose.goal_positionX = 0.0;
    _pGoal_pose.goal_positionY = 0.0;

    if(_pFlag_Value.m_bflag_ComebackHome) //Home Postion -> docking mode start
    {
        _pAR_tag_pose.m_iSelect_AR_tag_id = _pRobot_Status.HOME_ID; //0;
        ex_iDocking_CommandMode = 1;
        _pFlag_Value.m_bflag_ComebackHome = false;
    } 

    if(_pFlag_Value.m_bflag_Conveyor_docking) //Conveyor Postion -> docking mode start
    {
        _pAR_tag_pose.m_iSelect_AR_tag_id = _pRobot_Status.CONVEYOR_ID;
        ex_iDocking_CommandMode = 11;
        _pFlag_Value.m_bflag_Conveyor_docking = false;
    }

    m_flag_PREEMPTED = false;
    _pFlag_Value.m_bFlag_pub = false;
    _pGoal_pose.goal_positionX = 0.0;
    _pGoal_pose.goal_positionY = 0.0;

	//position
	_pReset_srv.init_position_x = _pTF_pose.poseTFx;
	_pReset_srv.init_position_y = _pTF_pose.poseTFy;
	_pReset_srv.init_position_z = _pTF_pose.poseTFz;
	//orientation
	_pReset_srv.init_orientation_x = _pTF_pose.poseTFqx;
	_pReset_srv.init_orientation_y = _pTF_pose.poseTFqy;
	_pReset_srv.init_orientation_z = _pTF_pose.poseTFqz;
	_pReset_srv.init_orientation_w = _pTF_pose.poseTFqw;
    // reset pose
    //Reset_Call_service();

  }
  else if( msgResult->status.status == ABORTED)
  {
    LED_Toggle_Control(1, 10,100,10,1);
    LED_Turn_On(18);
    printf("[ERROR]resultCallback _ RED LED On \n");
    _pFlag_Value.m_bflag_NextStep = false;
    ROS_INFO("[ERROR]resultCallback: %d ",msgResult->status.status);

    m_flag_setgoal = true;

    goto_goal_id.id = "";
    ROS_INFO("Goto Cancel call");
    GotoCancel_pub.publish(goto_goal_id);
    _pFlag_Value.m_bFlag_pub = false;

    if(m_iRetry_cnt >= MAX_RETRY_CNT)
    {
        m_iRetry_cnt = 0;
        ROS_INFO("[RETRY Behavior]: FAIL (%d)! \n", m_iRetry_cnt);
    }
    else
    {
        if(!ex_bMissionDocking_Flag)
        {
            //costmap clear call//
            clear_costmap_client.call(m_request);

            LED_Toggle_Control(1, 3,100,3,1);
            if(_pFlag_Value.m_bflag_Conveyor_docking)
                LED_Turn_On(45); //sky_blue
            else
                LED_Turn_On(63);

            ROS_INFO("[RETRY Behavior]: goto_ %s", goal.goal_id.id.c_str());
            setGoal(goal);
            m_iRetry_cnt++;
        }
    }

    m_flag_setgoal = false;
    m_flag_PREEMPTED = false;


  }
  else if(msgResult->status.status == PREEMPTED) //bumper On Check...
  {
    // if(_pFlag_Value.BUMPER_BT)
    //     ex_iDocking_CommandMode = 100;
    m_flag_PREEMPTED = true;
    
  }
  else
  {
    _pFlag_Value.m_bflag_NextStep = false;
    ROS_INFO("resultCallback: %d ",msgResult->status.status);
    _pRobot_Status.m_iMovebase_Result = msgResult->status.status;
    //costmap clear call//
    //clear_costmap_client.call(m_request);
  }

}

/////*******************************************************************************//////

int main (int argc, char** argv)
{
    signal(SIGINT,my_handler);

    ros::init(argc, argv, "tetraDS_service", ros::init_options::NoSigintHandler);

    ros::NodeHandle nh;
    cmdpub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel",100);
    ros::Subscriber cmdsub_ = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 100, cmd_vel_Callback);
    //Servo On/Off publish
    servo_pub = nh.advertise<std_msgs::Int32>("Servo_ON",10);
    service_pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 10);
    ros::Subscriber sub_amcl = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 100 ,poseAMCLCallback);
    //Navigation Result//
    ros::Subscriber result_sub = nh.subscribe<move_base_msgs::MoveBaseActionResult>("move_base/result", 10, resultCallback);
    ros::Subscriber status_sub = nh.subscribe<actionlib_msgs::GoalStatusArray>("move_base/status", 10, statusCallback);
    //Navigation Cancel//
    GotoCancel_pub = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel",10);
    //PoseReset//
    PoseReset_pub = nh.advertise<std_msgs::Int32>("PoseRest",10);
    //Acceleration input//
    Accel_pub = nh.advertise<std_msgs::Int32>("accel_vel",10);
    //AR_TAG_subscriber//
    ros::Subscriber AR_sub = nh.subscribe("ar_pose_marker", 100, AR_tagCallback);
    //Laser Scan subscriber//
    ros::Subscriber scan_sub = nh.subscribe("scan", 100, LaserScanCallback);
    //Depthimage to scan subscriber//
    ros::Subscriber pcl1_sub = nh.subscribe("pcl_1", 100, PCL1_Callback); //Front Realsense D435F
    //ros::Subscriber pcl2_sub = nh.subscribe("pcl_2", 100, PCL2_Callback); //Rear Realsense D435F
    //Cygbot LiDARto scan subscriber//
    ros::Subscriber cygbot_sub = nh.subscribe("scan_laser", 100, Cygbot_Callback); //Rear Cygbot LiDAR

    //virtual costmap_pub
    virtual_obstacle_pub = nh.advertise<virtual_costmap_layer::Obstacles>("virtual_costamp_layer/obsctacles", 100);
    virtual_obstacle2_pub = nh.advertise<virtual_costmap_layer2::Obstacles2>("virtual_costamp_layer2/obsctacles", 100);
    //amcl particlecloud Subscribe
    ros::Subscriber pacticle_sub = nh.subscribe<geometry_msgs::PoseArray>("particlecloud", 3000, Particle_Callback);
    //teb Markers Subscribe
    ros::Subscriber tebmarksers_sub = nh.subscribe<visualization_msgs::Marker>("move_base/TebLocalPlannerROS/teb_markers", 100, TebMarkers_Callback);
    //teb_localPlan Subscribe
    ros::Subscriber teblocalplan_sub = nh.subscribe<geometry_msgs::PoseArray>("move_base/TebLocalPlannerROS/teb_poses", 100, Teblocalplan_Callback);
    //Initialpose publish//
    initialpose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 100);
    //Initialpose Subscribe
    ros::Subscriber sub_initialpose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 100 ,InitialposeCallback);
    //Joystick//
    ros::NodeHandle njoy;
    ros::Subscriber joy_sub = njoy.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

    //Read HOME Docking ID Param Read//
    nh.getParam("HOME_ID", _pRobot_Status.HOME_ID);
    printf("##HOME_ID: %d \n", _pRobot_Status.HOME_ID);
    //add Bumper to Behavior
    nh.getParam("bumper_behavior", _pFlag_Value.BUMPER_BT);
    printf("##bumper_behavior: %d \n", _pFlag_Value.BUMPER_BT);
    //Read Max_linear_Velocity
    nh.getParam("max_vel_x", _pDynamic_param.MAX_Linear_velocity);
    printf("##max_vel_x: %f \n", _pDynamic_param.MAX_Linear_velocity);
    _pDynamic_param.MAX_Linear_velocity_origin = _pDynamic_param.MAX_Linear_velocity;
	
    ros::param::get("tf_prefix", tf_prefix_);

    //add GUI...
    ros::NodeHandle nGUI;
    ros::Subscriber GUI_sub = nGUI.subscribe("/rviz_visual_tools_gui_btn", 10, RVIZ_GUI_Callback);
    ros::Subscriber GUI_Str_sub = nGUI.subscribe("/rviz_visual_tools_gui_location_name", 10, RVIZ_GUI_Str_Callback);
    ros::Subscriber GUI_goto_sub = nGUI.subscribe("/rviz_visual_tools_gui_goto_location_name", 10, RVIZ_GUI_Goto_Callback);
	
    //add rviz tool wall create...    
    ros::Subscriber Wall_data_sub = nGUI.subscribe<geometry_msgs::Point>("/wall_rectangle_data", 10, RVIZ_Wall_Callback);
    //add rviz tool restrict_settings...
    ros::Subscriber Wall_clear_sub = nGUI.subscribe<geometry_msgs::Point>("/restrict_settings", 10, RVIZ_Restrict_setting_Callback);
    landmark_pub = nGUI.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    //add rviz tool zone create...
    ros::Subscriber Zone_data_sub = nGUI.subscribe<geometry_msgs::Point>("/zone_rectangle_data", 10, RVIZ_Zone_Callback);
    //add rviz tool restrict_settings2...
    ros::Subscriber Zone_clear_sub = nGUI.subscribe<geometry_msgs::Point>("/restrict_settings2", 10, RVIZ_Restrict_setting2_Callback);
    landmark_pub2 = nGUI.advertise<visualization_msgs::Marker>("visualization_marker2", 1);
    
    ros::NodeHandle nTest;
    landmark_pub = nTest.advertise<visualization_msgs::Marker>("visualization_marker", 1); // 240314 mwcha

    //Command Service//
    ros::NodeHandle service_h;
    getlocation_service = service_h.advertiseService("getlocation_cmd", GetLocation_Command);
    goto_service = service_h.advertiseService("goto_cmd", Goto_Command);
    goto_service2 = service_h.advertiseService("goto_cmd2", Goto_Command2);
    setlocation_service = service_h.advertiseService("setlocation_cmd", SetLocation_Command);
    save_map_service = service_h.advertiseService("savemap_cmd", SetSavemap_Command);
    getinfo_service = service_h.advertiseService("getinfo_cmd", GetInformation_Command);
    docking_service = service_h.advertiseService("docking_cmd", Docking_Command);
    locationlist_service = service_h.advertiseService("locationlist_cmd", LocationList_Command);
    delete_location_service = service_h.advertiseService("delete_location_cmd", DeleteLocation_Command);
    //Land mark Service//
    landmarklist_service = service_h.advertiseService("landmarklist_cmd", LandmarkList_Command);
    delete_landmark_service = service_h.advertiseService("delete_landmark_cmd", DeleteLandmark_Command);
    //Map Service//
    maplist_service = service_h.advertiseService("maplist_cmd", MapList_Command);
    delete_map_service = service_h.advertiseService("delete_map_cmd", DeleteMap_Command);
    gotocancel_service = service_h.advertiseService("gotocancel_cmd", GotoCancel_Command);
    sloptime_service = service_h.advertiseService("sloptime_cmd", SlopTime_Command);
    servo_service = service_h.advertiseService("servo_cmd", Servo_Command);
    //Docking Exit Service//
    docking_exit = service_h.advertiseService("docking_Stop", DockingStop_Command);
    //Dynamic reconfigure Service//
    setspeed_service = service_h.advertiseService("setspeed_cmd", Setspeed_Command);
    //rosrun & roslaunch command//
    mapping_service = service_h.advertiseService("mapping_cmd", mapping_Command);
    navigation_service = service_h.advertiseService("navigation_cmd", navigation_Command);
    nodekill_service = service_h.advertiseService("nodekill_cmd", nodekill_Command);
    //set initPose command//
    setinitpose_service = service_h.advertiseService("setinitpose_cmd", SetInitPose_Command);
    //set 2D_Pose_Estimate command//
    pose_Estimate_service = service_h.advertiseService("pose_estimate_cmd", Set2D_Pose_Estimate_Command);
    //Convetor Service//
    gotoconveyor_service = service_h.advertiseService("gotoconveyor_cmd", Goto_Conveyor_Command);
    loadingcheck_service = service_h.advertiseService("loadingcheck_service_cmd", Loading_check_Command);
    unloadingcheck_service = service_h.advertiseService("unloadingcheck_service_cmd", Unloading_check_Command);
    //Patrol Service//
    patrol_service = service_h.advertiseService("patrol_cmd", Patrol_Command);
    //patrol_conveyor_service = service_h.advertiseService("patrol_conveyor_cmd", Patrol_Conveyor_Command);
    //Delete Data All Service//
    deletedataall_service = service_h.advertiseService("deletedataall_cmd", DeleteData_All_Command);
    //Virtual costmap Service//
    virtual_obstacle_service = service_h.advertiseService("virtual_obstacle_cmd", Virtual_Obstacle_Command);
    //Set EKF & IMU Reset Service//
    set_ekf_service = service_h.advertiseService("set_ekf_cmd", SetEKF_Command);
    
    set_weight_obstacle_service = service_h.advertiseService("setweightobstacle_cmd", Set_Weight_obstacle_Command);
    
    //usb_cam Service Client...
    ros::NodeHandle client_h;
    usb_cam_On_client = client_h.serviceClient<std_srvs::Empty>("usb_cam/start_capture");
    usb_cam_Off_client = client_h.serviceClient<std_srvs::Empty>("usb_cam/stop_capture");
    //Charging Port Service Client...
    charging_port_On_client = client_h.serviceClient<std_srvs::Empty>("charging_port_on");
    charging_port_Off_client = client_h.serviceClient<std_srvs::Empty>("charging_port_off");
    //request_nomotion_update Service Client
    request_nomotion_update_client = client_h.serviceClient<std_srvs::Empty>("request_nomotion_update");
    //LED Control Client//
    led_cmd_client = client_h.serviceClient<tetraDS_service::ledcontrol>("led_cmd");
    ledtoggle_cmd_client = client_h.serviceClient<tetraDS_service::ledtogglecontrol>("ledtoggle_cmd");
    turnon_cmd_client = client_h.serviceClient<tetraDS_service::toggleon>("turnon_cmd");
    //Clear costmaps//
    clear_costmap_client = client_h.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
    //Conveyor Move Client
    //Conveyor_cmd_client = client_h.serviceClient<tetraDS_service::conveyor_auto_movement>("Auto_Move_cmd");
    //IMU Service Client//
    euler_angle_reset_cmd_client = client_h.serviceClient<tetraDS_service::euler_angle_reset>("euler_angle_reset_cmd");
    //robot_localization Service Client//
    SetPose_cmd_client = client_h.serviceClient<tetraDS_service::SetPose>("set_pose");

    //cygbot_mark toggle_enabled Service Client//
    toggle_enabled_client = client_h.serviceClient<std_srvs::SetBool>("move_base/local_costmap/cygbot_obstacle_layer/cygbot_mark/toggle_enabled");

    //Infomation_subscriber//
    ros::NodeHandle nInfo;
    ros::Subscriber tetra_battery = nInfo.subscribe<std_msgs::Int32>("tetra_battery", 1, BatteryCallback);
    ros::Subscriber emg_state = nInfo.subscribe<std_msgs::Int32>("emg_state", 1, EMGCallback);
    ros::Subscriber bumper_data = nInfo.subscribe<std_msgs::Int32>("bumper_data", 1, BumperCallback);
    ros::Subscriber docking_status = nInfo.subscribe<std_msgs::Int32>("docking_status", 1, ChargingCallback);
    //Conveyor_Info Subscriber//
    ros::Subscriber loadcell_status = nInfo.subscribe<std_msgs::Float64>("conveyor_loadcell", 1, LoadcellCallback);
    ros::Subscriber sensor_status = nInfo.subscribe<std_msgs::Int32>("conveyor_sensor", 1, SensorCallback);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //bumper_data to Pointcloud2_data///
    ros::NodeHandle nbumper;
    pointcloud_.header.frame_id = tf_prefix_ + "/Front_bumper";
    pointcloud_.width  = 3;
    pointcloud_.height = 1;
    pointcloud_.fields.resize(3);
    // Set x/y/z as the only fields
    pointcloud_.fields[0].name = "x";
    pointcloud_.fields[1].name = "y";
    pointcloud_.fields[2].name = "z";
    int offset = 0; 
    // All offsets are *4, as all field data types are float32
    for (size_t d = 0; d < pointcloud_.fields.size(); ++d, offset += 4)
    {
        pointcloud_.fields[d].count    = 1;
        pointcloud_.fields[d].offset   = offset;
        pointcloud_.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
    }
    pointcloud_.point_step = offset;
    pointcloud_.row_step   = pointcloud_.point_step * pointcloud_.width;

    pointcloud_.data.resize(3 * pointcloud_.point_step);
    pointcloud_.is_bigendian = false;
    pointcloud_.is_dense     = true;

    // y: always 0 for central bumper
    memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[1].offset], &ZERO, sizeof(float));
    // z: constant elevation from base frame
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));
    memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));
    pointcloud_pub_ = nbumper.advertise <sensor_msgs::PointCloud2> ("bumper_pointcloud", 100);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Docking Loop 
    ros::NodeHandle docking_nh;
    docking_progress.data = 0;
    docking_progress_pub = docking_nh.advertise<std_msgs::Int32>("docking_progress", 1);
    //Docking positioning publish ... 240314 mwcha
    positioning_pub = docking_nh.advertise<geometry_msgs::Pose2D>("positioning", 10);
    
    /*Thread Create...*/
    int docking_thread_id, auto_thread_id;
    int a = 1;
    int b = 1;
    docking_thread_id = pthread_create(&p_docking_thread, NULL, DockingThread_function, (void *)&a);
    if (docking_thread_id < 0)
    {
        printf("docking thread create error !!");
        exit(0);
    }
    auto_thread_id = pthread_create(&p_auto_thread, NULL, AutoThread_function, (void *)&b);
    if (auto_thread_id < 0)
    {
        printf("auto thread create error !!");
        exit(0);
    }  

    //TF transform//
    tf::TransformListener listener;
    tf::TransformListener listener2;
	
    ros::Rate loop_rate(30); //30hz

    LED_Toggle_Control(1, 3,100,3,1);
    LED_Turn_On(63);

    int iCheckCnt = 0;
    //Robot IP Check//
    m_strRobotIP = GetWIFI_IPAddress();
    printf("###[Robot IP]: %s ###\n", m_strRobotIP.c_str());
    printf("HOME_strLOCATION: %s \n", _pHomePose.HOME_strLOCATION.c_str());
    printf("HOME_dPOSITION_X: %f \n", _pHomePose.HOME_dPOSITION_X);
    printf("HOME_dPOSITION_Y: %f \n", _pHomePose.HOME_dPOSITION_Y);
    printf("HOME_dPOSITION_Z: %f \n", _pHomePose.HOME_dPOSITION_Z);
    printf("HOME_dQUATERNION_X: %f \n", _pHomePose.HOME_dQUATERNION_X);
    printf("HOME_dQUATERNION_Y: %f \n", _pHomePose.HOME_dQUATERNION_Y);
    printf("HOME_dQUATERNION_Z: %f \n", _pHomePose.HOME_dQUATERNION_Z);
    printf("HOME_dQUATERNION_W: %f \n", _pHomePose.HOME_dQUATERNION_W);
    
    std::string node_name = "/" + tf_prefix_ + "/move_base"; // add move_base Die Check node_name
    std::string node_name2 = "/" + tf_prefix_ + "/cartographer_node"; // add cartographer_node Die Check node_name

    while(ros::ok())
    {
        ros::spinOnce();

        // add move_base Die Check Loop
        nh.getParam("active_map", m_bActive_map_check);
        if(m_bActive_map_check)
        {
            if(checkNode(node_name, node_name2) == true)
            {
                // ROS_ERROR("move_base alive");
            }   
            else
            {
                // ROS_ERROR("move_base die");
            }
        }

        if(_pFlag_Value.m_bFlag_Obstacle_Center || m_iViaPoint_Index <= 1)
        {
            if(!m_flag_Dynamic_Linear_velocity_major_update)
            {
                Dynamic_reconfigure_Teb_Set_DoubleParam("max_vel_x", _pDynamic_param.MAX_Linear_velocity / 2.0);
                m_flag_Dynamic_Linear_velocity_major_update = true;
                m_flag_Dynamic_Linear_velocity_minor_update = false;
                _pFlag_Value.m_bTebMarker_reconfigure_flag = true;
            }
            else
            {
                _pFlag_Value.m_bTebMarker_reconfigure_flag = false;
            }
        }
        else
        {
            if(!_pFlag_Value.m_bTebMarker_reconfigure_flag)
            {
                if(!m_flag_Dynamic_Linear_velocity_minor_update)
                {
                    Dynamic_reconfigure_Teb_Set_DoubleParam("max_vel_x", _pDynamic_param.MAX_Linear_velocity);
                    m_flag_Dynamic_Linear_velocity_minor_update = true;
                    m_flag_Dynamic_Linear_velocity_major_update = false;
                    _pFlag_Value.m_bTebMarker_reconfigure_flag = true;
                }
            }
            else
            {
                _pFlag_Value.m_bTebMarker_reconfigure_flag = false;
            }
        }

        //IMU Reset Loop//
        if(m_iTimer_cnt >= 500) //10 sec_polling
        {
            //costmap clear call//
            clear_costmap_client.call(m_request);

            m_iTimer_cnt = 0;
            Reset_Robot_Pose();
            //ROS_INFO("Reset_Robot_Pose Call !");

            if(_pFlag_Value.m_bflag_auto_patrol)
            {
                ex_iDocking_CommandMode = 200;
            }
        }
        else
        {
            if(_pRobot_Status.m_iCallback_Charging_status == 2 || _pRobot_Status.m_iCallback_Charging_status == 3 || _pRobot_Status.m_iCallback_Charging_status == 6 || _pRobot_Status.m_iCallback_Charging_status == 7)
            {
  
                //todo.. 240315
                LED_Toggle_Control(1, 3,100,3,100); //(int de_index, int light_acc, int High_brightness, int light_dec, int Low_brightness)
                LED_Turn_On(63); //White led
                m_iTimer_cnt ++;
            }
            else
            {
                m_iTimer_cnt = 0;
            }
        }

        //Reset service call check//
        if(_pReset_srv.bflag_reset)
        {
            Reset_Call_service();
            _pReset_srv.bflag_reset = false;
        }

        if(_pRobot_Status.m_iCallback_Charging_status == 1)
        {
            //Get Active map param..//
            nh.getParam("active_map", m_bActive_map_check);
            if(m_bActive_map_check)
            {
                //map to base_footprint TF Pose////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                tf::StampedTransform transform;
                geometry_msgs::TransformStamped ts_msg;
                try
                {
                    listener.waitForTransform("/map", tf_prefix_ + "/base_footprint", ros::Time(0), ros::Duration(5.0)); //3.0
                    listener.lookupTransform("/map", tf_prefix_ + "/base_footprint", ros::Time(0), transform);
                    tf::transformStampedTFToMsg(transform, ts_msg);

                    _pTF_pose.poseTFx = ts_msg.transform.translation.x;
                    _pTF_pose.poseTFy = ts_msg.transform.translation.y;
                    _pTF_pose.poseTFz = ts_msg.transform.translation.z;
                    _pTF_pose.poseTFqx = ts_msg.transform.rotation.x;
                    _pTF_pose.poseTFqy = ts_msg.transform.rotation.y;
                    _pTF_pose.poseTFqz = ts_msg.transform.rotation.z;
                    _pTF_pose.poseTFqw = ts_msg.transform.rotation.w;
                      
                }
                catch (tf::TransformException ex)
                {
                    ROS_ERROR("[TF_Transform_Error(map to base_footprint)]: %s", ex.what());
                    continue;
                }

                //Speed Zone Check Loop //////////////////////////////////////////////////////////////////////////////
                if(m_flag_SpeedZone)
                {
                    POINT m_ptPos = {_pTF_pose.poseTFx, _pTF_pose.poseTFy};
                    for(int i=0; i < _pZoneDataPoint.m_iZone_count; i++)
                    {
                        if(isinner(m_ptPos, vV[i]))
                        {
                            if(!m_flag_major_update[i])
                            {
                                Dynamic_reconfigure_Teb_Set_DoubleParam("max_vel_x", _pTagList[i].dSpeed_value);
                                _pDynamic_param.MAX_Linear_velocity = _pTagList[i].dSpeed_value;
                                //printf("_pTagList[%d].dSpeed_value: %.3f \n", i, _pTagList[i].dSpeed_value);

                                //printf(" Inside Check[%d] !!\n", i);
                                m_flag_major_update[i] = true;
                                m_flag_minor_update[i] = false;
                            }
                        }
                        else
                        {

                            if(!m_flag_minor_update[i])
                            {
                                Dynamic_reconfigure_Teb_Set_DoubleParam("max_vel_x", _pDynamic_param.MAX_Linear_velocity_origin);
                                _pDynamic_param.MAX_Linear_velocity = _pDynamic_param.MAX_Linear_velocity_origin;

                                //printf(" Outside Check[%d] !!\n", i);
                                m_flag_minor_update[i] = true;
                                m_flag_major_update[i] = false;
                            }
                        }
                    }
                }    

                //map to odom TF Pose////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                tf::StampedTransform transform2;
                geometry_msgs::TransformStamped ts_msg2;
                bool bCheck_waitForTransform = false;
                try
                {
                    bCheck_waitForTransform = listener2.waitForTransform("/map", tf_prefix_ + "/odom", ros::Time(0), ros::Duration(5.0));
                    listener2.lookupTransform("/map", tf_prefix_ + "/odom", ros::Time(0), transform2);
                    tf::transformStampedTFToMsg(transform2, ts_msg2);

                    _pTF_pose2.poseTFx2 = ts_msg2.transform.translation.x;
                    _pTF_pose2.poseTFy2 = ts_msg2.transform.translation.y;
                    _pTF_pose2.poseTFz2 = ts_msg2.transform.translation.z;
                    _pTF_pose2.poseTFqx2 = ts_msg2.transform.rotation.x;
                    _pTF_pose2.poseTFqy2 = ts_msg2.transform.rotation.y;
                    _pTF_pose2.poseTFqz2 = ts_msg2.transform.rotation.z;
                    _pTF_pose2.poseTFqw2 = ts_msg2.transform.rotation.w;

                    //itself call client service loop
                    m_dTF_Yaw = Quaternion2Yaw_rad(_pTF_pose2.poseTFqw2, _pTF_pose2.poseTFqx2, _pTF_pose2.poseTFqy2, _pTF_pose2.poseTFqz2);
                    m_dTF_New_Pose_X = (((_pTF_pose2.poseTFx2 * cos(m_dTF_Yaw)) + (_pTF_pose2.poseTFy2 * sin(m_dTF_Yaw))));
                    m_dTF_New_Pose_Y = (((_pTF_pose2.poseTFx2 * -sin(m_dTF_Yaw)) + (_pTF_pose2.poseTFy2 * cos(m_dTF_Yaw))));

                }
                catch (tf::TransformException ex2)
                {
                    ROS_ERROR("[TF_Transform_Error2(map to odom)]: %s", ex2.what());
                    continue;
                }

                if(bCheck_waitForTransform)
                {
                    m_iList_Count = virtual_obstacle.list.size();
                    if(m_iList_Count > 0 || m_bFlag_virtualWallCheck)
                    {
                        if(m_bFlag_nomotion_call || !_pFlag_Value.m_bFlag_nomotion || m_flag_Dynamic_reconfigure_call || m_flag_setgoal || _pFlag_Value.m_bTebMarker_reconfigure_flag)
                        {
                            loop_rate.sleep();
                            continue;
                        }

                        //message copy...
                        virtual_obstacle2.list.clear();
                        virtual_obstacle2.list.resize(m_iList_Count);
                        m_iList_Count2 = virtual_obstacle2.list.size();
                        if(m_iList_Count2 >= 0)
                        {
                            for(int i=0; i<m_iList_Count2; i++)
                            {
                                m_iMode_Count = virtual_obstacle.list[i].form.size();
                                //virtual_obstacle2.list[i].form.clear();
                                virtual_obstacle2.list[i].form.resize(m_iMode_Count);
                                m_iMode_Count2 = virtual_obstacle2.list[i].form.size();
                                if(m_iMode_Count2 > 0)
                                {
                                    for(int j=0; j<m_iMode_Count; j++)
                                    {
                                        virtual_obstacle2.list[i].form[j].x = floor(((((virtual_obstacle.list[i].form[j].x *  cos(m_dTF_Yaw)) + (virtual_obstacle.list[i].form[j].y * sin(m_dTF_Yaw)))) - m_dTF_New_Pose_X)*1000.f + 0.5) /1000.f;
                                        virtual_obstacle2.list[i].form[j].y = floor(((((virtual_obstacle.list[i].form[j].x * -sin(m_dTF_Yaw)) + (virtual_obstacle.list[i].form[j].y  * cos(m_dTF_Yaw)))) - m_dTF_New_Pose_Y)*1000.f + 0.5) /1000.f;
                                        virtual_obstacle2.list[i].form[j].z = floor((virtual_obstacle.list[i].form[j].z)*1000.f + 0.5) /1000.f;
                                    }
                                }
                            }

                            if(m_bFlag_nomotion_call || !_pFlag_Value.m_bFlag_nomotion || m_flag_Dynamic_reconfigure_call || m_flag_setgoal || _pFlag_Value.m_bTebMarker_reconfigure_flag)
                            {
                                loop_rate.sleep();
                                continue;
                            }
                            virtual_obstacle2_pub.publish(virtual_obstacle2);
                        }
			            // 231115 add
                        if(m_iList_Count2 == 0)
                        {
                            m_bFlag_virtualWallCheck = false;
                        }

                	}
                }
                else
                {
                    printf("[Error]listener2.waitForTransform Fail & lookupTransform Fail!! \n");
                }
                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            }
        }

        loop_rate.sleep();
    }

    return 0;
}
