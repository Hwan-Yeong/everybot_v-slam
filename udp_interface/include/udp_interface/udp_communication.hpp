#ifndef UDP_COMMUNICATION_HPP
#define UDP_COMMUNICATION_HPP

#include "udp_interface/libNetwork.h" //#include "libNetwork.h"

#include <fstream>
#include <filesystem>
#include <chrono>
#include <vector>
#include <algorithm>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <dlfcn.h>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <deque>


#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int8.hpp"
#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "everybot_custom_msgs/msg/block_area.hpp"
#include "everybot_custom_msgs/msg/block_area_list.hpp"
#include "everybot_custom_msgs/msg/position.hpp"
#include "everybot_custom_msgs/msg/error_list.hpp"
#include "everybot_custom_msgs/msg/error_list_array.hpp"
#include "everybot_custom_msgs/msg/battery_status.hpp"
#include "everybot_custom_msgs/msg/camera_data.hpp"
#include "everybot_custom_msgs/msg/camera_data_array.hpp"
#include "everybot_custom_msgs/msg/tof_data.hpp"
#include "everybot_custom_msgs/msg/motor_status.hpp"
#include "everybot_custom_msgs/msg/station_data.hpp"
#include "everybot_custom_msgs/msg/bottom_ir_data.hpp"
//testcode
#include "everybot_custom_msgs/msg/move_n_rotation.hpp"

#include "everybot_custom_msgs/msg/rpm_control.hpp"
#include "everybot_custom_msgs/msg/imu_calibration.hpp"

#include "everybot_custom_msgs/msg/robot_state.hpp"
#include "everybot_custom_msgs/msg/navi_state.hpp"
#include "everybot_custom_msgs/msg/ai_temperature.hpp"
#include "everybot_custom_msgs/msg/ap_temperature.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <nlohmann/json.hpp>

#define USE_LINELASER_SENSOR 0

#if USE_LINELASER_SENSOR > 0
#include "everybot_custom_msgs/msg/line_laser_data.hpp"
#include "everybot_custom_msgs/msg/line_laser_data_array.hpp"
#endif

#define ENABLE_FOLLOW true

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29577951308232)
#endif

enum class REQUEST_STATUS
{
    VOID,
    START,
    STOP,
    RUN,
    COMPLETE,
    FAIL,
};

inline std::string enumToString(REQUEST_STATUS in) {
  std::string out;
  switch (in) {
  case REQUEST_STATUS::VOID:
    out = std::string("VOID");
    break;
    case REQUEST_STATUS::START:
    out = std::string("START");
    break;    
  case REQUEST_STATUS::STOP:
    out = std::string("STOP");
    break;
  case REQUEST_STATUS::RUN:
    out = std::string("RUN");
    break;
  case REQUEST_STATUS::COMPLETE:
    out = std::string("COMPLETE");
    break;
  case REQUEST_STATUS::FAIL:
    out = std::string("FAIL");
    break;
  default:
    out = std::string("REQUEST_STATUS-UNKOWN");
    break;          
  }
  return out;
};

enum class REQUEST_SOC_CMD {
  VOID,
  START_AUTO_MAPPING,   // SOC 명령
  START_MANUAL_MAPPING, // SOC 명령
  START_NAVIGATION,     // SOC 명령
  START_RETURN_CHARGER, // SOC 명령
  START_DOCKING,        // SOC 명령
  START_CHARGING,       // SOC 명령
  PAUSE_WORKING,     // SOC 명령
  RESUME_WORKING,    // SOC 명령
  START_FACTORY_NAVIGATION, // SOC명령
  STOP_WORKING, //  SOC명령
  #if ENABLE_FOLLOW
  START_FOLLOWING
  #endif
};

inline std::string enumToString(REQUEST_SOC_CMD in) {
  std::string out;
  switch (in) {
  case REQUEST_SOC_CMD::VOID:
    out = std::string("VOID");
    break;
    case REQUEST_SOC_CMD::START_AUTO_MAPPING:
    out = std::string("START_AUTO_MAPPING");
    break;    
  case REQUEST_SOC_CMD::START_MANUAL_MAPPING:
    out = std::string("START_MANUAL_MAPPING");
    break;
  case REQUEST_SOC_CMD::START_NAVIGATION:
    out = std::string("START_NAVIGATION");
    break;
  case REQUEST_SOC_CMD::START_RETURN_CHARGER:
    out = std::string("START_RETURN_CHARGER");
    break;
  case REQUEST_SOC_CMD::START_DOCKING:
    out = std::string("START_DOCKING");
    break;
  case REQUEST_SOC_CMD::START_CHARGING:
    out = std::string("START_CHARGING");
    break; 
  case REQUEST_SOC_CMD::PAUSE_WORKING:
    out = std::string("PAUSE_WORKING");
    break;       
  case REQUEST_SOC_CMD::RESUME_WORKING:
    out = std::string("RESUME_WORKING");
    break;
  case REQUEST_SOC_CMD::START_FACTORY_NAVIGATION:
    out = std::string("START_FACTORY_NAVIGATION");
    break;
  case REQUEST_SOC_CMD::STOP_WORKING:
    out = std::string("STOP_WORKING");
    break;
  case REQUEST_SOC_CMD::START_FOLLOWING:
    out = std::string("START_FOLLOWING");
    break;
  default:
    out = std::string("REQUEST_SOC_CMD-UNKOWN");
    break;           
  }
  return out;
};

enum class ROBOT_STATE:int{
  IDLE = 0,
  AUTO_MAPPING,
  MANUAL_MAPPING,
  NAVIGATION,
  RETURN_CHARGER,
  DOCKING,
  UNDOCKING,
  ONSTATION,
  FACTORY_NAVIGATION,
  ERROR,
  #if ENABLE_FOLLOW
  FOLLOWING
  #endif
};

inline std::string enumToString(ROBOT_STATE in) {
  std::string out;
  switch (in) {
  case ROBOT_STATE::IDLE:
    out = std::string("IDLE");
    break;
    case ROBOT_STATE::AUTO_MAPPING:
    out = std::string("AUTO_MAPPING");
    break;    
  case ROBOT_STATE::MANUAL_MAPPING:
    out = std::string("MANUAL_MAPPING");
    break;
  case ROBOT_STATE::NAVIGATION:
    out = std::string("NAVIGATION");
    break;
  case ROBOT_STATE::RETURN_CHARGER:
    out = std::string("RETURN_CHARGER");
    break;
  case ROBOT_STATE::DOCKING:
    out = std::string("DOCKING");
    break;
  case ROBOT_STATE::UNDOCKING:
    out = std::string("UNDOCKING");
    break;
  case ROBOT_STATE::ONSTATION:
    out = std::string("ONSTATION");
    break; 
  case ROBOT_STATE::FACTORY_NAVIGATION:
    out = std::string("FACTORY_NAVIGATION");
    break;       
  case ROBOT_STATE::ERROR:
    out = std::string("ERROR");
    break;
  case ROBOT_STATE::FOLLOWING:
    out = std::string("FOLLOWING");
    break;  
  default:
    out = std::string("ROBOT_STATE-UNKNOWN");
    break;           
  }
  return out;
};

enum class ROBOT_STATUS : int{
  VOID = 0,
  READY,
  START,
  PAUSE,
  RESUME,
  COMPLETE,
  FAIL,
};

inline std::string enumToString(ROBOT_STATUS in) {
  std::string out;
  switch (in) {
  case ROBOT_STATUS::VOID:
    out = std::string("VOID");
    break;
    case ROBOT_STATUS::READY:
    out = std::string("READY");
    break;    
  case ROBOT_STATUS::START:
    out = std::string("START");
    break;
  case ROBOT_STATUS::PAUSE:
    out = std::string("PAUSE");
    break;
  case ROBOT_STATUS::RESUME:
    out = std::string("RESUME");
    break;
  case ROBOT_STATUS::COMPLETE:
    out = std::string("COMPLETE");
    break;
  case ROBOT_STATUS::FAIL:
    out = std::string("FAIL");
    break; 
  default:
    out = std::string("ROBOT_STATUS-UNKNOWN");
    break;           
  }
  return out;
};

enum class NODE_STATUS
{
    IDLE,
    AUTO_MAPPING,
    MANUAL_MAPPING,
    NAVI,
    FT_NAVI,
    #if ENABLE_FOLLOW
    FOLLOWING,
    #endif
};

inline std::string enumToString(NODE_STATUS in) {
  std::string out;
  switch (in) {
  case NODE_STATUS::IDLE:
    out = std::string("IDLE");
    break;
    case NODE_STATUS::AUTO_MAPPING:
    out = std::string("AUTO_MAPPING");
    break;    
  case NODE_STATUS::MANUAL_MAPPING:
    out = std::string("MANUAL_MAPPING");
    break;
  case NODE_STATUS::NAVI:
    out = std::string("NAVI");
    break;
  case NODE_STATUS::FT_NAVI:
    out = std::string("FT_NAVI");
    break;
  case NODE_STATUS::FOLLOWING:
    out = std::string("FOLLOWING");
    break;
  default:
    out = std::string("NODE_STATUS-UNKNOWN");
    break;         
  }
  return out;
};

enum class NAVI_STATE
{
    IDLE = 0,
    MOVE_GOAL,
    ARRIVED_GOAL,
    PAUSE,
    FAIL,
    START_ROTAION,
    ROTATION_COMPLETE,
    READY,
    ALTERNATE_GOAL,
};

inline std::string enumToString(NAVI_STATE in) {
  std::string out;
  switch (in) {
  case NAVI_STATE::IDLE:
    out = std::string("IDLE");
    break;
    case NAVI_STATE::MOVE_GOAL:
    out = std::string("MOVE_GOAL");
    break;    
  case NAVI_STATE::ARRIVED_GOAL:
    out = std::string("ARRIVED_GOAL");
    break;
  case NAVI_STATE::PAUSE:
    out = std::string("PAUSE");
    break;
  case NAVI_STATE::FAIL:
    out = std::string("FAIL");
    break;
  case NAVI_STATE::START_ROTAION:
    out = std::string("START_ROTAION");
    break;
  case NAVI_STATE::ROTATION_COMPLETE:
    out = std::string("ROTATION_COMPLETE");
    break; 
  case NAVI_STATE::READY:
    out = std::string("READY");
    break;       
  case NAVI_STATE::ALTERNATE_GOAL:
    out = std::string("ALTERNATE_GOAL");
    break;
  default:
    out = std::string("NAVI_STATE-UNKWON");
    break;           
  }
  return out;
};

enum class NAVI_FAIL_REASON
{
    VOID,
    NODE_OFF,
    SERVER_NO_ACTION,
    GOAL_ABORT,
    GOAL_REJECT,
    UNKWON,
};

inline std::string enumToString(NAVI_FAIL_REASON in) {
  std::string out;
  switch (in) {
  case NAVI_FAIL_REASON::VOID:
    out = std::string("VOID");
    break;
    case NAVI_FAIL_REASON::NODE_OFF:
    out = std::string("NODE_OFF");
    break;    
  case NAVI_FAIL_REASON::SERVER_NO_ACTION:
    out = std::string("SERVER_NO_ACTION");
    break;
  case NAVI_FAIL_REASON::GOAL_ABORT:
    out = std::string("GOAL_ABORT");
    break;
  case NAVI_FAIL_REASON::GOAL_REJECT:
    out = std::string("GOAL_REJECT");
    break;
  case NAVI_FAIL_REASON::UNKWON:
    out = std::string("UNKWON");
    break;
  default:
    out = std::string("NAVI_FAIL_REASON-NOT VALID");
    break;           
  }
  return out;
};

enum class UDP_COMMUNICATION
{
    NORMAL,
    AP_JIG_MODE,
    AMR_JIG_MODE,
};

inline std::string enumToString(UDP_COMMUNICATION in) {
  std::string out;
  switch (in) {
  case UDP_COMMUNICATION::NORMAL:
    out = std::string("CHECK_NODE");
    break;
    case UDP_COMMUNICATION::AP_JIG_MODE:
    out = std::string("AP_JIG_MODE");
    break;    
  case UDP_COMMUNICATION::AMR_JIG_MODE:
    out = std::string("AMR_JIG_MODE");
    break;
  default:
    out = std::string("UDP_COMMUNICATION-UNKNOWN");
    break;           
  }
  return out;
};

enum JIG_HEADER
{
    AMR_MODE = 0x00,
    WHEEL_MOTOR = 0x01,
    BATTERY = 0x02,
    CLIFF_LIFT = 0x03,
    DOCK_RECEIVER = 0x04,
    TOF = 0x05,
    FRONT_LIDAR = 0x06,
    REAR_LIDAR = 0x07,
    IMU_CALIBRATION = 0x08,
    SW_FW_VERSION = 0x09,
    BATTERY_SHIPPING_MODE = 0x0A,
    TOF_LEFT_OFFSET_SETUP = 0x0B,
    TOF_RIGHT_OFFSET_SETUP = 0x0C,
    CMD_END
};

inline std::string enumToString(JIG_HEADER in) {
  std::string out;
  switch (in) {
  case JIG_HEADER::AMR_MODE:
    out = std::string("AMR_MODE 0x00");
    break;
  case JIG_HEADER::WHEEL_MOTOR:
    out = std::string("WHEEL_MOTOR 0x01");
    break;    
  case JIG_HEADER::BATTERY:
    out = std::string("BATTERY 0x02");
    break;
  case JIG_HEADER::CLIFF_LIFT:
    out = std::string("CLIFF_LIFT 0x03");
    break;
  case JIG_HEADER::DOCK_RECEIVER:
    out = std::string("DOCK_RECEIVER 0x04");
    break;
  case JIG_HEADER::TOF:
    out = std::string("TOF 0x05");
    break;
  case JIG_HEADER::FRONT_LIDAR:
    out = std::string("FRONT_LIDAR 0x06");
    break; 
  case JIG_HEADER::REAR_LIDAR:
    out = std::string("REAR_LIDAR 0x07");
    break;       
  case JIG_HEADER::IMU_CALIBRATION:
    out = std::string("IMU_CALIBRATION 0x08");
    break;
  case JIG_HEADER::SW_FW_VERSION:
    out = std::string("SW_FW_VERSION 0x09");
    break; 
  case JIG_HEADER::BATTERY_SHIPPING_MODE:
    out = std::string("BATTERY_SHIPPING_MODE 0x0A");
    break;
  case JIG_HEADER::TOF_LEFT_OFFSET_SETUP:
    out = std::string("TOF_LEFT_OFFSET_SETUP 0x0B");
    break;
  case JIG_HEADER::TOF_RIGHT_OFFSET_SETUP:
    out = std::string("TOF_RIGHT_OFFSET_SETUP 0x0C");
    break;
  case JIG_HEADER::CMD_END:
    out = std::string("CMD_END 0x0D");
    break;
  default:
    out = std::string("-JIG_HEADER-UNKNOWN");
    break;           
  }
  return out;
};

enum JIG_AP_HEADER
{
    AP_JIG = 0xF0,
    AP_JIG_RAM_MEMORY = 0xF1,
    AP_JIG_DISK_MEMORY = 0xF2,
    AP_JIG_FRONT_LIDAR = 0xF3,
    AP_JIG_BACK_LIDAR = 0xF4,
    AP_JIG_VERSION = 0xF5,
    AP_CMD_END
};

inline std::string enumToString(JIG_AP_HEADER in) {
  std::string out;
  switch (in) {
  case JIG_AP_HEADER::AP_JIG:
    out = std::string("AP_JIG 0xF0");
    break;
    case JIG_AP_HEADER::AP_JIG_RAM_MEMORY:
    out = std::string("AP_JIG_RAM_MEMORY 0xF1");
    break;    
  case JIG_AP_HEADER::AP_JIG_DISK_MEMORY:
    out = std::string("AP_JIG_DISK_MEMORY 0xF2");
    break;
  case JIG_AP_HEADER::AP_JIG_FRONT_LIDAR:
    out = std::string("AP_JIG_FRONT_LIDAR 0xF3");
    break;
  case JIG_AP_HEADER::AP_JIG_BACK_LIDAR:
    out = std::string("AP_JIG_BACK_LIDAR 0xF4");
    break;
  case JIG_AP_HEADER::AP_JIG_VERSION:
    out = std::string("AP_JIG_VERSION 0xF5");
    break;  
  case JIG_AP_HEADER::AP_CMD_END:
    out = std::string("AP_CMD_END 0xF6");
    break;
  default: 
    out = std::string("JIG_AP_HEADER-UNKNOWN");
    break;           
  }
  return out;
};

enum class JIG_DATA_KEY
{
    MODE,
    WHEEL_MOTOR,
    BATTERY,
    CLIFF_LIFT,
    DOCK_RECEIVER,
    TOF,
    FRONT_LIDAR,
    REAR_LIDAR,
    IMU_CALIBRATION,
    SW_FW_VERSION,
    BATTERY_SHIPPING_MODE,
    TOF_LEFT_OFFSET_SETUP,
    TOF_RIGHT_OFFSET_SETUP
};

inline std::string enumToString(JIG_DATA_KEY in) {
  std::string out;
  switch (in) {
  case JIG_DATA_KEY::MODE:
    out = std::string("MODE");
    break;
    case JIG_DATA_KEY::WHEEL_MOTOR:
    out = std::string("WHEEL_MOTOR");
    break;    
  case JIG_DATA_KEY::BATTERY:
    out = std::string("BATTERY");
    break;
  case JIG_DATA_KEY::CLIFF_LIFT:
    out = std::string("CLIFF_LIFT");
    break;
  case JIG_DATA_KEY::DOCK_RECEIVER:
    out = std::string("DOCK_RECEIVER");
    break;
  case JIG_DATA_KEY::TOF:
    out = std::string("TOF");
    break;
  case JIG_DATA_KEY::FRONT_LIDAR:
    out = std::string("FRONT_LIDAR");
    break; 
  case JIG_DATA_KEY::REAR_LIDAR:
    out = std::string("REAR_LIDAR");
    break;       
  case JIG_DATA_KEY::IMU_CALIBRATION:
    out = std::string("IMU_CALIBRATION");
    break;
  case JIG_DATA_KEY::SW_FW_VERSION:
    out = std::string("SW_FW_VERSION");
    break;
  case JIG_DATA_KEY::BATTERY_SHIPPING_MODE:
    out = std::string("BATTERY_SHIPPING_MODE");
    break;
  case JIG_DATA_KEY::TOF_LEFT_OFFSET_SETUP:
    out = std::string("TOF_LEFT_OFFSET_SETUP");
    break;
  case JIG_DATA_KEY::TOF_RIGHT_OFFSET_SETUP:
    out = std::string("TOF_RIGHT_OFFSET_SETUP");
    break;
  default:
    out = std::string("JIG_DATA_KEY-UNKNOWN");
    break;               
  }
  return out;
};

enum ROTATION_COMMAND
{
    ROTATE_RELATIVE,
    ROTATE_ABSOLUTE,
    ROTATE_360_CCW,
    ROTATE_360_CW,
};

inline std::string enumToString(ROTATION_COMMAND in) {
  std::string out;
  switch (in) {
  case ROTATION_COMMAND::ROTATE_RELATIVE:
    out = std::string("ROTATE_RELATIVE");
    break;
    case ROTATION_COMMAND::ROTATE_ABSOLUTE:
    out = std::string("ROTATE_ABSOLUTE");
    break;    
  case ROTATION_COMMAND::ROTATE_360_CCW:
    out = std::string("ROTATE_360_CCW");
    break;
  case ROTATION_COMMAND::ROTATE_360_CW:
    out = std::string("ROTATE_360_CW");
    break;
  default:
    out = std::string("ROTATION_COMMAND-UNKNOWN");
    break;          
  }
  return out;
};

struct pose
{
    bool valid;
    double x;
    double y;
    double theta;
    double timestamp;

    pose() : valid(false),x(0.0), y(0.0),theta(0.0), timestamp(0.0) {}
};

struct MapInfo
{
    bool bReceived;
    double resolution; 
    uint32_t width;
    uint32_t height;
    double origin_x;
    double origin_y;
    std::vector<uint8_t> map_data;
};

struct LidarSensorInfo
{
    int front_distance;
    int rear_distance;
};

struct motorData
{
    uint8_t type; // 0x01 : type-C , 0x02 : type-A
    uint8_t status;
    int32_t encoder;
    int16_t rpm;
    int16_t current; //mA
    int8_t tempterature;
};

struct MotorInfo
{
    uint8_t mode;
    motorData left;
    motorData right;
};

struct cameraData
{
    uint8_t class_id;
    uint8_t confidence;
    pose position;
    double width;
    double height;
    double distance;
};

struct CarmeraSensorInfo
{
    uint8_t num;

    std::vector<ObjectDataV2> data;
    #if 0
    std::vector<cameraData> data;
    #endif
};

struct lineLaserData
{
    pose position;
    uint8_t direction;
    double height;
    double distance;
};

struct LineLaserInfo
{
    uint8_t num;
    std::vector<LLDataV2> data;
    #if 0
    std::vector<lineLaserData> data;
    #endif
    
};

enum CALIB_TOF_DATA_IDX {
  SIDE,
  IDX13,
  IDX14,
  IDX15,
  MIN,
  MIN_REF,
  MAX,
  MAX_REF,
  MEDIAN,
  RESULT,
  ARRAY_SIZE,
};
namespace TOF_CALIB_SIDE {
  inline const float LEFT = 0.0;
  inline const float RIGHT = 1.0;
}
namespace TOF_CALIB_RESULT_JIG {
  inline const uint8_t PASS = 1;
  inline const uint8_t FAIL = 10;
  inline const uint8_t FAIL_OUT_OF_RANGE = 11;
  inline const uint8_t FAIL_UNSTABLE_RANGE = 12;
  inline const uint8_t FAIL_NON_RENEWAL = 13;
}

namespace TOF_CALIB_RESULT_ORIGIN {
  inline const uint8_t PASS = 0x02;
  inline const uint8_t FAIL_OUT_OF_RANGE = 0x03;
  inline const uint8_t FAIL_UNSTABLE_RANGE = 0x04;
  inline const uint8_t FAIL_NON_RENEWAL = 0x08;
}
struct Mtof_Calib {
  Mtof_Calib() {
    left.raw.clear();
    right.raw.clear();
    left.result = 0;
    right.result = 0;
  }
  uint8_t getLeftResult (float _result) {
    return (static_cast<uint8_t>(_result));
  }
  uint8_t getRightResult (float _result) {
    return (static_cast<uint8_t>(_result));
  }
  int convertCalibResultToJigFormat(uint8_t _result) {
    switch (_result) {
      case TOF_CALIB_RESULT_ORIGIN::PASS : return TOF_CALIB_RESULT_JIG::PASS;
      case TOF_CALIB_RESULT_ORIGIN::FAIL_OUT_OF_RANGE : return TOF_CALIB_RESULT_JIG::FAIL_OUT_OF_RANGE;
      case TOF_CALIB_RESULT_ORIGIN::FAIL_UNSTABLE_RANGE : return TOF_CALIB_RESULT_JIG::FAIL_UNSTABLE_RANGE;
      case TOF_CALIB_RESULT_ORIGIN::FAIL_NON_RENEWAL : return TOF_CALIB_RESULT_JIG::FAIL_NON_RENEWAL;
      default : return TOF_CALIB_RESULT_JIG::FAIL;
    }
  }
  bool isLeftSide(float _side) {
    return ((fabs(_side - TOF_CALIB_SIDE::LEFT)) < 1e-6);
  }
  bool isRightSide(float _side) {
    return ((fabs(_side - TOF_CALIB_SIDE::RIGHT)) < 1e-6);
  }
  void setCalibData(std::vector<float> &_data) {
    if (isLeftSide(_data[CALIB_TOF_DATA_IDX::SIDE])) {
      setLeftCalibData(_data);
    } else if (isRightSide(_data[CALIB_TOF_DATA_IDX::SIDE])) {
      setRightCalibData(_data);
    }
  }
  Aggregate makeAggregate(int _idx, float _min, float _minRef, float _max, float _maxRef, float _median) {
    return Aggregate{
      _idx,
      {
        {"min", _min},
        {"minRef", _minRef},
        {"max", _max},
        {"maxRef", _maxRef},
        {"median", _median}
      }
    };
  }
  void setLeftCalibData(std::vector<float> &_data) {
    left.raw = {{57, _data[CALIB_TOF_DATA_IDX::IDX13]}, {60, _data[CALIB_TOF_DATA_IDX::IDX14]}, {63, _data[CALIB_TOF_DATA_IDX::IDX15]}};
    Aggregate agg14 = makeAggregate(14, _data[CALIB_TOF_DATA_IDX::MIN], _data[CALIB_TOF_DATA_IDX::MIN_REF], _data[CALIB_TOF_DATA_IDX::MAX], _data[CALIB_TOF_DATA_IDX::MAX_REF], _data[CALIB_TOF_DATA_IDX::MEDIAN]);
    left.aggregates = {agg14};
    left.result = convertCalibResultToJigFormat(getLeftResult(_data[CALIB_TOF_DATA_IDX::RESULT]));
  }
  void setRightCalibData(std::vector<float> &_data) {
    right.raw = {{56, _data[CALIB_TOF_DATA_IDX::IDX13]}, {59, _data[CALIB_TOF_DATA_IDX::IDX14]}, {62, _data[CALIB_TOF_DATA_IDX::IDX15]}};
    Aggregate agg14 = makeAggregate(14, _data[CALIB_TOF_DATA_IDX::MIN], _data[CALIB_TOF_DATA_IDX::MIN_REF], _data[CALIB_TOF_DATA_IDX::MAX], _data[CALIB_TOF_DATA_IDX::MAX_REF], _data[CALIB_TOF_DATA_IDX::MEDIAN]);
    right.aggregates = {agg14};
    right.result = convertCalibResultToJigFormat(getLeftResult(_data[CALIB_TOF_DATA_IDX::RESULT]));
  }

  SideData left;
  SideData right;

};
struct TofData
{
    uint8_t status;
    double distance;
};


struct multiTofData //4x4 array
{
    uint8_t status;
    std::vector<int> data;
    multiTofData() : status(0), data(64) {}
};

struct TofSensorInfo
{
    TofData top;
    multiTofData left_bottom;
    multiTofData right_bottom;
};


union CliffLiftInfo {
    struct bits{
        uint8_t cliff_front_center : 1;
        uint8_t cliff_front_left : 1;
        uint8_t cliff_back_left : 1;
        uint8_t cliff_back_center : 1;
        uint8_t cliff_back_right : 1;
        uint8_t cliff_front_right : 1;
        uint8_t wheel_lift_left : 1;
        uint8_t wheel_lift_right : 1;
    }b;
    uint8_t value;

};

struct BottomIrADC
{
    uint16_t front_center;
    uint16_t front_left;
    uint16_t back_left;
    uint16_t back_center;
    uint16_t back_right;
    uint16_t front_right;
};

struct ImuData
{
  // 자세 (라디안)
  double roll;
  double pitch;
  double yaw;

  // 가속도 (m/s²)
  double ax;
  double ay;
  double az;
};

union SigReceiver {
    struct bits{
        uint8_t reserved : 4; //no use lower 4 bit
        uint8_t side_left : 1;//0x10
        uint8_t center_left : 1;//0x20  
        uint8_t center_right : 1;//0x40
        uint8_t side_right : 1;//0x80
    }b;
    uint8_t value;
};

struct DockingInfo
{
    uint8_t status;
    SigReceiver receiver;
};

struct BatteryInfo
{
    uint8_t status;
    int16_t cell_voltage1; // Cell Voltage 1-5 (mV) (p)
    int16_t cell_voltage2; // Cell Voltage 1-5 (mV) (p)
    int16_t cell_voltage3; // Cell Voltage 1-5 (mV) (p)
    int16_t cell_voltage4; // Cell Voltage 1-5 (mV) (p)
    int16_t cell_voltage5; // Cell Voltage 1-5 (mV) (p)

    int16_t total_capacity;     // Total Capacity (mAh) (p)
    int16_t remaining_capacity; // Remaining Capacity (mAh) (p)
    uint8_t manufacturer;       // Battery Manufacturer: 0x00, 0x01, or 0x02 (p)

    uint8_t percent;          // Battery Percentage (0-100%) (p)
    double voltage;          // Battery Voltage (mV) (p)
    double current;          // Battery Current (+charge, -discharge, 1mA) (p)
    uint8_t temperature1;     // Battery Temperature (°C) (Sensor 1 and 2) (p)
    uint8_t temperature2;     // Battery Temperature (°C) (Sensor 1 and 2) (p)
    int16_t design_capacity;  // Design Capacity (mAh) (p)
    int16_t number_of_cycles; // Number of Cycles (uint16_t) (p)

    uint8_t charge_status; // Charge Status (p)
    int16_t charging_mode; // Charging Mode (p)
    uint8_t shipping_mode;
    uint8_t precharge_state;
    uint8_t charge_mode;
    uint8_t fet_state;
};

struct SoftWareVersion
{
    bool bSet;
    double timestamp;
    std::string total_ver;
    std::string mcu_ver;
    std::string ai_ver;
    std::string sw_ver;
};


struct RobotSpeed
{
    double v;
    double w;
};

struct SocData
{
    pose robotPosition;
    LidarSensorInfo lidarInfo;
    MotorInfo motorInfo;
    CarmeraSensorInfo cameraInfo;
    LineLaserInfo lineLaserInfo;
    TofSensorInfo tofInfo;
    CliffLiftInfo cliffLiftInfo;
    BottomIrADC bottomIrData;
    ImuData imuData;
    DockingInfo dockingInfo;
    BatteryInfo battInfo;
    SoftWareVersion version;
    pose targetPosition;
    MapInfo mapInfo;
    RobotSpeed velocity;
    uint8_t robotStatus;
    uint8_t actionStatus;
    std::string notification;
    uint8_t lineLaserCalib;
    uint8_t movingStatus;
    std::vector<ErrorList_t> errorList;
    uint8_t camera_type;
};

struct rotationData
{
    bool progress;
    int type;
    double target;
    double accAngle;
    double preTheta;
};

struct mainState
{
  uint8_t robotStatus;
  uint8_t actionStatus;
};

struct movingState
{
  uint8_t movingStatus;
  uint8_t failReason;
};


using namespace std::chrono_literals;

class UdpCommunication : public rclcpp::Node
{
private :
    std::mutex map_mutex_;
    // std::mutex robot_pose_mutex_;
    std::mutex scan_front_mutex_;
    std::mutex scan_back_mutex_;

    // std::mutex battery_mutex_;
    // std::mutex wheel_motor_mutex_;
    // std::mutex tof_mutex_;

    #if USE_LINELASER_SENSOR > 0
    std::mutex line_laser_mutex_;
    #endif
    // std::mutex imu_cal_mutex_;
    // std::mutex imu_mutex_;
    // std::mutex bottom_status_mutex_;
    // std::mutex bottom_ir_mutex_;
    // std::mutex charger_data_mutex_;
    // std::mutex camera_mutex_;
    std::mutex soc_error_list_mutex_;
    std::mutex main_state_queue_mutex_;
    std::mutex moving_state_queue_mutex_;
    //subscriber
    //mcu_sub
    rclcpp::Subscription<everybot_custom_msgs::msg::TofData>::SharedPtr tof_status_sub;
    rclcpp::Subscription<everybot_custom_msgs::msg::BatteryStatus>::SharedPtr battery_status_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<everybot_custom_msgs::msg::MotorStatus>::SharedPtr motor_status_sub;
    rclcpp::Subscription<everybot_custom_msgs::msg::MotorStatus>::SharedPtr motor_data_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr bottom_status_sub_;
    rclcpp::Subscription<everybot_custom_msgs::msg::BottomIrData>::SharedPtr bottom_ir_sub_;
    rclcpp::Subscription<everybot_custom_msgs::msg::StationData>::SharedPtr station_data_sub_;
    rclcpp::Subscription<everybot_custom_msgs::msg::ImuCalibration>::SharedPtr imu_calib_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fw_version_sub;
    
    //ai_sub
    #if USE_LINELASER_SENSOR > 0
    rclcpp::Subscription<everybot_custom_msgs::msg::LineLaserDataArray>::SharedPtr line_laser_sub_;
    #endif
    rclcpp::Subscription<everybot_custom_msgs::msg::CameraDataArray>::SharedPtr camera_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ai_version_sub;
    rclcpp::Subscription<everybot_custom_msgs::msg::AiTemperature>::SharedPtr aitemperature_data_sub;

    //ap_sub
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_front_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_back_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_pose_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<everybot_custom_msgs::msg::ErrorListArray>::SharedPtr error_list_sub_;
    //state_sub
    rclcpp::Subscription<everybot_custom_msgs::msg::RobotState>::SharedPtr req_state_sub;
    rclcpp::Subscription<everybot_custom_msgs::msg::NaviState>::SharedPtr req_navi_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr node_status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_speed_sub_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr tof_calibData_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr recovery_local_sub_;

    //publisher
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr inspection_mode_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr manual_move_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr dock_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr charge_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lidar_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tof_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_cmd_pub_;
    #if USE_LINELASER_SENSOR > 0
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr linelaser_cmd_pub_;
    #endif
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr batterySleep_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr factory_mode_pub_;

    rclcpp::Publisher<everybot_custom_msgs::msg::Position>::SharedPtr move_target_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr move_charger_pub_;
    rclcpp::Publisher<everybot_custom_msgs::msg::MoveNRotation>::SharedPtr move_rotation_pub_;
    rclcpp::Publisher<everybot_custom_msgs::msg::MoveNRotation>::SharedPtr rotation_pub_;
    rclcpp::Publisher<everybot_custom_msgs::msg::BlockAreaList>::SharedPtr block_area_pub_;
    rclcpp::Publisher<everybot_custom_msgs::msg::BlockAreaList>::SharedPtr block_wall_pub_;     
	  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr req_version_pub_;
    rclcpp::Publisher<everybot_custom_msgs::msg::RpmControl>::SharedPtr jig_request_motor_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr jig_request_battery_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr jig_request_imu_pub_;
    rclcpp::Publisher<everybot_custom_msgs::msg::RobotState>::SharedPtr req_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr station_pose_pub_;
    
    //rclcpp::Publisher<everybot_custom_msgs::msg::MotorRpm>::SharedPtr cmd_rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr req_soc_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sw_version_pub_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr fw_reset_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr ai_reset_pub_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr req_camera_type_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr start_tofcalib_pub_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr map_copy_pub_;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr camera_type_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr reset_odom_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr odom_status_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr tof_calibState_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr save_map_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr calib_complete_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr maneuver_state_str_sub_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr reboot_ready_complete_sub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr ready_reboot_pub_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr mapinfo_changed_pub_;
    rclcpp::Publisher<everybot_custom_msgs::msg::ApTemperature>::SharedPtr ap_temperature_pub_;

    std::thread udp_communication_thread_;
    //timer
    //rclcpp::TimerBase::SharedPtr udpTimer_;
    rclcpp::TimerBase::SharedPtr autoPublishTimer;
    rclcpp::TimerBase::SharedPtr linear_target_timer_;
    rclcpp::TimerBase::SharedPtr tof_calib_timer_;
    rclcpp::TimerBase::SharedPtr selftest_calib_timer_;

    //udp protocol data
    Mapping_t ReqMapping;
    Mapping_t& reqMapping = ReqMapping;
    Driving_t ReqNavigation;
    Driving_t& reqNavigation = ReqNavigation;
    TargetPosition_t ReqTargetPosition;
    TargetPosition_t& reqTargetPosition = ReqTargetPosition;
    TargetPositionType_t ReqMovenRotation;
    TargetPositionType_t& reqMovenRotation = ReqMovenRotation;
    DockingStatus_t ReqDocking;
    DockingStatus_t& reqDocking = ReqDocking;
    MotorManual_VW_t ReqManualVWMove;
    MotorManual_VW_t& reqManualVWMove = ReqManualVWMove;
    MotorManual_RPM_t ReqManualRpmMove;
    MotorManual_RPM_t& reqManualRpmMove = ReqManualRpmMove;
    ModifiedMapDataB_t modifiedMap;
    ExcelSteps_t ReqExcelator;
    ExcelSteps_t &reqExcelator = ReqExcelator;
    TargetPosition_t ReqCalculateTargetPose;
    Rotation_t ReqRotation;
    Rotation_t& reqRotation = ReqRotation;

    FactoryMode_t ReqFactoryMode;
    FactoryMode_t& reqFactoryMode = ReqFactoryMode;
    BatterySleepMode_t ReqBatterySleep;
    BatterySleepMode_t& reqBatterySleep = ReqBatterySleep;
    InspectionMode_t ReqInspectionMode;
    InspectionMode_t &reqInspectionMode = ReqInspectionMode;
    StationRepositioning_t ReqStationPose;
    StationRepositioning_t& reqStationPose = ReqStationPose;

    SelfDiagnosisMotor_t ReqSelfDiagnosisMotor;
    SelfDiagnosisMotor_t& reqSelfDiagnosisMotor = ReqSelfDiagnosisMotor;

    FollowMe_t ReqFollowMe;
    FollowMe_t& reqFollowMe = ReqFollowMe;


    bool bInspectionMode;
    NAVI_FAIL_REASON movefail_reason;
    NODE_STATUS nodeState;

    SocData socData; //soc
    std::unordered_map<JIG_DATA_KEY,std::vector<uint8_t>> jigData; //jig
    sensor_msgs::msg::LaserScan::SharedPtr apJigFrontLaserData;
    sensor_msgs::msg::LaserScan::SharedPtr apJigBackLaserData;

    Mtof_Calib mtof_calib_data_;
    bool left_calib_data_set_done_;
    bool right_calib_data_set_done_;
    bool mtof_calib_file_generated_;
    bool is_copy_jig_calibration_file_;
    unsigned int selftest_calib_timer_reset_cnt_;

    UDP_COMMUNICATION communicationMode;
    rotationData rotation;
    pose base_odom;
    pose odom;
    bool end_linear_target;
    std::size_t temp_mapsize;

    double inspection_motor_Velocity;
    double inspection_motor_Distance;
    double acceleration;
    bool bInspectionMotor;
    bool bSetBaseOdom;
    bool bFactoryNaviMode;

    bool bRequestFrontLidarDist;
    bool bRequestBackLidarDist;

    Temperature_t aiTemp;
    int leftTerminalTemp;
    int rightTerminalTemp;

    bool m_is_starting_comm_SoC;

    pose m_prev_robot_pose;
    pose m_prev_odom;
    uint8_t odom_status;
    //std::chrono::time_point<std::chrono::steady_clock> reset_odom_start_time_;

    std::string prev_fw_version;
    std::string prev_ai_version;
    bool bOdomResetDone;
    bool bReceiveOdom;
    bool bReceiveOdomStatus;

    double temp_ap_soc = 0;
    double temp_ap_bigcore0 = 0;
    double temp_ap_bigcore1 = 0;
    double temp_ap_littlecore = 0;
    double temp_ap_center = 0;
    double temp_ap_gpu = 0;
    double temp_ap_npu = 0;

    int fan_control_power_auto_mapping;
    int fan_control_power_battery_overheat;
    int fan_control_power_ap_overheat;

    double fan_control_ap_overheat_on_th;
    double fan_control_ap_overheat_off_th;
    int fan_control_ap_overheat_duration;
    int fan_control_battery_overheat_on_th;
    int fan_control_battery_overheat_off_th;

    double tof_calib_timeout_;
    uint8_t temp_tof_calibcmd = 0;
    uint8_t tof_calibState = 0;
    std::vector<float> tof_calibData;
    std::chrono::time_point<std::chrono::steady_clock> tof_calib_start_time_;

    std::vector<int> mtof_left_target_indices;
    std::vector<int> mtof_right_target_indices;
    int mtof_left_target_indices_size;
    int mtof_right_target_indices_size;

    bool bOnStation = false;
    pose station_pose;
    pose m_prev_station_pose;

    bool bSendMovingInfo = false;

    std::chrono::time_point<std::chrono::steady_clock> auto_send_time_;

    std::deque<mainState> main_state_queue_;
    std::deque<movingState> moving_state_queue_;

    uint8_t reservedResumeCount = 0;
    bool bReserveResumeAfter3sec = false;

    bool bEnableVertionTimer = false;

    std::string popo_maneuver_state_str_;

public:
    UdpCommunication();
    ~ UdpCommunication();

    void initParams();
    rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter>& params);
    void initializeData();
    void setCommnicationMode(UDP_COMMUNICATION set);
    UDP_COMMUNICATION getCommunicationMode();
    
    void versionGenerator();

    void enableSensorTimer(UDP_COMMUNICATION mode);
    void disableSensorTimer();

    void tofCalibrationTimerCallback();
    void selftestCalibrationTimerCallback();
    bool tofCalibrationChecker(uint8_t cmd, uint8_t state,double runtime);

    void enableMapSub();
    void disableMapSub();

    void enable_amclposeSub();
    void disable_amclposeSub();
    
    void enableSlamposeSub();
    void disableSlamposeSub();

    void enableJigMode(UDP_COMMUNICATION mode);
    void disableJigMode();

    void enableLinearTargetMoving(double v, double distance);
    void disableLinearTargetMovoing();
    void processLinearMoving();

    void procSocCommunication();
    void procAmrJigCommunication();
    void procApJigCommunication();
    std::string readVersionFromIni(const std::string& filename, const std::string& section, const std::string& key);

    void autoPublisher();
    void reqSocActionChecker();
    void reqSocOptionChecker();
    void reqSocDataChecker();
    void generateSocCommand();
    void directRequestCommand();

    void generateVersion();

    void setNodeState(uint8_t set);
    void setNaviFailReason(NAVI_FAIL_REASON set, uint8_t reason);

    void setRobotPoseValid(bool valid);
    void setSocRobotPoseData(bool valid, double x, double y, double theta);
    void setSocFrontLidarData(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void setSocRearLidarData(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void setSocWheelMotorData(const everybot_custom_msgs::msg::MotorStatus::SharedPtr msg);
    void setSocIMUData(const sensor_msgs::msg::Imu::SharedPtr msg);
    void setSocCameraData(const everybot_custom_msgs::msg::CameraDataArray::SharedPtr msg);
    #if USE_LINELASER_SENSOR > 0
    void setSocLineLaserData(const everybot_custom_msgs::msg::LineLaserDataArray::SharedPtr msg);
    #endif
    void setTofStatus(uint8_t top, uint8_t bottom_left, uint8_t bottom_right);
    void setSocTofData(const everybot_custom_msgs::msg::TofData::SharedPtr msg);
    void setSocCliffLiftData(const std_msgs::msg::UInt8::SharedPtr msg);
    void setSocBottomIrData(const everybot_custom_msgs::msg::BottomIrData::SharedPtr msg);
    void setSocDockReceiverData(const everybot_custom_msgs::msg::StationData::SharedPtr msg);
    void setSocBatteryData(const everybot_custom_msgs::msg::BatteryStatus::SharedPtr msg);
    void setSocTargetPosition(double x, double y, double theta);
    void setSocMapData(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void setSocRobotVelocity(double v, double w);
    void setSocRobotState(uint8_t state, uint8_t status);
    void setSocMovingState(uint8_t status, uint8_t fail_reason);
    void setSocError(ErrorList_t error);

    void setJigData(JIG_DATA_KEY key, const std::vector<uint8_t>& data);
    std::vector<uint8_t> getJigData(JIG_DATA_KEY key);

    bool isValidTargetPose();
    pose getTargetPose();
    bool isValidRobotPose();
    pose getRobotPose();
    MapInfo getMapInfo();

    //callback
    void udp_callback();
    void lidarFrontCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void lidarBackCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    #if USE_LINELASER_SENSOR > 0
    void lineLaserCallback(const everybot_custom_msgs::msg::LineLaserDataArray::SharedPtr msg);
    #endif
    void cameraCallback(const everybot_custom_msgs::msg::CameraDataArray::SharedPtr msg);
    void tofCallback(const everybot_custom_msgs::msg::TofData::SharedPtr msg);
    void errorListCallback(const everybot_custom_msgs::msg::ErrorListArray::SharedPtr msg);
    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void slamPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
	  void fw_version_callback(const std_msgs::msg::String::SharedPtr msg);
    void ai_version_callback(const std_msgs::msg::String::SharedPtr msg);
    void bottomStatusCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void bottomIrCallback(const everybot_custom_msgs::msg::BottomIrData::SharedPtr msg);
    void stationDataCallback(const everybot_custom_msgs::msg::StationData::SharedPtr msg);
    void batteryCallback(const everybot_custom_msgs::msg::BatteryStatus::SharedPtr msg);
    // IMU Data Processing
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void motorCallback(const everybot_custom_msgs::msg::MotorStatus::SharedPtr msg);
    void imuCalibrationCallback(const everybot_custom_msgs::msg::ImuCalibration::SharedPtr msg);
    void robotStateCallback(const everybot_custom_msgs::msg::RobotState::SharedPtr msg);
    void aiTemperatureCallback(const everybot_custom_msgs::msg::AiTemperature::SharedPtr msg);
    void calibCompleteStateCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void maneuverStateStrCallback(const std_msgs::msg::String::SharedPtr msg);
    void movingStateCallback(const everybot_custom_msgs::msg::NaviState::SharedPtr msg);
    void nodeStateCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void robotSpeedCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void cameraTypeCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void odom_status_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    void tofCalibStateCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void tofCalibDataCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    void readyCompleteRebootCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void recoveryLocalCallback(const std_msgs::msg::Int8::SharedPtr msg);
    //response - data
    void resSocMap();
    void resSocEncMap();
    void resSocRobotPose();
    void resSocSoftWareVersion();
    void resSocNodeStatus(); //삭제 예정

    void resSocRobotState();
    void resSocRobotStatus();
    void resSocTemperature();
    void resSocTemperatureNotice();

    void resSocTargetPosition();
    void resSocBattData();
    void resSocDockingStatus();
    void resSocRobotInfo();
    void resSocModifiedMap();
    void resSocMovingInfo();
    void resSocRobotVelocity();
    void resSocWheelMotorData();
    void resSocCameraData();
    void resSocLineLaserData();
    void resSocTofData();
    void resSocErrorList();
    void resSocLidarData();

    void resSocCliffLiftData();
    void resSocDockReceiverData();

    void resSocAllSensor();

    void resPonseJigCommand(int command, const std::vector<uint8_t>& packet);
    void resPonseJigData(JIG_DATA_KEY key);
    void logCommandAndData(int command, const std::vector<uint8_t>& data, int option); 
    void printLidar(int command, const std::vector<uint8_t>& data);
    void printTof(const std::vector<uint8_t>& data);

    UDP_COMMUNICATION checkUdpCommunicationMode();

    std::vector<uint8_t> makeResponseJigCommandPacket(JIG_HEADER header, std::vector<uint8_t> packet);
    void jigCheckWheelMotor(const std::vector<uint8_t>& packet);
    void jigCheckBattery(const std::vector<uint8_t>& packet);
    void jigCheckCliffLift();
    void jigCheckDockReceiver();
    void jigCheckTofSensor(const std::vector<uint8_t>& packet);
    void jigCheckFrontLidar();
    void jigCheckRearLidar();
    void jigCheckImuCalibration(const std::vector<uint8_t>& packet);
    void jigCheckVersion();
    void jigCheckBatteryShippingMode();
    void jigCheckToFLeftOffsetSetup();
    void jigCheckToFRightOffsetSetup();
    void jigProcessor(int header, const std::vector<uint8_t> &packet);

    void sendRamData(const std::vector<uint8_t>& ram_info);
    void apJigCheckRam();
    void sendEmmcData(const std::vector<uint8_t>& emmc_info);
    void apJigCheckEmmc();
    void sendLidarData(int header, const std::vector<uint8_t>& lidar_info);
    void sendVersionData(int header, const std::vector<uint8_t>& ver_info);
    void apJigCheckFrontLiDAR();
    void apJigCheckBackLiDAR();
    void apJigVersion();
    void apJigProcessor(int header, const std::vector<uint8_t>& packet);


    //data publisher
    void publishEmergencyCommand();
    void publishVelocityCommand(double v, double w);
    void publishChargingCommand(bool start);
    void publishBlockArea(const ByPassOne_t& parsedData);
    void publishBlockWall(const ByPassOne_t& parsedData);
    void publishClearVirtualWall();
    void publishMoveGoal(double x,double y,double theta);
    void publishMoveCharger();
    void publishMoveNRotation(double x,double y,double theta, int type);
    void publishRotation(int type,double theta);
    void publishLidarOnOff(bool on_off);
    void publishSensorOnOff(bool set);
    void publishBatterySleep();
    void publishStationPose(double x, double y, double theta);
    void publishDockingCommand(bool start);
    void publishVersionRequest(uint8_t cmd);
    void publishInpectionStart();
    void publishInpectionStop();
    void publishFactoryStart();
    void publishFactoryStop();
    void publishStateCommand(REQUEST_SOC_CMD cmd);
    void publishReqCameraType();
    void publishReqFWReset();
    void publishReqAIReset();
    void publishHeartbeatVersion();
    void publishStartOdomReset();
    void publishClearOdomReset();
    void publishTofLeftCalibration();
    void publishTofRightCalibration();
    void publishMapCopyComplete();
    void publishReadyReboot();
    void publishMapChaged();
    void clearMapInfo();
    void clearErrorList();

    void systemRebootCommand();

    //mapping function
    bool copyFile(const std::string& source, const std::string& destination);
    bool removeFile(const std::string& filepath);

    void savePGMFile(const ModifiedMapDataB_t &settings, const std::string &filename);
    bool convertYamlImagePath(const std::string& sourceYaml, const std::string& destinationYaml, const std::string& newImageName) ;
    bool readPGM(const std::string& pgm_filename, int& width_map, int& height_map, std::vector<uint8_t>& mapData, double& origin_x, double& origin_y);
    bool updateMapFiles(const std::string& pgmSrc, const std::string& yamlSrc, const std::string& pgmDst, const std::string& yamlDst);
    double read_temperature_from_file(const std::string& file_path);

    double quaternion_to_euler(const geometry_msgs::msg::Quaternion& quat);
    std::vector<uint8_t> mapDataTypeConvert(const std::vector<int8_t>& src, uint32_t height, uint32_t width);
    std::vector<uint8_t> jigDataConvertWheelMotor(const everybot_custom_msgs::msg::MotorStatus::SharedPtr msg);
    std::vector<uint8_t> jigDataConvertBattery(const everybot_custom_msgs::msg::BatteryStatus::SharedPtr msg);
    std::vector<uint8_t> jigDataConvertTof(const everybot_custom_msgs::msg::TofData::SharedPtr msg);
    std::vector<uint8_t> jigDataConvertLidar(int long_dist, int short_dist);
    std::vector<uint8_t> jigDataConvertImuCalibration(const everybot_custom_msgs::msg::ImuCalibration::SharedPtr msg);
    uint8_t getLowerBits(uint8_t byte);
    uint8_t getUpperBits(uint8_t byte);
    
    void generateFrontBackScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, std::vector<int>& vecDist);
    void splitLidarScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, std::vector<int>& long_dist, std::vector<int>& short_dist);

    int getMinDistanceFromLidarSensor(const std::vector<int>& vecDistance);
    double getDistance(pose base, pose current);
    bool isFileExist(const std::string& filename);

    bool isValidateResetOdom();

    double getParamTofCalibTimeout();

    bool loadStationPose(const std::string& file_path);
    void sendstationPoseChecker();
    
    void udpCommunicationThread();
    double getSteadyClockRunningSeconds(const std::chrono::time_point<std::chrono::steady_clock> &start_time);

    void enqueueMainState(const mainState& data);
    std::vector<mainState> dequeueMainState();

    void enqueueMovingState(const movingState& data);
    std::vector<movingState> dequeueMovingState();

    bool checkDockingStatusOnDemend();
    bool checkVersionGenerate();
};

#endif // UDP_COMMUNICATION_HPP
