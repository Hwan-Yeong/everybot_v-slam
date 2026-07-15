#ifndef UART_COMMUNICATION_HPP
#define UART_COMMUNICATION_HPP

#include <chrono>
#include <vector>
#include <mutex>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <atomic>
#include <thread>
#include <dlfcn.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "everybot_custom_msgs/msg/battery_status.hpp"
#include "everybot_custom_msgs/msg/motor_status.hpp"
#include "everybot_custom_msgs/msg/tof_data.hpp"
#include "everybot_custom_msgs/msg/imu_calibration.hpp"
#include "everybot_custom_msgs/msg/rpm_control.hpp"
#include "everybot_custom_msgs/msg/station_data.hpp"
#include "everybot_custom_msgs/msg/bottom_ir_data.hpp"

#define PREVENT_SAME_CMD_VEL 0
#if PREVENT_SAME_CMD_VEL > 0
#include "everybot_custom_msgs/msg/navi_state.hpp"
#endif

#include "everybot_bringup/uart_helpers.h"


using namespace std::chrono_literals;

// ErrorMapping 구조체 정의
struct ErrorMapping
{
    int rank;
    std::string error_string;
};

typedef struct TransmissionData
{
    double linear_velocity;  // Linear Velocity (m/s)
    double angular_velocity; // Angular Velocity (radians/s)
    int16_t left_mt_rpm;     // Left Motor RPM
    int16_t right_mt_rpm;    // Right Motor RPM
    uint8_t reset_flags;     // Reset Flags
    uint8_t motor_flags;     // Motor Enable Flags
    uint8_t docking_flags;   // DOCKING&CHARGE Flags
    uint8_t imu_flags;       // IMU Calibration Flags
    uint8_t tofnBatt_flags;  // tof Batt on/off Flags
    uint8_t index;           // Index (0~255)
} TransmissionData;

enum class ResetCommand : uint8_t
{
    // 하위 4 비트 (Reset)
    NORMAL_OPERATION = 0x0, // Normal Operation (Default)
    ODOMETRY_RESET = 0x1,   // Odometry Reset
    IMU_RESET = 0x2,        // IMU Reset
    MCU_RESET = 0xF         // MCU Reset
};

enum class MotornRemoteMode : uint8_t
{
    // 상위 4 비트 (Motor Mode)
    VW_MODE = 0x0,      // Motor VW Mode (Default)
    RPM_MODE = 0x1,     // Motor RPM Mode
    MANUAL_MODE = 0x2,  // Motor Manual Mode
    BRAKE_MODE = 0x3,   // Motor Brake Mode
    DISABLE_MODE = 0xF, // Motor Disable Mode

    // 하위 4 비트 ( remote-control lock flag )
    UNBLOCK_REMOTE = 0x0, // 0x00
    BLOCK_REMOTE = 0x1    // 0x01
};

// DockingChargeCommand enum (상위 4 비트와 하위 4 비트에 해당하는 충전 및 도킹 명령)
enum class DockingChargeCommand : uint8_t
{
    // 상위 4 비트 (CHARGE)
    CHARGE_CONTROL_MCU = 0x0,      // MCU에서 충전 제어 (Auto) - 3.8A 고속충전 (Default)
    CHARGE_CONTROL_AP = 0x1,       // MCU에서 충전 제어 (Auto) - 1A 저속충전
    HIGH_SPEED_CHARGE_START = 0x2, // 고속 충전 시작 (3.8A 고속충전)
    LOW_SPEED_CHARGE_START = 0x3,  // 저속 충전 시작 (1A 저속충전)
    CHARGE_STOP = 0xF,             // 충전 중지

    // 하위 4 비트 (DOCKING)
    DOCKING_START = 0x1, // 도킹 시작
    DOCKING_STOP = 0x0   // 도킹 스탑
};

enum class ImuCalibrationCommand : uint8_t
{
    // 하위 4 비트 ( Calibration )
    NORMAL = 0x0, // (Default)
    CALIBRATION_START = 0x1,
    END_OF_ROTATION = 0x2
};

enum class ToFnBatt : uint8_t
{
    // 상위 4 비트
    BATT_NORMAL = 0x0,        // Batt Normal
    BATT_SLEEP = 0x1,           // Batt Sleep
    // 하위 4 비트 ( ToF On / oFF  )
    ON = 0x0, // ToF On  // (Default)
    OFF = 0x1 // ToF Off
};

enum ErrorCode1 {
  ERROR_LEFT_MT_UART = 0x01,
  ERROR_LEFT_MT_STALL = 0x02,
  ERROR_LEFT_MT_OVERHEAT = 0x04,
  ERROR_LEFT_MT_CURRENT = 0x08,
  ERROR_RIGHT_MT_UART = 0x10,
  ERROR_RIGHT_MT_STALL = 0x20,
  ERROR_RIGHT_MT_OVERHEAT = 0x40,
  ERROR_RIGHT_MT_CURRENT = 0x80
};

enum ErrorCode2 {
  ERROR_Left_TOF_I2C = 0x01,
  ERROR_Right_TOF_I2C = 0x02,
  ERROR_1D_TOF_UART = 0x04,
  ERROR_IMU_UART = 0x08,
  ERROR_AP_RX_UART = 0x10,
  ERROR_BATTERY_COMMUNICATION = 0x20,
  ERROR_CHARGE_OVERCURRENT = 0x40,
  ERROR_DISCHARGE_OVERCURRENT = 0x80
};

enum ErrorCode3 {
    ERROR_BATTERY_CHARGE_OVER_HEAT = 0x01,
    ERROR_BATTERY_DISCHAR_OVER_HEAT = 0x02,
    ERROR_POGO_PIN_OVER_HEAT = 0x04,
    ERROR_DOCKING_SIG_NOT_FOUND = 0x08
};

/* Docking State String *
static const char* docking_state_str[] = {
    "UNKNOWN",               // 0
    "Turn_Angle_0",          // 1
    "Turn_Angle_Stop",       // 2
    "Detect_Robot_Position", // 3
    "Detect_Turn",           // 4
    "Detect_Turn_Stop",      // 5
    "Position_Save",         // 6
    "Position_Cal_Dist",     // 7
    "Right_CCW_To_Station",  // 8
    "Right_To_Center",       // 9
    "Right_Find_Center",     // 10
    "Left_CW_To_Station",    // 11
    "Left_To_Center",        // 12
    "Left_Find_Center",      // 13
    "Right_Stop_Center",     // 14
    "Right_CW_To_Center",    // 15
    "Right_Restart",         // 16
    "Left_Stop_Center",      // 17
    "Left_CCW_To_Center",    // 18
    "Left_Restart",          // 19
    "Center_Update_Position",// 20
    "Center_Go",             // 21
    "Docking_OK",            // 22
    "Past_Right_Stop",       // 23
    "Past_Right_Turn",       // 24
    "Past_Right_Center",     // 25
    "Past_Right_Restart",    // 26
    "Past_Left_Stop",        // 27
    "Past_Left_Turn",        // 28
    "Past_Left_Center",      // 29
    "Past_Left_Restart"      // 30
};
*/

#if PREVENT_SAME_CMD_VEL > 0
enum class NAVI_STATE : int {
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
  case NAVI_STATE::READY:
    out = std::string("READY");
    break;  
  case NAVI_STATE::MOVE_GOAL:
    out = std::string("MOVE_GOAL");
    break;
  case NAVI_STATE::ARRIVED_GOAL:
    out = std::string("ARRIVED_GOAL");
    break;
    case NAVI_STATE::ALTERNATE_GOAL:
    out = std::string("ALTERNATE_GOAL");
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
  }
  return out;
};
#endif

class UARTCommunication : public rclcpp::Node
{
public:
    UARTCommunication();
    ~UARTCommunication();

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr odom_status_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr bottom_status_pub_;
    rclcpp::Publisher<everybot_custom_msgs::msg::BottomIrData>::SharedPtr bottom_ir_data_pub_;
    rclcpp::Publisher<everybot_custom_msgs::msg::BatteryStatus>::SharedPtr battery_status_pub_;
    rclcpp::Publisher<everybot_custom_msgs::msg::TofData>::SharedPtr tof_data_pub_;
    rclcpp::Publisher<everybot_custom_msgs::msg::MotorStatus>::SharedPtr motor_status_pub_;
    rclcpp::Publisher<everybot_custom_msgs::msg::StationData>::SharedPtr station_data_pub_;
    rclcpp::Publisher<everybot_custom_msgs::msg::ImuCalibration>::SharedPtr jig_imu_calibration_pub_;

    // error
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_left_mt_stall_or_overcurrent_pub_; //E04
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_right_mt_stall_or_overcurrent_pub_; //E04-1
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_left_mt_overheat_pub_; //E04-2
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_right_mt_overheat_pub_; //E04-3
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_docking_sig_pub_; //E07
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_charge_overcurrent_pub_; //F01
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_discharge_overcurrent_pub_; //F01-1
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_1d_tof_uart_pub_; //F07
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_right_mt_uart_pub_; //F11
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_left_mt_uart_pub_; //F12
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_battery_charge_overheat_pub_; //F15
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_battery_discharge_overheat_pub_; //F16
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_left_tof_i2c_pub_; //F17
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_right_tof_i2c_pub_; //F17-1
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_pogo_pin_overheat_pub_; //S11
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_imu_uart_pub_; //???
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_battery_communication_pub_; //???
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_ap_rx_pub_; //???

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fw_version_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr version_request_sub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr navi_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr docking_sub_;
    //rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr motor_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr remote_block_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_stop_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr charging_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr odom_imu_reset_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tof_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr battery_sleep_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr fw_reset_sub_;

    // jig
    rclcpp::Subscription<everybot_custom_msgs::msg::RpmControl>::SharedPtr jig_moter_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr jig_battery_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr jig_imu_sub;
    #if PREVENT_SAME_CMD_VEL > 0
    rclcp::Subscription<everybot_custom_msgs::msg::NaviState>::SharedPtr navi_state_sub;
    #endif

    UARTHelpers uart_;
    rclcpp::Duration timeout_;
    
    //std::chrono::time_point<std::chrono::steady_clock> start_node_time_;
    std::chrono::time_point<std::chrono::steady_clock> motor_communication_debug_time_;
    std::chrono::time_point<std::chrono::steady_clock> last_cmd_vel_time_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer2_;
    rclcpp::TimerBase::SharedPtr timer3_;
    std::mutex amcl_pose_;
    std::mutex odom_mutex_;
    std::mutex imu_mutex_;
    std::mutex imu_odom_status_mutex_;
    std::mutex bottom_ir_mutex_;
    std::mutex station_mutex_;
    std::mutex battery_mutex_;
    std::mutex tof_mutex_;
    std::mutex motor_mutex_;
    std::mutex jig_imu_calibration_mutex_;
    std::mutex error_mutex_;

    #if 0 //hjkim : block code not be used
    std::atomic<bool> serial_thread_running_;
    std::thread serial_thread_;
    #endif

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::Quaternion imu_orientation;

    nav_msgs::msg::Odometry odom_msg;
    sensor_msgs::msg::Imu imu_msg;
    std_msgs::msg::UInt8 imu_odom_status_msg;
    std_msgs::msg::UInt8 bottom_status_msg;          // cliff & lift
    everybot_custom_msgs::msg::BottomIrData bottom_ir_msg;
    everybot_custom_msgs::msg::StationData station_msg; // signal & docking status
    everybot_custom_msgs::msg::BatteryStatus battery_msg;
    everybot_custom_msgs::msg::TofData tof_msg;
    std_msgs::msg::UInt8 imu_calib_status;
    everybot_custom_msgs::msg::MotorStatus motor_msg;
    std_msgs::msg::String fwVersion_msg;
    std_msgs::msg::String cradleVersion_msg;
    everybot_custom_msgs::msg::ImuCalibration jig_imu_calibration_msg;//jig related variables
    std_msgs::msg::Bool error_left_mt_stall_or_overcurrent_msg, error_right_mt_stall_or_overcurrent_msg,
        error_left_mt_overheat_msg, error_right_mt_overheat_msg, error_right_mt_uart_msg, error_left_mt_uart_msg,
        error_charge_overcurrent_msg, error_discharge_overcurrent_msg, error_battery_communication_msg,
        error_docking_sig_msg, error_1d_tof_uart_msg, error_left_tof_i2c_msg, error_right_tof_i2c_msg, error_imu_uart_msg,
        error_battery_charge_overheat_msg, error_battery_discharge_overheat_msg, error_pogo_pin_overheat_msg, error_ap_rx_msg; // error variables
    std::map<std::string, bool> prev_error_state_;

    const uint8_t PRE_AMBLE_0 = 0xAA;
    const uint8_t PRE_AMBLE_1 = 0x55;

    TransmissionData g_transmission_data;
    double previous_roll, previous_pitch, previous_yaw;
    double amcl_pose_x_, amcl_pose_y_, amcl_pose_angle_;

    uint8_t prev_precharge_state = 0;
    uint8_t prev_charge_mode = 0;
    uint8_t prev_fet_state = 0;

    bool errorStates[3][8] = {}; 
    bool bEmergencyStop = false;
    bool bJigMotorInspection = false;

    #if PREVENT_SAME_CMD_VEL > 0
    NAVI_STATE navi_state = NAVI_STATE::IDLE;
    #endif

private:
    // 명령어 타입을 정의하는 enum
    enum class CommandType : uint8_t
    {
        // RX_LEGACYPROTOCOL_  = 0x10,
        RX_V1PROTOCOL_ = 0x11,
        RX_V2PROTOCOL_ = 0x12,

        TX_V1PROTOCOL_ = 0x51,
    };

#pragma pack(push, 1) // Ensure no padding between struct members
    typedef struct ErrorCode
    {
        uint32_t ERROR_LEFT_MT_UART : 1;     // Bit 0
        uint32_t ERROR_LEFT_MT_STALL : 1;    // Bit 1
        uint32_t ERROR_LEFT_MT_SENSOR : 1;   // Bit 2
        uint32_t ERROR_LEFT_MT_OVERHEAT : 1; // Bit 3
        uint32_t ERROR_LEFT_MT_CURRENT : 1;  // Bit 4
        uint32_t ERROR_LEFT_MT_CONTROL : 1;  // Bit 5
        uint32_t RESERVED_1 : 1;             // Bit 6 (Reserved)
        uint32_t ERROR_RIGHT_MT_UART : 1;    // Bit 7

        uint32_t ERROR_RIGHT_MT_STALL : 1;    // Bit 8
        uint32_t ERROR_RIGHT_MT_SENSOR : 1;   // Bit 9
        uint32_t ERROR_RIGHT_MT_OVERHEAT : 1; // Bit 10
        uint32_t ERROR_RIGHT_MT_CURRENT : 1;  // Bit 11
        uint32_t ERROR_RIGHT_MT_CONTROL : 1;  // Bit 12
        uint32_t RESERVED_2 : 1;              // Bit 13 (Reserved)
        uint32_t ERROR_LEFT_TOF_I2C : 1;      // Bit 14
        uint32_t ERROR_RIGHT_TOF_I2C : 1;     // Bit 15

        uint32_t ERROR_1D_TOF_UART : 1;           // Bit 16
        uint32_t ERROR_IMU_UART : 1;              // Bit 17
        uint32_t ERROR_TOF_CHECK : 1;             // Bit 18
        uint32_t ERROR_BATTERY_UART : 1;          // Bit 19
        uint32_t ERROR_CHARGE_OVERCURRENT : 1;    // Bit 20
        uint32_t ERROR_DISCHARGE_OVERCURRENT : 1; // Bit 21
        uint32_t ERROR_BATTERY_TEMP : 1;          // Bit 22
        uint32_t ERROR_OVERVOLTAGE : 1;           // Bit 23

        uint32_t ERROR_SHORT_CIRCUIT : 1;      // Bit 24
        uint32_t ERROR_CHARGE : 1;             // Bit 25
        uint32_t ERROR_BATTERY_PROTECTION : 1; // Bit 26
        uint32_t RESERVED_3 : 1;               // Bit 27 (Reserved)
        uint32_t ERROR_FALLING : 1;            // Bit 28 (Warning)
        uint32_t ERROR_DOCKING : 1;            // Bit 29 (Warning)
        uint32_t RESERVED_4 : 1;               // Bit 30 (Reserved)
        uint32_t RESERVED_5 : 1;               // Bit 31 (Reserved)
    } ErrorCode;
#pragma pack(pop)     // Reset to default packing

#pragma pack(push, 1) // Ensure no padding between struct members
    typedef struct V1DataPacket
    {
        uint8_t pre_amble_0; // 0xAA
        uint8_t pre_amble_1; // 0x55
        uint8_t data_size;   // Data Size: 166
        uint8_t command;     // 0x11

        double x_position; // Odometry X (double), unit: mm (8 bytes for double) (p)
        double y_position; // Odometry Y (double), unit: mm (8 bytes for double) (p)

        uint8_t left_motor_status;  // Left Motor Status (p)
        uint8_t right_motor_status; // Right Motor Status (p)

        uint8_t motor_mode;       // Motor Mode (p)
        uint8_t left_motor_type;  // Left Motor Type: 0x01 or 0x02 (p)
        uint8_t right_motor_type; // Right Motor Type: 0x01 or 0x02 (p)

        uint8_t error_code_1; // Error Code 1 (0x00: No Error, 0x01: Error)
        uint8_t error_code_2; // Error Code 2
        uint8_t error_code_3;      // Reserved field

        uint8_t imu_index; // IMU Index (0-255)
        int16_t imu_yaw;   // IMU Yaw (Angle in degrees, +/- 180˚)
        int16_t imu_pitch; // IMU Pitch (Angle in degrees, +/- 90˚)
        int16_t imu_roll;  // IMU Roll (Angle in degrees, +/- 180˚)

        int16_t imu_x_acc; // IMU X-axis Acceleration (mg) (p)
        int16_t imu_y_acc; // IMU Y-axis Acceleration (mg) (p)
        int16_t imu_z_acc; // IMU Z-axis Acceleration (mg) (p)

        int16_t cell_voltage1; // Cell Voltage 1-5 (mV) (p)
        int16_t cell_voltage2; // Cell Voltage 1-5 (mV) (p)
        int16_t cell_voltage3; // Cell Voltage 1-5 (mV) (p)
        int16_t cell_voltage4; // Cell Voltage 1-5 (mV) (p)
        int16_t cell_voltage5; // Cell Voltage 1-5 (mV) (p)

        int16_t total_capacity;       // Total Capacity (mAh) (p)
        int16_t remaining_capacity;   // Remaining Capacity (mAh) (p)
        uint8_t battery_manufacturer; // Battery Manufacturer: 0x00, 0x01, or 0x02 (p)

        uint8_t charge_status; // Charge Status

        uint8_t imu_odometry_status;         // IMU and Odometry Status
        uint8_t bottom_wheel_lifting_sensor; // Bottom & Wheel Lifting Sensor
        uint8_t dock_sig_status;             // Dock Signal Status (m)
        uint8_t dock_short_ir_position;      // Dock Short IR Position (m)
        uint8_t docking_status;              // Docking Status (m)
        uint8_t dock_long_ir_position;       // Dock Long IR Position (m)
        uint8_t battery_percent;             // Battery Percentage (0-100%) (p)
        int16_t battery_voltage;             // Battery Voltage (mV) (p)
        int16_t battery_current;             // Battery Current (+charge, -discharge, 1mA) (p)
        uint8_t battery_temperature1;        // Battery Temperature (°C) (Sensor 1 and 2) (p)
        uint8_t battery_temperature2;        // Battery Temperature (°C) (Sensor 1 and 2) (p)
        int16_t design_capacity;             // Design Capacity (mAh) (p)
        int16_t number_of_cycles;            // Number of Cycles (uint16_t) (p)

        int16_t top_tof;             // Top TOF (mm) (p)
        uint8_t top_1d_tof_status;   // Top 1D TOF Status (p)
        int16_t lower_left_tof[16];  // Lower Left TOF (mm) (p)
        int16_t lower_right_tof[16]; // Lower Right TOF (mm) (p)
        uint8_t lower_tof_status;    // Left/Right TOF Status (p)

        uint8_t imu_calibration_status; // IMU Calibration Status

        int32_t left_motor_encoder;  // Left Motor Encoder (int32_t) (p)
        int32_t right_motor_encoder; // Right Motor Encoder (int32_t) (p)

        int16_t left_motor_current;  // Left Motor Current (10mA) (p)
        int16_t right_motor_current; // Right Motor Current (10mA) (p)
        int16_t left_motor_rpm;      // Left Motor RPM (p)
        int16_t right_motor_rpm;     // Right Motor RPM (p)

        uint8_t cradle_adc0; // Cradle ADC (Temperature in °C)
        uint8_t cradle_adc1; // Cradle ADC (Temperature in °C)

        uint8_t cradle_fw_ver_msb; // Cradle FW Major Version
        uint8_t cradle_fw_ver_lsb; // Cradle FW Minor Version

        uint8_t charging_mode; // Charging Mode (p)

        int8_t left_motor_temperature;  // Left Motor Temperature (°C) (p)
        int8_t right_motor_temperature; // Right Motor Temperature (°C) (p)

        uint8_t firmware_ver_msb; // Firmware Major Version
        uint8_t firmware_ver_lsb; // Firmware Minor Version
        uint8_t checksum;         // Checksum (from byte 3 to byte 167)
    } V1DataPacket;
#pragma pack(pop) // Reset to default packing

#pragma pack(push, 1) // Ensure no padding between struct members
    typedef struct V2DataPacket
    {
        uint8_t pre_amble_0; // 0xAA
        uint8_t pre_amble_1; // 0x55
        uint8_t data_size;   // Data Size: 166
        uint8_t command;     // 0x11

        double x_position; // Odometry X (double), unit: mm (8 bytes for double) (p)
        double y_position; // Odometry Y (double), unit: mm (8 bytes for double) (p)

        uint8_t left_motor_status;  // Left Motor Status (p)
        uint8_t right_motor_status; // Right Motor Status (p)

        uint8_t motor_mode;       // Motor Mode (p)
        uint8_t left_motor_type;  // Left Motor Type: 0x01 or 0x02 (p)
        uint8_t right_motor_type; // Right Motor Type: 0x01 or 0x02 (p)

        uint8_t error_code_1; // Error Code 1 (0x00: No Error, 0x01: Error)
        uint8_t error_code_2; // Error Code 2
        uint8_t error_code_3;      // Reserved field

        uint8_t imu_index; // IMU Index (0-255)
        int16_t imu_yaw;   // IMU Yaw (Angle in degrees, +/- 180˚)
        int16_t imu_pitch; // IMU Pitch (Angle in degrees, +/- 90˚)
        int16_t imu_roll;  // IMU Roll (Angle in degrees, +/- 180˚)

        int16_t imu_x_acc; // IMU X-axis Acceleration (mg) (p)
        int16_t imu_y_acc; // IMU Y-axis Acceleration (mg) (p)
        int16_t imu_z_acc; // IMU Z-axis Acceleration (mg) (p)

        int16_t cell_voltage1; // Cell Voltage 1-5 (mV) (p)
        int16_t cell_voltage2; // Cell Voltage 1-5 (mV) (p)
        int16_t cell_voltage3; // Cell Voltage 1-5 (mV) (p)
        int16_t cell_voltage4; // Cell Voltage 1-5 (mV) (p)
        int16_t cell_voltage5; // Cell Voltage 1-5 (mV) (p)

        int16_t total_capacity;       // Total Capacity (mAh) (p)
        int16_t remaining_capacity;   // Remaining Capacity (mAh) (p)
        uint8_t battery_manufacturer; // Battery Manufacturer: 0x00, 0x01, or 0x02 (p)

        uint8_t charge_status; // Charge Status

        uint8_t imu_odometry_status;         // IMU and Odometry Status
        uint8_t bottom_wheel_lifting_sensor; // Bottom & Wheel Lifting Sensor
        uint8_t dock_sig_status;             // Dock Signal Status (m)
        uint8_t dock_short_ir_position;      // Dock Short IR Position (m)
        uint8_t docking_status;              // Docking Status (m)
        uint8_t dock_long_ir_position;       // Dock Long IR Position (m)
        uint8_t battery_percent;             // Battery Percentage (0-100%) (p)
        int16_t battery_voltage;             // Battery Voltage (mV) (p)
        int16_t battery_current;             // Battery Current (+charge, -discharge, 1mA) (p)
        uint8_t battery_temperature1;        // Battery Temperature (°C) (Sensor 1 and 2) (p)
        uint8_t battery_temperature2;        // Battery Temperature (°C) (Sensor 1 and 2) (p)
        int16_t design_capacity;             // Design Capacity (mAh) (p)
        int16_t number_of_cycles;            // Number of Cycles (uint16_t) (p)

        int16_t top_tof;             // Top TOF (mm) (p)
        uint8_t top_1d_tof_status;   // Top 1D TOF Status (p)
        int16_t lower_left_tof[16];  // Lower Left TOF (mm) (p)
        int16_t lower_right_tof[16]; // Lower Right TOF (mm) (p)
        uint8_t lower_tof_status;    // Left/Right TOF Status (p)

        uint8_t imu_calibration_status; // IMU Calibration Status

        int32_t left_motor_encoder;  // Left Motor Encoder (int32_t) (p)
        int32_t right_motor_encoder; // Right Motor Encoder (int32_t) (p)

        int16_t left_motor_current;  // Left Motor Current (10mA) (p)
        int16_t right_motor_current; // Right Motor Current (10mA) (p)
        int16_t left_motor_rpm;      // Left Motor RPM (p)
        int16_t right_motor_rpm;     // Right Motor RPM (p)

        uint8_t cradle_adc0; // Cradle ADC (Temperature in °C)
        uint8_t cradle_adc1; // Cradle ADC (Temperature in °C)

        uint8_t cradle_fw_ver_msb; // Cradle FW Major Version
        uint8_t cradle_fw_ver_lsb; // Cradle FW Minor Version

        uint8_t charging_mode; // Charging Mode (p)

        int8_t left_motor_temperature;  // Left Motor Temperature (°C) (p)
        int8_t right_motor_temperature; // Right Motor Temperature (°C) (p)
        
        uint16_t bottom_ir_1; // BOTTOM IR 1
        uint16_t bottom_ir_2; // BOTTOM IR 2
        uint16_t bottom_ir_3; // BOTTOM IR 3
        uint16_t bottom_ir_4; // BOTTOM IR 4
        uint16_t bottom_ir_5; // BOTTOM IR 5
        uint16_t bottom_ir_6; // BOTTOM IR 6

        
        uint8_t shipping_mode;
        // Reserve (예약 영역)
        // uint8_t reserve_1;
        uint8_t charge_control_flags;
        uint8_t docking_state;
        uint8_t battery_version;

        uint8_t firmware_ver_msb; // Firmware Major Version
        uint8_t firmware_ver_lsb; // Firmware Minor Version
        uint8_t checksum;         // Checksum (from byte 3 to byte 167)
    } V2DataPacket;
#pragma pack(pop) // Reset to default packing

    struct ToFData16Data
    {
        uint8_t pre_amble_0;
        uint8_t pre_amble_1;
        uint8_t data_size;
        uint8_t command;
        std::vector<uint16_t> left_tof;
        std::vector<uint16_t> right_tof;
        uint8_t checksum;
    };

private:
    void initializeData();
    void setCurrentVelocity(double v, double w);
    void pulishOdom(bool bOdomTfPub);
    void pulishImuOdomStatus();
    void pulishImu();
    void pulishBottomIr();
    void pulishTof();
    void pulishMotorStatus();
    void pulishStationData();
    void pulishJigImuCalibration();
    void pulishError();
    void pulishBatteryStatus();
    void pubTopic();

    void batterySleepCallback(const std_msgs::msg::Empty::SharedPtr);
    //void motorModeCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void remoteBlockCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void tofCommandCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void dockingCommandCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void e_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void charge_cmd_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    void odom_imu_reset_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void reqVersionCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void jig_motor_callback(const everybot_custom_msgs::msg::RpmControl msg);
    void jig_bettery_callback(const std_msgs::msg::UInt8 msg);
    void jig_imu_callback(const std_msgs::msg::UInt8 msg);
    void onFWResetReceived(const std_msgs::msg::Empty::SharedPtr);
    void timerCallback();

    #if PREVENT_SAME_CMD_VEL > 0
    void movingStateCallback(const everybot_custom_msgs::msg::NaviState::SharedPtr msg);
    #endif
    void receiveData();
    void searchAndParseData(std::vector<uint8_t> &buffer);

    uint8_t getUpperBits(uint8_t byte);
    uint8_t getLowerBits(uint8_t byte);
    uint16_t combineBytesToUint16(const std::vector<uint8_t> &data, int index);
    int16_t combineBytesToInt16(const std::vector<uint8_t> &data, int index);
    uint64_t combineBytesToUint64(const std::vector<uint8_t> &data, int startIndex);
    double combineBytesToDouble(const std::vector<uint8_t> &data, int index);

    /************************************************************************* */
    void sendProtocolV1Data(TransmissionData &gData);
    
    // Default Mode
    // Reset            : Normal
    // Motor            : VW
    // charge           : 고속충전 
    // Docking          : 도킹 스탑
    // ImuCalibration   : Normal 
    // ToF              : On
    void defaultTransmissionData();

    /************************************************************************* */

    /**************************************************************************************/
    // RESET
    /**************************************************************************************/

    void setReset(ResetCommand status);
    ResetCommand getResetStatus() const;
    bool isResetStatus(ResetCommand status) const;

    /**************************************************************************************/
    // Motor Enable & Remote Block
    /**************************************************************************************/
    void setMotorMode(MotornRemoteMode mode);
    void setRemoteMode(MotornRemoteMode mode);
    MotornRemoteMode getMotorMode() const;
    MotornRemoteMode getRemoteMode() const;
    bool isMotorMode(MotornRemoteMode mode) const;
    bool isRemoteMode(MotornRemoteMode mode) const;
    /**************************************************************************************/
    // Docking & Charge Flag
    /**************************************************************************************/

    void setCharge(DockingChargeCommand set);
    void setDocking(DockingChargeCommand set);
    DockingChargeCommand getChargeMode() const;
    DockingChargeCommand getDockingMode() const;
    bool isChargeMode(DockingChargeCommand mode) const;
    bool isDockingMode(DockingChargeCommand mode) const;

    /**************************************************************************************/
    // Calibration
    /**************************************************************************************/

    void setIMUCalibration(ImuCalibrationCommand set);
    ImuCalibrationCommand getIMUCalibration() const;
    bool isIMUCalibrationMode(ImuCalibrationCommand mode) const;

    /**************************************************************************************/
    // ToF On/Off n Batt Mode Change
    /**************************************************************************************/
    void setToF(ToFnBatt cmd);
    void setBattMode(ToFnBatt cmd);
    ToFnBatt getToFStatus() const;
    ToFnBatt getBattMode() const;
    bool isToFStatus(ToFnBatt status) const;
    bool isBattMode(ToFnBatt status) const;
   
    /**************************************************************************************/
    // Motor Control
    /**************************************************************************************/

    void setLinearVelocity(double velocity);
    void setAngularVelocity(double velocity);
    void setLeftMotorRpm(int16_t rpm);
    void setRightMotorRpm(int16_t rpm);

    double getLinearVelocity() const;
    double getAngularVelocity() const;
    int16_t getLeftMotorRpm() const;
    int16_t getRightMotorRpm() const;

    /************************************************************************* */
    void parseDataFields(const std::vector<uint8_t> &data);

    template <typename PacketType>
    void setOdomMsg(const PacketType  &packet);
    template <typename PacketType>
    void setOdomStatusMsg(const PacketType  &packet);
    template <typename PacketType>
    void setImuMsg(const PacketType  &packet);
    template <typename PacketType>
    void setBatteryMsg(const PacketType  &packet);
    template <typename PacketType>
    void setTofMsg(const PacketType  &packet);
    template <typename PacketType>
    void setMotorMsg(const PacketType  &packet);
    template <typename PacketType>
    void setDockingMsg(const PacketType  &packet);
    template <typename PacketType>
    void setBottomMsg(const PacketType  &packet);
    template <typename PacketType>
    void setFwVersionMsg(const PacketType  &packet);
    template <typename PacketType>
    void setMcuErrorMsg(const PacketType  &packet);
    template <typename PacketType>
    void setCradleADCMsg(const PacketType  &packet);
    template <typename PacketType>
    void setButtomIRMsg(const PacketType  &packet);

    // void setProtocolModeMsg(CommandType protocol);
    template <typename PacketType>
    void setJigCalibrationMsg(const PacketType &packet);

    void ParseV1Data(const std::vector<uint8_t> &data, V1DataPacket &packet);
    void ParseV2Data(const std::vector<uint8_t> &data, V2DataPacket &packet);

    void printV1DataPacket(const V1DataPacket &packet);

    uint8_t calculateChecksum(const std::vector<uint8_t> &data, size_t start, size_t end);
    std::vector<double> computeAngularVelocity(double roll, double pitch, double roll_dot, double pitch_dot, double yaw_dot);
    double quaternion_to_euler(const geometry_msgs::msg::Quaternion &quat);
    void updateErrorStatus(uint8_t errorCode, uint8_t mask, const char *errorMessage, std_msgs::msg::Bool &error_msg, uint8_t errorCategory);
    
    size_t getUartAvailableData();
    int getUartReadData(unsigned char* buffer, size_t read_size);

    bool isEqualbyEpsilon(double a, double b, double epsilon = 1e-6);
    bool preventUnintendAccelationChecker(double current_linear_velocity, double current_angular_velocity, bool preventState);
};

#endif // UART_COMMUNICATION_HPP