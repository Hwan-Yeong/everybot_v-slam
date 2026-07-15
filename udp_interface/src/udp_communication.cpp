#include "udp_communication.hpp"

#define FW_VERSION_REQUEST 0x01
#define AI_VERSION_REQUEST 0x02

#define AUTO_CHARGE_HIGH          0x0         // MCU에서 충전 제어 (Auto) - 3.8A 고속충전
#define AUTO_CHARGE_LOW           0x1         // MCU에서 충전 제어 (Auto) - 1A 저속충전
#define CHARGE_STOP               0xF         // 충전 중지

#define FRONT_REAR_MIN_ANGLE (178.0f * M_PI / 180.0f)
#define FRONT_REAR_MAX_ANGLE (182.0f * M_PI / 180.0f)

#define JIG_LIDAR_LONG_DIST_ANGLE_MIN  (220.0f * M_PI / 180.0f) // 로봇 헤딩 기준 CCW 44-4 deg
#define JIG_LIDAR_LONG_DIST_ANGLE_MAX  (228.0f * M_PI / 180.0f) // 로봇 헤딩 기준 CCW 44+4 deg
#define JIG_LIDAR_SHORT_DIST_ANGLE_MIN  (86.0f * M_PI / 180.0f) // 로봇 헤딩 기준 CW 90-4 deg
#define JIG_LIDAR_SHORT_DIST_ANGLE_MAX  (94.0f * M_PI / 180.0f) // 로봇 헤딩 기준 CW 90+4 deg

#define AP_JIG_CHECK_ON_AMR 1 // AMR에서 단품지그 테스트할 때 : 1

#define MOTOR_DEFAULT_MODE 0x00
#define MOTOR_RPM_MODE     0x01
#define MOTOR_BREAK_MODE   0x03
#define MOTOR_DISABLE_MODE 0x0F
#define MOTOR_MODE_ERROR   0xFF

#define BATTERY_DEFAULT_MODE 0x00
#define BATTERY_HIGH_SPEED_MODE 0x01
#define BATTERY_LOW_SPEED_MODE 0x02
#define BATTERY_MODE_ERROR 0xFF

#define BOOT_TIME_OUT 120

#define DOCK_START 0x01
#define DOCK_STOP 0x00

auto initNodeTime = std::chrono::steady_clock::now();

std::string station_pose_path = "/home/airbot/app_rw/stationPose/station_pose.json";

UdpCommunication::UdpCommunication() : rclcpp::Node("udp_communication")
{
    rclcpp::QoS qos_state_profile = rclcpp::QoS(5)
                            .reliable()
                            .durability_volatile();

    rclcpp::QoS qos_best_effort_profile = rclcpp::QoS(5)
                            .best_effort()
                            .durability_volatile();

    initializeData();

    req_state_sub = this->create_subscription<everybot_custom_msgs::msg::RobotState>("/state_datas", qos_state_profile, std::bind(&UdpCommunication::robotStateCallback, this, std::placeholders::_1)); 
    req_navi_sub = this->create_subscription<everybot_custom_msgs::msg::NaviState>("/navi_datas", 10, std::bind(&UdpCommunication::movingStateCallback, this, std::placeholders::_1)); 
    node_status_sub_ = this->create_subscription<std_msgs::msg::UInt8>("/node_status", 10, std::bind(&UdpCommunication::nodeStateCallback, this, std::placeholders::_1)); 
    battery_status_sub = this->create_subscription<everybot_custom_msgs::msg::BatteryStatus>( "/battery_status", 10, std::bind(&UdpCommunication::batteryCallback, this, std::placeholders::_1));
    /*hjkim : robot pose와 odom 로그를 추가하기 위해 odom을 상시 callback 하고 imu callback을 sensor timer enable/disable 하도록 변경.
    현재 udp에서 udp 통신 100ms delay( 모든 response API 함수에 포함)로 UDP timer와 subcribe callback에 부하를 주지 않도록 주의 필요.*/
    //imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/imu_data", 10, std::bind(&UdpCommunication::imuCallback, this, std::placeholders::_1)); 
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&UdpCommunication::odomCallback, this, std::placeholders::_1));
    station_data_sub_ = this->create_subscription<everybot_custom_msgs::msg::StationData>("/station_data", 10, std::bind(&UdpCommunication::stationDataCallback, this, std::placeholders::_1));
    error_list_sub_ = this->create_subscription<everybot_custom_msgs::msg::ErrorListArray>("/error_list", 10, std::bind(&UdpCommunication::errorListCallback, this, std::placeholders::_1));
    //robot_speed_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&UdpCommunication::robotSpeedCallback, this, std::placeholders::_1));
    aitemperature_data_sub = this->create_subscription<everybot_custom_msgs::msg::AiTemperature>("/aitemperature_data", 10, std::bind(&UdpCommunication::aiTemperatureCallback, this, std::placeholders::_1)); 
    calib_complete_state_sub_ = this->create_subscription<std_msgs::msg::Bool>("/perception/calibration/complete", 10, std::bind(&UdpCommunication::calibCompleteStateCallback, this, std::placeholders::_1));
    maneuver_state_str_sub_ = this->create_subscription<std_msgs::msg::String>("/maneuver/state/string", 10, std::bind(&UdpCommunication::maneuverStateStrCallback, this, std::placeholders::_1));

    sw_version_pub_ = this->create_publisher<std_msgs::msg::String>("/sw_version", 1);
    req_version_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/req_version", 1);
    req_camera_type_pub_ = this->create_publisher<std_msgs::msg::Empty>("/req_camera_type", 10);

    req_soc_cmd_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/soc_cmd", 10);
    emergency_stop_pub_ = this->create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10);
    move_target_pub_ = this->create_publisher<everybot_custom_msgs::msg::Position>("/move_target", 10);
    move_charger_pub_ = this->create_publisher<std_msgs::msg::Empty>("/move_charger", 10);
    move_rotation_pub_ = this->create_publisher<everybot_custom_msgs::msg::MoveNRotation>("/move_n_rotation", 10);
    rotation_pub_ = this->create_publisher<everybot_custom_msgs::msg::MoveNRotation>("/rotation", 10);
    manual_move_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    charge_pub = this->create_publisher<std_msgs::msg::UInt8>("/charging_cmd", 10);
    dock_pub = this->create_publisher<std_msgs::msg::UInt8>("/docking_cmd", 10);
    block_area_pub_ = this->create_publisher<everybot_custom_msgs::msg::BlockAreaList>("/block_areas", 10);
    block_wall_pub_ = this->create_publisher<everybot_custom_msgs::msg::BlockAreaList>("/block_walls", 10);
    
    lidar_cmd_pub_ = this->create_publisher<std_msgs::msg::Bool>("/cmd_lidar", 10);
    tof_cmd_pub_ = this->create_publisher<std_msgs::msg::Bool>("/cmd_tof", 10);
	camera_cmd_pub_ = this->create_publisher<std_msgs::msg::Bool>("/cmd_camera", 10);
    #if USE_LINELASER_SENSOR > 0
    linelaser_cmd_pub_ = this->create_publisher<std_msgs::msg::Bool>("/cmd_linelaser", 10);
    #endif
    batterySleep_cmd_pub_ = this->create_publisher<std_msgs::msg::Empty>("/cmd_battery_sleep", 10);
    station_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/station_pose", 1);
    inspection_mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/inspection_mode", 10);
    factory_mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/factory_mode", 10);

    start_tofcalib_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/start_tofcalib", 10); // left : 0x01 , right : 0x02
    tof_calibState_sub_ = this->create_subscription<std_msgs::msg::UInt8>("/tof_calib_state", 10, std::bind(&UdpCommunication::tofCalibStateCallback, this, std::placeholders::_1));
    tof_calibData_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/tof_calib_data", 10, std::bind(&UdpCommunication::tofCalibDataCallback, this, std::placeholders::_1));

    map_copy_pub_ = this->create_publisher<std_msgs::msg::Empty>("/map_copy_complete", 10);

    // req_state_pub_ = this->create_publisher<everybot_custom_msgs::msg::RobotState>("/request_state", 10);
    //cmd_rpm_pub_ = this->create_publisher<everybot_custom_msgs::msg::MotorRpm>("/cmd_rpm", 10);
    
    ai_reset_pub_ = this->create_publisher<std_msgs::msg::Empty>("/req_ai_reset", 10);
    fw_reset_pub_ = this->create_publisher<std_msgs::msg::Empty>("/req_fw_reset", 10);
    reboot_ready_complete_sub_ = this->create_subscription<std_msgs::msg::UInt8>("/reboot_ready_complete", 10, std::bind(&UdpCommunication::readyCompleteRebootCallback, this, std::placeholders::_1));
    
    ready_reboot_pub_ = this->create_publisher<std_msgs::msg::Empty>("/ready_reboot", rclcpp::SystemDefaultsQoS());
    reset_odom_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/odom_imu_reset_cmd", 1);

    mapinfo_changed_pub_ = this->create_publisher<std_msgs::msg::Empty>("/mapinfo_changed", qos_best_effort_profile);
    ap_temperature_pub_ = this->create_publisher<everybot_custom_msgs::msg::ApTemperature>("/ap_temperature_data", 10);

    recovery_local_sub_ = this->create_subscription<std_msgs::msg::Int8>("/recovery_local", rclcpp::SystemDefaultsQoS(), std::bind(&UdpCommunication::recoveryLocalCallback, this, std::placeholders::_1));
    camera_type_sub_ = this->create_subscription<std_msgs::msg::UInt8>("/camera_type", 10, std::bind(&UdpCommunication::cameraTypeCallback, this, std::placeholders::_1));

    fw_version_sub = this->create_subscription<std_msgs::msg::String>("/fw_version", 10, std::bind(&UdpCommunication::fw_version_callback, this, std::placeholders::_1));
    rclcpp::QoS qos_profile_ai = rclcpp::QoS(5).reliable().transient_local();
    ai_version_sub = this->create_subscription<std_msgs::msg::String>("/ai_version", qos_profile_ai, std::bind(&UdpCommunication::ai_version_callback, this, std::placeholders::_1));

    autoPublishTimer = this->create_wall_timer(1000ms, std::bind(&UdpCommunication::autoPublisher, this));
    udp_communication_thread_ = std::thread(&UdpCommunication::udpCommunicationThread, this);

    RCLCPP_INFO(this->get_logger(), "node initialized");
}

UdpCommunication::~UdpCommunication()
{
    if(udp_communication_thread_.joinable()){
        udp_communication_thread_.join();
    }
    RCLCPP_INFO(this->get_logger(), "node terminated");
}

void UdpCommunication::initParams()
{
    this->declare_parameter("tof_calib_timeout", 5.0);
    this->declare_parameter("fan_control.power.auto_mapping", 2);
    this->declare_parameter("fan_control.power.battery_overheat", 1);
    this->declare_parameter("fan_control.power.ap_overheat", 1);
    this->declare_parameter("fan_control.threshold.ap_overheat.on_celsius", 73.0);
    this->declare_parameter("fan_control.threshold.ap_overheat.off_celsius", 70.0);
    this->declare_parameter("fan_control.threshold.ap_overheat.duration_sec", 30);
    this->declare_parameter("fan_control.threshold.battery_overheat.on_celsius", 43);
    this->declare_parameter("fan_control.threshold.battery_overheat.off_celsius", 40);

    this->get_parameter("tof_calib_timeout", tof_calib_timeout_);
    this->get_parameter("fan_control.power.auto_mapping", fan_control_power_auto_mapping);
    this->get_parameter("fan_control.power.battery_overheat", fan_control_power_battery_overheat);
    this->get_parameter("fan_control.power.ap_overheat", fan_control_power_ap_overheat);
    this->get_parameter("fan_control.threshold.ap_overheat.on_celsius", fan_control_ap_overheat_on_th);
    this->get_parameter("fan_control.threshold.ap_overheat.off_celsius", fan_control_ap_overheat_off_th);
    this->get_parameter("fan_control.threshold.ap_overheat.duration_sec", fan_control_ap_overheat_duration);
    this->get_parameter("fan_control.threshold.battery_overheat.on_celsius", fan_control_battery_overheat_on_th);
    this->get_parameter("fan_control.threshold.battery_overheat.off_celsius", fan_control_battery_overheat_off_th);

    RCLCPP_INFO(this->get_logger(), "tof_calib_timeout: %.2f", tof_calib_timeout_);

    std::ostringstream oss;
    oss << "UDP Parameters:\n"
        << "  tof_calib_timeout = " << std::fixed << std::setprecision(2) << tof_calib_timeout_ << "\n"
        << "  fan_control.power.auto_mapping = Level " << fan_control_power_auto_mapping << "\n"
        << "  fan_control.power.battery_overheat = Level " << fan_control_power_battery_overheat << "\n"
        << "  fan_control.power.ap_overheat = Level " << fan_control_power_ap_overheat << "\n"
        << "  fan_control.threshold.ap_overheat.on_celsius = " << fan_control_ap_overheat_on_th << " C\n"
        << "  fan_control.threshold.ap_overheat.off_celsius = " << fan_control_ap_overheat_off_th << " C\n"
        << "  fan_control.threshold.ap_overheat.duration_sec = " << fan_control_ap_overheat_duration << " sec\n"
        << "  fan_control.threshold.battery_overheat.on_celsius = " << fan_control_battery_overheat_on_th << " C\n"
        << "  fan_control.threshold.battery_overheat.off_celsius = " << fan_control_battery_overheat_off_th << " C";

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
}
void UdpCommunication::initializeData()
{
    bOnStation = false;
    station_pose.x = 0.0;
    station_pose.y = 0.0;
    station_pose.theta = 0.0;
    if(isFileExist(station_pose_path)){
        loadStationPose(station_pose_path);
    }

    RCLCPP_INFO(this->get_logger(), "[initializeData] station_pose(%.2f,%.2f,%.2f)", station_pose.x, station_pose.y, station_pose.theta);
    initParams();
    //init State
    movefail_reason = NAVI_FAIL_REASON::VOID;
    nodeState = NODE_STATUS::IDLE;
    bInspectionMode = false;
    //init Version
    std::string iniFile = "/home/airbot/vslam_ws/install/udp_interface/share/udp_interface/launch/config.ini";
    std::string version = readVersionFromIni(iniFile, "info", "version");
    socData.version.bSet = false;
    socData.version.sw_ver = version;
    socData.version.mcu_ver = "0.0";
    socData.version.ai_ver = "0.0";
    generateVersion();
    prev_fw_version = "0.0";
    prev_ai_version = "0.0";
    //initRobotPosition
    socData.robotPosition.valid = false;
    socData.robotPosition.x = 0.0;
    socData.robotPosition.y = 0.0;
    socData.robotPosition.theta = 0.0;

    mtof_calib_data_ = Mtof_Calib();
    mtof_calib_file_generated_ = false;
    selftest_calib_timer_reset_cnt_ = 0;
    left_calib_data_set_done_ = false;
    right_calib_data_set_done_ = false;
    is_copy_jig_calibration_file_ = false;
    
    socData.mapInfo.bReceived = false;
    socData.mapInfo.resolution = 0.05;
    socData.mapInfo.width = 0;
    socData.mapInfo.height = 0;
    socData.mapInfo.origin_x = 0.0;
    socData.mapInfo.origin_y = 0.0;
    socData.camera_type = 0;
    bInspectionMotor = false;
    bFactoryNaviMode = false;

    bRequestFrontLidarDist = false;
    bRequestBackLidarDist = false;

    m_is_starting_comm_SoC = false;

    mtof_left_target_indices = {
        0, 3, 6,
        17, 20, 23,
        28, 31,
        44, 47,
        49, 52, 55,
        57, 60, 63
    };

    mtof_right_target_indices = {
        1, 4, 7,
        16, 19, 22,
        24, 27,
        40, 43,
        48, 51, 54,
        56, 59, 62
    };

    mtof_left_target_indices_size = static_cast<int>(mtof_left_target_indices.size());
    mtof_right_target_indices_size = static_cast<int>(mtof_right_target_indices.size());

    for (int i = 0; i < 64; ++i) { // 모든 mtof 기본값 -1로 세팅
        socData.tofInfo.left_bottom.data[i] = -1;
        socData.tofInfo.right_bottom.data[i] = -1;
    }

    setCommnicationMode(UDP_COMMUNICATION::NORMAL);
    bEnableVertionTimer = true;
}

rcl_interfaces::msg::SetParametersResult UdpCommunication::paramCallback(const std::vector<rclcpp::Parameter>& params)
{  
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& param : params) {
      if (param.get_name() == "tof_calib_timeout") {
        tof_calib_timeout_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated tof_calib_timeout: %.2f", tof_calib_timeout_);
      }
  }
  return result;
}

void UdpCommunication::cameraTypeCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    if(socData.camera_type != msg->data){
        RCLCPP_INFO(this->get_logger(), "[cameraTypeCallback] Camera Type previous : %u, --> new: %u",socData.camera_type, msg->data);
    }
    socData.camera_type = msg->data;
}

void UdpCommunication::odom_status_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    if(!bReceiveOdomStatus){
        bReceiveOdomStatus = true;
        RCLCPP_INFO(this->get_logger(), "receive check odom_status OK hex[%02x]", msg->data);
    }
    if(odom_status != msg->data){
      RCLCPP_INFO(this->get_logger(), "odom_status hex[%02x]", msg->data);
    }
    odom_status = msg->data;
}

void UdpCommunication::tofCalibStateCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    /**
     * @brief Left (LSB) / Right (MSB)
     *   Running:         0x01 (Left), 0x10 (Right)
     *   Complete:        0x02 (Left), 0x20 (Right)
     *   Out of Range:    0x03 (Left), 0x30 (Right)
     *   Unstable Range:  0x04 (Left), 0x40 (Right)
     */
    if(tof_calibState != msg->data){
      RCLCPP_INFO(this->get_logger(), "[tofCalibStateCallback] Changed tofCalibState cmd[%02x] previous[%02x] --> new[%02x]",temp_tof_calibcmd,tof_calibState, msg->data);
    }
    tof_calibState = msg->data;
}

void UdpCommunication::tofCalibDataCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() > 0) {
        mtof_calib_data_.setCalibData(msg->data);
        if (mtof_calib_data_.isLeftSide(msg->data[CALIB_TOF_DATA_IDX::SIDE])) { // left
            left_calib_data_set_done_ = true;
        } else {
            right_calib_data_set_done_ = true;
        }
    }

    if (msg->data.size() == CALIB_TOF_DATA_IDX::ARRAY_SIZE) {
        RCLCPP_INFO(this->get_logger(),
            "[tofCalibDataCallback] Valid Data -> size: %zu | side: %s | values: [%.3f, %.3f, %.3f]",
            msg->data.size(), ((msg->data[CALIB_TOF_DATA_IDX::SIDE]==0)?"LEFT":"RIGHT"), msg->data[CALIB_TOF_DATA_IDX::IDX13], msg->data[CALIB_TOF_DATA_IDX::IDX14], msg->data[CALIB_TOF_DATA_IDX::IDX15]
        );
    } else {
        RCLCPP_INFO(this->get_logger(), "[tofCalibDataCallback] Invalid Data -> size: %zu", msg->data.size());
    }
}

void UdpCommunication::readyCompleteRebootCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "[readyCompleteRebootCallback] reboot start");
    publishReqAIReset();
    publishReqFWReset(); 
    sleep(3); // reset 명령 발행 후 3초 대기
    systemRebootCommand();
}

void UdpCommunication::recoveryLocalCallback(const std_msgs::msg::Int8::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "recoveryLocalCallback [%d]", msg->data);
  if(msg->data == 1){
    publishStateCommand(REQUEST_SOC_CMD::PAUSE_WORKING);
    RCLCPP_INFO(this->get_logger(), "detect recovery localization recovery publish pause working");
  }else if(msg->data == 2){
    reservedResumeCount = 0;
    bReserveResumeAfter3sec = true;
    RCLCPP_INFO(this->get_logger(), "recovery complete, reserve resume after 3sec");
  }else{
    RCLCPP_INFO(this->get_logger(), "recovery localization state msg is wrong");
  }
}

void UdpCommunication::versionGenerator()
{
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_sec = now-initNodeTime;
    double runtime = elapsed_sec.count();

    if (prev_fw_version != socData.version.mcu_ver) {
        RCLCPP_INFO(this->get_logger(), "MCU version updated: %s", socData.version.mcu_ver.c_str());
        generateVersion();
    }

    if (prev_ai_version != socData.version.ai_ver) {
        RCLCPP_INFO(this->get_logger(), "AI version updated: %s", socData.version.ai_ver.c_str());
        generateVersion();
    }

    prev_fw_version = socData.version.mcu_ver;
    prev_ai_version = socData.version.ai_ver;

    if(runtime >= BOOT_TIME_OUT || (socData.version.mcu_ver != "0.0"/* && socData.version.ai_ver != "0.0"*/)){
        if(runtime >= BOOT_TIME_OUT){
            // RCLCPP_INFO(this->get_logger(), "Version Check Timeout %f, MCU version %s, AI version %s",runtime,socData.version.mcu_ver.c_str(),socData.version.ai_ver.c_str());
        }else{
            RCLCPP_INFO(this->get_logger(), "Version Check Complete runTime %f, MCU version %s, AI version %s",runtime,socData.version.mcu_ver.c_str(),socData.version.ai_ver.c_str());
        }
        socData.version.bSet = true;
        return;
    }else{
        uint8_t command = 0;
        if (socData.version.mcu_ver == "0.0"){
            command |= FW_VERSION_REQUEST;
            RCLCPP_INFO(this->get_logger(), "FW_VERSION_REQUEST");
        }
        if (socData.version.ai_ver == "0.0"){
            command |= AI_VERSION_REQUEST;
            RCLCPP_INFO(this->get_logger(), "AI_VERSION_REQUEST");
        }
        publishVersionRequest(command);
    }
    RCLCPP_INFO(this->get_logger(), "versionGenerator runTime(%.2f)",runtime);
}

void UdpCommunication::enableSensorTimer(UDP_COMMUNICATION mode)
{
    RCLCPP_INFO(this->get_logger(), "enableSensorTimer");
    publishSensorOnOff(true);
    if(!lidar_front_sub_){
        lidar_front_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan_front", 10, std::bind(&UdpCommunication::lidarFrontCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "lidar_front_sub_ created");
    }else{
        RCLCPP_INFO(this->get_logger(), "lidar_front_sub_ already exist");
    }
    if(!lidar_back_sub_){
        lidar_back_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan_back", 10, std::bind(&UdpCommunication::lidarBackCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "lidar_back_sub_ created");
    }else{
        RCLCPP_INFO(this->get_logger(), "lidar_back_sub_ already exist");
    }
	#if USE_LINELASER_SENSOR > 0
    if(!line_laser_sub_){
        line_laser_sub_ = this->create_subscription<everybot_custom_msgs::msg::LineLaserDataArray>("/line_laser_data", 10, std::bind(&UdpCommunication::lineLaserCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "line_laser_sub_ created");
    }else{
        RCLCPP_INFO(this->get_logger(), "line_laser_sub_ already exist");
    }
	#endif
    if(!camera_sub_){
        camera_sub_ = this->create_subscription<everybot_custom_msgs::msg::CameraDataArray>("/camera_data", 10, std::bind(&UdpCommunication::cameraCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "camera_sub_ created");
    }else{
        RCLCPP_INFO(this->get_logger(), "camera_sub_ already exist");
    }

    if(!tof_status_sub){
        tof_status_sub = this->create_subscription<everybot_custom_msgs::msg::TofData>("/tof_data", 10, std::bind(&UdpCommunication::tofCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "tof_status_sub_ created");
    }else{
        RCLCPP_INFO(this->get_logger(), "tof_status_sub_ already exist");
    }
    if(!bottom_status_sub_){
        bottom_status_sub_ = this->create_subscription<std_msgs::msg::UInt8>("bottom_status", 10, std::bind(&UdpCommunication::bottomStatusCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "bottom_status_sub_ created");
    }else{
        RCLCPP_INFO(this->get_logger(), "bottom_status_sub_ already exist");
    }
    if(!bottom_ir_sub_){
        bottom_ir_sub_ = this->create_subscription<everybot_custom_msgs::msg::BottomIrData>("bottom_ir_data", 10, std::bind(&UdpCommunication::bottomIrCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "bottom_ir_sub_ created");
    }else{
        RCLCPP_INFO(this->get_logger(), "bottom_ir_sub_ already exist");
    }
    if(!motor_status_sub){
        motor_status_sub = this->create_subscription<everybot_custom_msgs::msg::MotorStatus>("/motor_status", 10, std::bind(&UdpCommunication::motorCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "motor_status_sub created");
    }else{
        RCLCPP_INFO(this->get_logger(), "motor_status_sub already exist");
    }

    //reset_odom_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/odom_imu_reset_cmd", 1);

    if( !odom_status_sub_){
        odom_status_sub_ = this->create_subscription<std_msgs::msg::UInt8>("/odom_status", 1,
        std::bind(&UdpCommunication::odom_status_callback, this, std::placeholders::_1));
    } else{
        RCLCPP_INFO(this->get_logger(), "[odom_status] subscription already exists. skip create subscription");
    }

    if(!imu_sub){
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/imu_data", 10, std::bind(&UdpCommunication::imuCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "imu_sub_ created");
    }else{
        RCLCPP_INFO(this->get_logger(), "imu_sub_ already exist");
    }
}

void UdpCommunication::disableSensorTimer()
{
    RCLCPP_INFO(this->get_logger(), "disableSensorTimer");
    publishSensorOnOff(false);
    if(lidar_front_sub_){
        lidar_front_sub_.reset();
        RCLCPP_INFO(this->get_logger(), "reset-lidar_front_sub");
    }else{
        RCLCPP_INFO(this->get_logger(), "lidar_front_sub allready reset ");
    }
    if(lidar_back_sub_){
        lidar_back_sub_.reset();
        RCLCPP_INFO(this->get_logger(), "reset-lidar_back_sub");
    }else{
        RCLCPP_INFO(this->get_logger(), "lidar_back_sub allready reset ");
    }
	#if USE_LINELASER_SENSOR > 0
    if(line_laser_sub_){
        line_laser_sub_.reset();
        RCLCPP_INFO(this->get_logger(), "reset-line_laser_sub");
    }else{
        RCLCPP_INFO(this->get_logger(), "line_laser_sub allready reset ");
    }
	#endif
    if(camera_sub_){
        camera_sub_.reset();
        RCLCPP_INFO(this->get_logger(), "reset-camera_sub");
    }else{
        RCLCPP_INFO(this->get_logger(), "camera_sub allready reset ");
    }
    if(tof_status_sub){
        tof_status_sub.reset();
        RCLCPP_INFO(this->get_logger(), "reset-tof_status_sub");
    }else{
        RCLCPP_INFO(this->get_logger(), "tof_status_sub allready reset ");
    }
    if(bottom_status_sub_){
        bottom_status_sub_.reset();
        RCLCPP_INFO(this->get_logger(), "reset-bottom_status_sub");
    }else{
        RCLCPP_INFO(this->get_logger(), "bottom_status_sub allready reset ");
    }
    if(bottom_ir_sub_){
        bottom_ir_sub_.reset();
        RCLCPP_INFO(this->get_logger(), "reset-bottom_ir_sub");
    }else{
        RCLCPP_INFO(this->get_logger(), "bottom_ir_sub allready reset ");
    }
    if(motor_status_sub){
        motor_status_sub.reset();
        RCLCPP_INFO(this->get_logger(), "reset-motor_status_sub");
    }else{
        RCLCPP_INFO(this->get_logger(), "motor_status_sub allready reset ");    
    }
    
    if(imu_sub){
        imu_sub.reset();
        RCLCPP_WARN(this->get_logger(), "imu_sub_ reset");
    }else{
        RCLCPP_WARN(this->get_logger(), "imu_sub_ is already reset");
    }
    if (odom_status_sub_) {  // 등록되어 있다면 해제
        odom_status_sub_.reset();
        RCLCPP_INFO(this->get_logger(), "odom status callback disabled");
    }else{
        RCLCPP_INFO(this->get_logger(), "odom status callback already disabled");
    }

    #if 0 // hjkim :  odom reset publish는 셀프테스트 모드에서만 호출되기 때문에 enable/disable 하는것이 dds 부하를 줄수 있어 초기 생성 후 유지하는 것으로 변경.
    if(reset_odom_pub_){
        reset_odom_pub_.reset();
        RCLCPP_WARN(this->get_logger(), "reset_odom_pub_ reset");
    }else{
        RCLCPP_WARN(this->get_logger(), "reset_odom_pub_ is already reset");
    }
    #endif
}

void UdpCommunication::tofCalibrationTimerCallback()
{
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_sec = now-tof_calib_start_time_;
    double runtime = elapsed_sec.count();
    if(tofCalibrationChecker(temp_tof_calibcmd, tof_calibState, runtime)){
         if(tof_calib_timer_){
            tof_calib_timer_.reset();
            RCLCPP_INFO(this->get_logger(), "[tofCalibrationTimerCallback] reset tof calibration timer");
        }else{
            RCLCPP_INFO(this->get_logger(), "[tofCalibrationTimerCallback] tof calibration timer already reset");
        }
        RCLCPP_INFO(this->get_logger(), "[tofCalibrationTimerCallback] finish timer");
    }
}

void UdpCommunication::selftestCalibrationTimerCallback()
{
    if(!tof_calib_timer_){
        tof_calib_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&UdpCommunication::tofCalibrationTimerCallback, this));
        RCLCPP_INFO(this->get_logger(), "[selftestCalibrationTimerCallback] Start tof calibration timer");
    }else{
        RCLCPP_INFO(this->get_logger(), "[selftestCalibrationTimerCallback] tof calibration timer already started");
    }

    if (tof_calibState == 0x02) { // Left Complete
        RCLCPP_INFO(this->get_logger(), "[selftestCalibrationTimerCallback] Left calibration completed.");
        // Right 상태 Running 으로 세팅
        tof_calibState = (tof_calibState & 0x0F) | (0x01 << 4);
        temp_tof_calibcmd = 0x02;
        publishTofRightCalibration();
        tof_calib_start_time_ = std::chrono::steady_clock::now();
    }
    else if (tof_calibState == 0x20) { // Right Complete
        RCLCPP_INFO(this->get_logger(), "[selftestCalibrationTimerCallback] Right calibration completed.");
        // SoC에 calibration 결과 데이터 전송

        if (mtof_calib_file_generated_ && left_calib_data_set_done_ && right_calib_data_set_done_) {
            resGetToFCalibrationData(mtof_calib_data_.left, mtof_calib_data_.right);
            RCLCPP_INFO(this->get_logger(), "[selftestCalibrationTimerCallback] Send left result = %d, right result = %d Complete. %d", mtof_calib_data_.left.result, mtof_calib_data_.right.result, __LINE__);
            mtof_calib_data_ = Mtof_Calib();
            left_calib_data_set_done_ = false;
            right_calib_data_set_done_ = false;
            selftest_calib_timer_.reset();
        } else {
            RCLCPP_INFO(this->get_logger(), "[selftestCalibrationTimerCallback] Waiting for A1_perception to generate the calibration file && calibration data set ...");
            selftest_calib_timer_reset_cnt_++;
            if (selftest_calib_timer_reset_cnt_ >= 10) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "[selftestCalibrationTimerCallback] Reset tof calibration timer (file generate time over 10 sec) file generated: %d, left done: %d, right done: %d",
                    mtof_calib_file_generated_, left_calib_data_set_done_, right_calib_data_set_done_
                );
                mtof_calib_data_ = Mtof_Calib();
                left_calib_data_set_done_ = false;
                right_calib_data_set_done_ = false;
                selftest_calib_timer_.reset();
                selftest_calib_timer_reset_cnt_ = 0;
            }
        }
    } else if (!(tof_calibState == 0x01 || tof_calibState == 0x10)) {
        RCLCPP_INFO(this->get_logger(), "[selftestCalibrationTimerCallback] Calibration Fail. STATE: %d", tof_calibState);

        if (left_calib_data_set_done_ && right_calib_data_set_done_) {
            resGetToFCalibrationData(mtof_calib_data_.left, mtof_calib_data_.right);
            RCLCPP_INFO(this->get_logger(), "[selftestCalibrationTimerCallback] Send left result = %d, right result = %d Complete. %d", mtof_calib_data_.left.result, mtof_calib_data_.right.result, __LINE__);
            mtof_calib_data_ = Mtof_Calib();
            left_calib_data_set_done_ = false;
            right_calib_data_set_done_ = false;
            selftest_calib_timer_.reset();
        } else {
            RCLCPP_INFO(this->get_logger(), "[selftestCalibrationTimerCallback] Waiting for calibration data setting ...");
            selftest_calib_timer_reset_cnt_++;
            if (selftest_calib_timer_reset_cnt_ >= 10) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "[selftestCalibrationTimerCallback] Reset tof calibration timer (file generate time over 10 sec) left done: %d, right done: %d",
                    left_calib_data_set_done_, right_calib_data_set_done_
                );
                mtof_calib_data_ = Mtof_Calib();
                left_calib_data_set_done_ = false;
                right_calib_data_set_done_ = false;
                selftest_calib_timer_.reset();
                selftest_calib_timer_reset_cnt_ = 0;
            }
        }
    }
}

bool UdpCommunication::tofCalibrationChecker(uint8_t cmd, uint8_t state,double runtime)
{
    bool ret = false;
    double wait_timeout = getParamTofCalibTimeout();
    std::vector<uint8_t> data;

    auto isCmdValid = [](uint8_t val) {
        return (val == 0x01 || val == 0x02);
    };

    if(!isCmdValid(cmd)){
        RCLCPP_INFO(this->get_logger(), "[tofCalibrationChecker] return notValid cmd[%02x]",cmd);
        return true;
    }

    auto isStateValid = [](uint8_t val) {
        return (val == 0x00 || val == 0x01 || val == 0x02 || val == 0x0F);
    };

    if (cmd == 0x01 || cmd == 0x02) {
        switch (state) {
            case 0x01: // Running (Left)
            case 0x10: // Running (Right)
                data.push_back(0x01);
                RCLCPP_INFO(this->get_logger(),
                            "[tofCalibrationChecker] tof calibration Running! cmd[%02x] current_state[%02x] runtime[%.2f]",
                            cmd, state, runtime);
                break;
            case 0x02: // Complete (Left)
                data.push_back(0x02);
                RCLCPP_INFO(this->get_logger(),
                            "[tofCalibrationChecker] tof calibration complete! cmd[%02x] current_state[%02x] runtime[%.2f]",
                            cmd, state, runtime);
                ret = true;
                break;
            case 0x20: // Complete (Right)
                data.push_back(0x02);
                RCLCPP_INFO(this->get_logger(),
                            "[tofCalibrationChecker] tof calibration complete! cmd[%02x] current_state[%02x] runtime[%.2f]",
                            cmd, state, runtime);
                break;
            case 0x03: // Out of Range (Left)
            case 0x30: // Out of Range (Right)
                data.push_back(0x0F);
                RCLCPP_INFO(this->get_logger(),
                            "[tofCalibrationChecker] tof calibration [Out of Range] fail cmd[%02x] current_state[%02x] runtime[%.2f]",
                            cmd, state, runtime);
                ret = true;
                break;
            case 0x04: // Unstable Range (Left)
            case 0x40: // Unstable Range (Right)
                data.push_back(0x0F);
                RCLCPP_INFO(this->get_logger(),
                            "[tofCalibrationChecker] tof calibration [Unstable Range] fail cmd[%02x] current_state[%02x] runtime[%.2f]",
                            cmd, state, runtime);
                ret = true;
                break;
            case 0x08: // NON RENEWAL (Left)
            case 0x80: // NON RENEWAL (Right)
                data.push_back(0x0F);
                RCLCPP_INFO(this->get_logger(),
                            "[tofCalibrationChecker] tof calibration [NON RENEWAL] fail cmd[%02x] current_state[%02x] runtime[%.2f]",
                            cmd, state, runtime);
                ret = true;
                break;
            default:
                RCLCPP_INFO(this->get_logger(),
                            "[tofCalibrationChecker] inValid ToF State cmd[%02x] current_state[%02x]",
                            cmd, state);
                return true;
        }
    }

    if (communicationMode == UDP_COMMUNICATION::AMR_JIG_MODE) {
        if(cmd == 0x01){
            setJigData(JIG_DATA_KEY::TOF_LEFT_OFFSET_SETUP, data);
            resPonseJigData(JIG_DATA_KEY::TOF_LEFT_OFFSET_SETUP);
        }else{
            if (state == 0x20) { // jig calibration에서 Calibration 파일 생성 완료된 후 amr jig 로 데이터 전송하도록
                if (mtof_calib_file_generated_) {
                    std::string calib_file = "/home/airbot/app_rw/perception/params/calibration.yaml";
                    std::string copy_calib_file = "/home/airbot/app_rw/log/jig_calibration.yaml";
                    bool bExist = std::filesystem::exists(calib_file) && std::filesystem::is_regular_file(calib_file);
                    if (bExist) {
                        bool copy_result = copyFile(calib_file, copy_calib_file);
                        if (copy_result) {
                            RCLCPP_INFO(this->get_logger(), "[Self Test Calibration] Calibration file copied successfully.");
                            is_copy_jig_calibration_file_ = true;
                        } else { // copy 실패 시 캘리브레이션 완료 response 안보냄
                            RCLCPP_INFO(this->get_logger(), "[Self Test Calibration] Failed to copy calibration file.");
                            ret = true;
                        }
                    } else { // 캘리브레이션 결과 파일 존재하지 않을 시 완료 response 안보냄
                        RCLCPP_INFO(this->get_logger(), "[Self Test Calibration] Required file not found: %s", calib_file.c_str());
                        ret = true;
                    }

                    if (is_copy_jig_calibration_file_) {
                        is_copy_jig_calibration_file_ = false;
                        setJigData(JIG_DATA_KEY::TOF_RIGHT_OFFSET_SETUP, data);
                        resPonseJigData(JIG_DATA_KEY::TOF_RIGHT_OFFSET_SETUP);
                        ret = true;
                    }
                }
            } else {
                setJigData(JIG_DATA_KEY::TOF_RIGHT_OFFSET_SETUP, data);
                resPonseJigData(JIG_DATA_KEY::TOF_RIGHT_OFFSET_SETUP);
            }
        }
    } else {
        if (!(cmd == 0x01) && state == 0x20 && mtof_calib_file_generated_) ret = true;
    }
    
    return ret;
}


/*
    공기청정 Fan 구동 판단 로직 (Send On/Off Flag To SoC)

    [Fan On 조건]
    AP 보드 온도 73도 이상 30초 유지 시
    or
    배터리 온도 43도 이상

    [Fan Off 조건]
    AP 보드 온도 70 미만
    and
    배터리 온도 40 미만
*/
void UdpCommunication::resSocTemperatureNotice()
{
    static bool bTurnOnFan = false;
    static bool prev_bTurnOnFan = false;
    static int apOverheatCnt = 0;

    int numOverheat = 0;
    int numCool = 0;

    bool bCharging = socData.dockingInfo.status & 0X70;

    // SW temperature
    temp_ap_soc = read_temperature_from_file("/sys/class/thermal/thermal_zone0/temp");             // soc-thermal
    temp_ap_bigcore0 = read_temperature_from_file("/sys/class/thermal/thermal_zone1/temp");        // bigcore0-thermal
    temp_ap_bigcore1 = read_temperature_from_file("/sys/class/thermal/thermal_zone2/temp");        // bigcore1-thermal
    temp_ap_littlecore = read_temperature_from_file("/sys/class/thermal/thermal_zone3/temp");      // littlecore-thermal
    temp_ap_center = read_temperature_from_file("/sys/class/thermal/thermal_zone4/temp");          // center-thermal
    temp_ap_gpu = read_temperature_from_file("/sys/class/thermal/thermal_zone5/temp");             // gpu-thermal
    temp_ap_npu = read_temperature_from_file("/sys/class/thermal/thermal_zone6/temp");             // npu-thermal

    everybot_custom_msgs::msg::ApTemperature ap_temp_msg;
    ap_temp_msg.ap = temp_ap_soc;
    ap_temp_msg.bigcore0 = temp_ap_bigcore0;
    ap_temp_msg.bigcore1 = temp_ap_bigcore1;
    ap_temp_msg.littlecore = temp_ap_littlecore;
    ap_temp_msg.center = temp_ap_center;
    ap_temp_msg.gpu = temp_ap_gpu;
    ap_temp_msg.npu = temp_ap_npu;
    ap_temperature_pub_->publish(ap_temp_msg);


    if (!bTurnOnFan) {
        numOverheat = (temp_ap_soc >= fan_control_ap_overheat_on_th)
                    + (temp_ap_bigcore0 >= fan_control_ap_overheat_on_th)
                    + (temp_ap_bigcore1 >= fan_control_ap_overheat_on_th)
                    + (temp_ap_littlecore >= fan_control_ap_overheat_on_th)
                    + (temp_ap_center >= fan_control_ap_overheat_on_th)
                    + (temp_ap_gpu >= fan_control_ap_overheat_on_th)
                    + (temp_ap_npu >= fan_control_ap_overheat_on_th);

        if (numOverheat >= 1) {
            apOverheatCnt++;
        } else if (apOverheatCnt > 0) {
            apOverheatCnt--;
        }
    } else {
        numCool = (temp_ap_soc < fan_control_ap_overheat_off_th)
                + (temp_ap_bigcore0 < fan_control_ap_overheat_off_th)
                + (temp_ap_bigcore1 < fan_control_ap_overheat_off_th)
                + (temp_ap_littlecore < fan_control_ap_overheat_off_th)
                + (temp_ap_center < fan_control_ap_overheat_off_th)
                + (temp_ap_gpu < fan_control_ap_overheat_off_th)
                + (temp_ap_npu < fan_control_ap_overheat_off_th);
    }

    if ((bCharging && (fan_control_battery_overheat_on_th <= socData.battInfo.temperature1 || fan_control_battery_overheat_on_th <= socData.battInfo.temperature2))
         || apOverheatCnt >= fan_control_ap_overheat_duration)
    {
        bTurnOnFan = true;
    }

    if ((!bCharging || (fan_control_battery_overheat_off_th > socData.battInfo.temperature1 && fan_control_battery_overheat_off_th > socData.battInfo.temperature2))
        && numCool == 7) // 모든 AP 온도가 70도 미만이어야 fan OFF
    {
        bTurnOnFan = false;
    }

    if (static_cast<ROBOT_STATE>(socData.robotStatus) == ROBOT_STATE::AUTO_MAPPING) {
        bTurnOnFan = true;
    }

    if (prev_bTurnOnFan != bTurnOnFan) {
        /** 프로토콜 형식 (string):
         *
            "Notification": {
                "code": 1,                  // int
                "description": "on,0"       // string
            }
            "Notification": {
                "code": 1,                  // int
                "description": "off"        // string
            }
         *
            0: 취침 / 1: 약풍 / 2: 중풍 / 3: 강풍 / 4: 터보
         *
         */
        std::string description;
        if (bTurnOnFan) {
            if (static_cast<ROBOT_STATE>(socData.robotStatus) == ROBOT_STATE::AUTO_MAPPING) {
                description = "on," + std::to_string(fan_control_power_auto_mapping);
            } else {
                if (fan_control_power_ap_overheat == fan_control_power_battery_overheat) {
                    description = "on," + std::to_string(fan_control_power_battery_overheat);
                } else {
                    if (fan_control_battery_overheat_on_th <= socData.battInfo.temperature1 || fan_control_battery_overheat_on_th <= socData.battInfo.temperature2) {
                        description = "on," + std::to_string(fan_control_power_battery_overheat);
                    } else if (apOverheatCnt >= fan_control_ap_overheat_duration) {
                        description = "on," + std::to_string(fan_control_power_ap_overheat);
                    }
                }
            }
        } else {
            description = "off";
        }
        resGetNotification(0x01, description.c_str()); // 0x01 : Fan Control Notification

        if (bTurnOnFan) {
            apOverheatCnt = 0;
        }

        RCLCPP_INFO(this->get_logger(),
            "[resSocTemperatureNotice] Fan state changed to [%s] => [%s], description: [%s]",
            prev_bTurnOnFan ? "ON" : "OFF", bTurnOnFan ? "ON" : "OFF", description.c_str()
        );

        RCLCPP_INFO(this->get_logger(),
            "[resSocTemperatureNotice] soc:%.1f, big0:%.1f, big1:%.1f, little:%.1f, center:%.1f, gpu:%.1f, npu:%.1f, battery1:%d, battery2:%d",
            temp_ap_soc, temp_ap_bigcore0, temp_ap_bigcore1, temp_ap_littlecore, temp_ap_center, temp_ap_gpu, temp_ap_npu,
            socData.battInfo.temperature1, socData.battInfo.temperature2
        );
    }

    prev_bTurnOnFan = bTurnOnFan;

    if (apOverheatCnt > 86400) apOverheatCnt = 0; // 하루 이상 지속될 경우
}

void UdpCommunication::enableMapSub()
{
    if(!map_sub){
        map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 10, std::bind(&UdpCommunication::mapCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "map_sub created");
    }else{
        RCLCPP_INFO(this->get_logger(), "map_sub already exist");
    }
}
void UdpCommunication::disableMapSub()
{
    if(map_sub){
        map_sub.reset();
        RCLCPP_INFO(this->get_logger(), "reset-map_sub");    
    }else{
        RCLCPP_INFO(this->get_logger(), "map_sub allready reset ");
    }
}

void UdpCommunication::enable_amclposeSub()
{
    disableSlamposeSub();
    if(!amcl_pose_sub){
        amcl_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose",10,std::bind(&UdpCommunication::amclCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "amcl_pose_sub created");
    }else{
        RCLCPP_INFO(this->get_logger(), "amcl_pose_sub already exist");
    }
}
void UdpCommunication::disable_amclposeSub()
{
    if(amcl_pose_sub){
        amcl_pose_sub.reset();
        RCLCPP_INFO(this->get_logger(), "reset-amcl_pose_sub");
    }else{
        RCLCPP_INFO(this->get_logger(), "amcl_pose_sub allready reset ");
    }
}

void UdpCommunication::enableSlamposeSub()
{
    disable_amclposeSub();
    if(!slam_pose_sub){
        slam_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/robot_pose",10,std::bind(&UdpCommunication::slamPoseCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "slam_pose_sub created");
    }else{
        RCLCPP_INFO(this->get_logger(), "slam_pose_sub already exist");
    }
}
void UdpCommunication::disableSlamposeSub()
{
    if(slam_pose_sub){
        slam_pose_sub.reset();
        RCLCPP_INFO(this->get_logger(), "reset-slam_pose_sub");
    }else{
        RCLCPP_INFO(this->get_logger(), "slam_pose_sub allready reset ");
    }
}

void UdpCommunication::enableLinearTargetMoving(double v, double distance)
{
    inspection_motor_Velocity = 0;//v;
    inspection_motor_Distance = 0.5;//distance;
    acceleration = 0.01;
    end_linear_target = false;
    bSetBaseOdom = false;
    publishStartOdomReset();
    if(!linear_target_timer_){
        linear_target_timer_ = this->create_wall_timer( 20ms, std::bind(&UdpCommunication::processLinearMoving, this));
        RCLCPP_WARN(this->get_logger(), "enableLinearTargetMoving");
    }else{
        RCLCPP_WARN(this->get_logger(), "LinearTargetMoving is already enabled!!");
    }
}

void UdpCommunication::disableLinearTargetMovoing()
{
    RCLCPP_WARN(this->get_logger(), "disableLinearTargetMovoing");
    publishVelocityCommand(0.0,0.0);
    if(linear_target_timer_){
        linear_target_timer_.reset();
        RCLCPP_WARN(this->get_logger(), "linear_target_timer_ reset");
    }else{
        RCLCPP_WARN(this->get_logger(), "linear_target_timer_ is already reset");
    }
}

void UdpCommunication::processLinearMoving()
{
    if(!bSetBaseOdom && isValidateResetOdom()){
        base_odom = odom;
        bSetBaseOdom = true;
        publishClearOdomReset();
        //resSocMovingInfo();
        RCLCPP_INFO(this->get_logger(), "Set Base Odom : %.2f, %.2f, %.2f", base_odom.x, base_odom.y, base_odom.theta);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    if(bSetBaseOdom && !end_linear_target)
    {
        double distance = getDistance(base_odom,odom);
        if(distance >= inspection_motor_Distance){
            inspection_motor_Velocity = 0.0;
            end_linear_target = true;
        }else if(distance >= 0.4){
            if(inspection_motor_Velocity > 0.1){
                inspection_motor_Velocity-=acceleration;
            }
        }else{
            inspection_motor_Velocity+=acceleration;
        }
        if(inspection_motor_Velocity > 0.1){
            inspection_motor_Velocity = 0.1;
        }else if(inspection_motor_Velocity < 0.0){
            inspection_motor_Velocity = 0.0;
        }

        RCLCPP_WARN(this->get_logger(), "moving 0.5m over velocity : %.2f, distance : %.2f , base X : %.2f, Y : %.2f, current X : %.2f, Y : %.2f ",inspection_motor_Velocity,distance,base_odom.x,base_odom.y,odom.x,odom.y);
        publishVelocityCommand(inspection_motor_Velocity,0.0);
    }
}

void UdpCommunication::enableJigMode(UDP_COMMUNICATION mode) 
{   
    RCLCPP_INFO(this->get_logger(), "enableJigMode");
    enableSensorTimer(mode);

    if(!imu_calib_sub_){
        imu_calib_sub_ = this->create_subscription<everybot_custom_msgs::msg::ImuCalibration>("/jig_response_imu_calibration", 100, std::bind(&UdpCommunication::imuCalibrationCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "imu_calib_sub_ created");
    }else{
        RCLCPP_INFO(this->get_logger(), "imu_calib_sub_ already exist");
    }
    if(!jig_request_motor_pub_){
        jig_request_motor_pub_ = this->create_publisher<everybot_custom_msgs::msg::RpmControl>("jig_request_motor", 10);
        RCLCPP_INFO(this->get_logger(), "jig_request_motor_pub_ created");
    }else{
        RCLCPP_INFO(this->get_logger(), "jig_request_motor_pub_ already exist");
    }
    if(!jig_request_battery_pub_){
        jig_request_battery_pub_ = this->create_publisher<std_msgs::msg::UInt8>("jig_request_battery", 10);
        RCLCPP_INFO(this->get_logger(), "jig_request_battery_pub_ created");
    }else{
        RCLCPP_INFO(this->get_logger(), "jig_request_battery_pub_ already exist");
    }
    if(!jig_request_imu_pub_){
        jig_request_imu_pub_ = this->create_publisher<std_msgs::msg::UInt8>("jig_request_imu", 10);
        RCLCPP_INFO(this->get_logger(), "jig_request_imu_pub_ created");
    }else{
        RCLCPP_INFO(this->get_logger(), "jig_request_imu_pub_ already exist");
    }
}
void UdpCommunication::disableJigMode()
{
    RCLCPP_INFO(this->get_logger(), "disableJigMode");
    if(imu_calib_sub_){
        imu_calib_sub_.reset();
        RCLCPP_INFO(this->get_logger(), "imu_calib_sub_ reset");
    }else{
        RCLCPP_INFO(this->get_logger(), "imu_calib_sub_ is already reset");
    }
    if(jig_request_motor_pub_){
        jig_request_motor_pub_.reset();
        RCLCPP_INFO(this->get_logger(), "jig_request_motor_pub_ reset");
    }else{
        RCLCPP_INFO(this->get_logger(), "jig_request_motor_pub_ is already reset");
    }
    if(jig_request_battery_pub_){
        jig_request_battery_pub_.reset();
        RCLCPP_INFO(this->get_logger(), "jig_request_battery_pub_ reset");
    }else{
        RCLCPP_INFO(this->get_logger(), "jig_request_battery_pub_ is already reset");
    }
    if(jig_request_imu_pub_){
        jig_request_imu_pub_.reset();
        RCLCPP_INFO(this->get_logger(), "jig_request_imu_pub_ reset");
    }else{
        RCLCPP_INFO(this->get_logger(), "jig_request_imu_pub_ is already reset");
    }
    disableSensorTimer();
}

void UdpCommunication::setCommnicationMode(UDP_COMMUNICATION set)
{
    if(communicationMode != set){
        if(set == UDP_COMMUNICATION::NORMAL){
            disableJigMode();
            publishInpectionStop();//block off state_manager
        }else{
            enableJigMode(set);
            publishInpectionStart(); //block state_manager
        }
        RCLCPP_INFO(this->get_logger(), "setCommnicationMode prev[%s] next[%s]", enumToString(communicationMode).c_str(), enumToString(set).c_str());
    }
    communicationMode = set;
}

UDP_COMMUNICATION UdpCommunication::getCommunicationMode()
{
    return communicationMode;
}

 void UdpCommunication::setRobotPoseValid(bool valid)
 {
    if(socData.robotPosition.valid != valid){
        RCLCPP_INFO(this->get_logger(), "setRobotPoseValid prev[%d] next[%d]", socData.robotPosition.valid, valid);
    }
    socData.robotPosition.valid = valid;
 }
void UdpCommunication::setSocRobotPoseData(bool valid, double x, double y, double theta)
{
    //std::lock_guard<std::mutex> lock(robot_pose_mutex_);
    setRobotPoseValid(valid);
    socData.robotPosition.x = x;
    socData.robotPosition.y = y;
    socData.robotPosition.theta = theta; 
}

void UdpCommunication::setSocFrontLidarData(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    ROBOT_STATE convert_state = static_cast<ROBOT_STATE>(socData.robotStatus);
    if(convert_state != ROBOT_STATE::ONSTATION && msg->ranges.size() <= 0){
        RCLCPP_INFO(this->get_logger(), "Front ScanData is Empty!");
    }else{
        std::vector<int> dist_vec;
        generateFrontBackScan(msg,dist_vec);
        socData.lidarInfo.front_distance = getMinDistanceFromLidarSensor(dist_vec);
    }
}

void UdpCommunication::setSocRearLidarData(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    ROBOT_STATE convert_state = static_cast<ROBOT_STATE>(socData.robotStatus);
    if(convert_state != ROBOT_STATE::ONSTATION && msg->ranges.size() <= 0){
        RCLCPP_INFO(this->get_logger(), "Rear ScanData is Empty!");
    }else{
        std::vector<int> dist_vec;
        generateFrontBackScan(msg,dist_vec);
        socData.lidarInfo.rear_distance = getMinDistanceFromLidarSensor(dist_vec);
    }
}

void UdpCommunication::setSocWheelMotorData(const everybot_custom_msgs::msg::MotorStatus::SharedPtr msg)
{
    //std::lock_guard<std::mutex> lock(wheel_motor_mutex_);
    socData.motorInfo.mode = msg->motor_mode;
    socData.motorInfo.left.current = msg->left_motor_current;
    socData.motorInfo.left.encoder = msg->left_motor_encoder;
    socData.motorInfo.left.rpm = msg->left_motor_rpm;
    socData.motorInfo.left.status = msg->left_motor_status;
    socData.motorInfo.left.type = msg->left_motor_type;
    socData.motorInfo.left.tempterature = msg->left_motor_temperature;
    socData.motorInfo.right.current = msg->right_motor_current;
    socData.motorInfo.right.encoder = msg->right_motor_encoder;
    socData.motorInfo.right.rpm = msg->right_motor_rpm;
    socData.motorInfo.right.status = msg->right_motor_status;
    socData.motorInfo.right.type = msg->right_motor_type;
    socData.motorInfo.right.tempterature = msg->right_motor_temperature;
}

void UdpCommunication::setSocIMUData(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // 1) Orientation → Roll/Pitch/Yaw (rad)
    //std::lock_guard<std::mutex> lock(imu_mutex_);
    tf2::Quaternion q;
    q.setX(msg->orientation.x);
    q.setY(msg->orientation.y);
    q.setZ(msg->orientation.z);
    q.setW(msg->orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    socData.imuData.roll  = roll;
    socData.imuData.pitch = pitch;
    socData.imuData.yaw   = yaw;

    // 2) Angular velocity → wx, wy, wz (rad/s)
    // socData.imuData.wx = msg->angular_velocity.x;
    // socData.imuData.wy = msg->angular_velocity.y;
    // socData.imuData.wz = msg->angular_velocity.z;

    // 3) Linear acceleration → ax, ay, az (m/s²)
    socData.imuData.ax = msg->linear_acceleration.x;
    socData.imuData.ay = msg->linear_acceleration.y;
    socData.imuData.az = msg->linear_acceleration.z;
}

void UdpCommunication::setSocCameraData(const everybot_custom_msgs::msg::CameraDataArray::SharedPtr msg)
{
    //to-do : data sturture fix : pose
    //std::lock_guard<std::mutex> lock(camera_mutex_);
    uint8_t num = msg->num;
    if(num > 0){

        ObjectDataV2 temp;
        socData.cameraInfo.num = num;
        socData.cameraInfo.data.clear();
        for(int i = 0; i < socData.cameraInfo.num; i++){
            temp.class_id = msg->data_array[i].id;
            temp.confidence = msg->data_array[i].score;
            temp.height = msg->data_array[i].height*1000;
            temp.width = msg->data_array[i].width*1000;
            temp.x = msg->data_array[i].x;
            temp.y = msg->data_array[i].y;
            temp.theta = msg->data_array[i].theta;
            temp.distance = msg->data_array[i].distance*1000;
            socData.cameraInfo.data.push_back(temp);
        }
    }
}

#if USE_LINELASER_SENSOR > 0
void UdpCommunication::setSocLineLaserData(const everybot_custom_msgs::msg::LineLaserDataArray::SharedPtr msg)
{
    uint8_t num = msg->num;
    
    if(num  > 0){

        LLDataV2 temp;
        socData.lineLaserInfo.num = num;
        socData.lineLaserInfo.data.clear();
        
        for(int i = 0; i < socData.lineLaserInfo.num; i++){
            temp.direction = msg->data_array[i].direction;
            temp.distance = msg->data_array[i].distance*1000;
            temp.height = msg->data_array[i].height*1000;
            temp.x = msg->data_array[i].x;
            temp.y = msg->data_array[i].y;
            temp.theta = msg->data_array[i].theta;
            socData.lineLaserInfo.data.push_back(temp);
        }
    }
}
#endif

void UdpCommunication::setTofStatus(uint8_t top, uint8_t bottom_left, uint8_t bottom_right)
{
    if(socData.tofInfo.top.status != top){
        if(top == 0x0){
            RCLCPP_INFO(this->get_logger(), "TOP-TOF ON");
        }else if(top == 0xF){
            RCLCPP_INFO(this->get_logger(), "TOP-TOF ERROR");
        }
    }
    if(socData.tofInfo.left_bottom.status != bottom_left){
        if(bottom_left == 0x0){
            RCLCPP_INFO(this->get_logger(), "BOTTOMLEFT-TOF ON");
        }else if(bottom_left == 0x1){
            RCLCPP_INFO(this->get_logger(), "BOTTOMLEFT-TOF OFF");
        }else if(bottom_left == 0xF){
            RCLCPP_INFO(this->get_logger(), "BOTTOMLEFT-TOF ERROR");
        }
    }
    if(socData.tofInfo.right_bottom.status != bottom_right){
        if(bottom_right == 0x0){
            RCLCPP_INFO(this->get_logger(), "BOTTOMRIGHT-TOF ON");
        }else if(bottom_right == 0x1){
            RCLCPP_INFO(this->get_logger(), "BOTTOMRIGHT-TOF OFF");
        }else if(bottom_right == 0xF){
            RCLCPP_INFO(this->get_logger(), "BOTTOMRIGHT-TOF ERROR");
        }
    }
    socData.tofInfo.top.status = top;
    socData.tofInfo.left_bottom.status = bottom_left;
    socData.tofInfo.right_bottom.status = bottom_right;
}
void UdpCommunication::setSocTofData(const everybot_custom_msgs::msg::TofData::SharedPtr msg)
{
    //std::lock_guard<std::mutex> lock(tof_mutex_);
    uint8_t bottom_left = getLowerBits(msg->bot_status);
    uint8_t bottom_right = getUpperBits(msg->bot_status);
    setTofStatus(msg->top_status,bottom_left,bottom_right);
    socData.tofInfo.top.distance = msg->top*1000;
    socData.tofInfo.left_bottom.status = bottom_left;
    socData.tofInfo.right_bottom.status = bottom_right;

    int left_msg_size = static_cast<int>(msg->bot_left.size());
    int right_msg_size = static_cast<int>(msg->bot_right.size());

    if ((mtof_left_target_indices_size == 16) && (mtof_right_target_indices_size == 16)) {
        if ((left_msg_size == 16) && (right_msg_size == 16)) {
            for(int i = 0; i < 16; i++) {
                int left_idx = mtof_left_target_indices[i];
                int right_idx = mtof_right_target_indices[i];
                socData.tofInfo.left_bottom.data[left_idx] = msg->bot_left[i]*1000;
                socData.tofInfo.right_bottom.data[right_idx] = msg->bot_right[i]*1000;
            }
        } else {
            RCLCPP_INFO(this->get_logger(),
                "[setSocTofData] Invalid MTOF Msg size: left_size=%d, right_size=%d",
                left_msg_size, right_msg_size
            );
        }
    } else {
        RCLCPP_INFO(this->get_logger(),
            "[setSocTofData] Invalid MTOF target index size: left_size=%d, right_size=%d",
            mtof_left_target_indices_size, mtof_right_target_indices_size
        );
    }
}

void UdpCommunication::setSocCliffLiftData(const std_msgs::msg::UInt8::SharedPtr msg)
{
    //std::lock_guard<std::mutex> lock(bottom_status_mutex_);
    socData.cliffLiftInfo.value = msg->data;
}

void UdpCommunication::setSocBottomIrData(const everybot_custom_msgs::msg::BottomIrData::SharedPtr msg)
{
    //std::lock_guard<std::mutex> lock(bottom_ir_mutex_);
    socData.bottomIrData.front_center = msg->adc_ff;
    socData.bottomIrData.front_left = msg->adc_fl;
    socData.bottomIrData.back_left = msg->adc_bl;
    socData.bottomIrData.back_center = msg->adc_bb;
    socData.bottomIrData.back_right = msg->adc_br;
    socData.bottomIrData.front_right = msg->adc_fr;
}

void UdpCommunication::setSocDockReceiverData(const everybot_custom_msgs::msg::StationData::SharedPtr msg)
{
    //std::lock_guard<std::mutex> lock(charger_data_mutex_);
    if(bInspectionMotor){
        if(msg->docking_status & 0x80){
            publishDockingCommand(false);
        }
    }
    
    socData.dockingInfo.status = msg->docking_status;
    socData.dockingInfo.receiver.value = msg->receiver_status;
    leftTerminalTemp = msg->left_terminal;
    rightTerminalTemp = msg->right_terminal;
}

void UdpCommunication::setSocBatteryData(const everybot_custom_msgs::msg::BatteryStatus::SharedPtr msg)
{
    //std::lock_guard<std::mutex> lock(battery_mutex_);
    socData.battInfo.cell_voltage1 = msg->cell_voltage1;
    socData.battInfo.cell_voltage2 = msg->cell_voltage2;
    socData.battInfo.cell_voltage3 = msg->cell_voltage3;
    socData.battInfo.cell_voltage4 = msg->cell_voltage4;
    socData.battInfo.cell_voltage5 = msg->cell_voltage5;

    socData.battInfo.current = (static_cast<double>(msg->battery_current) * 10) / 10.0;
    socData.battInfo.voltage = msg->battery_voltage;
    socData.battInfo.percent = msg->battery_percent;
    socData.battInfo.number_of_cycles = msg->number_of_cycles;
    socData.battInfo.temperature1 = msg->battery_temperature1;
    socData.battInfo.temperature2 = msg->battery_temperature2;
    socData.battInfo.manufacturer = msg->battery_manufacturer;
    socData.battInfo.remaining_capacity = msg->remaining_capacity;
    socData.battInfo.total_capacity = msg->total_capacity;
    socData.battInfo.design_capacity = msg->design_capacity;

    socData.battInfo.charge_status = msg->charge_status;
    socData.battInfo.charging_mode = msg->charging_mode;

    socData.battInfo.shipping_mode = msg->shipping_mode;

    socData.battInfo.precharge_state = msg->precharge_state;
    socData.battInfo.charge_mode = msg->charge_mode;
    socData.battInfo.fet_state = msg->fet_state;

#if false
// Status string arrays
const char *precharge_str[]    = { "Not charging", "Precharge ON", "Precharge OFF", "Reserved" };
const char *charge_mode_str[]  = { "Not charging", "Constant Current (CC)", "Constant Voltage (CV)", "Reserved" };
const char *fet_str[]          = { "-", "FET OFF", "FET ON", "Reserved" };

// Log battery status
RCLCPP_INFO(this->get_logger(),
    "Battery Status - Shipping: %u | Precharge: %s | Charge Mode: %s | FET: %s",
    msg->shipping_mode,
    precharge_str[msg->precharge_state],
    charge_mode_str[msg->charge_mode],
    fet_str[msg->fet_state]
);
#endif
}

void UdpCommunication::generateVersion()
{
    socData.version.timestamp = this->now().seconds();
    socData.version.total_ver = (socData.version.sw_ver + ":" + socData.version.mcu_ver + ":" + socData.version.ai_ver);
    RCLCPP_INFO(this->get_logger(), "generateVersion SW VER : %s", socData.version.total_ver.c_str());
    //Set Total Version for the purpose of OTA
    APISetTotalVersion(socData.version.total_ver.c_str());
}

void UdpCommunication::setSocTargetPosition(double x, double y, double theta)
{
    socData.targetPosition.x = x;
    socData.targetPosition.y = y;
    socData.targetPosition.theta = theta;
    RCLCPP_INFO(this->get_logger(), "setSocTargetPosition (%.1f, %.1f, %.1f(deg))", x, y, RAD2DEG(theta));
}

void UdpCommunication::setSocMapData(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    socData.mapInfo.height = msg->info.height;
    socData.mapInfo.width = msg->info.width;
    socData.mapInfo.origin_x = msg->info.origin.position.x;
    socData.mapInfo.origin_y = msg->info.origin.position.y;
    socData.mapInfo.resolution = msg->info.resolution; //로봇을 껐다 켜면 0이됨. yaml 파일에서 읽어와야함 
    socData.mapInfo.map_data = mapDataTypeConvert(msg->data,socData.mapInfo.height,socData.mapInfo.width);
    
	//hjkim : logging for first map data
    if(!socData.mapInfo.bReceived){
        RCLCPP_INFO(this->get_logger(), "[setSocMapData] first map received with height : %u, width : %u, size : %zu",socData.mapInfo.height,socData.mapInfo.width,socData.mapInfo.map_data.size());
    }
    socData.mapInfo.bReceived = true;
    temp_mapsize = socData.mapInfo.map_data.size();
}

void UdpCommunication::setSocRobotVelocity(double v, double w)
{
    socData.velocity.v = v;
    socData.velocity.w = w;
}

void UdpCommunication::setSocRobotState(uint8_t state, uint8_t status)
{
    ROBOT_STATE convert_state = static_cast<ROBOT_STATE>(state);
    ROBOT_STATE prev_state = static_cast<ROBOT_STATE>(socData.robotStatus);
    ROBOT_STATUS convert_status = static_cast<ROBOT_STATUS>(status);
    ROBOT_STATUS prev_status = static_cast<ROBOT_STATUS>(socData.actionStatus);
    
    bool print_msg = false;

    if(socData.robotStatus != state){
        if(bInspectionMode && bInspectionMotor && convert_state == ROBOT_STATE::ONSTATION){
            publishDockingCommand(false);
            bInspectionMotor = false;
            RCLCPP_INFO(this->get_logger(), "inspection motor Complete");
        }
#if 0 //hjkim : 맵핑 시작 시 가상벽 초기화 --> 매핑 완료 후 맵 저장 시 초기화 하도록 수정
        if(convert_state == ROBOT_STATE::IDLE || convert_state == ROBOT_STATE::ERROR){
            // publishVelocityCommand(0.0,0.0);
        }else if(convert_state == ROBOT_STATE::AUTO_MAPPING || convert_state == ROBOT_STATE::MANUAL_MAPPING){
            publishClearVirtualWall();
        }
#endif
        print_msg = true;
        //RCLCPP_INFO(this->get_logger(), "setSocRobotState State prev[%s] -->new[%s] status[%s]",enumToString(prev_state).c_str(),enumToString(convert_state).c_str(),enumToString(convert_status).c_str());
    }
    if(socData.actionStatus != status){
        //bSendActionState = true;
        print_msg = true;
        //RCLCPP_INFO(this->get_logger(), "setSocRobotState Status state[%s] prev[%s]-->new[%s]",enumToString(convert_state).c_str(),,enumToString(convert_status).c_str());
    }

    if(print_msg){
        RCLCPP_INFO(this->get_logger(), "SocRobot State: Set [%s(%d) , %s(%d)] -> [%s(%d) , %s(%d)]"
        , enumToString(prev_state).c_str(), socData.robotStatus, enumToString(prev_status).c_str(), socData.actionStatus
        , enumToString(convert_state).c_str(), state, enumToString(convert_status).c_str(), status);
    }

    socData.robotStatus = state;
    socData.actionStatus = status;
}

void UdpCommunication::setSocMovingState(uint8_t status,uint8_t fail_reason)
{
    NAVI_STATE prev_status = static_cast<NAVI_STATE>(socData.movingStatus);
    NAVI_STATE convert_status = static_cast<NAVI_STATE>(status);
    NAVI_FAIL_REASON convert_reason = static_cast<NAVI_FAIL_REASON>(fail_reason);
    if(socData.movingStatus != status){
        RCLCPP_INFO(this->get_logger(), "setSocMovingState Status prev[%s]-->new[%s]",enumToString(prev_status).c_str(),enumToString(convert_status).c_str());
    }
    socData.movingStatus = status;
    setNaviFailReason(convert_reason,fail_reason);
}

void UdpCommunication::setSocError(ErrorList_t error)
{
    std::lock_guard<std::mutex> lock(soc_error_list_mutex_);
    auto it = std::find_if(socData.errorList.begin(), socData.errorList.end(),
        [&](const ErrorList_t& e) {
            return e.errorCode == error.errorCode;
        });

    if (it != socData.errorList.end() && error.resolved == 2) {
        socData.errorList.erase(it);
        RCLCPP_INFO(this->get_logger(), "[setSocError] sameError clear..erase this error : %d (1:occured, 2:released), error_code = %s", error.resolved, error.errorCode.c_str());    
    } else {
        // 새 항목 추가
        socData.errorList.push_back(error);
        RCLCPP_INFO(this->get_logger(), "[setSocError] add: %d (1:occured, 2:released), error_code = %s", error.resolved, error.errorCode.c_str());
    }
}

void UdpCommunication::setNodeState(uint8_t set)
{
    NODE_STATUS prev_state = static_cast<NODE_STATUS>(nodeState);
    NODE_STATUS convert_state = static_cast<NODE_STATUS>(set);
    if(nodeState != convert_state){
        if(convert_state == NODE_STATUS::MANUAL_MAPPING || convert_state == NODE_STATUS::AUTO_MAPPING){
            enableMapSub();
        }else{
            disableMapSub();
        }
        if(convert_state == NODE_STATUS::AUTO_MAPPING || convert_state == NODE_STATUS::NAVI || convert_state == NODE_STATUS::FT_NAVI){
            enable_amclposeSub();
        }else if(convert_state == NODE_STATUS::MANUAL_MAPPING){
            enableSlamposeSub();
        }else{
            //setRobotPoseValid(false);
            disable_amclposeSub();
            disableSlamposeSub();
        }
        RCLCPP_INFO(this->get_logger(), "setNodeState prev[%s]-->new[%s]",enumToString(prev_state).c_str(),enumToString(convert_state).c_str());
    }
    nodeState = convert_state;
}

void UdpCommunication::setNaviFailReason(NAVI_FAIL_REASON set, uint8_t reason)
{
    NAVI_FAIL_REASON prev_state = static_cast<NAVI_FAIL_REASON>(movefail_reason);
    NAVI_FAIL_REASON convert_state = static_cast<NAVI_FAIL_REASON>(set);
    if(movefail_reason != set){
        RCLCPP_INFO(this->get_logger(), "setNaviFailReason prev[%s]-->new[%s]",enumToString(prev_state).c_str(),enumToString(convert_state).c_str());
    }
    movefail_reason = set;
}

void UdpCommunication::setJigData(JIG_DATA_KEY key, const std::vector<uint8_t>& data) {
    jigData[key] = data; // key에 해당하는 데이터를 저장
}

// 데이터를 조회하는 함수
std::vector<uint8_t> UdpCommunication::getJigData(JIG_DATA_KEY key)
{
    auto it = jigData.find(key);
    if (it != jigData.end()) {
        return it->second; // key가 존재하면 해당 데이터를 반환
    }

    return {}; // key가 존재하지 않으면 빈 벡터 반환
}

void UdpCommunication::nodeStateCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    setNodeState(msg->data);
}

void UdpCommunication::robotStateCallback(const everybot_custom_msgs::msg::RobotState::SharedPtr msg)
{
    mainState newstate;
    newstate.robotStatus = msg->state;
    newstate.actionStatus = msg->status;
    enqueueMainState(newstate);
    //resSocRobotState();
    //resSocRobotStatus();
}

void UdpCommunication::aiTemperatureCallback(const everybot_custom_msgs::msg::AiTemperature::SharedPtr msg)
{
    aiTemp.npu = msg->npu;
    aiTemp.gpu = msg->gpu;
    aiTemp.center = msg->center;
    aiTemp.soc = msg->soc;
    aiTemp.bigcore0 = msg->bigcore0;
    aiTemp.bigcore1 = msg->bigcore1;
    aiTemp.littlecore = msg->littlecore;
}

void UdpCommunication::calibCompleteStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg) {
        if (msg->data) {
            mtof_calib_file_generated_ = true;
        } else {
            mtof_calib_file_generated_ = false;
        }
        RCLCPP_INFO(this->get_logger(), "[calibCompleteStateCallback] M-ToF Calibration Result received from A1_perception: %d", msg->data);
    }
}

void UdpCommunication::maneuverStateStrCallback(const std_msgs::msg::String::SharedPtr msg)
{
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[Debug PoPo] RobotInfo : %s", msg->data.c_str());
    popo_maneuver_state_str_ = msg->data;
}

void UdpCommunication::movingStateCallback(const everybot_custom_msgs::msg::NaviState::SharedPtr msg)
{
    movingState newState;
    newState.movingStatus = msg->state;
    newState.failReason = msg->fail_reason;
    enqueueMovingState(newState);
    //resSocMovingInfo();
}

void UdpCommunication::batteryCallback(const everybot_custom_msgs::msg::BatteryStatus::SharedPtr msg)
{
    if(getCommunicationMode() == UDP_COMMUNICATION::AMR_JIG_MODE){
        std::vector<uint8_t> data = jigDataConvertBattery(msg);
        setJigData(JIG_DATA_KEY::BATTERY,data);
        setJigData(JIG_DATA_KEY::BATTERY_SHIPPING_MODE,{static_cast<uint8_t>(msg->shipping_mode)});
    } else if(getCommunicationMode() == UDP_COMMUNICATION::AP_JIG_MODE){
        return ;
    } else {
        setSocBatteryData(msg);
    }
}

void UdpCommunication::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    setSocIMUData(msg);
}

void UdpCommunication::motorCallback(const everybot_custom_msgs::msg::MotorStatus::SharedPtr msg)
{
    if(getCommunicationMode() == UDP_COMMUNICATION::AMR_JIG_MODE){
        std::vector<uint8_t> data = jigDataConvertWheelMotor(msg);
        setJigData(JIG_DATA_KEY::WHEEL_MOTOR,data);
    } else if(getCommunicationMode() == UDP_COMMUNICATION::AP_JIG_MODE){
        return ;
    } else {
        setSocWheelMotorData(msg);
    }
}

void UdpCommunication::slamPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    const auto& pose = msg->pose.pose;
    double x = static_cast<int>(pose.position.x * 1000000 + 0.5) / 1000000.0;
    double y = static_cast<int>(pose.position.y * 1000000 + 0.5) / 1000000.0;
    double theta = quaternion_to_euler(pose.orientation);
    if(!bInspectionMode){
        setSocRobotPoseData(true,x,y,theta);
    }
}

void UdpCommunication::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom.x = msg->pose.pose.position.x;
    odom.y = msg->pose.pose.position.y;
    odom.theta = quaternion_to_euler(msg->pose.pose.orientation);
     if(!bReceiveOdom){
        RCLCPP_INFO(this->get_logger(), "receive check odom OK x,y,theta[%.2f, %.2f, %.2f]", odom.x, odom.y, odom.theta);
        bReceiveOdom = true;
    }
    if(bInspectionMode){
        setSocRobotPoseData(true,odom.x,odom.y,odom.theta);
    }
}

void UdpCommunication::amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    const auto& pose = msg->pose.pose;
    double x = static_cast<int>(pose.position.x * 1000000 + 0.5) / 1000000.0;
    double y = static_cast<int>(pose.position.y * 1000000 + 0.5) / 1000000.0;
    double theta = quaternion_to_euler(pose.orientation);
    if(!bInspectionMode){
        setSocRobotPoseData(true,x,y,theta);
    }
}

void UdpCommunication::robotSpeedCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    setSocRobotVelocity(msg->linear.x,msg->angular.z);
}

void UdpCommunication::fw_version_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if(socData.version.mcu_ver != msg->data){
        RCLCPP_INFO(this->get_logger(), "[fw_version_callback] FW Version previous : %s, --> new: %s",socData.version.mcu_ver.c_str(), msg->data.c_str());
    }
    socData.version.mcu_ver = msg->data;
}

void UdpCommunication::ai_version_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if(socData.version.ai_ver != msg->data){
        RCLCPP_INFO(this->get_logger(), "[ai_version_callback] AI Version previous : %s, --> new: %s",socData.version.ai_ver.c_str(), msg->data.c_str());
    }
    socData.version.ai_ver = msg->data;
}
	

void UdpCommunication::generateFrontBackScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, std::vector<int>& vecDist)
{
    for (std::size_t i = 0; i < msg->ranges.size(); i++) {
        float angle = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
        if (angle >= FRONT_REAR_MIN_ANGLE && angle <= FRONT_REAR_MAX_ANGLE) {
            int dist = static_cast<int>(msg->ranges[i] * 1000);
            if (std::abs(dist) > 1) {  // abs() 대신 std::abs() 사용
                vecDist.push_back(dist);
            }
        }
    }
}

void UdpCommunication::lidarFrontCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(scan_front_mutex_);

    if(getCommunicationMode() == UDP_COMMUNICATION::AMR_JIG_MODE){
        std::vector<int> vec_lidar_long, vec_lidar_short;
        splitLidarScan(msg, vec_lidar_long, vec_lidar_short);
        std::vector<uint8_t> data = jigDataConvertLidar(getMinDistanceFromLidarSensor(vec_lidar_long), getMinDistanceFromLidarSensor(vec_lidar_short));
        setJigData(JIG_DATA_KEY::FRONT_LIDAR,data);
        bRequestFrontLidarDist = false;
    } else if(getCommunicationMode() == UDP_COMMUNICATION::AP_JIG_MODE){
        apJigFrontLaserData = msg;
        // RCLCPP_INFO(this->get_logger(), "ranges[0], ranges[100] : '%f', '%f'", msg->ranges[0],
        //             msg->ranges[100]);
    }else{
        setSocFrontLidarData(msg);
    }
}

void UdpCommunication::lidarBackCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(scan_back_mutex_);

    if(getCommunicationMode() == UDP_COMMUNICATION::AMR_JIG_MODE){
        std::vector<int> vec_lidar_long, vec_lidar_short;
        splitLidarScan(msg, vec_lidar_long, vec_lidar_short);
        std::vector<uint8_t> data = jigDataConvertLidar(getMinDistanceFromLidarSensor(vec_lidar_long), getMinDistanceFromLidarSensor(vec_lidar_short));
        setJigData(JIG_DATA_KEY::REAR_LIDAR,data);
        bRequestBackLidarDist = false;
    } else if(getCommunicationMode() == UDP_COMMUNICATION::AP_JIG_MODE){
        apJigBackLaserData = msg;
        // RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", msg->ranges[0],
        //         msg->ranges[100]);
    }else{
        setSocRearLidarData(msg);
    }
}

#if USE_LINELASER_SENSOR > 0
void UdpCommunication::lineLaserCallback(const everybot_custom_msgs::msg::LineLaserDataArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(line_laser_mutex_);
    setSocLineLaserData(msg);
}
#endif

void UdpCommunication::cameraCallback(const everybot_custom_msgs::msg::CameraDataArray::SharedPtr msg)
{
    setSocCameraData(msg);
}

void UdpCommunication::tofCallback(const everybot_custom_msgs::msg::TofData::SharedPtr msg)
{
    if(getCommunicationMode() == UDP_COMMUNICATION::AMR_JIG_MODE){
        std::vector<uint8_t> data = jigDataConvertTof(msg);
        setJigData(JIG_DATA_KEY::TOF,data);
    } else if(getCommunicationMode() == UDP_COMMUNICATION::AP_JIG_MODE){
        return ;
    }else{
        setSocTofData(msg);
    }
}

void UdpCommunication::errorListCallback(const everybot_custom_msgs::msg::ErrorListArray::SharedPtr msg)
{
    size_t array_size = msg->data_array.size();
    for (size_t i = 0; i < array_size; ++i) {  // `i`를 size_t로 변경 (경고 해결)
        const auto& error = msg->data_array[i];
        ErrorList_t error_entry;
        if(error.error_occurred){
            RCLCPP_INFO(this->get_logger(), "[errorListCallback] occurred error_code = %s",error.error_code.c_str());
            error_entry.resolved = 1;    
        }else{
            RCLCPP_INFO(this->get_logger(), "[errorListCallback] release error_code = %s",error.error_code.c_str());
            error_entry.resolved = 2;
        }
        error_entry.rank = 0;
        error_entry.errorCode = error.error_code;  // 기존 코드 유지
        setSocError(error_entry);
    }
    //resSocErrorList();
}

void UdpCommunication::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    setSocMapData(msg);
}

void UdpCommunication::bottomStatusCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    if(getCommunicationMode() == UDP_COMMUNICATION::AMR_JIG_MODE){
        return;
    } else if(getCommunicationMode() == UDP_COMMUNICATION::AP_JIG_MODE){
        return;
    }else{
        setSocCliffLiftData(msg);
    }
}

void UdpCommunication::bottomIrCallback(const everybot_custom_msgs::msg::BottomIrData::SharedPtr msg)
{
    if(getCommunicationMode() == UDP_COMMUNICATION::AMR_JIG_MODE){
        std::vector<uint8_t> data(12);
        data[0] = static_cast<uint8_t>((msg->adc_ff >> 8) & 0xFF);
        data[1] = static_cast<uint8_t>(msg->adc_ff & 0xFF);
        data[2] = static_cast<uint8_t>((msg->adc_fl >> 8) & 0xFF);
        data[3] = static_cast<uint8_t>(msg->adc_fl & 0xFF);
        data[4] = static_cast<uint8_t>((msg->adc_bl >> 8) & 0xFF);
        data[5] = static_cast<uint8_t>(msg->adc_bl & 0xFF);
        data[6] = static_cast<uint8_t>((msg->adc_bb >> 8) & 0xFF);
        data[7] = static_cast<uint8_t>(msg->adc_bb & 0xFF);
        data[8] = static_cast<uint8_t>((msg->adc_br >> 8) & 0xFF);
        data[9] = static_cast<uint8_t>(msg->adc_br & 0xFF);
        data[10] = static_cast<uint8_t>((msg->adc_fr >> 8) & 0xFF);
        data[11] = static_cast<uint8_t>(msg->adc_fr & 0xFF);
        setJigData(JIG_DATA_KEY::CLIFF_LIFT,data);
    } else if(getCommunicationMode() == UDP_COMMUNICATION::AP_JIG_MODE){
        return;
    }else{
        setSocBottomIrData(msg);
    }
}


void UdpCommunication::stationDataCallback(const everybot_custom_msgs::msg::StationData::SharedPtr msg)
{
    if(getCommunicationMode() == UDP_COMMUNICATION::AMR_JIG_MODE){
        std::vector<uint8_t> data(1);
        uint8_t receiver = getUpperBits(msg->receiver_status);
        data[0] = receiver;
        setJigData(JIG_DATA_KEY::DOCK_RECEIVER,data);
    } else if(getCommunicationMode() == UDP_COMMUNICATION::AP_JIG_MODE){
        return ;
    }else{
        setSocDockReceiverData(msg);
    }
}

void UdpCommunication::imuCalibrationCallback(const everybot_custom_msgs::msg::ImuCalibration::SharedPtr msg)
{
    //std::lock_guard<std::mutex> lock(imu_cal_mutex_);
    std::vector<uint8_t> data = jigDataConvertImuCalibration(msg);
    setJigData(JIG_DATA_KEY::IMU_CALIBRATION,data);
}

void UdpCommunication::resSocRobotPose()
{
    if(bOnStation){
        resGetPosition(station_pose.x, station_pose.y, station_pose.theta);
    }else{
        resGetPosition(socData.robotPosition.x, socData.robotPosition.y, socData.robotPosition.theta);
    }
    //RCLCPP_INFO(this->get_logger(), "resSocRobotPose (%.1f, %.1f, %.1f)", pose.x, pose.y, RAD2DEG(pose.theta));    
}

void UdpCommunication::resSocMap()
{
    ROBOT_STATE convert_state = static_cast<ROBOT_STATE>(socData.robotStatus);
    if(convert_state == ROBOT_STATE::AUTO_MAPPING || convert_state == ROBOT_STATE::MANUAL_MAPPING || nodeState == NODE_STATUS::AUTO_MAPPING || nodeState == NODE_STATUS::MANUAL_MAPPING){
        MapInfo mapInfo = getMapInfo();
        const size_t map_size = mapInfo.map_data.size();
        const size_t expected_size = static_cast<size_t>(mapInfo.width) * static_cast<size_t>(mapInfo.height);
        if(!mapInfo.bReceived){
            RCLCPP_INFO(this->get_logger(), "realtime map is not received yet [%d]",socData.mapInfo.bReceived);
        }else if(mapInfo.map_data.empty()){
            RCLCPP_INFO(this->get_logger(), "realtime map date is empty received [%d] size : %zu",socData.mapInfo.bReceived,map_size);
        }else if(map_size != expected_size){
            RCLCPP_INFO(this->get_logger(), "realtime map is not correct size, width : %u, height : %u, size : %zu, received : %d",mapInfo.width,mapInfo.height,map_size,socData.mapInfo.bReceived);
        }else{
            if(temp_mapsize != map_size){
                RCLCPP_INFO(this->get_logger(), "realtime update-map send with : %u, height : %u, resolution : %f, size : %zu, received : %d",mapInfo.width,mapInfo.height,mapInfo.resolution,map_size,socData.mapInfo.bReceived);
            }else{
                RCLCPP_INFO(this->get_logger(), "realtime map is same with last send  width : %u, height : %u, size : %zu, received : %d",mapInfo.width,mapInfo.height,map_size,socData.mapInfo.bReceived);
            }
            resGetMapDataB(mapInfo.width,mapInfo.height,mapInfo.resolution,mapInfo.origin_x,mapInfo.origin_y, mapInfo.map_data.data() ,map_size);
        }
    }else{
        bool bReadMap = false;
        int height = 0,width = 0;
        double origin_x{0.0},origin_y{0.0};
        std::vector<uint8_t> saved_map;
        if(convert_state == ROBOT_STATE::FACTORY_NAVIGATION){
            bReadMap = readPGM("/home/airbot/app_rw/factorymap/everybot_factorymap_00.pgm", width, height, saved_map, origin_x, origin_y);
            const size_t map_size = saved_map.size();
            const size_t expected_size = static_cast<size_t>(width) * static_cast<size_t>(height);
            if(bReadMap){
                 if((saved_map.empty())){
                    RCLCPP_INFO(this->get_logger(), "factory map is empty");
                }else if(map_size != expected_size){
                    RCLCPP_INFO(this->get_logger(), "factory map is not correct size, width : %u, height : %u, size : %zu",width,height,map_size);
                }else{
                    MapInfo mapInfo = getMapInfo();
                    RCLCPP_INFO(this->get_logger(), "factory map send with : %u, height : %u, resolution : %.2f, size : %zu",width,height,mapInfo.resolution,map_size);
                    resGetMapDataB(width,height,mapInfo.resolution,origin_x,origin_y, saved_map.data(),map_size);
                }
            }else{
                RCLCPP_INFO(this->get_logger(), "factory map read fail error");
            }
        }
        else{
            bReadMap = readPGM("/home/airbot/app_rw/map/everybot_map_00.pgm", width, height, saved_map, origin_x, origin_y);
            const size_t map_size = saved_map.size();
            const size_t expected_size = static_cast<size_t>(width) * static_cast<size_t>(height);
            if(bReadMap){
                if((map_size == 0)){
                    RCLCPP_INFO(this->get_logger(), "saved map is empty");
                }else if(map_size != expected_size){
                    RCLCPP_INFO(this->get_logger(), "saved map is not correct size, width : %u, height : %u, size : %zu",width,height,map_size);
                }else{
                    MapInfo mapInfo = getMapInfo();
                    RCLCPP_INFO(this->get_logger(), "saved map send with : %u, height : %u, resolution : %.2f, size : %zu",width,height,mapInfo.resolution,map_size);
                    resGetMapDataB(width,height,mapInfo.resolution,origin_x,origin_y, saved_map.data(),map_size);
                }
            }else{
                RCLCPP_INFO(this->get_logger(), "savedmap read fail error");
            }
        }
    }
}

void UdpCommunication::resSocEncMap()
{
    double theta = 0.0;
    int negate = 0;
    double occupied_thresh = 0.65;
    double free_thresh = 0.196;
    std::string originPath = "/home/airbot/app_rw/map/";
    std::string originPGMName = "everybot_map_00.pgm";
    std::string originYamlName = "everybot_map_00.yaml";

    ROBOT_STATE convert_state = static_cast<ROBOT_STATE>(socData.robotStatus);

    if(convert_state == ROBOT_STATE::AUTO_MAPPING || convert_state == ROBOT_STATE::MANUAL_MAPPING || nodeState == NODE_STATUS::AUTO_MAPPING || nodeState == NODE_STATUS::MANUAL_MAPPING)
    {
        originPath = "/home/airbot/app_rw/tempmap/"; //맵이 있는 상태에서 맵핑을 다시 시작하는 순간 저장된 맵을 전송맵(map topic을 전송하기 위해 임시로 만들어지는 맵)이 덮어버려, sync 오류가 생겨  전송맵을 tempmap 폴더에 저장하도로 수정.
        MapInfo mapInfo = getMapInfo();
        const size_t map_size = mapInfo.map_data.size();
        const size_t expected_size = static_cast<size_t>(mapInfo.width) * static_cast<size_t>(mapInfo.height);
        if(!mapInfo.bReceived){
            RCLCPP_INFO(this->get_logger(), "realtime map is not received yet [%d]",socData.mapInfo.bReceived);
        }else if(map_size == 0){
            RCLCPP_INFO(this->get_logger(), "realtime map date is empty received [%d] size : %zu",socData.mapInfo.bReceived,map_size);
        }else if(mapInfo.map_data.size() != expected_size){
            RCLCPP_INFO(this->get_logger(), "realtime map is not correct size, width : %u, height : %u, size : %zu, received : %d",mapInfo.width,mapInfo.height,map_size,socData.mapInfo.bReceived);
        }else{
            if(temp_mapsize != map_size){
                RCLCPP_INFO(this->get_logger(), "originMap send with : %u, height : %u, resolution : %.2f, size : %zu",mapInfo.width,mapInfo.height,mapInfo.resolution,map_size);
            }else{
                RCLCPP_INFO(this->get_logger(), "originMap is same size");
            }

            bool bMakepgm = APIsaveAndEncryptPGM(originPath,originPGMName,mapInfo.map_data.data(),mapInfo.width,mapInfo.height,false);
            bool bMakeyaml = APIsaveAndEncryptYAML(originPath,originYamlName,originPGMName,mapInfo.resolution,mapInfo.origin_x,mapInfo.origin_y,theta,negate,occupied_thresh,free_thresh,false);                

            if(bMakepgm && bMakeyaml){
                if(resGetOriginMapData(originPath,originYamlName,originPGMName)){
                    RCLCPP_INFO(this->get_logger(), "MapSendToSoc realtimemap encrypt success");
                }else{
                    RCLCPP_INFO(this->get_logger(), "MapSendToSoc realtimemap encrypt fail error");
                }
            }else{
                if(!bMakepgm){
                    RCLCPP_INFO(this->get_logger(), "MapSendToSoc realtimemap pgm save fail error");
                }
                if(!bMakeyaml){
                    RCLCPP_INFO(this->get_logger(), "MapSendToSoc realtimemap yaml save fail error");
                }
            }
        }
    }else
    {
        std::string inputPGMPath, outputPGMPath, inputYamlPath, outputYamlPath;
        bool bMakepgm = false;
        bool bMakeyaml = false;
        if(convert_state == ROBOT_STATE::FACTORY_NAVIGATION){
            originPath = "/home/airbot/app_rw/factorymap/";
            inputPGMPath = "/home/airbot/app_rw/factorymap/everybot_factorymap_00.pgm";
            inputYamlPath = "/home/airbot/app_rw/factorymap/everybot_factorymap_00.yaml";
            outputPGMPath = "/home/airbot/app_rw/factorymap/everybot_map_00.pgm.enc";
            outputYamlPath = "/home/airbot/app_rw/factorymap/everybot_map_00.yaml.enc";
        }else{
            inputPGMPath = "/home/airbot/app_rw/map/everybot_map_00.pgm";
            inputYamlPath = "/home/airbot/app_rw/map/everybot_map_00.yaml";
            outputPGMPath = "/home/airbot/app_rw/map/everybot_map_00.pgm.enc";
            outputYamlPath = "/home/airbot/app_rw/map/everybot_map_00.yaml.enc";
        }
#if 0 // [2025.09.18] hyjoe: 매핑 끝난 후 맵 요청 시, 기존 enc 파일의 존재 여부와 상관없이 무조건 파일 새로 만들도록 수정
        if(!isFileExist(outputPGMPath)){
            bMakepgm = APIencryptToFile(inputPGMPath,outputPGMPath,false);
            RCLCPP_INFO(this->get_logger(), "make encPGM result : %d",static_cast<int>(bMakepgm));
        }else{
            bMakepgm = true;
        }

        if(!isFileExist(outputYamlPath)){
            bMakeyaml = APIencryptToFile(inputYamlPath,outputYamlPath,false);
            RCLCPP_INFO(this->get_logger(), "make encYaml result : %d",static_cast<int>(bMakeyaml));
        }else{
            bMakeyaml = true;
        }
#endif
        bMakepgm = APIencryptToFile(inputPGMPath,outputPGMPath,false);
        bMakeyaml = APIencryptToFile(inputYamlPath,outputYamlPath,false);

        if(bMakepgm && bMakeyaml){
            if(resGetOriginMapData(originPath,originYamlName,originPGMName)){
                RCLCPP_INFO(this->get_logger(), "MapSendToSoc savedmap encrypt success");
            }else{
                RCLCPP_INFO(this->get_logger(), "MapSendToSoc savedmap encrypt fail error");
            }
        }else{
            if(!bMakepgm){
                RCLCPP_INFO(this->get_logger(), "MapSendToSoc savedmap pgm save fail error");
            }
            if(!bMakeyaml){
                RCLCPP_INFO(this->get_logger(), "MapSendToSoc savedmap yaml save fail error");
            }
        }
    }
}

void UdpCommunication::resSocSoftWareVersion()
{
	resGetSoftwareVersion(socData.version.total_ver);
    RCLCPP_INFO(this->get_logger(), "Send to SocSoftware Version : %s",socData.version.total_ver.c_str());
}

void UdpCommunication::resSocNodeStatus()
{
    switch (nodeState)
    {
    case NODE_STATUS::AUTO_MAPPING :
    case NODE_STATUS::MANUAL_MAPPING :
        resGetMapStatus(true, "mapping");
        break;
    case NODE_STATUS::NAVI :
        resGetMapStatus(false, "navigation");
        break;
    case NODE_STATUS::FT_NAVI :
        resGetMapStatus(false, "factory-navigation");
        break;
    case NODE_STATUS::FOLLOWING :
        resGetMapStatus(false, "following");
        break;
    default:
        resGetMapStatus(false, "IDLE");
        break;
    }
}


void UdpCommunication::resSocRobotState()
{
    static int prevState = 0;
    int newState;
    if(bFactoryNaviMode){
        newState = static_cast<int>(ROBOT_STATE::FACTORY_NAVIGATION);
    }else{
        newState = static_cast<int>(socData.robotStatus);
    }
    resGetRobotStatus(newState);
    if(prevState != newState){
        RCLCPP_INFO(this->get_logger(), "Send to SocRobot State prev: %d, new: %d",prevState,newState);
        prevState = newState;
    }
}

void UdpCommunication::resSocRobotStatus()
{
    static int prevStatus = 0;
    int newStatus;
    if(bFactoryNaviMode && ((ROBOT_STATE)socData.robotStatus != ROBOT_STATE::FACTORY_NAVIGATION) ){
        newStatus = static_cast<int>(ROBOT_STATUS::START);
    }else{
        newStatus = static_cast<int>(socData.actionStatus);
    }
    resGetActionStatus(newStatus);
    if(prevStatus != newStatus){
        RCLCPP_INFO(this->get_logger(), "Send to SocRobot Status prev: %d, new: %d",prevStatus,newStatus);
        prevStatus = newStatus;
    }
}

double UdpCommunication:: read_temperature_from_file(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file) {
        std::cerr << "Failed to open " << file_path << std::endl;
        return -1.0f;
    }

    int temp;
    file >> temp;
    return temp / 1000.0f;  // 온도는 보통 1000배로 표시됨
}

void UdpCommunication::resSocTemperature()
{
    // SW temperature
    double APsoc        = temp_ap_soc;            // soc-thermal
    double APbigcore0   = temp_ap_bigcore0;       // bigcore0-thermal
    double APbigcore1   = temp_ap_bigcore1;       // bigcore1-thermal
    double APlittlecore = temp_ap_littlecore;     // littlecore-thermal
    double APcenter     = temp_ap_center;         // center-thermal
    double APgpu        = temp_ap_gpu;            // gpu-thermal
    double APnpu        = temp_ap_npu;            // npu-thermal

    // AI temperature
    double AInpu = aiTemp.npu;
    double AIgpu = aiTemp.gpu;
    double AIcenter =  aiTemp.center;
    double AIsoc =  aiTemp.soc;
    double AIbigcore0 =  aiTemp.bigcore0;
    double AIbigcore1 =  aiTemp.bigcore1;
    double AIlittlecore =  aiTemp.littlecore;

    // Terminal status
    int leftTerminal = leftTerminalTemp;
    int rightTerminal =  rightTerminalTemp;

    // 전달
    resGetTemperature(APnpu, APgpu, APcenter, APsoc, APbigcore0, APbigcore1, APlittlecore,
                      AInpu, AIgpu, AIcenter, AIsoc, AIbigcore0, AIbigcore1, AIlittlecore,
                      leftTerminal, rightTerminal);    
    
    if(bInspectionMode && !bInspectionMotor) {        
        RCLCPP_INFO(this->get_logger(), "[resSocTemperature] AP[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f], AI[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f], POGO[%d,%d]"
                                    , APnpu, APgpu, APcenter, APsoc, APbigcore0, APbigcore1, APlittlecore
                                    , AInpu, AIgpu, AIcenter, AIsoc, AIbigcore0, AIbigcore1, AIlittlecore
                                    , leftTerminal, rightTerminal);        
    }
}

void UdpCommunication::resSocTargetPosition()
{
    pose pose = getTargetPose();
    resGetTargetPosition(pose.x, pose.y, pose.theta);
}

void UdpCommunication::resSocBattData()
{
    resGetBatteryStatusV2(socData.battInfo.cell_voltage1,socData.battInfo.cell_voltage2,socData.battInfo.cell_voltage3,socData.battInfo.cell_voltage4,socData.battInfo.cell_voltage5,
    socData.battInfo.total_capacity,socData.battInfo.remaining_capacity,socData.battInfo.manufacturer,
    socData.battInfo.percent,socData.battInfo.voltage,socData.battInfo.current,socData.battInfo.temperature1,socData.battInfo.temperature2,
    socData.battInfo.design_capacity,socData.battInfo.number_of_cycles,socData.battInfo.charge_status);
}

void UdpCommunication::resSocDockingStatus()
{
    static int prevStatus = 0;
    int newStatus = static_cast<int>(socData.dockingInfo.status);
    if(prevStatus != newStatus){
        RCLCPP_INFO(this->get_logger(), "Send to SocDockingStatus prev: %02x, new: %02x",prevStatus,socData.dockingInfo.status);
        prevStatus = newStatus;
    }
    resGetDockingStatus(newStatus);
}

void UdpCommunication::resSocRobotInfo()
{
    resGetRobotInfo(popo_maneuver_state_str_, 1);
}

void UdpCommunication::resSocModifiedMap()
{
    int height,width;
    double origin_x,origin_y;
    std::vector<uint8_t> modified_map;

    RCLCPP_INFO(this->get_logger(), "reqGetModifiedMapData");

    if (!readPGM("/home/airbot/app_rw/map/everybot_map_00.pgm", width, height, modified_map, origin_x, origin_y)) {
        RCLCPP_INFO(this->get_logger(), "Error reading PGM file");
        return;
    } 
    resGetModifiedMapDataB(width,height,modified_map.data(),modified_map.size()); 
}

void UdpCommunication::resSocMovingInfo()
{   
    static int prev_status = 0;
    int new_status = static_cast<int>(socData.movingStatus);
    
    sendstationPoseChecker(); //hjkim : set bOnstation before send moving info
    
    if(bOnStation){
        resGetAllMovingInfoV2(new_status,
        true,station_pose.x,station_pose.y,station_pose.theta,
        true,station_pose.x,station_pose.y,station_pose.theta);
        if(m_prev_station_pose.x != station_pose.x || m_prev_station_pose.y != station_pose.y || m_prev_station_pose.theta != station_pose.theta){
            RCLCPP_INFO(this->get_logger(), "Send to SocMovingInfo StationPose(%.2f,%.2f,%.1f(deg))",station_pose.x,station_pose.y,RAD2DEG(station_pose.theta));
        }
        m_prev_station_pose = station_pose;
    }else{
        resGetAllMovingInfoV2(new_status,
        socData.robotPosition.valid,socData.robotPosition.x,socData.robotPosition.y,socData.robotPosition.theta,
        socData.targetPosition.valid,socData.targetPosition.x,socData.targetPosition.y,socData.targetPosition.theta);
        if(prev_status != new_status){
            RCLCPP_INFO(this->get_logger(), "Send to SocMovingInfo : %d",new_status);
            prev_status = new_status;
        }
    }

    double diff_pose_x = fabs(socData.robotPosition.x - m_prev_robot_pose.x);
    double diff_pose_y = fabs(socData.robotPosition.y - m_prev_robot_pose.y);
    double diff_pose_theta = fabs(socData.robotPosition.theta - m_prev_robot_pose.theta);
    double diff_odom_x = fabs(odom.x - m_prev_odom.x);
    double diff_odom_y = fabs(odom.y - m_prev_odom.y);
    double diff_odom_theta = fabs(odom.theta - m_prev_odom.theta);

    if(diff_pose_x > 0.05 || diff_pose_y > 0.05 || diff_pose_theta > 0.05 || 
        diff_odom_x > 0.05 || diff_odom_y > 0.05 || diff_odom_theta > 0.05)
    {
        RCLCPP_INFO(this->get_logger(), "Current RobotPose [%d](%.3f, %.3f, %.1f(deg)) , odom(%.3f, %.3f, %.1f(deg))",
        socData.robotPosition.valid, socData.robotPosition.x, socData.robotPosition.y, RAD2DEG(socData.robotPosition.theta),
        odom.x, odom.y, RAD2DEG(odom.theta));
        //hjkim : 로봇이 낮은속도로 조금씩조금씩 움직이는 경우 좌표 추적이 안됨.
        m_prev_robot_pose.x = socData.robotPosition.x;
        m_prev_robot_pose.y = socData.robotPosition.y;
        m_prev_robot_pose.theta = socData.robotPosition.theta;

        m_prev_odom.x = odom.x;
        m_prev_odom.y = odom.y;
        m_prev_odom.theta = odom.theta;
    }
}

void UdpCommunication::resSocRobotVelocity()
{
    resGetRobotSpeed(socData.velocity.v,socData.velocity.w);
}

void UdpCommunication::resSocLidarData()
{
    resGetLidarData(static_cast<double>(socData.lidarInfo.front_distance), static_cast<double>(socData.lidarInfo.rear_distance));
}

void UdpCommunication::resSocWheelMotorData()
{
    resGetMotorStatusV2(socData.motorInfo.left.rpm,socData.motorInfo.right.rpm,socData.motorInfo.left.current,socData.motorInfo.right.current,socData.motorInfo.left.status,socData.motorInfo.right.status);
}

void UdpCommunication::resSocCameraData()
{
    resGetCameraStatusV2(0,socData.camera_type,socData.cameraInfo.data);
    RCLCPP_INFO(this->get_logger(), "send camera data to soc type : %d ,num : %d ,size : %zu ",socData.camera_type,socData.cameraInfo.num,socData.cameraInfo.data.size());
}

void UdpCommunication::resSocLineLaserData()
{
    resGetLineLaserStatusV2(0,socData.lineLaserInfo.data);
}
 
void UdpCommunication::resSocCliffLiftData()
{
    resGetCliffIRStatus(socData.bottomIrData.front_center, socData.bottomIrData.front_left, socData.bottomIrData.back_left,
         socData.bottomIrData.back_center, socData.bottomIrData.back_right, socData.bottomIrData.front_right);
}

void UdpCommunication::resSocDockReceiverData()
{
    resGetRecvIRStatus((bool)socData.dockingInfo.receiver.b.side_left,(bool)socData.dockingInfo.receiver.b.center_left,(bool)socData.dockingInfo.receiver.b.center_right,
    (bool)socData.dockingInfo.receiver.b.side_right);
}


void UdpCommunication::resSocTofData()
{
    resGetTofStatusV2(socData.tofInfo.left_bottom.status,socData.tofInfo.left_bottom.data,socData.tofInfo.right_bottom.status,socData.tofInfo.right_bottom.data,socData.tofInfo.top.status,socData.tofInfo.top.distance);
}

void UdpCommunication::resSocErrorList()
{
    std::lock_guard<std::mutex> lock(soc_error_list_mutex_);

    if (!socData.errorList.empty())
    {
        for(const auto& error : socData.errorList)
        {
            RCLCPP_INFO(this->get_logger(), "[resSocErrorList] response error: %d (1:occured, 2:released), error_code = %s", error.resolved, error.errorCode.c_str());
        }
                    
        resGetErrorList(socData.errorList);

        if(m_is_starting_comm_SoC){            
            RCLCPP_INFO(this->get_logger(), "[resSocErrorList] clearErrorList");
            socData.errorList.clear();            
        } else {
            RCLCPP_INFO(this->get_logger(), "[resSocErrorList] don't clear error_list before starting communicating with the SOC");
        }
        
    }else{
        RCLCPP_INFO(this->get_logger(), "[resSocErrorList] error_list is empty");
    } 
}

void UdpCommunication::resSocAllSensor()
{
    int camera_status = 0, lineLaser_status = 0;
    resGetAllStatusV2(socData.lidarInfo.front_distance,socData.lidarInfo.rear_distance,
    socData.motorInfo.left.rpm,socData.motorInfo.right.rpm,socData.motorInfo.left.current,socData.motorInfo.right.current,socData.motorInfo.left.type,socData.motorInfo.right.type,
    camera_status,socData.camera_type,socData.cameraInfo.data,lineLaser_status,socData.lineLaserInfo.data,
    socData.tofInfo.left_bottom.status,socData.tofInfo.right_bottom.status,socData.tofInfo.top.status,socData.tofInfo.left_bottom.data,socData.tofInfo.right_bottom.data,socData.tofInfo.top.distance,
    socData.bottomIrData.front_center, socData.bottomIrData.front_left, socData.bottomIrData.back_left,socData.bottomIrData.back_center, socData.bottomIrData.back_right, socData.bottomIrData.front_right,
    (bool)socData.dockingInfo.receiver.b.side_left,(bool)socData.dockingInfo.receiver.b.center_left,(bool)socData.dockingInfo.receiver.b.center_right,(bool)socData.dockingInfo.receiver.b.side_right,
    socData.battInfo.cell_voltage1,socData.battInfo.cell_voltage2,socData.battInfo.cell_voltage3,socData.battInfo.cell_voltage4,socData.battInfo.cell_voltage5,
    socData.battInfo.total_capacity,socData.battInfo.remaining_capacity,socData.battInfo.manufacturer,
    socData.battInfo.percent,socData.battInfo.voltage,socData.battInfo.current,socData.battInfo.temperature1,socData.battInfo.temperature2,
    socData.battInfo.design_capacity,socData.battInfo.number_of_cycles,socData.battInfo.charge_status,
	socData.imuData.ax, socData.imuData.ay, socData.imuData.az, socData.imuData.roll, socData.imuData.pitch, socData.imuData.yaw); 
    if(bInspectionMode && !bInspectionMotor){
        RCLCPP_INFO(this->get_logger(), "[resSocAllSensor] lidar front : %d, rear : %d",socData.lidarInfo.front_distance,socData.lidarInfo.rear_distance);
        
        RCLCPP_INFO(this->get_logger(), "[resSocAllSensor] l_motor rpm(%d), current(%d), type(%d)"
                    , socData.motorInfo.left.rpm, socData.motorInfo.left.current, socData.motorInfo.left.type);
        RCLCPP_INFO(this->get_logger(), "[resSocAllSensor] r_motor rpm(%d), current(%d), type(%d)"
                    , socData.motorInfo.right.rpm, socData.motorInfo.right.current, socData.motorInfo.right.type);        
        
        RCLCPP_INFO(this->get_logger(), "[resSocAllSensor] TOP ToF status : %d, distance : %.1f",socData.tofInfo.top.status, socData.tofInfo.top.distance);        

        std::string tof_left_bottom_str = "[resSocAllSensor] M-ToF L sts : ";
        tof_left_bottom_str += std::to_string(socData.tofInfo.left_bottom.status);
        tof_left_bottom_str += ", dist : ";
        int data_cnt = 0;
        int slash_cnt = 0;
        for(auto data : socData.tofInfo.left_bottom.data){
            tof_left_bottom_str += std::to_string(data);
            tof_left_bottom_str += " ";
            if(data_cnt >= 3){
                if(slash_cnt < 3){
                    tof_left_bottom_str += "/ ";
                }
                data_cnt = 0;
                slash_cnt++;
            }else{
                data_cnt++;
            }
        }
        RCLCPP_INFO(this->get_logger(), "%s", tof_left_bottom_str.c_str());
           
        std::string tof_right_bottom_str = "[resSocAllSensor] M-ToF R sts : ";
        tof_right_bottom_str += std::to_string(socData.tofInfo.right_bottom.status);
        tof_right_bottom_str += ", dist : ";
        data_cnt = 0;
        slash_cnt = 0;
        for(auto data : socData.tofInfo.right_bottom.data){
            tof_right_bottom_str += std::to_string(data);
            tof_right_bottom_str += " ";
            if(data_cnt >= 3){
                if(slash_cnt < 3){
                    tof_right_bottom_str += "/ ";
                }
                data_cnt = 0;
                slash_cnt++;
            }else{
                data_cnt++;
            }
        }
        RCLCPP_INFO(this->get_logger(), "%s", tof_right_bottom_str.c_str());
        RCLCPP_INFO(this->get_logger(), "[resSocAllSensor] camera type : %d, data num : %d, size : %zu ",socData.camera_type,socData.cameraInfo.num,socData.cameraInfo.data.size());
        for(auto data : socData.cameraInfo.data)
        {
            RCLCPP_INFO(this->get_logger(), "[resSocAllSensor] camera data id[%u]",data.class_id);
        }
        RCLCPP_INFO(this->get_logger(), "[resSocAllSensor] cliff fc(%d), fl(%d), bl(%d), bc(%d), br(%d), fr(%d)"
                   , socData.cliffLiftInfo.b.cliff_front_center, socData.cliffLiftInfo.b.cliff_front_left, socData.cliffLiftInfo.b.cliff_back_left
                   , socData.cliffLiftInfo.b.cliff_back_center, socData.cliffLiftInfo.b.cliff_back_right, socData.cliffLiftInfo.b.cliff_front_right);
        RCLCPP_INFO(this->get_logger(), "[resSocAllSensor] bottomIR-ADC fc(%d), fl(%d), bl(%d), bc(%d), br(%d), fr(%d)"
                   , socData.bottomIrData.front_center, socData.bottomIrData.front_left, socData.bottomIrData.back_left
                   ,socData.bottomIrData.back_center, socData.bottomIrData.back_right, socData.bottomIrData.front_right);
        RCLCPP_INFO(this->get_logger(), "[resSocAllSensor] receiver sl(%d), fl(%d), fr(%d), sr(%d)"
                    ,socData.dockingInfo.receiver.b.side_left, socData.dockingInfo.receiver.b.center_left
                    ,socData.dockingInfo.receiver.b.center_right, socData.dockingInfo.receiver.b.side_right);
        RCLCPP_INFO(this->get_logger(), "[resSocAllSensor] battery voltage : %.1f, current : %.1f, percent : %d, temperature : (%d,%d)"
                    , socData.battInfo.voltage, socData.battInfo.current, socData.battInfo.percent, socData.battInfo.temperature1, socData.battInfo.temperature2);
        RCLCPP_INFO(this->get_logger(), "[resSocAllSensor] battery design_capacity : %d, number_of_cycles : %d, charge status : %d"
                    , socData.battInfo.design_capacity, socData.battInfo.number_of_cycles, socData.battInfo.charge_status);
        RCLCPP_INFO(this->get_logger(),"[resSocAllSensor] imu Accel(m/s²): x[%.3f], y[%.3f]z[%.3f] roll(deg)[%.2f], pitch(deg)[%.2f], yaw(deg)[%.2f]"
                    ,socData.imuData.ax, socData.imuData.ay, socData.imuData.az, RAD2DEG(socData.imuData.roll), RAD2DEG(socData.imuData.pitch), RAD2DEG(socData.imuData.yaw));
                    
    }
}

void UdpCommunication::autoPublisher()
{        
    // if(publishReqCameraType == 0){
    //     publishReqCameraType();
    // }
    
    publishHeartbeatVersion();

    if(bReserveResumeAfter3sec && ++reservedResumeCount >= 3){
        bReserveResumeAfter3sec = false;
        reservedResumeCount = 0;
        publishStateCommand(REQUEST_SOC_CMD::RESUME_WORKING);
        RCLCPP_INFO(this->get_logger(), "RecoveryCompleteResumeAfter3sec Publish RESUME_WORKING");
    }

    if(bEnableVertionTimer){
        versionGenerator();
    }
}

void UdpCommunication::reqSocDataChecker()
{
    if (reqGetPosition()){
        RCLCPP_INFO(this->get_logger(), "Data request From Soc ROBOT_POSITION");
        resSocRobotPose();
        m_is_starting_comm_SoC = true;
    }

    if (reqGetMapData()){
        RCLCPP_INFO(this->get_logger(), "Data request From Soc MAP");
        resSocMap();
        m_is_starting_comm_SoC = true;
    }

    if(reqGetEncMapData()){
        RCLCPP_INFO(this->get_logger(), "Data request From Soc ENC_MAP");
        resSocEncMap();
        m_is_starting_comm_SoC = true;
    }
    bool isGenerateVersion = checkVersionGenerate();
    // bool reqVersionFromSoc = reqGetSoftwareVersion();
    // if (reqVersionFromSoc || isGenerateVersion){
    //     RCLCPP_INFO(this->get_logger(), "Data request From Soc SW_VERSION versionGenerate[%d]",isGenerateVersion);
    //     resSocSoftWareVersion();
    //     if(reqVersionFromSoc){
    //         //hjkim : on-demend 데이터 전송 시 m_is_starting_comm_SoC 설정 오류, m_is_starting_comm_SoC 는 Soc와 연동을 확인하기 위한 변수로, soc에서 데이터 요청이 왔을 때만 true로 설정되어야 함
    //         m_is_starting_comm_SoC = true;
    //     }
    //     //version timer enable && generate version complete --> send version & disable timer
    // }
    if (reqGetSoftwareVersion()) {
        resSocSoftWareVersion();
    }

    if (reqGetMapStatus()){
        RCLCPP_INFO(this->get_logger(), "Data request From Soc MAP_STATUS");
        resSocNodeStatus();
        m_is_starting_comm_SoC = true;
    }

    if (reqGetTargetPosition()){
        RCLCPP_INFO(this->get_logger(), "Data request From Soc TARGET_POSITION");
        resSocTargetPosition();
        m_is_starting_comm_SoC = true;
    }

    if (reqGetBatteryStatusV2() || reqGetBatteryStatus()){
        RCLCPP_INFO(this->get_logger(), "Data request From Soc BATTERY_STATUS2");
        resSocBattData();
        m_is_starting_comm_SoC = true;
    }

    bool reqDockingStatusFromSoc = reqGetDockingStatus();
    if (reqDockingStatusFromSoc || checkDockingStatusOnDemend()){
        //RCLCPP_INFO(this->get_logger(), "Data request From Soc DOCKING_STATUS"); // [250328] KKS : log 과다출력으로 제거
        resSocDockingStatus();
        if(reqDockingStatusFromSoc){
            //hjkim : on-demend 데이터 전송 시 m_is_starting_comm_SoC 설정 오류, m_is_starting_comm_SoC 는 Soc와 연동을 확인하기 위한 변수로, soc에서 데이터 요청이 왔을 때만 true로 설정되어야 함
            m_is_starting_comm_SoC = true;
        }
    }
    // if (reqGetRobotInfo()){
    //     RCLCPP_INFO(this->get_logger(), "Data request From Soc ROBOT_INFO");
    //     resSocRobotInfo();
    //     m_is_starting_comm_SoC = true;
    // }

    if (reqGetModifiedMapData()){
        RCLCPP_INFO(this->get_logger(), "Data request From Soc MODIFIED_MAP");
        resSocModifiedMap();  
        m_is_starting_comm_SoC = true;
    }

    if (reqGetAllMovingInfoV2() || reqGetAllMovingInfo()){
        RCLCPP_INFO(this->get_logger(), "Data request From Soc MOVING_INFO");
        resSocMovingInfo();
        m_is_starting_comm_SoC = true;
    }

    if(reqGetRobotSpeed()){
        RCLCPP_INFO(this->get_logger(), "Data request From Soc ROBOT_VELOCITY");
        resSocRobotVelocity();
        m_is_starting_comm_SoC = true;
    }

    if (reqGetMotorStatus()){
        RCLCPP_INFO(this->get_logger(), "Data request From Soc MOTOR_STATUS");
        resSocWheelMotorData();
        m_is_starting_comm_SoC = true;
    }

    if (reqGetCameraStatusV2() || reqGetCameraStatus()){
        RCLCPP_INFO(this->get_logger(), "Data request From Soc CAMERA_STATUS2");
        resSocCameraData();
        m_is_starting_comm_SoC = true;
    }

    if (reqGetLineLaserStatusV2() || reqGetLineLaserStatus()){
        RCLCPP_INFO(this->get_logger(), "Data request From Soc LINE_LASER_STATUS2");
        resSocLineLaserData();
        m_is_starting_comm_SoC = true;
    }

    if (reqGetTofStatusV2() || reqGetTofStatus()){
        RCLCPP_INFO(this->get_logger(), "Data request From Soc TOF_STATUS2");
        resSocTofData();
        m_is_starting_comm_SoC = true;
    }

    bool reqErrorListFromSoc = reqGetErrorList();
    if (reqErrorListFromSoc || !socData.errorList.empty()){
        resSocErrorList();
        if(reqErrorListFromSoc){
            RCLCPP_INFO(this->get_logger(), "Data request From Soc ERROR_LIST");
            m_is_starting_comm_SoC = true; 
        }else{
            RCLCPP_INFO(this->get_logger(), "On-demand Data Send to Soc ERROR_LIST");
        }
    }

    if (reqGetLidarData() ||reqGetLidarSensorStatus()){
        //RCLCPP_INFO(this->get_logger(), "Data request From Soc LIDAR_DATA"); log 과다출력으로 제거
        resSocLidarData();
        m_is_starting_comm_SoC = true;
    }

    if(reqGetRecvIRStatus()){
        RCLCPP_INFO(this->get_logger(), "Data request From Soc DOCK_RECEIVER");
        resSocDockReceiverData();
        m_is_starting_comm_SoC = true;
    }

    if(reqGetCliffIRStatus()){
        RCLCPP_INFO(this->get_logger(), "Data request From Soc CLIFF_LIFT_DATA");
        resSocCliffLiftData();
        m_is_starting_comm_SoC = true;
    }

    if(reqGetAllStatusV2()){
        //RCLCPP_INFO(this->get_logger(), "Data request From Soc ALL_STATUS"); //250610 KKS : SoC에서 주기적으로 요청이 와서 로그 제거
        resSocAllSensor();
        m_is_starting_comm_SoC = true;
    }
    
    if(reqGetRobotStatus()){
        RCLCPP_INFO(this->get_logger(), "Data request From Soc ROBOT_STATE");
        resSocRobotState();
        m_is_starting_comm_SoC = true;
    }
    
    if(reqGetActionStatus()){
        RCLCPP_INFO(this->get_logger(), "Data request From Soc ACTION_STATE");
        resSocRobotStatus();
        m_is_starting_comm_SoC = true;
    }
    
    if(reqGetNotification()){
        RCLCPP_INFO(this->get_logger(), "Data request From Soc NOTIFICATION");
        resGetNotification(0,"0");
        m_is_starting_comm_SoC = true;
    }if(reqGetTemperature()){
        //RCLCPP_INFO(this->get_logger(), "Data request From Soc TEMPERATURE"); //250610 KKS : SoC에서 주기적으로 요청이 와서 로그 제거
        resSocTemperature();  
    }

    auto now = std::chrono::steady_clock::now();
    if(auto_send_time_ == std::chrono::steady_clock::time_point()){
        auto_send_time_ = now;
    }
    bool bAutoSend = getSteadyClockRunningSeconds(auto_send_time_) > 1.0;
    bool bOndemandSendState = false, bOndemandSendMovingState = false;
    std::vector<mainState> front_state = dequeueMainState();
    if(!front_state.empty()){
        mainState data = front_state.front();
        setSocRobotState(data.robotStatus,data.actionStatus);
        resSocRobotState();
        resSocRobotStatus();
        RCLCPP_INFO(this->get_logger(), "SocRobot State: Send to Soc State(%d) status(%d), On-demand",static_cast<int>(data.robotStatus),static_cast<int>(data.actionStatus));
        bOndemandSendState = true;
    }
    std::vector<movingState> front_moving_state = dequeueMovingState();
    if(!front_moving_state.empty()){
        movingState data = front_moving_state.front();
        setSocMovingState(data.movingStatus,data.failReason);
        resSocMovingInfo();
        RCLCPP_INFO(this->get_logger(), "SocMoving State: Send to Soc MovingState(%d) fail_reason(%d), On-demand ",static_cast<int>(data.movingStatus),static_cast<int>(data.failReason));
        bOndemandSendMovingState = true;
    }

    if(bAutoSend){
        //hjkim : skip auto send in case ondemand send already
        if(!bOndemandSendState){
            resSocRobotState();
            resSocRobotStatus();
        }
        //hjkim : skip auto send in case ondemand send already
        if(!bOndemandSendMovingState){
            resSocMovingInfo();
        }
        resSocBattData();
        resSocTemperatureNotice();
        resSocRobotInfo(); // popo navigation : maneuver state 를 string 으로 전송하기 위해 추가
        auto_send_time_ = std::chrono::steady_clock::now();
    }
    //hjkim : inspection mode auto send all sensor & temperature for 100ms (before 500ms - sensor updte time is too slow)
    /*inspection-mode 실행시 thread에서 soc에 보내는 처리와 sensorTimer에서 동시에 보내게 되면 udp socket가 계속 생성되어 socket overflow 발생으로 통신 불능 상태가 되는 문제 개선.
    topic callback 이나 timerCallback 함수에서 송신 코드를 thread 내부에서만 송신해야함.*/
    if(bInspectionMode){
        resSocAllSensor();
        resSocTemperature();
    }
}

void UdpCommunication::reqSocOptionChecker()
{
    if (reqSetModifiedMapDataB(modifiedMap)){
        RCLCPP_INFO(this->get_logger(), "soc-cmd received : map-modify");
        std::string filename = "/home/airbot/app_rw/map/everybot_map_00.pgm";
        savePGMFile(modifiedMap, filename);
        RCLCPP_INFO(this->get_logger(), "PGM file saved as %s", filename.c_str());
        resSetModifiedMapData(true);
        publishMapChaged();
    }
    ByPassOne_t parsedData;
    if (reqSetByPassOneData(parsedData)){
        RCLCPP_INFO(this->get_logger(), "soc-cmd received : byPassOne");
        std::string pgmPath = "/home/airbot/app_rw/map/everybot_map_00.pgm";
        std::string yamlPath = "/home/airbot/app_rw/map/everybot_map_00.yaml";
        bool isPGMExist = isFileExist(pgmPath);
        bool isYamlExist = isFileExist(yamlPath);
        if(isPGMExist && isYamlExist){
            RCLCPP_INFO(this->get_logger(), "Request by Pass Data UID : %s , Version : %s, Modified : %s ",parsedData.uid.c_str(),parsedData.version.c_str(),parsedData.modified.c_str() );
            RCLCPP_INFO(this->get_logger(), "blockWall list size : %zu, blockArea list size : %zu", parsedData.block_wall.size(), parsedData.block_area.size()); 
            publishBlockWall(parsedData);
            publishBlockArea(parsedData);
            if(parsedData.charging_station.has_data){
                RCLCPP_INFO(this->get_logger(), "received charging_station data x:%f y:%f theta:%f",parsedData.charging_station.robot_position.x,parsedData.charging_station.robot_position.y,RAD2DEG(parsedData.charging_station.robot_position.theta));
                publishStationPose(parsedData.charging_station.robot_position.x,parsedData.charging_station.robot_position.y,parsedData.charging_station.robot_position.theta);
            }else{
                RCLCPP_INFO(this->get_logger(), "[WARN]charging_station data is empty");
            }
            publishMapChaged();
        }else{
            if(!isPGMExist){
                RCLCPP_INFO(this->get_logger(), "Robot don`t has PGM file need to create map");
            }
            if(!isYamlExist){
                RCLCPP_INFO(this->get_logger(), "Robot don`t has YAML file need to create map");
            }
        }
        
        resSetByPassOneData(true);
    }

    if(reqSetStationRepositioning(reqStationPose)){
        resSetStationRepositioning(true);
        RCLCPP_INFO(this->get_logger(), "soc-cmd received : stationReposition x:%f y:%f theta:%f(%.1f(deg))",reqStationPose.x,reqStationPose.y,reqStationPose.theta,RAD2DEG(reqStationPose.theta));
        publishStationPose(reqStationPose.x,reqStationPose.y,reqStationPose.theta);
        publishMapChaged();
    }

    if (reqSetExcelSteps(reqExcelator)){
        RCLCPP_INFO(this->get_logger(), "soc-cmd received : bExcelSteps"); 
        resSetExcelSteps(true);
    }

    if(reqGetTargetPositionCalculate(ReqCalculateTargetPose)){
        double Time = 0, Distance = 0;
        resGetTargetPositionCalculate( Time,Distance, ReqCalculateTargetPose.x,ReqCalculateTargetPose.y,ReqCalculateTargetPose.theta);
        RCLCPP_INFO(this->get_logger(), "soc-cmd received : reqGetTargetPositionCalculate"); 
    }
    TimeData_t parsedTimeData;
    if(reqSetTimeData(parsedTimeData)){
        RCLCPP_INFO(this->get_logger(), "soc-cmd received : reqSetTimeData year : %d, month : %d, day : %d, hour : %d, minute : %d, second : %d",
        parsedTimeData.year,parsedTimeData.month,parsedTimeData.day,parsedTimeData.hour,parsedTimeData.minute,parsedTimeData.second); 
        resSetTimeData(true);        
    }

    if(MapCopyAlarm())
    {
        RCLCPP_INFO(this->get_logger(), "soc-cmd received : MapCopyAlarm");
        ROBOT_STATE convert_state = static_cast<ROBOT_STATE>(socData.robotStatus);
        if(convert_state == ROBOT_STATE::AUTO_MAPPING || convert_state == ROBOT_STATE::MANUAL_MAPPING || 
            convert_state == ROBOT_STATE::NAVIGATION || convert_state == ROBOT_STATE::RETURN_CHARGER)
        {
           RCLCPP_INFO(this->get_logger(), "can`t do MapCopyAlarm state[%s] node[%s]",enumToString(convert_state).c_str(),enumToString(nodeState).c_str());
        }else{
            std::string encpgmSrc  = "/home/airbot/app_rw/mapCopy/office.pgm.enc";
            std::string encyamlSrc = "/home/airbot/app_rw/mapCopy/office.yaml.enc";
            std::string encJason = "/home/airbot/app_rw/mapCopy/map_meta_sample.json.enc";
            std::string pgmSrc  = "/home/airbot/app_rw/mapCopy/office.pgm";
            std::string yamlSrc = "/home/airbot/app_rw/mapCopy/office.yaml";
            std::string pgmDst  = "/home/airbot/app_rw/map/everybot_map_00.pgm";
            std::string yamlDst = "/home/airbot/app_rw/map/everybot_map_00.yaml";
            
            bool bDecryptPgm = APIdecryptToFile(encpgmSrc, pgmSrc);
            bool bDecryptYaml = APIdecryptToFile(encyamlSrc, yamlSrc);
            
            if(bDecryptPgm){
                RCLCPP_INFO(this->get_logger(), "decrypt pgm file successfully.");
            }else{
                RCLCPP_ERROR(this->get_logger(), "Failed to decrypt pgm file.");
            }
            if(bDecryptYaml){
                RCLCPP_INFO(this->get_logger(), "decrypt yaml file successfully.");
            }else{
                RCLCPP_ERROR(this->get_logger(), "Failed to decrypt yaml file.");
            }

            if(bDecryptPgm && bDecryptYaml){
                if(updateMapFiles(pgmSrc, yamlSrc, pgmDst, yamlDst)) {
                    RCLCPP_INFO(this->get_logger(), "Map files updated successfully.");
                    publishMapCopyComplete();
                    publishMapChaged();
                    ByPassOne_t parsedData;
                    if (reqSetMapCopyData(parsedData)){
                    //if(API_JsonFileReader(encJason,parsedData)){
                        RCLCPP_INFO(this->get_logger(), "MapCopy Json Data UID : %s , Version : %s, Modified : %s ",parsedData.uid.c_str(),parsedData.version.c_str(),parsedData.modified.c_str() );
                        RCLCPP_INFO(this->get_logger(), "blockWall list size : %zu, blockArea list size : %zu", parsedData.block_wall.size(), parsedData.block_area.size()); 
                        publishBlockWall(parsedData);
                        publishBlockArea(parsedData);
                        if(parsedData.charging_station.has_data){
                            RCLCPP_INFO(this->get_logger(), "received station data in MapCopy jason x:%f y:%f theta:%f",parsedData.charging_station.robot_position.x,parsedData.charging_station.robot_position.y,RAD2DEG(parsedData.charging_station.robot_position.theta));
                            publishStationPose(parsedData.charging_station.robot_position.x,parsedData.charging_station.robot_position.y,parsedData.charging_station.robot_position.theta);
                        }else{
                            RCLCPP_INFO(this->get_logger(), "[WARN]there is no station data in MapCopy jason");
                        }
                    }
                    removeFile("/home/airbot/app_rw/map/everybot_map_00.pgm.enc");
                    removeFile("/home/airbot/app_rw/map/everybot_map_00.yaml.enc");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to update map files.");
                }
            }else{
                RCLCPP_ERROR(this->get_logger(), "Failed to Decrypt map files");
            }
        }
    }
}


void UdpCommunication::generateSocCommand()
{  
    ROBOT_STATE convert_state = static_cast<ROBOT_STATE>(socData.robotStatus);
    ROBOT_STATUS convert_status = static_cast<ROBOT_STATUS>(socData.actionStatus);
    if (reqSetMapping(reqMapping)){
        resSetMapping(true);
        if(reqMapping.set == 1){
            clearMapInfo();
            RCLCPP_INFO(this->get_logger(), "soc-cmd received : Start-Manual-Mapping");
            publishStateCommand(REQUEST_SOC_CMD::START_MANUAL_MAPPING);   
        }else if(reqMapping.set == 2){
            clearMapInfo();
            RCLCPP_INFO(this->get_logger(), "soc-cmd received : Start-Auto-Mapping");
            publishStateCommand(REQUEST_SOC_CMD::START_AUTO_MAPPING);   
        }else if(reqMapping.set == 4){
            RCLCPP_INFO(this->get_logger(), "soc-cmd received : Mapping-Stop");
            if( convert_state == ROBOT_STATE::MANUAL_MAPPING || convert_state == ROBOT_STATE::AUTO_MAPPING){
                RCLCPP_INFO(this->get_logger(), "soc-cmd received : Mapping-Stop");
                publishStateCommand(REQUEST_SOC_CMD::STOP_WORKING);
            }else{
                RCLCPP_INFO(this->get_logger(),"soc-cmd received : Can`t stop mapping state[%s] status[%s]",enumToString(convert_state).c_str(),enumToString(convert_status).c_str());
            }
        }
    }else if (reqSetDriving(reqNavigation)){
        resSetDriving(true);
        if (reqNavigation.set == 1){
            RCLCPP_INFO(this->get_logger(), "soc-cmd received : Start-Navigation");
            if(bFactoryNaviMode){
                RCLCPP_INFO(this->get_logger(), "Factory Navi Mode On Start FactoryNavigation");
                publishStateCommand(REQUEST_SOC_CMD::START_FACTORY_NAVIGATION);

            }else{
                publishStateCommand(REQUEST_SOC_CMD::START_NAVIGATION);
            }
        }else if (reqNavigation.set == 4){
            if(bFactoryNaviMode){
                bFactoryNaviMode = false;
                RCLCPP_INFO(this->get_logger(), "Factory Navi Mode Off");
            }
            RCLCPP_INFO(this->get_logger(), "soc-cmd received : Stop-Navigation");
            publishStateCommand(REQUEST_SOC_CMD::STOP_WORKING);
        }else if (reqNavigation.set == 2){
            RCLCPP_INFO(this->get_logger(), "soc-cmd received : Pause");
            publishStateCommand(REQUEST_SOC_CMD::PAUSE_WORKING);
        }else if (reqNavigation.set == 3){
            RCLCPP_INFO(this->get_logger(), "soc-cmd received : Resume");
            publishStateCommand(REQUEST_SOC_CMD::RESUME_WORKING);
        }
        #if ENABLE_FOLLOW
        else if(reqNavigation.set == 5){
            RCLCPP_INFO(this->get_logger(), "soc-cmd received : Start-Follow me Mode");
            publishStateCommand(REQUEST_SOC_CMD::START_FOLLOWING);
            // RCLCPP_INFO(this->get_logger(), "soc-cmd received : Start-Follow me Mode [ But not support this function!!!]");
        }
        #endif
    }else if (reqSetTargetPosition(reqTargetPosition)){
            RCLCPP_INFO(this->get_logger(), "soc-cmd received : Move to Target (%.1f, %.1f, %.1f(deg))", reqTargetPosition.x, reqTargetPosition.y, RAD2DEG(reqTargetPosition.theta)); 
            if(convert_state != ROBOT_STATE::NAVIGATION && convert_state != ROBOT_STATE::FACTORY_NAVIGATION){
                if(bFactoryNaviMode){
                    RCLCPP_INFO(this->get_logger(), "Factory Navi Mode On Start FactoryNavigation");
                    publishStateCommand(REQUEST_SOC_CMD::START_FACTORY_NAVIGATION);
                }else{
                    RCLCPP_INFO(this->get_logger(), "Start Navigation");
                    publishStateCommand(REQUEST_SOC_CMD::START_NAVIGATION);
                }
            }
            resSetTargetPosition(true);
            setSocTargetPosition(reqTargetPosition.x,reqTargetPosition.y,reqTargetPosition.theta);
            publishMoveGoal(reqTargetPosition.x,reqTargetPosition.y,reqTargetPosition.theta);
    }
    else if(reqSetReturnToChargingStation()){
        resSetReturnToChargingStation(true);
        RCLCPP_INFO(this->get_logger(), "soc-cmd received : ReturnToCharger");
        if(convert_state == ROBOT_STATE::RETURN_CHARGER){
            publishMoveCharger();
            RCLCPP_INFO(this->get_logger(), "Already ReturnToCharger State, Move to Charger");
        }else{
            RCLCPP_INFO(this->get_logger(), "StateManager Start ReturnToCharger");
            publishStateCommand(REQUEST_SOC_CMD::START_RETURN_CHARGER); 
        }
    }else if (reqSetDockingState(reqDocking)){
        resSetDockingState(true);
        if(reqDocking.dock==true){
            RCLCPP_INFO(this->get_logger(), "soc-cmd received : start-docking");
            publishStateCommand(REQUEST_SOC_CMD::START_DOCKING);
        }else{
            RCLCPP_INFO(this->get_logger(), "soc-cmd received : stop-docking");
            publishStateCommand(REQUEST_SOC_CMD::STOP_WORKING);
        }
    }
    else if(reqSetFactoryMode(reqFactoryMode)){
        resSetFactoryMode(true);
        if(reqFactoryMode.start){
            RCLCPP_INFO(this->get_logger(), "soc-cmd received : Start-FactoryNavigation");
            std::string pgmSrc  = "/home/airbot/app_rw/factorymap/everybot_factorymap_00.pgm";
            std::string yamlSrc = "/home/airbot/app_rw/factorymap/everybot_factorymap_00.yaml";
            std::string pgmDst  = "/home/airbot/app_rw/map/everybot_map_00.pgm";
            std::string yamlDst = "/home/airbot/app_rw/map/everybot_map_00.yaml";
            publishClearVirtualWall(); // keepout 초기화.
            if( updateMapFiles(pgmSrc, yamlSrc, pgmDst, yamlDst) ){
                publishFactoryStart();
            } else{
                 RCLCPP_ERROR(this->get_logger(), "Failed to copy factory map files.");
            }
        }else{
            RCLCPP_INFO(this->get_logger(), "soc-cmd received : Stop-FactoryNavigation");
            publishStateCommand(REQUEST_SOC_CMD::STOP_WORKING);
            publishFactoryStop();
        }
        bFactoryNaviMode = reqFactoryMode.start; 
    }
    #if ENABLE_FOLLOW
    else if(reqSetFollowMe(reqFollowMe)){
        if(reqFollowMe.enable){
            RCLCPP_INFO(this->get_logger(), "soc-cmd received : Start-Follow Me Mode");
            publishStateCommand(REQUEST_SOC_CMD::START_FOLLOWING);
        }else{
            RCLCPP_INFO(this->get_logger(), "soc-cmd received : Stop-Follow Me Mode");
            publishStateCommand(REQUEST_SOC_CMD::STOP_WORKING);
        }
        // RCLCPP_INFO(this->get_logger(), "soc-cmd received : Follow me Mode [ But not support this function!!!]");
    }
    #endif
}

void UdpCommunication::directRequestCommand()
{
    if (reqSetSoftwareReset()){
        RCLCPP_INFO(this->get_logger(), "soc-cmd received : Reboot");
        resSetSoftwareReset(true); // 재부팅 시에는 응답 먼저 날리고 재부팅 실행 - 실행 후 보고 하면 보고를 할 수 없음
        publishReadyReboot();

#if 0 //hjkim : reboot_sequence
        publishReqAIReset();
        publishReqFWReset(); 
        sleep(3); // reset 명령 발행 후 3초 대기
        systemRebootCommand();
#endif
    }

    if (reqSetEmergencyStop()){
        RCLCPP_INFO(this->get_logger(), "soc-cmd received : EmergencyStop");
        publishEmergencyCommand();
        resSetEmergencyStop(true);
    }

    if (reqGetToFCalibrationData()){
        RCLCPP_INFO(this->get_logger(), "soc-cmd received : Self Test Multi-ToF Calibration");

        // 캘리브레이션 시작
        if (!selftest_calib_timer_) {
            // 변수 초기화
            mtof_calib_file_generated_ = false;
            mtof_calib_data_ = Mtof_Calib();
            selftest_calib_timer_reset_cnt_= 0;
            // Left 상태 Running 으로 세팅
            tof_calibState = (tof_calibState & 0xF0) | 0x01;
            temp_tof_calibcmd = 0x01;
            publishTofLeftCalibration();
            tof_calib_start_time_ = std::chrono::steady_clock::now();
            selftest_calib_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000),
                std::bind(&UdpCommunication::selftestCalibrationTimerCallback, this));
            RCLCPP_INFO(this->get_logger(), "[Self Test Calibration] Start selftest calibration timer");
        } else {
            RCLCPP_INFO(this->get_logger(), "selftest calibration timer already started");
        }
    }

    if (reqSetStartCharging()){
        RCLCPP_INFO(this->get_logger(), "soc-cmd received : StartCharging");
        resSetStartCharging(true);
    }else if (reqSetStopCharging()){
        RCLCPP_INFO(this->get_logger(), "soc-cmd received : StopCharging");
        resSetStopCharging(true);
    }

    if(reqSetMotorManual_VW(reqManualVWMove)){
        publishVelocityCommand(reqManualVWMove.mS,reqManualVWMove.radS);
        resSetMotorManual_VW(true);
    }else if (reqSetRotation(reqRotation)){
        resSetRotation(true);
        RCLCPP_INFO(this->get_logger(), "soc-cmd received : Rotation ");
        publishRotation(reqRotation.type,reqRotation.radian);
    }

    if(reqSetStartLidar()){
        RCLCPP_INFO(this->get_logger(), "soc-cmd received : start-Lidar");
        publishLidarOnOff(true);
    }else if(reqSetStopLidar()){
        RCLCPP_INFO(this->get_logger(), "soc-cmd received :t stop-Lidar");
        publishLidarOnOff(false);
    }

    if(reqSetSensorInspectionMode(reqInspectionMode)){
        resSetSensorInspectionMode(true);
        publishDockingCommand(false);
        if(reqInspectionMode.start){
            RCLCPP_INFO(this->get_logger(), "soc-cmd received : InspectionMode START");
            publishInpectionStart();
            bInspectionMode = true;
            enableSensorTimer(UDP_COMMUNICATION::NORMAL);
        }else{
            RCLCPP_INFO(this->get_logger(), "soc-cmd received : InspectionMode STOP");
            if(bInspectionMotor){
                publishDockingCommand(false);
                disableLinearTargetMovoing();
                bInspectionMotor = false;
                RCLCPP_INFO(this->get_logger(), "InspectionMotor Cancel");
            }
            publishInpectionStop();
            bInspectionMode = false;
            disableSensorTimer();
        }
    }

    if(reqSetBatterySleepMode(reqBatterySleep)){
        resSetBatterySleepMode(true);
        RCLCPP_INFO(this->get_logger(), "soc-cmd received : Battery Sleep Mode");
        if(socData.battInfo.manufacturer == 0x01){
            RCLCPP_INFO(this->get_logger(), "NEONIX-BATTERY sleep-mode start");
            publishBatterySleep();
        }else if(socData.battInfo.manufacturer == 0x02){
            RCLCPP_INFO(this->get_logger(), "UBATTERY sleep-mode start");
            publishBatterySleep();
        }else{
            RCLCPP_INFO(this->get_logger(), "UNKOWN BATTERY Can`t be sleep-mode");
        }
    }

    if(reqSetSelfDiagnosisMotor(reqSelfDiagnosisMotor)){
        
        resSetSelfDiagnosisMotor(true);
        RCLCPP_INFO(this->get_logger(), "soc-cmd received : SelfDiagnosisMotor");
        if(reqSelfDiagnosisMotor.enable){
            RCLCPP_INFO(this->get_logger(), "SelfDiagnosisMotor Start");
            publishDockingCommand(false);
            if(bInspectionMode){
                bInspectionMotor = true;
                enableLinearTargetMoving(reqSelfDiagnosisMotor.mS,reqSelfDiagnosisMotor.Distance);
            }else{
                RCLCPP_INFO(this->get_logger(), "Not InspectionMode Can`t be SelfDiagnosisMotor Start");
            }
        }else{
            RCLCPP_INFO(this->get_logger(), "SelfDiagnosisMotor Stop");
            if(bInspectionMode){
                disableLinearTargetMovoing();
            }else{
                RCLCPP_INFO(this->get_logger(), "Not InspectionMode Can`t be SelfDiagnosisMotor Stop");
            }
            publishDockingCommand(true);
        }
        
    }
}


void UdpCommunication::reqSocActionChecker()
{  
    generateSocCommand();
    directRequestCommand();
}

void UdpCommunication::publishVersionRequest(uint8_t cmd)
{
    std_msgs::msg::UInt8 msgs;
    msgs.data = cmd;
    req_version_pub_->publish(msgs);
    RCLCPP_INFO(this->get_logger(), "publishVersionRequest : %u",msgs.data);
}

void UdpCommunication::publishMoveGoal(double x,double y,double theta)
{
    everybot_custom_msgs::msg::Position msg;
    msg.x = x;
    msg.y = y;
    msg.theta = theta;
    move_target_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "publishMoveGoal X : %f, Y : %f",x,y);
}

void UdpCommunication::publishMoveCharger()
{
    std_msgs::msg::Empty msg;
    move_charger_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "publishMoveCharger");
}

void UdpCommunication::publishMoveNRotation(double x,double y,double theta, int type)
{
    everybot_custom_msgs::msg::MoveNRotation msg;
    msg.x = x;
    msg.y = y;
    msg.theta = theta;
    msg.type = static_cast<u_int8_t>(type);
    move_rotation_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "publishMoveNRotation type : %d, target(%.1f, %.1f, %.1f(deg))", type, x, y, RAD2DEG(theta));
}

void UdpCommunication::publishRotation(int type,double theta)
{
    everybot_custom_msgs::msg::MoveNRotation msg;
    msg.x = 0.0;
    msg.y = 0.0;
    msg.theta = theta;
    msg.type = static_cast<u_int8_t>(type);
    rotation_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "publishRotation type : %d, target heading : %.1f(deg)", type, RAD2DEG(theta));
}

void UdpCommunication::publishLidarOnOff(bool on_off)
{
    std_msgs::msg::Bool lidar_cmd;
    if(on_off){
        lidar_cmd.data = true;
        lidar_cmd_pub_->publish(lidar_cmd);
        resSetStartLidar(true);
    }else{
        lidar_cmd.data = false;
        lidar_cmd_pub_->publish(lidar_cmd);
    }
    resSetStopLidar(true);
}

void UdpCommunication::publishSensorOnOff(bool set)
{
    std_msgs::msg::Bool lidar_cmd;
    std_msgs::msg::Bool tof_cmd;
    std_msgs::msg::Bool camera_cmd;
    #if USE_LINELASER_SENSOR > 0
    std_msgs::msg::Bool linelaser_cmd;
    #endif
    if(set){
        lidar_cmd.data = true;
        tof_cmd.data = true;
        camera_cmd.data = true;
        #if USE_LINELASER_SENSOR > 0
        linelaser_cmd.data = true;
        #endif
    }else{
        lidar_cmd.data = false;
        tof_cmd.data = false;
        camera_cmd.data = false;
        #if USE_LINELASER_SENSOR > 0
        linelaser_cmd.data = false;
        #endif
    }
    lidar_cmd_pub_->publish(lidar_cmd);
    tof_cmd_pub_->publish(tof_cmd);
    camera_cmd_pub_->publish(camera_cmd);
    #if USE_LINELASER_SENSOR > 0
    linelaser_cmd_pub_->publish(linelaser_cmd);
    #endif
}

void UdpCommunication::publishBatterySleep()
{
    std_msgs::msg::Empty battery_sleep_cmd;
    batterySleep_cmd_pub_->publish(battery_sleep_cmd);
    RCLCPP_INFO(this->get_logger(), "publishBatterySleep");
}

void UdpCommunication::publishStationPose(double x, double y, double theta)
{
    station_pose.x = x;
    station_pose.y = y;
    station_pose.theta = theta;

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = rclcpp::Clock().now();
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    msg.pose.pose.orientation = tf2::toMsg(q);
    std::fill(msg.pose.covariance.begin(), msg.pose.covariance.end(), 0.0);

    station_pose_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "publishStationPose X : %f, Y: %f, Theta(DEG) : %f ",x,y,RAD2DEG(theta));
}

void UdpCommunication::publishDockingCommand(bool start)
{
    std_msgs::msg::UInt8 dock_cmd_;
    if(start){
        dock_cmd_.data = DOCK_START;
        RCLCPP_INFO(this->get_logger(),"publish Start-Docking");
    }else{
        dock_cmd_.data = DOCK_STOP;
    RCLCPP_INFO(this->get_logger(), "publish Stop-Docking");
    }
    dock_pub->publish(dock_cmd_);
}

void UdpCommunication::publishInpectionStart()
{
    std_msgs::msg::Bool msg;
    msg.data = true;
    inspection_mode_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "publishInpectionStart");
}
void UdpCommunication::publishInpectionStop()
{
    std_msgs::msg::Bool msg;
    msg.data = false;
    inspection_mode_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "publishInpectionStop");
}

void UdpCommunication::publishFactoryStart()
{
    std_msgs::msg::Bool msg;
    msg.data = true;
    factory_mode_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "publishFactoryStart");
}
void UdpCommunication::publishFactoryStop()
{
    std_msgs::msg::Bool msg;
    msg.data = false;
    factory_mode_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "publishFactoryStop");
}

void UdpCommunication::logCommandAndData(int command, const std::vector<uint8_t>& data, int option) 
{
    if(option == 1)
        RCLCPP_INFO(this->get_logger(), "=============== API_RecvJigData Start ================ ");
    else
        RCLCPP_INFO(this->get_logger(), "=============== API_SendJigData Start ================ ");
    
    RCLCPP_INFO(this->get_logger(), "send jig Command: %d", command);

    if(data.size() > 0){
        if(command == 5) {
            printTof(data);
        }
        else if(command == 6 || command == 7){
            printLidar(command,data);
        } 
        else 
        {
            int i = 0;
            for (const auto& byte : data) {
                RCLCPP_INFO(this->get_logger(), "Data[%d]: 0x%02X", i, byte);
                i++;
            }
        }
    }
    
    if(option == 1)
        RCLCPP_INFO(this->get_logger(), "=============== API_RecvJigData END ================ ");
    else 
        RCLCPP_INFO(this->get_logger(), "=============== API_SendJigData END ================ ");
}

void UdpCommunication::printLidar(int command, const std::vector<uint8_t>& data)
{
    if (data.size() < 4) {
        RCLCPP_ERROR(this->get_logger(), "Error: Insufficient data size for lidar distances.");
        return;
    }

    // Extract obstacle distance (mm)
    uint16_t lidar_long_obstacle_distance = (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
    uint16_t lidar_short_obstacle_distance = (static_cast<uint16_t>(data[2]) << 8) | static_cast<uint16_t>(data[3]);

    // Log the distances
    if(command == 6)
        RCLCPP_INFO(this->get_logger(), "front Lidar Distances (mm):");
    else if (command == 7)
        RCLCPP_INFO(this->get_logger(), "rear Lidar Distances (mm):");
    
    RCLCPP_INFO(this->get_logger(), "  Long Obstacle Distance: %d mm", lidar_long_obstacle_distance);
    RCLCPP_INFO(this->get_logger(), "  Short Obstacle Distance: %d mm", lidar_short_obstacle_distance);
}

void UdpCommunication::printTof(const std::vector<uint8_t>& data)
{
    if( !data.empty() && data.size() % 2 != 0)
    {
        RCLCPP_WARN(this->get_logger(), "Data size is odd, last byte will be ignored.");
        return ;
    }
    
    for (size_t i = 0; i < data.size(); i++) {
        RCLCPP_INFO(this->get_logger(), "Raw Data[%zu]: 0x%02X (%d)", i, data[i], data[i]);
    }

    for (size_t i = 0; i < data.size(); i += 2) {
        uint16_t tof_distance = (static_cast<uint16_t>(data[i]) << 8) | static_cast<uint16_t>(data[i+1]);
        RCLCPP_INFO(this->get_logger(), "tof Data[%zu]: %d", i/2 , tof_distance);
    }
}

void UdpCommunication::procSocCommunication()
{
    reqSocDataChecker();
    reqSocOptionChecker();
    reqSocActionChecker();
}

/**
 *  @brief AMR JIG 통신 처리 프로세스.
 */
void UdpCommunication::procAmrJigCommunication()
{
    std::vector<uint8_t> packet;
    
    //AMR JIG
    for(int i = 1; i < JIG_HEADER::CMD_END; i++)
    {
        if(API_RecvJigData(i, packet)){
            logCommandAndData(i, packet, 1);
            jigProcessor(i,packet);
            break;
        }
    }
}

void UdpCommunication::procApJigCommunication()
{
    std::vector<uint8_t> packet;

    //AP JIG
    for(int i = 241; i < JIG_AP_HEADER::AP_CMD_END; i++ )
    {
        if(API_RecvJigData(i, packet)){
            // logCommandAndData(i, packet, 1); 로그 자리
            apJigProcessor(i,packet);
            break;
        }
    }
}

UDP_COMMUNICATION UdpCommunication::checkUdpCommunicationMode()
{
    std::vector<uint8_t> packet;
    UDP_COMMUNICATION ret = getCommunicationMode();
    if(API_RecvJigData(static_cast<int>(JIG_HEADER::AMR_MODE), packet)){
        logCommandAndData(static_cast<int>(JIG_HEADER::AMR_MODE), packet, 1);
        if(!packet.empty() && packet[0] == 1){
            resPonseJigCommand(JIG_HEADER::AMR_MODE,packet);
            ret = UDP_COMMUNICATION::AMR_JIG_MODE;
            setCommnicationMode(UDP_COMMUNICATION::AMR_JIG_MODE);
        }
        else if(!packet.empty() && packet[0] == 0){
            setCommnicationMode(UDP_COMMUNICATION::NORMAL);
            ret = UDP_COMMUNICATION::NORMAL;
        }
    }
    else if (API_RecvJigData(JIG_AP_HEADER::AP_JIG, packet)){
        // logCommandAndData(static_cast<int>(JIG_AP_HEADER::AP_JIG), packet, 1);
        if(!packet.empty() && packet[0] == 1){
            resPonseJigCommand(JIG_AP_HEADER::AP_JIG,packet);
            ret = UDP_COMMUNICATION::AP_JIG_MODE;
            setCommnicationMode(UDP_COMMUNICATION::AP_JIG_MODE);
        }
        else if(!packet.empty() && packet[0] == 0){
            ret = UDP_COMMUNICATION::NORMAL;
            setCommnicationMode(UDP_COMMUNICATION::NORMAL);
        }
    }

    return ret;  
}

void UdpCommunication::udp_callback()
{
    UDP_COMMUNICATION udpCommucation = checkUdpCommunicationMode();

    switch (udpCommucation)
    {
    case UDP_COMMUNICATION::AMR_JIG_MODE: 
        procAmrJigCommunication();
        m_is_starting_comm_SoC = true; //250531 KKS : 지그모드는 통신체크 안함
        break;
    case UDP_COMMUNICATION::AP_JIG_MODE:
        procApJigCommunication();
        m_is_starting_comm_SoC = true; //250531 KKS : 지그모드는 통신체크 안함
        break;
    case UDP_COMMUNICATION::NORMAL:
        procSocCommunication();
        break;
    default:
        RCLCPP_INFO(this->get_logger(), "COMMUNICATION MODE Error");
        break;
    }
}

bool UdpCommunication::isValidRobotPose()
{
    return true;
}
//**********************************************************************************///
// END OF UDP Callback
pose UdpCommunication::getRobotPose(){
    return socData.robotPosition;
}

bool UdpCommunication::isValidTargetPose()
{
    return true;
}

pose UdpCommunication::getTargetPose(){
    return socData.targetPosition;
}

MapInfo UdpCommunication::getMapInfo(){
    return socData.mapInfo;
}

void UdpCommunication::clearMapInfo(){
    
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        socData.mapInfo.bReceived = false;
        socData.mapInfo.resolution = 0.05;
        socData.mapInfo.width = 0;
        socData.mapInfo.height = 0;
        socData.mapInfo.origin_x = 0.0;
        socData.mapInfo.origin_y = 0.0;
        socData.mapInfo.map_data.clear();
    }
    temp_mapsize = 0;
    RCLCPP_INFO(this->get_logger(), "[clearMapInfo] clearMapInfo");
}

void UdpCommunication::clearErrorList(){
    //250531 KKS : direct 호출 금지, mutex 고려 필요
    //socData.errorList.clear();
    RCLCPP_INFO(this->get_logger(), "[clearErrorList] clearErrorList");
}

void UdpCommunication::systemRebootCommand()
{
    RCLCPP_INFO(this->get_logger(), "[systemRebootCommand] start-Reboot");
    int reboot_cmd = std::system("sudo /home/airbot/reboot_script.sh");
    if (reboot_cmd == 0) {
        RCLCPP_INFO(this->get_logger(), "[systemRebootCommand] Reboot command executed successfully.");
    } else {
        RCLCPP_WARN(this->get_logger(), "[systemRebootCommand] Reboot command failed.");
    }          
}

void UdpCommunication::publishBlockArea(const ByPassOne_t& parsedData)
{
    everybot_custom_msgs::msg::BlockAreaList msgs;
    for (const auto& block_area : parsedData.block_area)
    {
        everybot_custom_msgs::msg::BlockArea message;  // Define the message here inside the function

        message.id = block_area.id;  // Set the id for the block area

        for (const auto& position : block_area.robot_path)
        {
            everybot_custom_msgs::msg::Position area_data;
            area_data.x = position.x;
            area_data.y = position.y;
            message.robot_path.push_back(area_data);
            RCLCPP_INFO(this->get_logger(), "blockArea robot path X : %f, Y : %f",area_data.x,area_data.y);
        }

        for (const auto& position : block_area.image_path)
        {
            everybot_custom_msgs::msg::Position area_data;
            area_data.x = position.x;
            area_data.y = position.y;
            RCLCPP_INFO(this->get_logger(), "blockArea image path X : %f, Y : %f",area_data.x,area_data.y);
            //message.image_path.push_back(pos_msg);
        }
        msgs.block_area_list.push_back(message);
    }
    block_area_pub_->publish(msgs);  // Publish the BlockArea message
    if(parsedData.block_area.empty()){
        RCLCPP_INFO(this->get_logger(), "publishBlockArea empty");
    }
}

void UdpCommunication::publishBlockWall(const ByPassOne_t& parsedData)
{
    everybot_custom_msgs::msg::BlockAreaList msgs;
    for (const auto& block_wall : parsedData.block_wall)
    {
        // Assuming block_wall is a BlockArea-like structure or a list of positions
        everybot_custom_msgs::msg::BlockArea wall_list;  // Create BlockArea message for walls

        wall_list.id = block_wall.id;
        // Populate robot_path with block_wall positions (if they're positions)
        for (const auto& wall_position : block_wall.robot_path)
        {
            everybot_custom_msgs::msg::Position wall_data;
            wall_data.x = wall_position.x;
            wall_data.y = wall_position.y;
            wall_list.robot_path.push_back(wall_data);
            RCLCPP_INFO(this->get_logger(), "blockWall robot path X : %f, Y : %f",wall_data.x,wall_data.y);
        }

        for (const auto& position : block_wall.image_path)
        {
            everybot_custom_msgs::msg::Position wall_data;
            wall_data.x = position.x;
            wall_data.y = position.y;
            //wall_list.image_path.push_back(wall_data);
            RCLCPP_INFO(this->get_logger(), "blockWall image path X : %f, Y : %f",wall_data.x,wall_data.y);
        }

        msgs.block_area_list.push_back(wall_list);
    }
    block_wall_pub_->publish(msgs);  // Publish the BlockArea message
    
    if(parsedData.block_wall.empty()){
        RCLCPP_INFO(this->get_logger(), "publishBlockWall empty");
    }
}

void UdpCommunication::publishClearVirtualWall()
{
    everybot_custom_msgs::msg::BlockAreaList empty_msgs;
    block_wall_pub_->publish(empty_msgs);
    block_area_pub_->publish(empty_msgs);
    RCLCPP_INFO(this->get_logger(), "publishClearVirtualWall");
}

void UdpCommunication::publishEmergencyCommand()
{
    std_msgs::msg::Bool e_stop_cmd;
    if(socData.motorInfo.mode != MOTOR_BREAK_MODE){
        e_stop_cmd.data = true; //e stop is called
        emergency_stop_pub_->publish(e_stop_cmd);
    }else{
        RCLCPP_INFO(this->get_logger(), "Motor is Aready Break!");
    }
}

void UdpCommunication::publishChargingCommand(bool start)
{
    std_msgs::msg::UInt8 charge_cmd;
    if(start){
        charge_cmd.data = AUTO_CHARGE_HIGH;
        charge_pub->publish(charge_cmd);
        RCLCPP_INFO(this->get_logger(), "Request-Startcharging");
    }else{
        charge_cmd.data = CHARGE_STOP;
        charge_pub->publish(charge_cmd);
        RCLCPP_INFO(this->get_logger(), "Request- Stopcharging");
    }
}

void UdpCommunication::publishVelocityCommand(double v, double w)
{
    setSocRobotVelocity(v,w);
    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_msg.linear.x = v;
    cmd_msg.angular.z = w;
    manual_move_pub_->publish(cmd_msg);
    //RCLCPP_INFO(this->get_logger(), "publishVelocityCommand V : %f, W : %f ", v,w);
}

void UdpCommunication::publishStateCommand(REQUEST_SOC_CMD cmd)
{   
    std_msgs::msg::UInt8 state_cmd;
    state_cmd.data = static_cast<uint8_t>(cmd);
    RCLCPP_INFO(this->get_logger(), "publishStateCommand data: %u , enum : %s",state_cmd.data,enumToString(cmd).c_str());
    req_soc_cmd_pub_->publish(state_cmd);
}

void UdpCommunication::publishReqCameraType(){
    std_msgs::msg::Empty msg;
    req_camera_type_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "publishReqCameraType");
}

void UdpCommunication::publishReqFWReset(){
    std_msgs::msg::Empty msg;
    fw_reset_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "publishReqFWReset");
}

void UdpCommunication::publishReqAIReset(){
    std_msgs::msg::Empty msg;
    ai_reset_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "publishReqAIReset");
}

void UdpCommunication::publishHeartbeatVersion()
{
    //publishing all verison continously for checking heartbeat(OTA)
    std_msgs::msg::String sw_version_data;
    sw_version_data.data = socData.version.sw_ver;
    sw_version_pub_->publish(sw_version_data);
}
void UdpCommunication::publishStartOdomReset()
{
  bReceiveOdom = false;
  bReceiveOdomStatus = false;
  std_msgs::msg::UInt8 odom_reset_cmd_;
  odom_reset_cmd_.data = 0x01; // Odom IMU Reset
  reset_odom_pub_->publish(odom_reset_cmd_);
  RCLCPP_INFO(this->get_logger(), "publish-StartOdomReset");
  //reset_odom_start_time_ = std::chrono::steady_clock::now();
}

void UdpCommunication::publishClearOdomReset()
{
  std_msgs::msg::UInt8 odom_reset_cmd_;
  odom_reset_cmd_.data = 0x00; // Odom IMU Reset
  reset_odom_pub_->publish(odom_reset_cmd_);
  RCLCPP_INFO(this->get_logger(), "publish-ClearOdomReset");
}

void UdpCommunication::publishTofLeftCalibration(){
    std_msgs::msg::UInt8 msg;
    msg.data = 0x01; //Start-LeftCalibration
    start_tofcalib_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "publishTofLeftCalibration hex[%02x]", msg.data);
}

void UdpCommunication::publishTofRightCalibration(){
   std_msgs::msg::UInt8 msg;
    msg.data = 0x02; //Start-RightCalibration
    start_tofcalib_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "publishTofRightCalibration hex[%02x]", msg.data);
}

void UdpCommunication::publishMapCopyComplete(){
    std_msgs::msg::Empty msg;
    map_copy_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "publishMapCopyComplete");
}

void UdpCommunication::publishReadyReboot(){
    std_msgs::msg::Empty msg;
    ready_reboot_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "publishReadyReboot");
}

void UdpCommunication::publishMapChaged(){
    std_msgs::msg::Empty msg;
    mapinfo_changed_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "publishMapChaged");
}

bool UdpCommunication::copyFile(const std::string& source, const std::string& destination) {
    try {
        std::filesystem::copy_file(source, destination, std::filesystem::copy_options::overwrite_existing);

        // copy 후 fsync 추가
        int fd = ::open(destination.c_str(), O_RDWR);
        if (fd != -1) {
            if (::fsync(fd) == 0) {
                RCLCPP_INFO(this->get_logger(), "copyFile sync success: %s", destination.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "copyFile sync failed (fsync error): %s", destination.c_str());
            }
            ::close(fd);
        } else {
            RCLCPP_WARN(this->get_logger(), "copyFile sync failed (open error): %s", destination.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "source[%s]--> destination[%s] copyFile success", source.c_str(), destination.c_str());
        return true;
    } catch (const std::filesystem::filesystem_error& e) {
        RCLCPP_INFO(this->get_logger(), "copyFile error");
        return false;
    }
}

bool UdpCommunication::removeFile(const std::string& filepath) {
    try {
        if (std::filesystem::exists(filepath)) {
            std::filesystem::remove(filepath);
            RCLCPP_INFO(this->get_logger(), "Deleted file: %s", filepath.c_str());
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "File does not exist: %s", filepath.c_str());
            return false;
        }
    } catch (const std::filesystem::filesystem_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to delete file %s: %s", filepath.c_str(), e.what());
        return false;
    }
}

bool UdpCommunication::convertYamlImagePath(const std::string& sourceYaml, const std::string& destinationYaml, const std::string& newImageName) {
    std::ifstream inFile(sourceYaml);
    if (!inFile.is_open()) {
        RCLCPP_INFO(this->get_logger(), "convertYamlImagePath sourcefile open error");
        return false;
    }

    std::ofstream outFile(destinationYaml);
    if (!outFile.is_open()) {
        RCLCPP_INFO(this->get_logger(), "convertYamlImagePath destfile open error");
        return false;
    }

    std::string line;
    while (std::getline(inFile, line)) {
        if (line.find("image:") != std::string::npos) { // "image:"로 시작하면
            outFile << "image: " << newImageName << "\n";
            RCLCPP_INFO(this->get_logger(), "convertYamlImagePath success newImageName : %s", newImageName.c_str());
        } else {
            outFile << line << "\n";
            RCLCPP_INFO(this->get_logger(), "convertYaml data success line : %s", line.c_str());
        }
    }

    inFile.close();
    outFile.close();
    return true;
}

bool UdpCommunication::updateMapFiles(const std::string& pgmSrc, const std::string& yamlSrc, const std::string& pgmDst, const std::string& yamlDst) {
    try {
        std::filesystem::create_directories(std::filesystem::path(pgmDst).parent_path());
    RCLCPP_INFO(this->get_logger(), "Ensured destination directory exists: %s", std::filesystem::path(pgmDst).parent_path().c_str());

    if (!copyFile(pgmSrc, pgmDst)) return false;

    std::string pgmFileName = std::filesystem::path(pgmDst).filename().string();
    if (!convertYamlImagePath(yamlSrc, yamlDst, pgmFileName)) return false;

    return true;
    } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "std::exception occurred during map update: %s", e.what());
    return false;
    }
}
void UdpCommunication::savePGMFile(const ModifiedMapDataB_t &settings, const std::string &filename)
{
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open())
    {
        RCLCPP_INFO(this->get_logger(), "Unable to open file for writing");
        throw std::runtime_error("Unable to open file for writing");
    }
    RCLCPP_INFO(this->get_logger(), "settings size : %lu, width : %f, height : %f", settings.data.size(), settings.width, settings.height);

    file.write(reinterpret_cast<const char *>(settings.data.data()), settings.data.size());

    file.close();
}

bool UdpCommunication::readPGM(const std::string& pgm_filename, int& width_map, int& height_map, 
            std::vector<uint8_t>& mapData, double& origin_x, double& origin_y) {

    // Reading the PGM file
    std::ifstream file(pgm_filename, std::ios::binary);
    if (!file.is_open()) {
        RCLCPP_INFO(this->get_logger(), "Error: Could not open file named %s", pgm_filename.c_str());
        return false;
    }

    std::string line;
    std::getline(file, line);
    if (line != "P5") {
        RCLCPP_INFO(this->get_logger(), "Error: Unsupported file format (only P5 is supported)");
        return false;
    }

    // Skip comments
    while (std::getline(file, line) && line[0] == '#') {
        // Skip comment lines
    }

    std::istringstream sizeStream(line);
    sizeStream >> width_map >> height_map;

    std::getline(file, line);
    std::istringstream maxValueStream(line);
    int maxValue;
    maxValueStream >> maxValue;

    // Check for valid maxValue
    if (maxValue != 255) {
        RCLCPP_INFO(this->get_logger(), "Error: Only 8-bit PGM files are supported");
        return false;
    }

    // Read pixel data
    mapData.resize(width_map * height_map);
    file.read(reinterpret_cast<char*>(mapData.data()), mapData.size());

    if (!file) {
        RCLCPP_INFO(this->get_logger(), "Error: Reading pixel data failed");
        return false;
    }

    file.close(); // Close the PGM file after reading

    // Derive the YAML file name from the PGM file name
    std::string yaml_filename = pgm_filename.substr(0, pgm_filename.find_last_of('.')) + ".yaml";
    
    std::ifstream file1(yaml_filename);
    if (!file1.is_open()) {
        RCLCPP_INFO(this->get_logger(), "Error: Could not open file %s", yaml_filename.c_str());
        return false;
    }
    // Reading the YAML file
    std::string line1;
    while (std::getline(file1, line1)) {
        if (line1.find("origin:") != std::string::npos) {
            std::istringstream ss(line1.substr(line1.find('[') + 1, line1.find(']') - line1.find('[') - 1));
            char comma;
            ss >> origin_x >> comma >> origin_y;
            if (ss.fail() || comma != ',') {
                RCLCPP_INFO(this->get_logger(), "Error: Failed to parse 'origin' in YAML file");
                return false;
            }
            return true;
        }
    }

    return true;
}

std::vector<uint8_t> UdpCommunication::mapDataTypeConvert(const std::vector<int8_t>& src, uint32_t height, uint32_t width)
{
    // 동적 메모리 할당
    if (height == 0 || width == 0) {
        RCLCPP_WARN(this->get_logger(),
                    "mapDataTypeConvert: invalid dims width=%u height=%u", width, height);
        return {};
    }
    //unsigned char* mapData = new unsigned char[height * width];
    std::vector<uint8_t> mapData(height * width);
    const size_t map_size = src.size();
    const size_t expected_size = static_cast<size_t>(width) * static_cast<size_t>(height);

    if((map_size != expected_size) || src.empty())
    {
        if(src.empty()){
            RCLCPP_INFO(this->get_logger(), "mapDataTypeConvert Error: src is empty");
        }else{
            RCLCPP_INFO(this->get_logger(), "mapDataTypeConvert Error: src size is unexpected width : %u, height : %u, size : %zu",width,height,map_size);
        }
        return {};
    }

    for (size_t y = 0; y < height; ++y)
    {
        for (size_t x = 0; x < width; ++x)
        {
            const size_t index = y*width + x;//width
            const int value = static_cast<int>(src[index]);
            const size_t transformed_x = x;  //
            const size_t transformed_y = (height-1) - y;  //
            const size_t transformed_index = transformed_y * width + transformed_x;

            uint8_t pixel_value;
            if (value == -1)  // OccupancyGrid에서 -1은 보통 'unknown'을 의미
            {
                pixel_value = 127; // Gray for unknown
            }
            else if (value >= 0 && value <= 100)
            {
                pixel_value = static_cast<uint8_t>(255 - (value * 255) / 100); // Scale 0-100 to 0-255                
            }
            else
            {
                pixel_value = 128; // Default gray for unexpected values
            }

            mapData[transformed_index] = pixel_value;                           
        }
    }
    
    return mapData;  // 동적으로 할당된 메모리를 반환
}

double UdpCommunication::quaternion_to_euler(const geometry_msgs::msg::Quaternion& quat)
{
    tf2::Quaternion q;
    tf2::fromMsg(quat, q);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw; // Return yaw as theta
}

uint8_t UdpCommunication::getLowerBits(uint8_t byte) {
    return (byte & 0x0F);
}

// 상위 4비트를 반환하는 함수
uint8_t UdpCommunication::getUpperBits(uint8_t byte) {
    return ((byte >> 4) & 0x0F);
}

//응답
void UdpCommunication::resPonseJigCommand(int command, const std::vector<uint8_t>& packet)
{
    logCommandAndData(command, packet, 2);
    API_SendJigData(command, packet);
}

//데이터 파싱
void UdpCommunication::resPonseJigData(JIG_DATA_KEY key)
{
    logCommandAndData(static_cast<int>(key),getJigData(key), 2);
    API_SendJigData(static_cast<int>(key),getJigData(key));
}
/**
 * @brief 지그 응답 패킷에 0값을 넣어주는 함수
 * 
 *       지그 패킷 갯수가 항상 일정해야하기 때문에 0값을 생성하여 맞춰줌
 *  
 * @param header 
 * @param packet 
 * @return std::vector<uint8_t> 
 */
std::vector<uint8_t> UdpCommunication::makeResponseJigCommandPacket(JIG_HEADER header, std::vector<uint8_t> packet)
{
    std::vector<uint8_t> ret;
    JIG_DATA_KEY jig_data_key;

    switch(header) {
        case JIG_HEADER::WHEEL_MOTOR:
            jig_data_key = JIG_DATA_KEY::WHEEL_MOTOR;
            break;
        case JIG_HEADER::BATTERY:
            jig_data_key = JIG_DATA_KEY::BATTERY;
            break;
        case JIG_HEADER::IMU_CALIBRATION:
            jig_data_key = JIG_DATA_KEY::IMU_CALIBRATION;
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "jig Command Response Protocol Error!!");
            ret.resize(1);
            return ret;
            break;
    }

    ret.resize(jigData[jig_data_key].size());
    std::copy(jigData[jig_data_key].begin(), jigData[jig_data_key].end(), ret.begin());

    if (packet.size() > ret.size()) {
        RCLCPP_WARN(this->get_logger(), "Packet size (%zu) exceeds response size (%zu), truncating.", packet.size(), ret.size());
    }

    if (!packet.empty()) {
        size_t copy_size = std::min(ret.size(), packet.size());
        std::copy(packet.begin(), packet.begin() + copy_size, ret.begin());
    }

    return ret;
}

void UdpCommunication::jigCheckWheelMotor(const std::vector<uint8_t>& packet)
{
    if(packet[0] == 0x00){
        resPonseJigData(JIG_DATA_KEY::WHEEL_MOTOR);
    }
    else if (packet[0] == 0x01){
        std::vector<uint8_t> newPacket = makeResponseJigCommandPacket(JIG_HEADER::WHEEL_MOTOR,packet);
        resPonseJigCommand(JIG_HEADER::WHEEL_MOTOR,newPacket);
        everybot_custom_msgs::msg::RpmControl rpmCtrlMsg;
        rpmCtrlMsg.motor_enable = packet[0];
        rpmCtrlMsg.left_motor_rpm_msb = packet[1];
        rpmCtrlMsg.left_motor_rpm_lsb = packet[2];
        rpmCtrlMsg.right_motor_rpm_msb = packet[3];
        rpmCtrlMsg.right_motor_rpm_lsb = packet[4];
        jig_request_motor_pub_->publish(rpmCtrlMsg);
    }else{
        RCLCPP_INFO(this->get_logger(), "jigCheckWheelMotor Protocol Error!!");
    }
}

void UdpCommunication::jigCheckBattery(const std::vector<uint8_t>& packet)
{
    if(packet[0] == 0x00){
        resPonseJigData(JIG_DATA_KEY::BATTERY); 
    }else if(packet[0] & 0x03){
        std::vector<uint8_t> newPacket = makeResponseJigCommandPacket(JIG_HEADER::BATTERY,packet);
        resPonseJigCommand(JIG_HEADER::BATTERY,newPacket);
        std_msgs::msg::UInt8 chargingModeMsg;
        chargingModeMsg.data = packet[0];
        jig_request_battery_pub_->publish(chargingModeMsg);
        //everybot_custom_msgs::msg::RpmControl rpmCtrlMsg;
    }else{
        RCLCPP_INFO(this->get_logger(), "jigCheckBattery Protocol Error!!");
    }  
}

void UdpCommunication::jigCheckCliffLift()
{
    resPonseJigData(JIG_DATA_KEY::CLIFF_LIFT);
}

void UdpCommunication::jigCheckDockReceiver()
{
    resPonseJigData(JIG_DATA_KEY::DOCK_RECEIVER);
}

void UdpCommunication::jigCheckTofSensor(const std::vector<uint8_t>& packet)
{   
    RCLCPP_INFO(this->get_logger(), "jigCheckTofSensor Received packet of size: %zu", packet.size());
    resPonseJigData(JIG_DATA_KEY::TOF);
} 

void UdpCommunication::jigCheckFrontLidar()
{
    bRequestFrontLidarDist = true;
    resPonseJigData(JIG_DATA_KEY::FRONT_LIDAR);
}

void UdpCommunication::jigCheckRearLidar()
{
    bRequestBackLidarDist = true;
    resPonseJigData(JIG_DATA_KEY::REAR_LIDAR);
}

void UdpCommunication::jigCheckImuCalibration(const std::vector<uint8_t>& packet)
{
    if(packet[0] == 0x00){
        resPonseJigData(JIG_DATA_KEY::IMU_CALIBRATION);
    }else if(packet[0] & 0x03){
        std::vector<uint8_t> newPacket = makeResponseJigCommandPacket(JIG_HEADER::IMU_CALIBRATION,packet);
        resPonseJigCommand(JIG_HEADER::IMU_CALIBRATION,newPacket);
        std_msgs::msg::UInt8 imu_cal_msg;
        imu_cal_msg.data = packet[0];
        jig_request_imu_pub_->publish(imu_cal_msg);
    }else{
        RCLCPP_INFO(this->get_logger(), "jigCheckImuCalibration Protocol Error!!");
    }
}

void UdpCommunication::jigCheckVersion()
{
    std::string ver = socData.version.sw_ver + ":" + socData.version.mcu_ver;
    std::vector<uint8_t> data;
    data.insert(data.end(), ver.begin(), ver.end());
    setJigData(JIG_DATA_KEY::SW_FW_VERSION, data);
    resPonseJigData(JIG_DATA_KEY::SW_FW_VERSION);
}

void UdpCommunication::jigCheckBatteryShippingMode()
{
    publishBatterySleep();
    resPonseJigData(JIG_DATA_KEY::BATTERY_SHIPPING_MODE);
}

void UdpCommunication::jigCheckToFLeftOffsetSetup()
{
    std::vector<uint8_t> data;
    data.push_back(0x01);
    setJigData(JIG_DATA_KEY::TOF_LEFT_OFFSET_SETUP, data);
    resPonseJigData(JIG_DATA_KEY::TOF_LEFT_OFFSET_SETUP);
    tof_calibState = (tof_calibState & 0xF0) | 0x01;
    temp_tof_calibcmd = 0x01;
    publishTofLeftCalibration();
    tof_calib_start_time_ = std::chrono::steady_clock::now();

    if(!tof_calib_timer_){
        tof_calib_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&UdpCommunication::tofCalibrationTimerCallback, this));
        RCLCPP_INFO(this->get_logger(), "Start tof calibration timer");
    }else{
        RCLCPP_INFO(this->get_logger(), "tof calibration timer already started");
    }
    RCLCPP_INFO(this->get_logger(), "[jigCheckToFLeftOffsetSetup] Start tof left calibration");
}

void UdpCommunication::jigCheckToFRightOffsetSetup()
{
    std::vector<uint8_t> data;
    data.push_back(0x01);
    setJigData(JIG_DATA_KEY::TOF_RIGHT_OFFSET_SETUP, data);
    resPonseJigData(JIG_DATA_KEY::TOF_RIGHT_OFFSET_SETUP);
    tof_calibState = (tof_calibState & 0x0F) | (0x01 << 4);
    temp_tof_calibcmd = 0x02;
    publishTofRightCalibration();
    tof_calib_start_time_ = std::chrono::steady_clock::now();

    if(!tof_calib_timer_){
        tof_calib_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&UdpCommunication::tofCalibrationTimerCallback, this));
        RCLCPP_INFO(this->get_logger(), "Start tof calibration timer");
    }else{
        RCLCPP_INFO(this->get_logger(), "tof calibration timer already started");
    }
    RCLCPP_INFO(this->get_logger(), "[jigCheckToFRightOffsetSetup] Start tof right calibration");
}

void UdpCommunication::jigProcessor(int header, const std::vector<uint8_t>& packet)
{
    switch (header)
    { 
    case JIG_HEADER::WHEEL_MOTOR:
        jigCheckWheelMotor(packet);
        break;
    case JIG_HEADER::BATTERY:
        jigCheckBattery(packet);
        break;
    case JIG_HEADER::CLIFF_LIFT:
        jigCheckCliffLift();
        break;
    case JIG_HEADER::DOCK_RECEIVER:
        jigCheckDockReceiver();
        break;
    case JIG_HEADER::TOF:
        jigCheckTofSensor(packet);
        break;
    case JIG_HEADER::FRONT_LIDAR:
        jigCheckFrontLidar();
        break;
    case JIG_HEADER::REAR_LIDAR:
        jigCheckRearLidar();
        break;
    case JIG_HEADER::IMU_CALIBRATION:
        jigCheckImuCalibration(packet);
        break;
    case JIG_HEADER::SW_FW_VERSION:
        jigCheckVersion();
        break;
    case JIG_HEADER::BATTERY_SHIPPING_MODE:
        jigCheckBatteryShippingMode();
        break;
    case JIG_HEADER::TOF_LEFT_OFFSET_SETUP:
        jigCheckToFLeftOffsetSetup();
        break;
    case JIG_HEADER::TOF_RIGHT_OFFSET_SETUP:
        jigCheckToFRightOffsetSetup();
        break;
    default:
        break;
    }
}

std::vector<uint8_t> UdpCommunication::jigDataConvertWheelMotor(const everybot_custom_msgs::msg::MotorStatus::SharedPtr msg)
{
    std::vector<uint8_t> ret(11);
    
    uint8_t motor_mode = msg->motor_mode & 0xF0; // 상위 4비트: 모터 모드
    // uint8_t romotecontroller_mode = msg->motor_mode & 0x0F; // 하위 4비트: 리모컨 모드
   
    switch(motor_mode){
        case 0x00: ret[0] = MOTOR_DEFAULT_MODE; break;
        case 0x10: ret[0] = MOTOR_RPM_MODE; break;  //hjkim 250806 : remote block 기능 추가하면서 motor mode 가 상위 비트로 변경되었음. --> 지그 검사 시 오류 메세지 출력되어 수정
        default: 
            ret[0] = MOTOR_MODE_ERROR; 
            RCLCPP_WARN(this->get_logger(), "Invalid motor mode received: 0x%02X", msg->motor_mode);
            break;
    } 

    ret[1] = static_cast<uint8_t>((static_cast<int>(msg->left_motor_rpm) >> 8) & 0xFF);
    ret[2] = static_cast<uint8_t>(static_cast<int>(msg->left_motor_rpm) & 0xFF);
    ret[3] = static_cast<uint8_t>((static_cast<int>(msg->right_motor_rpm) >> 8) & 0xFF);
    ret[4] = static_cast<uint8_t>(static_cast<int>(msg->right_motor_rpm) & 0xFF);
    ret[5] = static_cast<uint8_t>((static_cast<int>(msg->left_motor_current) >> 8) & 0xFF);
    ret[6] = static_cast<uint8_t>(static_cast<int>(msg->left_motor_current) & 0xFF);
    ret[7] = static_cast<uint8_t>((static_cast<int>(msg->right_motor_current) >> 8) & 0xFF);
    ret[8] = static_cast<uint8_t>(static_cast<int>(msg->right_motor_current) & 0xFF);
    ret[9] = static_cast<uint8_t>(msg->left_motor_temperature);
    ret[10] = static_cast<uint8_t>(msg->right_motor_temperature);
    return ret;
}

std::vector<uint8_t> UdpCommunication::jigDataConvertBattery(const everybot_custom_msgs::msg::BatteryStatus::SharedPtr msg)
{
    std::vector<uint8_t> ret;
    ret.resize(11);

    switch (msg->charge_status) {
        case 0x00: ret[0] = BATTERY_DEFAULT_MODE; break;
        case 0x02: ret[0] = BATTERY_HIGH_SPEED_MODE; break;
        case 0x03: ret[0] = BATTERY_LOW_SPEED_MODE; break;
        default:   
            ret[0] = BATTERY_MODE_ERROR; 
            RCLCPP_WARN(this->get_logger(), "Invalid battery mode received: 0x%02X", msg->charge_status);
            break;
    }
    
    ret[1] = static_cast<uint8_t>((msg->number_of_cycles >> 8) & 0xFF);
    ret[2] = static_cast<uint8_t>(msg->number_of_cycles & 0xFF);
    ret[3] = static_cast<uint8_t>((static_cast<int>(msg->battery_voltage) >> 8) & 0xFF);
    ret[4] = static_cast<uint8_t>(static_cast<int>(msg->battery_voltage) & 0xFF);
    ret[5] = static_cast<uint8_t>(msg->battery_temperature1 & 0xFF);
    ret[6] = static_cast<uint8_t>(msg->battery_temperature2 & 0xFF);
    ret[7] = static_cast<uint8_t>(msg->battery_percent & 0xFF);
    ret[8] = static_cast<uint8_t>((static_cast<int>(msg->battery_current) >> 8) & 0xFF);
    ret[9] = static_cast<uint8_t>(static_cast<int>(msg->battery_current) & 0xFF);
    ret[10] = static_cast<uint8_t>(static_cast<int>(msg->battery_version) & 0xFF);

    return ret;
}

std::vector<uint8_t> UdpCommunication::jigDataConvertTof(const everybot_custom_msgs::msg::TofData::SharedPtr msg)
{
    std::vector<uint8_t> ret(64);
     for(int i = 0; i < 32; i += 2){
        int idx_right = i;
        int idx_left = i+32;
        uint16_t temp_right = static_cast<uint16_t>(msg->bot_right[i/2]*1000);
        uint16_t temp_left = static_cast<uint16_t>(msg->bot_left[i/2]*1000);
        ret[idx_right]  = static_cast<uint8_t>((temp_right >> 8) & 0xFF);
        ret[idx_right+1] = static_cast<uint8_t>(temp_right & 0xFF);
        ret[idx_left] = static_cast<uint8_t>((temp_left >> 8) & 0xFF);  
        ret[idx_left+1] = static_cast<uint8_t>(temp_left & 0xFF);
    }
    return ret;
}

std::vector<uint8_t> UdpCommunication::jigDataConvertLidar(int long_dist, int short_dist)
{
    std::vector<uint8_t> ret;
    if(long_dist != std::numeric_limits<int>::max() && short_dist != std::numeric_limits<int>::max()){
        ret.push_back(static_cast<uint8_t>((long_dist >> 8) & 0xFF));
        ret.push_back(static_cast<uint8_t>(long_dist & 0xFF));
        ret.push_back(static_cast<uint8_t>((short_dist >> 8) & 0xFF));
        ret.push_back(static_cast<uint8_t>(short_dist & 0xFF));
    }
    
    return ret;
}

std::vector<uint8_t> UdpCommunication::jigDataConvertImuCalibration(const everybot_custom_msgs::msg::ImuCalibration::SharedPtr msg)
{
    std::vector<uint8_t> ret;
    ret.push_back(msg->calibration_status);
    ret.push_back(static_cast<uint8_t>((msg->roll >> 8) & 0xFF)); // high byte
    ret.push_back(static_cast<uint8_t>(msg->roll & 0xFF)); // low byte
    ret.push_back(static_cast<uint8_t>((msg->pitch >> 8) & 0xFF)); // high byte
    ret.push_back(static_cast<uint8_t>(msg->pitch & 0xFF)); // low byte
    ret.push_back(static_cast<uint8_t>((msg->yaw >> 8) & 0xFF)); // high byte
    ret.push_back(static_cast<uint8_t>(msg->yaw & 0xFF)); // low byte

    return ret;
}

void UdpCommunication::splitLidarScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, std::vector<int>& long_dist, std::vector<int>& short_dist)
{
    float current_angle = msg->angle_min;
    // 'i'를 size_t로 변경하여 워닝 제거
    for (size_t i = 0; i < msg->ranges.size(); i++, current_angle += msg->angle_increment) {
        float range = msg->ranges[i];

        if (!std::isfinite(range) || range <= 0.001) {
            continue;
        }

        int dist = static_cast<int>(range * 1000);
        if (abs(dist) <= 1) {
            continue;
        }

        // 라이다 긴 장애물 범위 확인
        if (current_angle >= JIG_LIDAR_LONG_DIST_ANGLE_MIN && current_angle <= JIG_LIDAR_LONG_DIST_ANGLE_MAX) {
            long_dist.push_back(dist);
        }
        // 라이다 짧은 장애물 범위 확인
        else if (current_angle >= JIG_LIDAR_SHORT_DIST_ANGLE_MIN && current_angle <= JIG_LIDAR_SHORT_DIST_ANGLE_MAX) {
            short_dist.push_back(dist);
        }

        // 지그 각도 디버깅
        if (bRequestFrontLidarDist || bRequestBackLidarDist) {
            if (dist >= 900 && dist <= 1100) {
                float deg = current_angle * 180.0f / M_PI;
                if (deg >= 120.0f && deg <= 250.0f) {
                    RCLCPP_INFO(this->get_logger(), "Long angle in target distance range: %.2f deg (dist: %d mm)", deg, dist);
                }
            }
            if (dist >= 200 && dist <= 400) {
                float deg = current_angle * 180.0f / M_PI;
                if (deg >= 30.0f && deg <= 150.0f) {
                    RCLCPP_INFO(this->get_logger(), "Short angle in target distance range: %.2f deg (dist: %d mm)", deg, dist);
                }
            }
        }
    }
}

int UdpCommunication::getMinDistanceFromLidarSensor(const std::vector<int>& vecDistance)
{
    int ret = 0;
    if(!vecDistance.empty()){
        ret = *std::min_element(vecDistance.begin(), vecDistance.end());
    }
    return ret;
}

void UdpCommunication::apJigProcessor(int header, const std::vector<uint8_t>& packet)
{
    RCLCPP_INFO(this->get_logger(), "apJigProcessor Received packet of size: %zu", packet.size());

    switch (header)
    {
    case JIG_AP_HEADER::AP_JIG_RAM_MEMORY:
        apJigCheckRam();
        break;
    case JIG_AP_HEADER::AP_JIG_DISK_MEMORY:
        apJigCheckEmmc();
        break;
    case JIG_AP_HEADER::AP_JIG_FRONT_LIDAR:
        apJigCheckFrontLiDAR();
        break;
    case JIG_AP_HEADER::AP_JIG_BACK_LIDAR:
        apJigCheckBackLiDAR();
        break;
    case JIG_AP_HEADER::AP_JIG_VERSION:
        apJigVersion();
        break;
    default:
        RCLCPP_INFO(this->get_logger(), "Unknown command");
        break;
    }
}
//------------------------------------------------------
//everybot@everybot-edu:~/ros2_ws$ cat /proc/meminfo 
//MemTotal:       15992188 kB
//MemFree:         6279864 kB
//MemAvailable:   11137800 kB
// 구조체 데이터를 전송하는 함수
void UdpCommunication::sendRamData(const std::vector<uint8_t>& ram_info)
{
    try {
        // UDP 값전송
        resPonseJigCommand(JIG_AP_HEADER::AP_JIG_RAM_MEMORY,ram_info);
        // serial_.write(reinterpret_cast<const uint8_t*>(&ram_info.size[0]), ram_info.size.size());
        
        // 전송된 데이터 값을 정수 및 헥사로 출력
        for (size_t i = 0; i < ram_info.size(); i += 4) {
            uint32_t value = *reinterpret_cast<const uint32_t*>(&ram_info[i]);
            value = __builtin_bswap32(value);
            RCLCPP_INFO(this->get_logger(), "RAM value (int): %u, Hex: 0x%08X", value, value);
        }

        RCLCPP_INFO(this->get_logger(), "RAM data sent successfully.");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "std::exception Failed to send RAM data: %s", e.what());
    }
}

void UdpCommunication::apJigCheckRam()
{
    RCLCPP_INFO(this->get_logger(), "Starting RAM check...");

    std::ifstream meminfo("/proc/meminfo");
    if (!meminfo.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Could not open /proc/meminfo");
        return;
    }

    std::string line;
    std::vector<uint8_t> ram_info(12);  // RAM 정보를 담을 벡터 초기화

    // /proc/meminfo 파일에서 필요한 정보를 파싱
    while (std::getline(meminfo, line)) {
        if (line.find("MemTotal:") == 0) {
            uint32_t mem_total;
            sscanf(line.c_str(), "MemTotal: %u kB", &mem_total);  // MemTotal 값을 추출
            mem_total = __builtin_bswap32(mem_total); // 바이트 순서 변경
            memcpy(ram_info.data(), &mem_total, sizeof(mem_total)); // MemTotal 값을 구조체에 저장
        } else if (line.find("MemFree:") == 0) {
            uint32_t mem_free;
            sscanf(line.c_str(), "MemFree: %u kB", &mem_free);    // MemFree 값을 추출
            mem_free = __builtin_bswap32(mem_free);
            memcpy(ram_info.data() + 4, &mem_free, sizeof(mem_free)); // MemFree 값을 구조체에 저장
        } else if (line.find("MemAvailable:") == 0) {
            uint32_t mem_available;
            sscanf(line.c_str(), "MemAvailable: %u kB", &mem_available);  // MemAvailable 값을 추출
            mem_available = __builtin_bswap32(mem_available);
            memcpy(ram_info.data() + 8, &mem_available, sizeof(mem_available)); // MemAvailable 값을 구조체에 저장
        }
    }

    meminfo.close();

    sendRamData(ram_info);
}

//구조체 데이터를 전송하는 함수
void UdpCommunication::sendEmmcData(const std::vector<uint8_t>& emmc_info)
{
    try {
        // UDP 값전송
        resPonseJigCommand(JIG_AP_HEADER::AP_JIG_DISK_MEMORY, emmc_info);
        // Transmit eMMC data (4 bytes)
        // serial_.write(reinterpret_cast<const uint8_t*>(emmc_info), emmc_info.size());
        
        // Log sent eMMC data
        for (size_t i = 0; i < emmc_info.size(); ++i) {
            uint32_t value = static_cast<uint32_t>(emmc_info[i]);
            RCLCPP_INFO(this->get_logger(), "eMMC value (int): %u, Hex: 0x%02X", value, value);
        }

        RCLCPP_INFO(this->get_logger(), "eMMC data sent successfully.");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "std::exception Failed to send eMMC data: %s", e.what());
    }
}
//------------------------------------------------------
//EMMC Information:
//mmcblk0    14.9G disk
//구조체 데이터를 전송하는 함수
void UdpCommunication::apJigCheckEmmc()
{
    RCLCPP_INFO(rclcpp::get_logger("EMMC"), "Starting EMMC check...");

    std::array<char, 128> buffer;
    std::string result;

    // EMMC 정보 확인을 위한 'lsblk' 명령어 실행 (mmcblk로 시작하는 디바이스 필터링)
    FILE* pipe = popen("lsblk -o NAME,SIZE --noheadings | grep -i '^mmcblk0'", "r");
    if (!pipe) {
        RCLCPP_ERROR(rclcpp::get_logger("EMMC"), "Failed to run EMMC check command");
        return;
    }

    // 명령어 출력 결과를 읽어서 result에 저장
    if (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
        result = buffer.data();
    }

    // 명령어 종료
    int returnCode = pclose(pipe);
    if (returnCode != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("EMMC"), "Command exited with code %d", returnCode);
        return;
    }

    // 디바이스가 발견되지 않은 경우 예외 처리
    if (result.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("EMMC"), "No EMMC device found.");
        return;
    }

    // 결과 로그 출력
    RCLCPP_INFO(rclcpp::get_logger("EMMC"), "EMMC check completed: \n%s", result.c_str());

    std::vector<uint8_t> emmc_info(4);  // EMMC 정보를 담을 벡터 초기화

    // 결과에서 사이즈 파싱
    std::istringstream iss(result);
    std::string device_name, size_str;
    if (iss >> device_name >> size_str) {
        // 사이즈 문자열에서 단위 문자("G", "M" 등) 제거
        char unit = size_str.back();
        size_str.pop_back();
        uint32_t size_in_kb = std::stoi(size_str);

        // 단위에 따라 KB로 변환
        if (unit == 'G') {
            size_in_kb *= 1024 * 1024;  // GB to KB
        } else if (unit == 'M') {
            size_in_kb *= 1024;  // MB to KB
        }

        // EMMC 크기를 구조체에 저장
        emmc_info[0] = static_cast<uint8_t>(size_in_kb >> 24);  // 가장 상위 바이트
        emmc_info[1] = static_cast<uint8_t>(size_in_kb >> 16);  // 두 번째 바이트
        emmc_info[2] = static_cast<uint8_t>(size_in_kb >> 8);   // 세 번째 바이트
        emmc_info[3] = static_cast<uint8_t>(size_in_kb);        // 가장 하위 바이트

        // EMMC 크기를 로그로 출력
        RCLCPP_INFO(rclcpp::get_logger("EMMC"), "EMMC Size: %u KB (size[0]: 0x%02X, size[1]: 0x%02X, size[2]: 0x%02X, size[3]: 0x%02X)",
                size_in_kb, emmc_info[0], emmc_info[1], emmc_info[2], emmc_info[3]);
    }

    // 구조체 데이터를 바이너리로 전송
    sendEmmcData(emmc_info);
}
//----------------------------------------------
// LiDAR 데이터 전송 함수
void UdpCommunication::sendLidarData(int header, const std::vector<uint8_t>& lidar_info)
{
    try {
        // LiDAR 데이터 전송 (2바이트)
        // UDP 값전송
        resPonseJigCommand(header, lidar_info);
        
        // 전송된 데이터 값을 정수 및 헥사로 출력
        for (size_t i = 0; i < lidar_info.size(); ++i) { // 벡터의 크기 사용
            int distance_value = static_cast<int>(lidar_info[i]);
            RCLCPP_INFO(this->get_logger(), "Distance value (int): %d, Hex: 0x%02X", distance_value, distance_value);
        }

        RCLCPP_INFO(this->get_logger(), "LiDAR data sent successfully.");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "std::exception Failed to send LiDAR data: %s", e.what());
    }
}

void UdpCommunication::sendVersionData(int header, const std::vector<uint8_t>& ver_info)
{
    try {
        // LiDAR 데이터 전송 (2바이트)
        // UDP 값전송
        resPonseJigCommand(header, ver_info);

        // 전송된 데이터 값을 정수 및 헥사로 출력
        for (size_t i = 0; i < ver_info.size(); ++i) { // 벡터의 크기 사용
            int version_ascii_value = static_cast<int>(ver_info[i]);
            RCLCPP_INFO(this->get_logger(), "Version ASCII value (int): %d, Hex: 0x%02X", version_ascii_value, version_ascii_value);
        }

        RCLCPP_INFO(this->get_logger(), "Version data sent successfully.");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "std::exception Failed to send Version data: %s", e.what());
    }
}

void UdpCommunication::apJigCheckFrontLiDAR()
{
    std::lock_guard<std::mutex> lock(scan_front_mutex_);
    RCLCPP_INFO(this->get_logger(), "Starting front check...");

    std::vector<uint8_t> lidar_info(2);
    unsigned int valid_scan_cnt = 0;
    if (apJigFrontLaserData && !apJigFrontLaserData->ranges.empty()) {
#if AP_JIG_CHECK_ON_AMR
        for (auto range : apJigFrontLaserData->ranges) {
            if (range > 0.0 && range < std::numeric_limits<float>::infinity()) {
                valid_scan_cnt += 1;
            }
        }
#else
        float first_range = apJigFrontLaserData->ranges[0];
#endif
    RCLCPP_INFO(this->get_logger(), "Total Num of Front LiDAR ranges: %ld", apJigFrontLaserData->ranges.size());
    RCLCPP_INFO(this->get_logger(), "Num of Front LiDAR ranges: %d", valid_scan_cnt);
    } else {
        RCLCPP_WARN(this->get_logger(), "No front LiDAR data available.");
    }
    lidar_info[0] = static_cast<uint8_t>((valid_scan_cnt >> 8) & 0xFF); // 상위 바이트
    lidar_info[1] = static_cast<uint8_t>(valid_scan_cnt & 0xFF); // 하위 바이트
    sendLidarData(JIG_AP_HEADER::AP_JIG_FRONT_LIDAR, lidar_info);
    apJigFrontLaserData.reset();
}

void UdpCommunication::apJigCheckBackLiDAR()
{
    std::lock_guard<std::mutex> lock(scan_back_mutex_);    
    RCLCPP_INFO(this->get_logger(), "Starting back check...");

    std::vector<uint8_t> lidar_info(2);
    unsigned int valid_scan_cnt = 0;
    if (apJigBackLaserData && !apJigBackLaserData->ranges.empty()) {
#if AP_JIG_CHECK_ON_AMR
        for (auto range : apJigBackLaserData->ranges) {
            if (range > 0.0 && range < std::numeric_limits<float>::infinity()) {
                valid_scan_cnt += 1;
            }
        }
#else
        float first_range = apJigBackLaserData->ranges[0];
#endif
        RCLCPP_INFO(this->get_logger(), "Total Num of Back LiDAR ranges: %ld", apJigBackLaserData->ranges.size());
        RCLCPP_INFO(this->get_logger(), "Num of Back LiDAR ranges: %d", valid_scan_cnt);
    } else {
        RCLCPP_WARN(this->get_logger(), "No back LiDAR data available.");
    }
    lidar_info[0] = static_cast<uint8_t>((valid_scan_cnt >> 8) & 0xFF); // 상위 바이트
    lidar_info[1] = static_cast<uint8_t>(valid_scan_cnt & 0xFF); // 하위 바이트
    sendLidarData(JIG_AP_HEADER::AP_JIG_BACK_LIDAR, lidar_info);
    apJigBackLaserData.reset();
}

void UdpCommunication::apJigVersion() {
    RCLCPP_INFO(this->get_logger(), "Starting Version check...");
    std::vector<uint8_t> ap_jig_version;
    ap_jig_version.insert(ap_jig_version.end(), socData.version.total_ver.begin(), socData.version.total_ver.end());
    sendVersionData(JIG_AP_HEADER::AP_JIG_VERSION, ap_jig_version);
}


double UdpCommunication::getDistance(pose base, pose current) {
    return std::sqrt((current.x - base.x) * (current.x - base.x) + (current.y - base.y) * (current.y - base.y));
}

std::string UdpCommunication::readVersionFromIni(const std::string& filename, const std::string& section, const std::string& key) {
    std::ifstream file(filename);
    if (!file.is_open()) {        
        return "";
    }

    std::string line;
    bool inSection = false;
    while (std::getline(file, line)) {
        // 공백 제거
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);

        // 섹션 확인
        if (line.empty() || line[0] == ';' || line[0] == '#') continue;  // 주석 무시
        if (line[0] == '[') {  // 섹션 시작
            inSection = (line == "[" + section + "]");
            continue;
        }

        // 현재 섹션에서 key 찾기
        if (inSection) {
            size_t pos = line.find('=');
            if (pos != std::string::npos) {
                std::string currentKey = line.substr(0, pos);
                std::string value = line.substr(pos + 1);

                // 공백 제거
                currentKey.erase(currentKey.find_last_not_of(" \t") + 1);
                value.erase(0, value.find_first_not_of(" \t"));

                if (currentKey == key) {
                    return value;
                }
            }
        }
    }
    return "";
}

bool UdpCommunication::isFileExist(const std::string& filename) {
    try{
        return std::filesystem::exists(filename);
    }catch(const std::exception& e){
        RCLCPP_INFO(this->get_logger(), "[isFileExist] std::exception occurred : %s", e.what());
        return false;
    }
}

bool UdpCommunication::isValidateResetOdom() {
  bool ret = false;
  
  if ( bReceiveOdom && bReceiveOdomStatus&& odom_status == 0x22 && (odom.x >= -0.1 && odom.x <= 0.1) && (odom.y >= -0.1 && odom.y <= 0.1) &&
      (odom.theta >= -0.2 && odom.theta <= 0.1)) {
    RCLCPP_INFO(this->get_logger(), "[isValidateResetOdom] ODOM RESET Success status : %02x, current:(%.2f, %.2f, %.2f)",odom_status,odom.x, odom.y, odom.theta);
    ret = true;
  } else {
    RCLCPP_INFO(this->get_logger(), "[isValidateResetOdom] current:(%.2f, %.2f, %.2f)", odom.x, odom.y, odom.theta);
  }
  return ret;
}

double UdpCommunication::getParamTofCalibTimeout() {
    return tof_calib_timeout_;
}

bool UdpCommunication::loadStationPose(const std::string& file_path) {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_INFO(this->get_logger(), "[loadStationPose] can`t open file: %s", file_path.c_str());
    return false;
  }

  try {
      nlohmann::json load_json;
      file >> load_json;
      if(load_json.contains("station_pose")) {
        station_pose.x = load_json["station_pose"]["x"].get<double>();
        station_pose.y = load_json["station_pose"]["y"].get<double>();
        station_pose.theta = load_json["station_pose"]["theta"].get<double>();
        RCLCPP_INFO(this->get_logger(), "[loadStationPose]station_pose parsing complete: %s ,pose(%.2f,%.2f,%.2f)", file_path.c_str(),station_pose.x,station_pose.y,station_pose.theta);
        file.close();
        return true;
      } else {
        RCLCPP_INFO(this->get_logger(), "[loadStationPose]station_pose parsing failed not found key : %s", file_path.c_str());
        file.close();
        return false;
      }
  } catch (const std::exception& e) {
    RCLCPP_INFO(this->get_logger(), "[loadStationPose] std::exception station_pose parsing failed : %s", e.what());
    file.close();
    return false;
  }
}

void UdpCommunication::sendstationPoseChecker()
{
    ROBOT_STATE state = static_cast<ROBOT_STATE>(socData.robotStatus);
    if(!bOnStation && state == ROBOT_STATE::ONSTATION){
        bOnStation = true;
        RCLCPP_INFO(this->get_logger(), "[sendstationPoseChecker] on-station send station pose to soc");
    }else if(bOnStation && state != ROBOT_STATE::ONSTATION){
        if(nodeState != NODE_STATUS::NAVI){
            bOnStation = false;
            RCLCPP_INFO(this->get_logger(), "[sendstationPoseChecker] [%s]node start not on station --> send amcl pose to soc",enumToString(nodeState).c_str());
        }else{
            NAVI_STATE moving_state = static_cast<NAVI_STATE>(socData.movingStatus);
            if(moving_state == NAVI_STATE::MOVE_GOAL){
                bOnStation = false;
                RCLCPP_INFO(this->get_logger(), "[sendstationPoseChecker] navigation start moving --> send amcl pose to soc");
            }
        }
    }
}

void UdpCommunication::udpCommunicationThread(){
    while (rclcpp::ok())
    {
        auto start_time = std::chrono::steady_clock::now();
        udp_callback();
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        auto sleep_time = std::chrono::milliseconds(100) - elapsed;

        if (sleep_time > std::chrono::milliseconds(0)) {
            //RCLCPP_INFO(this->get_logger(), "sleep time: %d ms", sleep_time.count());
            std::this_thread::sleep_for(sleep_time);
        }else if(elapsed > std::chrono::milliseconds(1000)){
            RCLCPP_INFO(this->get_logger(), "runtime: %ld ms", elapsed.count());
        }
    }
}

double UdpCommunication::getSteadyClockRunningSeconds(const std::chrono::time_point<std::chrono::steady_clock> &start_time)
{
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
}


void UdpCommunication::enqueueMainState(const mainState& data)
{
    {
        std::lock_guard<std::mutex> lock(main_state_queue_mutex_);
        main_state_queue_.push_back(data);
    }
    
}
std::vector<mainState> UdpCommunication::dequeueMainState()
{
    std::vector<mainState> front_state;
    {
        std::lock_guard<std::mutex> lock(main_state_queue_mutex_);
        if(main_state_queue_.empty()) return {};
        front_state.push_back(main_state_queue_.front());
        main_state_queue_.pop_front();
    }
    return front_state;
}

void UdpCommunication::enqueueMovingState(const movingState& data)
{
    {
        std::lock_guard<std::mutex> lock(moving_state_queue_mutex_);
        moving_state_queue_.push_back(data);
    }
}

std::vector<movingState> UdpCommunication::dequeueMovingState()
{
    std::vector<movingState> front_moving_state;
    {
        std::lock_guard<std::mutex> lock(moving_state_queue_mutex_);
        if(moving_state_queue_.empty()) return{};
        front_moving_state.push_back(moving_state_queue_.front());
        moving_state_queue_.pop_front();
    }
    return front_moving_state;
}

bool UdpCommunication::checkDockingStatusOnDemend()
{
    bool ret = false;
    static uint8_t prev_docking_status = 0;
    uint8_t current_docking_status = (socData.dockingInfo.status & 0xF0); //current charging status (upperbits)
    if(prev_docking_status != current_docking_status){
        RCLCPP_INFO(this->get_logger(), "Detect Docking Status Change send On-demand : prev(%02x) -> new(%02x)", prev_docking_status, current_docking_status);
        ret = true;
    }
    prev_docking_status = current_docking_status;
    return ret;
}

bool UdpCommunication::checkVersionGenerate()
{
    bool ret = false;
    static uint8_t send_version_count = 0;
    if(bEnableVertionTimer){
        if( socData.version.bSet){
            bEnableVertionTimer = false;
            send_version_count = 0;
            ret = true;
        }else if(++send_version_count >= 10){
            send_version_count = 0;
            ret = true;
        }
    }else{
        send_version_count = 0;
    }
    return ret;
}