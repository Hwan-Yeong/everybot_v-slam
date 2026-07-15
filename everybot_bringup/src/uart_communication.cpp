#include "everybot_bringup/uart_communication.hpp"
#include <numeric> // std::accumulate
#include <cstdlib>
#include <string>
#include <chrono>
#include <iomanip>  // for std::setw, std::setfill
#include <optional> // for std::optional

//uint8_t temp_tof_command; //250524 KKS : legacy
uint8_t temp_tofnbatt_command;
uint8_t temp_reset_command;
uint8_t temp_imu_calib_command;
uint8_t temp_docking_command;
uint8_t temp_motor_command;
uint8_t temp_rpm_command;
uint8_t temp_tof_status_from_mcu;

auto received_time_ = std::chrono::steady_clock::now();
auto transmit_time = std::chrono::steady_clock::now();
auto published_time_ = std::chrono::steady_clock::now();
auto previous_time = std::chrono::steady_clock::now();

#define DEG_TO_RAD_0_01 (0.01 * (M_PI / 180.0))
#define ACCEL_SCALE (9.81 / 1000.0)

UARTCommunication::UARTCommunication()
    : Node("uart_communication"), uart_("/dev/ttydriver", 230400),timeout_(rclcpp::Duration::from_seconds(3))
{
    initializeData();
    uart_.openPort();

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    battery_status_pub_ = this->create_publisher<everybot_custom_msgs::msg::BatteryStatus>("/battery_status", 10);

    tof_data_pub_ = this->create_publisher<everybot_custom_msgs::msg::TofData>("tof_data", 10);

    motor_status_pub_ = this->create_publisher<everybot_custom_msgs::msg::MotorStatus>("/motor_status", 10);

    station_data_pub_ = this->create_publisher<everybot_custom_msgs::msg::StationData>("/station_data", 10);

    bottom_status_pub_ = this->create_publisher<std_msgs::msg::UInt8>("bottom_status", 10);

    bottom_ir_data_pub_ = this->create_publisher<everybot_custom_msgs::msg::BottomIrData>("bottom_ir_data", 10);

    imu_data_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_data", 10);

    odom_status_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/odom_status", 10);

    fw_version_pub_ = this->create_publisher<std_msgs::msg::String>("/fw_version", 10);

    jig_imu_calibration_pub_ = this->create_publisher<everybot_custom_msgs::msg::ImuCalibration>("/jig_response_imu_calibration", 1);

    // error publisher
    error_left_mt_stall_or_overcurrent_pub_ = this->create_publisher<std_msgs::msg::Bool>("/error/e_code/left_motor_stuck", 10);
    error_right_mt_stall_or_overcurrent_pub_ = this->create_publisher<std_msgs::msg::Bool>("/error/e_code/right_motor_stuck", 10);
    error_left_mt_overheat_pub_ = this->create_publisher<std_msgs::msg::Bool>("/error/e_code/left_motor_overheat", 10);
    error_right_mt_overheat_pub_ = this->create_publisher<std_msgs::msg::Bool>("/error/e_code/right_motor_overheat", 10);
    error_docking_sig_pub_ = this->create_publisher<std_msgs::msg::Bool>("/error/e_code/docking_station", 10);
    error_charge_overcurrent_pub_ = this->create_publisher<std_msgs::msg::Bool>("/error/f_code/battery_charging_overcurrent", 10);
    error_discharge_overcurrent_pub_ = this->create_publisher<std_msgs::msg::Bool>("/error/f_code/battery_discharging_overcurrent", 10);
    error_1d_tof_uart_pub_ = this->create_publisher<std_msgs::msg::Bool>("/error/f_code/top_tof", 10);
    error_right_mt_uart_pub_ = this->create_publisher<std_msgs::msg::Bool>("/error/f_code/right_motor", 10);
    error_left_mt_uart_pub_ = this->create_publisher<std_msgs::msg::Bool>("/error/f_code/left_motor", 10);
    error_battery_charge_overheat_pub_ = this->create_publisher<std_msgs::msg::Bool>("/error/f_code/battery_charging_overheat", 10);
    error_battery_discharge_overheat_pub_ = this->create_publisher<std_msgs::msg::Bool>("/error/f_code/battery_discharging_overheat", 10);
    error_left_tof_i2c_pub_ = this->create_publisher<std_msgs::msg::Bool>("/error/f_code/bot_tof_left", 10);
    error_right_tof_i2c_pub_ = this->create_publisher<std_msgs::msg::Bool>("/error/f_code/bot_tof_right", 10);
    error_pogo_pin_overheat_pub_ = this->create_publisher<std_msgs::msg::Bool>("/error/s_code/station_overheat", 10);
    error_imu_uart_pub_ = this->create_publisher<std_msgs::msg::Bool>("/error/f_code/imu", 10);
    // error_battery_communication_pub_ = this->create_publisher<std_msgs::msg::Bool>("/???", 10);
    // error_ap_rx_pub_ = this->create_publisher<std_msgs::msg::Bool>("/???", 10);

    // 1109 change
    version_request_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/req_version",
        1,
        std::bind(&UARTCommunication::reqVersionCallback, this, std::placeholders::_1));

    navi_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(&UARTCommunication::cmdVelCallback, this, std::placeholders::_1));

    docking_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/docking_cmd",
        10,
        std::bind(&UARTCommunication::dockingCommandCallback, this, std::placeholders::_1));

    charging_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/charging_cmd",
        10,
        std::bind(&UARTCommunication::charge_cmd_callback, this, std::placeholders::_1));

    odom_imu_reset_sub = this->create_subscription<std_msgs::msg::UInt8>(
        "/odom_imu_reset_cmd",
        10,
        std::bind(&UARTCommunication::odom_imu_reset_callback, this, std::placeholders::_1));

    tof_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/cmd_tof",
        10,
        std::bind(&UARTCommunication::tofCommandCallback, this, std::placeholders::_1));
        
    battery_sleep_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/cmd_battery_sleep",
        10,
        std::bind(&UARTCommunication::batterySleepCallback, this, std::placeholders::_1));    
    
    // motor_mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
    //     "/cmd_motor_mode",
    //     10,
    //     std::bind(&UARTCommunication::motorModeCallback, this, std::placeholders::_1));
    
    remote_block_sub_ = this->create_subscription<std_msgs::msg::Bool>("/cmd_remote_block",
        10,
        std::bind(&UARTCommunication::remoteBlockCallback, this, std::placeholders::_1));
 
    e_stop_sub = this->create_subscription<std_msgs::msg::Bool>(
        "/emergency_stop",
        10,
        std::bind(&UARTCommunication::e_stop_callback, this, std::placeholders::_1));

    amcl_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "amcl_pose",
        10,
        std::bind(&UARTCommunication::amcl_pose_callback, this, std::placeholders::_1));

    jig_moter_sub = this->create_subscription<everybot_custom_msgs::msg::RpmControl>(
        "jig_request_motor",
        10,
        std::bind(&UARTCommunication::jig_motor_callback, this, std::placeholders::_1));

    jig_battery_sub = this->create_subscription<std_msgs::msg::UInt8>(
        "jig_request_battery",
        10,
        std::bind(&UARTCommunication::jig_bettery_callback, this, std::placeholders::_1));

    jig_imu_sub = this->create_subscription<std_msgs::msg::UInt8>(
        "jig_request_imu",
        10,
        std::bind(&UARTCommunication::jig_imu_callback, this, std::placeholders::_1));

    fw_reset_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "req_fw_reset", 
        10,
        std::bind(&UARTCommunication::onFWResetReceived, this, std::placeholders::_1));
    
    #if PREVENT_SAME_CMD_VEL > 0
    navi_state_sub = this->create_subscription<everybot_custom_msgs::msg::NaviState>("/navi_datas", 10, std::bind(&UARTCommunication::movingStateCallback, this, std::placeholders::_1));
    #endif

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
        10ms, // Timer period: 10 milliseconds
        std::bind(&UARTCommunication::timerCallback, this));

    timer2_ = this->create_wall_timer(
        10ms, // Timer period: 10 milliseconds
        std::bind(&UARTCommunication::receiveData, this));

    // timer3_ = this->create_wall_timer(
    //     10ms, // Timer period: 10 milliseconds
    //     std::bind(&UARTCommunication::pubTopic, this));

    //serial_thread_running_ = true; //hjkim : block code not be used
    RCLCPP_INFO(this->get_logger(), "node initialized");
}

UARTCommunication::~UARTCommunication()
{
    //serial_thread_running_ = false; //hjkim : block code not be used
    uart_.closePort();
    #if 0 //hjkim : block code not be used
    if (serial_thread_.joinable()){
        serial_thread_.join();
    }
    #endif
    RCLCPP_INFO(this->get_logger(), "node terminated");
}

void UARTCommunication::initializeData()
{
    defaultTransmissionData();
    setCurrentVelocity(0.0, 0.0);
    //temp_tof_command = 0; //250524 KKS : legacy
    temp_tofnbatt_command = 0;
    temp_reset_command = 0;
    temp_imu_calib_command = 0;
    temp_docking_command = 0;
    temp_motor_command = 0;
    temp_tof_status_from_mcu = 0;
    previous_roll = 0;
    previous_pitch = 0;
    previous_yaw = 0;
    amcl_pose_x_ = 0;
    amcl_pose_y_ = 0;
    amcl_pose_angle_ = 0;
    motor_communication_debug_time_ = std::chrono::steady_clock::now();
    //start_node_time_ = std::chrono::steady_clock::now();
    fwVersion_msg.data = "0.0";
    bEmergencyStop = false;
    bJigMotorInspection = false;
    setRemoteMode(MotornRemoteMode::BLOCK_REMOTE); //hjkim250814 : remote mode default setting (unblock--> block) change : unblock case only manual-mapping
}

void UARTCommunication::setCurrentVelocity(double v, double w)
{
    //hjkim : Jig Motor Inspection Mode once(RPM_MODE) --> cat`use VW_MODE if you want to use VW_MODE should RE-BOOT
    if(bJigMotorInspection){
        RCLCPP_INFO(this->get_logger(), "Jig Motor Inspection Can`t use cmdVelCallback : V : %.2f, W : %.2f ", v,w);
        return;
    }
    setMotorMode(MotornRemoteMode::VW_MODE);
    if(bEmergencyStop){
        //RCLCPP_INFO(this->get_logger(), "set Emergency stop cmdVelCallback : V : %.2f, W : %.2f ", v,w);
        setLinearVelocity(0.0);
        setAngularVelocity(0.0);
    }else{
        setLinearVelocity(v);
        setAngularVelocity(w);
    }
}


void UARTCommunication::pulishOdom(bool bOdomTfPub)
{
    nav_msgs::msg::Odometry tem_msg;
    {
        // std::lock_guard<std::mutex> lock(odom_mutex_);
        tem_msg = odom_msg;
    }
    rclcpp::Time cur_time = this->get_clock()->now();
    tem_msg.header.stamp = cur_time;
    odom_pub_->publish(tem_msg);

    if(bOdomTfPub){
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = cur_time;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = tem_msg.pose.pose.position.x;
        tf_msg.transform.translation.y = tem_msg.pose.pose.position.y;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = tem_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg);
    }else{
        RCLCPP_INFO(this->get_logger(), "skip odom TF Publish!!");
    }
}

void UARTCommunication::pulishImuOdomStatus()
{
    std_msgs::msg::UInt8 temp_msg;
    {
        // std::lock_guard<std::mutex> lock(imu_odom_status_mutex_);
        temp_msg = imu_odom_status_msg;
    }
    odom_status_pub_->publish(temp_msg);
}

void UARTCommunication::pulishImu()
{
    sensor_msgs::msg::Imu temp_msg;
    {
        // std::lock_guard<std::mutex> lock(imu_mutex_);
        temp_msg = imu_msg;
    }
    imu_data_pub_->publish(temp_msg);
}


void UARTCommunication::pulishBottomIr()
{
    std_msgs::msg::UInt8 temp_status_msg;
    everybot_custom_msgs::msg::BottomIrData temp_ir_msg;
    {
        // std::lock_guard<std::mutex> lock(bottom_ir_mutex_);
        temp_ir_msg = bottom_ir_msg;
        temp_status_msg = bottom_status_msg;
    }
    bottom_ir_data_pub_->publish(temp_ir_msg);
    bottom_status_pub_->publish(temp_status_msg);
}

void UARTCommunication::pulishTof()
{
    everybot_custom_msgs::msg::TofData temp_msg;
    {
        // std::lock_guard<std::mutex> lock(tof_mutex_);
        temp_msg = tof_msg;
    }
    tof_data_pub_->publish(temp_msg);
}

void UARTCommunication::pulishMotorStatus()
{
    everybot_custom_msgs::msg::MotorStatus temp_msg;
    
    {
        // std::lock_guard<std::mutex> lock(motor_mutex_);
        temp_msg = motor_msg;
    }
    motor_status_pub_->publish(temp_msg);
}

uint8_t temp_docking_status = 0;
void UARTCommunication::pulishStationData()
{
    uint8_t cur_data;
    everybot_custom_msgs::msg::StationData temp_msg;
    {
        // std::lock_guard<std::mutex> lock(station_mutex_);
        cur_data = (station_msg.docking_status&0xF0);
        temp_msg = station_msg;
    }
    station_data_pub_->publish(temp_msg);

    if(temp_docking_status != cur_data){
        RCLCPP_INFO(this->get_logger(), "docking status changed!! 0x%02X -> 0x%02X", temp_docking_status, cur_data);
        temp_docking_status = cur_data;
    }
}

void UARTCommunication::pulishJigImuCalibration()
{
    everybot_custom_msgs::msg::ImuCalibration temp_msg;
    
    {
        // std::lock_guard<std::mutex> lock(jig_imu_calibration_mutex_);
        temp_msg = jig_imu_calibration_msg;
    }
    jig_imu_calibration_pub_->publish(temp_msg);
}

void UARTCommunication::pulishError()
{
    std_msgs::msg::Bool temp_left_mt_stall_or_overcurrent_msg, temp_right_mt_stall_or_overcurrent_msg,
    temp_left_mt_overheat_msg, temp_right_mt_overheat_msg, temp_right_mt_uart_msg, temp_left_mt_uart_msg,
        temp_charge_overcurrent_msg, temp_discharge_overcurrent_msg, temp_docking_sig_msg, temp_1d_tof_uart_msg, temp_left_tof_i2c_msg, temp_right_tof_i2c_msg,
        temp_battery_charge_overheat_msg, temp_battery_discharge_overheat_msg, temp_pogo_pin_overheat_msg;

    
    {
        // std::lock_guard<std::mutex> lock(error_mutex_);
        temp_left_mt_stall_or_overcurrent_msg = error_left_mt_stall_or_overcurrent_msg;
        temp_right_mt_stall_or_overcurrent_msg = error_right_mt_stall_or_overcurrent_msg;
        temp_left_mt_overheat_msg = error_left_mt_overheat_msg;
        temp_right_mt_overheat_msg = error_right_mt_overheat_msg;
        temp_right_mt_uart_msg = error_right_mt_uart_msg;
        temp_left_mt_uart_msg = error_left_mt_uart_msg;
        temp_charge_overcurrent_msg = error_charge_overcurrent_msg;
        temp_discharge_overcurrent_msg = error_discharge_overcurrent_msg;
        temp_docking_sig_msg = error_docking_sig_msg;
        temp_1d_tof_uart_msg = error_1d_tof_uart_msg;
        temp_left_tof_i2c_msg = error_left_tof_i2c_msg;
        temp_right_tof_i2c_msg = error_right_tof_i2c_msg;
        temp_battery_charge_overheat_msg = error_battery_charge_overheat_msg;
        temp_battery_discharge_overheat_msg = error_battery_discharge_overheat_msg;
        temp_pogo_pin_overheat_msg = error_pogo_pin_overheat_msg;
    }

    auto publishErrorIfChanged = [&](const std::string &key, const std_msgs::msg::Bool &msg, const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr &pub)
    {
        bool current = msg.data;
        bool &prev = prev_error_state_[key]; // key가 없어도 초기화된 값(false) 반환

        if (current != prev) { // 이전값과 다른 에러 상태인 경우에만 퍼블리싱 (에러발생: false->true / 에러해제: true->false)
            pub->publish(msg);
            RCLCPP_INFO(this->get_logger(), "[%s] Publish Error: %s", msg.data ? "Occured" : "Released", key.c_str());
            prev = current;
        }
    };

    publishErrorIfChanged("left_mt_stall_or_overcurrent", temp_left_mt_stall_or_overcurrent_msg, error_left_mt_stall_or_overcurrent_pub_);
    publishErrorIfChanged("right_mt_stall_or_overcurrent", temp_right_mt_stall_or_overcurrent_msg, error_right_mt_stall_or_overcurrent_pub_);
    publishErrorIfChanged("left_mt_overheat", temp_left_mt_overheat_msg, error_left_mt_overheat_pub_);
    publishErrorIfChanged("right_mt_overheat", temp_right_mt_overheat_msg, error_right_mt_overheat_pub_);
    publishErrorIfChanged("right_mt_uart", temp_right_mt_uart_msg, error_right_mt_uart_pub_);
    publishErrorIfChanged("left_mt_uart", temp_left_mt_uart_msg, error_left_mt_uart_pub_);
    publishErrorIfChanged("charge_overcurrent", temp_charge_overcurrent_msg, error_charge_overcurrent_pub_);
    publishErrorIfChanged("discharge_overcurrent", temp_discharge_overcurrent_msg, error_discharge_overcurrent_pub_);
    publishErrorIfChanged("docking_sig", temp_docking_sig_msg, error_docking_sig_pub_);
    publishErrorIfChanged("1d_tof_uart", temp_1d_tof_uart_msg, error_1d_tof_uart_pub_);
    publishErrorIfChanged("left_tof_i2c", temp_left_tof_i2c_msg, error_left_tof_i2c_pub_);
    publishErrorIfChanged("right_tof_i2c", temp_right_tof_i2c_msg, error_right_tof_i2c_pub_);
    publishErrorIfChanged("battery_charge_overheat", temp_battery_charge_overheat_msg, error_battery_charge_overheat_pub_);
    publishErrorIfChanged("battery_discharge_overheat", temp_battery_discharge_overheat_msg, error_battery_discharge_overheat_pub_);
    publishErrorIfChanged("pogo_pin_overheat", temp_pogo_pin_overheat_msg, error_pogo_pin_overheat_pub_);
    publishErrorIfChanged("imu_uart", error_imu_uart_msg, error_imu_uart_pub_);
    // publishErrorIfChanged("battery_comm", error_battery_communication_msg, error_battery_communication_pub_);
    // publishErrorIfChanged("ap_rx", error_ap_rx_msg, error_ap_rx_pub_);
}

void UARTCommunication::pulishBatteryStatus()
{
    static int battery_cnt = 0;
    if (++battery_cnt % 100 == 0){
        everybot_custom_msgs::msg::BatteryStatus temp_msg;
        
        {
            // std::lock_guard<std::mutex> lock(battery_mutex_);
            temp_msg = battery_msg;
        }
        battery_status_pub_->publish(temp_msg);
        battery_cnt = 0;
    }
}

void UARTCommunication::pubTopic()
{
	auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = now - published_time_;
    double milisec_interval = elapsed_seconds.count()*1000.0;
    static size_t call_count = 0;
    static size_t delayed_count = 0;
    bool bCheckOdomTfPub = false;
   
    if(call_count > 86400000){
        RCLCPP_INFO(this->get_logger(), "[pubTopic] reset check_cnt (%zu/%zu)", delayed_count, call_count);
        call_count = 0; 
        delayed_count = 0; 
    }

    ++call_count;
    
    if(now > published_time_){
        bCheckOdomTfPub = true;
        if(milisec_interval >= 100.0 && call_count > 0){
            ++delayed_count;
            double delay_rate = static_cast<double>(delayed_count) / static_cast<double>(call_count) * 100.0;
            RCLCPP_INFO(this->get_logger(), "[pubTopic]Publish interval : %.3f ms", milisec_interval);
            RCLCPP_INFO(this->get_logger(),"[pubTopic]Publish Delay rate (>=100ms): %.2f%% (%zu/%zu)",delay_rate, delayed_count, call_count);
        }
    }else{
        RCLCPP_INFO(this->get_logger(), "[pubTopic]time-stamp Error!! interval is revert : %.3f ms", milisec_interval);
    }

    pulishOdom(bCheckOdomTfPub);
    pulishImuOdomStatus();
    pulishImu();
    pulishBottomIr();
    pulishTof();
    pulishMotorStatus();
    pulishStationData();
    pulishJigImuCalibration();
    pulishError();
    pulishBatteryStatus();
	published_time_ = now;
}

void UARTCommunication::reqVersionCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    if (!msg) { // msg 가 nullptr 일 경우 죽을 수 있음
        RCLCPP_WARN(this->get_logger(), "[reqVersionCallback] Received nullptr");
        return;
    }

    if (msg->data & 0x01){
        fw_version_pub_->publish(fwVersion_msg);
    }
}

void UARTCommunication::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (!msg) { // msg 가 nullptr 일 경우 죽을 수 있음
        RCLCPP_WARN(this->get_logger(), "[cmdVelCallback] Received nullptr");
        return;
    }

    setCurrentVelocity(msg->linear.x, msg->angular.z);
    last_cmd_vel_time_ = std::chrono::steady_clock::now();
}

void UARTCommunication::jig_motor_callback(const everybot_custom_msgs::msg::RpmControl msg)
{
    RCLCPP_INFO(this->get_logger(), "jig_motor_callback ");
    uint8_t command = msg.motor_enable;
    if (command == 0x01){
        bJigMotorInspection = true;
        int16_t left_motor_rpm = (int16_t)((msg.left_motor_rpm_msb << 8) | msg.left_motor_rpm_lsb);
        int16_t right_motor_rpm = (int16_t)((msg.right_motor_rpm_msb << 8) | msg.right_motor_rpm_lsb);
        defaultTransmissionData();
        setMotorMode(MotornRemoteMode::RPM_MODE);
        setLeftMotorRpm(left_motor_rpm);
        setRightMotorRpm(right_motor_rpm);
    }else{
        RCLCPP_INFO(this->get_logger(), "motor_enable error : %u ", command);
    }
}
void UARTCommunication::jig_bettery_callback(const std_msgs::msg::UInt8 msg)
{
    RCLCPP_INFO(this->get_logger(), "jig_bettery_callback ");
    uint8_t command = msg.data;
    if (command == 0x01){
        defaultTransmissionData();
        setCharge(DockingChargeCommand::HIGH_SPEED_CHARGE_START);
    }else if (command == 0x02)
    {
        defaultTransmissionData();
        setCharge(DockingChargeCommand::LOW_SPEED_CHARGE_START);
    }else{
        RCLCPP_INFO(this->get_logger(), "command error : %u ", command);
    }
}


void UARTCommunication::jig_imu_callback(const std_msgs::msg::UInt8 msg)
{
    RCLCPP_INFO(this->get_logger(), "jig_imu_callback ");
    uint8_t command = msg.data;
    if (command == 0x01){
        setIMUCalibration(ImuCalibrationCommand::CALIBRATION_START);
    }else if(command == 0x02)
    {
        setIMUCalibration(ImuCalibrationCommand::END_OF_ROTATION);
    }else{
        RCLCPP_INFO(this->get_logger(), "command error : %u ", command);
    }
}

uint8_t UARTCommunication::calculateChecksum(const std::vector<uint8_t> &data, size_t start, size_t end)
{
    uint8_t checksum = 0;
    for (size_t i = start; i <= end; ++i)
    {
        checksum += data[i];
    }
    return checksum;
}


/**************************************************************************************/
// INDEX
/**************************************************************************************/
// std::mutex index_mutex;

/**************************************************************************************/
// RESET
/**************************************************************************************/
// std::mutex reset_mutex;

// Reset 상태를 설정하는 함수 (하위 4 비트에 Reset 명령 설정)
void UARTCommunication::setReset(ResetCommand status)
{
    // std::lock_guard<std::mutex> lock(reset_mutex);
    
    if(temp_reset_command != static_cast<uint8_t>(status)){
        if(status == ResetCommand::NORMAL_OPERATION){
            RCLCPP_INFO(this->get_logger(), "NORMAL_OPERATION ");
        }else if(status == ResetCommand::ODOMETRY_RESET){
            RCLCPP_INFO(this->get_logger(), "ODOMETRY_RESET ");
        }else if(status == ResetCommand::IMU_RESET){
            RCLCPP_INFO(this->get_logger(), "IMU_RESET ");
        }else if(status == ResetCommand::MCU_RESET){
            RCLCPP_INFO(this->get_logger(), "MCU_RESET ");
        }else{
            RCLCPP_INFO(this->get_logger(), "setReset error");
        }
    }
    // 기존 Reset 명령을 지우고 새 명령으로 설정
    g_transmission_data.reset_flags &= 0xF0;  // 상위 4 비트는 그대로 두고, 하위 4 비트 초기화
    g_transmission_data.reset_flags |= static_cast<uint8_t>(status); // 하위 4 비트에 새로운 Reset 명령 설정
}

// Reset 상태 값을 읽어오는 함수 (하위 4 비트)
ResetCommand UARTCommunication::getResetStatus() const
{
    // std::lock_guard<std::mutex> lock(reset_mutex);
    return static_cast<ResetCommand>(g_transmission_data.reset_flags & 0x0F); // 하위 4 비트 추출
}

// 설정된 Reset 상태가 특정 값과 일치하는지 확인하는 함수 (하위 4 비트)
bool UARTCommunication::isResetStatus(ResetCommand status) const
{
    return getResetStatus() == status;
}

/**************************************************************************************/
// Motor Enable
/**************************************************************************************/
// std::mutex motor_mutex;

// Motor Mode 설정 함수 (상위 4 비트에 Motor Mode 명령 설정)
void UARTCommunication::setMotorMode(MotornRemoteMode mode)
{
    // std::lock_guard<std::mutex> lock(motor_mutex);
    uint8_t prev_cmd = (temp_motor_command & 0xF0) >> 4;

    if(prev_cmd != static_cast<uint8_t>(mode)){
        if(mode == MotornRemoteMode::VW_MODE){
            RCLCPP_INFO(this->get_logger(), "[setMotorMode]VW_MODE");
        }else if(mode == MotornRemoteMode::RPM_MODE){
            RCLCPP_INFO(this->get_logger(), "[setMotorMode]RPM_MODE");
        }else if(mode == MotornRemoteMode::MANUAL_MODE){
            RCLCPP_INFO(this->get_logger(), "[setMotorMode]MANUAL_MODE");
        }else if(mode == MotornRemoteMode::BRAKE_MODE){
            RCLCPP_INFO(this->get_logger(), "[setMotorMode]BRAKE_MODE");
        }else if(mode == MotornRemoteMode::DISABLE_MODE){
            RCLCPP_INFO(this->get_logger(), "[setMotorMode]DISABLE_MODE");
        }else{
            RCLCPP_INFO(this->get_logger(), "[setMotorMode]UNKOWN_MODE");
        }
    }

    // 기존 모터 모드를 지우고 새 모드로 설정
    g_transmission_data.motor_flags &= 0x0F;  // 상위 4 비트 초기화 (하위 4 비트만 남기기)
    g_transmission_data.motor_flags |= (static_cast<uint8_t>(mode) << 4); // 새 모드를 상위 4 비트에 설정
}

// Remote Mode 설정 함수
void UARTCommunication::setRemoteMode(MotornRemoteMode mode)
{
    // std::lock_guard<std::mutex> lock(motor_mutex);
    uint8_t prev_cmd = temp_motor_command & 0x0F;

    if (prev_cmd != static_cast<uint8_t>(mode))
    {
        if (mode == MotornRemoteMode::BLOCK_REMOTE)
        {

            RCLCPP_INFO(this->get_logger(), "[setRemoteMode]BLOCK_REMOTE");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "[setRemoteMode]UNBLOCK_REMOTE");
        }
    }
    // 기존 도킹 명령을 지우고 새 명령으로 설정
    g_transmission_data.motor_flags &= 0xF0;                                // 상위 4 비트는 그대로 두고, 하위 4 비트 초기화
    g_transmission_data.motor_flags |= (static_cast<uint8_t>(mode) & 0x0F); // 하위 4 비트에 새로운 도킹 명령 설정
}

// Motor Mode 값을 읽어오는 함수 (상위 4 비트)
MotornRemoteMode UARTCommunication::getMotorMode() const
{
    // std::lock_guard<std::mutex> lock(motor_mutex);
    return static_cast<MotornRemoteMode>(g_transmission_data.motor_flags >> 4); // 상위 4 비트 추출
}

MotornRemoteMode UARTCommunication::getRemoteMode() const
{
    // std::lock_guard<std::mutex> lock(motor_mutex);
    return static_cast<MotornRemoteMode>(g_transmission_data.motor_flags & 0x0F); // 하위 4 비트 추출
}

// Motor Mode가 설정되었는지 확인하는 함수
bool UARTCommunication::isMotorMode(MotornRemoteMode mode) const
{
    return getMotorMode() == mode;
}

bool UARTCommunication::isRemoteMode(MotornRemoteMode mode) const
{
    return getRemoteMode() == mode;
}
/**************************************************************************************/
// Docking & Charge Flag
/**************************************************************************************/
// std::mutex docking_mutex;
// 충전 설정 함수 (상위 4 비트에 충전 명령 설정)
void UARTCommunication::setCharge(DockingChargeCommand set)
{
    // std::lock_guard<std::mutex> lock(docking_mutex);
    uint8_t prev_cmd = (temp_docking_command & 0xF0) >> 4;

    if(prev_cmd != static_cast<uint8_t>(set)){
        if (set == DockingChargeCommand::CHARGE_CONTROL_MCU){
            RCLCPP_INFO(this->get_logger(), "CHARGE_CONTROL MCU [0x%X->0x%X]", prev_cmd, static_cast<uint8_t>(set));
        }else if(set == DockingChargeCommand::CHARGE_CONTROL_AP){
            RCLCPP_INFO(this->get_logger(), "CHARGE_CONTROL AP [0x%X->0x%X]", prev_cmd, static_cast<uint8_t>(set));
        }else if(set == DockingChargeCommand::HIGH_SPEED_CHARGE_START){
            RCLCPP_INFO(this->get_logger(), "CHARGE_CONTROL HIGH_SPEED [0x%X->0x%X]", prev_cmd, static_cast<uint8_t>(set));
        }else if(set == DockingChargeCommand::LOW_SPEED_CHARGE_START){
            RCLCPP_INFO(this->get_logger(), "CHARGE_CONTROL LOW_SPEED [0x%X->0x%X]", prev_cmd, static_cast<uint8_t>(set));
        }else if(set == DockingChargeCommand::CHARGE_STOP){
            RCLCPP_INFO(this->get_logger(), "CHARGE_CONTROL STOP [0x%X->0x%X]", prev_cmd, static_cast<uint8_t>(set));
        }else{
            RCLCPP_INFO(this->get_logger(), "CHARGE_CONTROL UNDEFINED [0x%X->0x%X]", prev_cmd, static_cast<uint8_t>(set));
        }
    }
    // 기존 충전 명령을 지우고 새 명령으로 설정
    g_transmission_data.docking_flags &= 0x0F;  // 하위 4 비트는 그대로 두고, 상위 4 비트 초기화
    g_transmission_data.docking_flags |= (static_cast<uint8_t>(set) << 4); // 상위 4 비트에 새로운 충전 명령 설정
}

// 도킹 관련 설정 함수 (하위 4 비트)
void UARTCommunication::setDocking(DockingChargeCommand set)
{
    // std::lock_guard<std::mutex> lock(docking_mutex);
    uint8_t prev_cmd = temp_docking_command & 0x0F;

    if(prev_cmd != static_cast<uint8_t>(set)){
        if (set == DockingChargeCommand::DOCKING_START){
            RCLCPP_INFO(this->get_logger(), "DOCKING_CMD START [0x%X->0x%X]", prev_cmd, static_cast<uint8_t>(set));
        }else if(set == DockingChargeCommand::DOCKING_STOP){
            RCLCPP_INFO(this->get_logger(), "DOCKING_CMD STOP [0x%X->0x%X]", prev_cmd, static_cast<uint8_t>(set));
        }else{
            RCLCPP_INFO(this->get_logger(), "DOCKING_CMD UNDEFINED [0x%X->0x%X]", prev_cmd, static_cast<uint8_t>(set));
        }
    }
    // 기존 도킹 명령을 지우고 새 명령으로 설정
    g_transmission_data.docking_flags &= 0xF0;  // 상위 4 비트는 그대로 두고, 하위 4 비트 초기화
    g_transmission_data.docking_flags |= (static_cast<uint8_t>(set) & 0x0F); // 하위 4 비트에 새로운 도킹 명령 설정
}

// 충전 관련 값을 읽어오는 함수 (상위 4 비트)
DockingChargeCommand UARTCommunication::getChargeMode() const
{
    // std::lock_guard<std::mutex> lock(docking_mutex);
    return static_cast<DockingChargeCommand>(g_transmission_data.docking_flags >> 4); // 상위 4 비트 추출
}

// 도킹 관련 값을 읽어오는 함수 (하위 4 비트)
DockingChargeCommand UARTCommunication::getDockingMode() const
{
    // std::lock_guard<std::mutex> lock(docking_mutex);
    return static_cast<DockingChargeCommand>(g_transmission_data.docking_flags & 0x0F); // 하위 4 비트 추출
}

// 충전 모드가 설정되었는지 확인하는 함수 (상위 4 비트)
bool UARTCommunication::isChargeMode(DockingChargeCommand mode) const
{
    return getChargeMode() == mode;
}

// 도킹 모드가 설정되었는지 확인하는 함수 (하위 4 비트)
bool UARTCommunication::isDockingMode(DockingChargeCommand mode) const
{
    return getDockingMode() == mode;
}

/**************************************************************************************/
// Calibration
/**************************************************************************************/
// std::mutex imu_mutex;

// IMU Calibration 명령을 설정하는 함수 (하위 4 비트)
void UARTCommunication::setIMUCalibration(ImuCalibrationCommand set)
{
    // std::lock_guard<std::mutex> lock(imu_mutex);
    uint8_t prev_cmd = temp_imu_calib_command & 0x0F;

    if(prev_cmd != static_cast<uint8_t>(set)){
        if (set == ImuCalibrationCommand::NORMAL){
            RCLCPP_INFO(this->get_logger(), "IMU_Cal_CMD NORMAL [0x%X->0x%X]", prev_cmd, static_cast<uint8_t>(set));
        }else if(set == ImuCalibrationCommand::CALIBRATION_START){
            RCLCPP_INFO(this->get_logger(), "IMU_Cal_CMD START [0x%X->0x%X]", prev_cmd, static_cast<uint8_t>(set));
        }else if(set == ImuCalibrationCommand::END_OF_ROTATION){
            RCLCPP_INFO(this->get_logger(), "IMU_Cal_CMD END_OF_ROTATION [0x%X->0x%X]", prev_cmd, static_cast<uint8_t>(set));
        }else{
            RCLCPP_INFO(this->get_logger(), "IMU_Cal_CMD UNDEFINED [0x%X->0x%X]", prev_cmd, static_cast<uint8_t>(set));
        }
    }
    // 기존 IMU Calibration 명령을 지우고 새 명령으로 설정
    g_transmission_data.imu_flags &= 0xF0;  // 상위 4 비트는 그대로 두고, 하위 4 비트 초기화
    g_transmission_data.imu_flags |= (static_cast<uint8_t>(set) & 0x0F); // 하위 4 비트에 새로운 IMU Calibration 명령 설정
}

// IMU Calibration 값을 읽어오는 함수 (하위 4 비트)
ImuCalibrationCommand UARTCommunication::getIMUCalibration() const
{
    // std::lock_guard<std::mutex> lock(imu_mutex);

    return static_cast<ImuCalibrationCommand>(g_transmission_data.imu_flags & 0x0F); // 하위 4 비트 추출
}

// IMU Calibration 모드가 설정되었는지 확인하는 함수 (하위 4 비트)
bool UARTCommunication::isIMUCalibrationMode(ImuCalibrationCommand mode) const
{
    return getIMUCalibration() == mode;
}

/**************************************************************************************/
// ToF On/Off
/**************************************************************************************/
// std::mutex tof_mutex;

// 배터리 설정 함수 (상위 4 비트에 충전 명령 설정)
void UARTCommunication::setBattMode(ToFnBatt cmd)
{
    // std::lock_guard<std::mutex> lock(tof_mutex);
    uint8_t prev_cmd = (temp_tofnbatt_command & 0xF0) >> 4;    

    if(cmd == ToFnBatt::BATT_NORMAL || cmd == ToFnBatt::BATT_SLEEP){
        // 기존 배터리 명령을 지우고 새 명령으로 설정
        g_transmission_data.tofnBatt_flags &= 0x0F;  // 하위 4 비트는 그대로 두고, 상위 4 비트 초기화
        g_transmission_data.tofnBatt_flags |= (static_cast<uint8_t>(cmd) << 4); // 상위 4 비트에 새로운 배터리 명령 설정
    }

    if(prev_cmd != static_cast<uint8_t>(cmd)){
        if (cmd == ToFnBatt::BATT_NORMAL){
            RCLCPP_INFO(this->get_logger(), "SET_BATT_CMD NORMAL CMD:[%d->%d] TO_MCU:[0x%02x->0x%02x]"
                    , prev_cmd, static_cast<uint8_t>(cmd), temp_tofnbatt_command, g_transmission_data.tofnBatt_flags);
        }else if(cmd == ToFnBatt::BATT_SLEEP){
            RCLCPP_INFO(this->get_logger(), "SET_BATT_CMD SLEEP CMD:[%d->%d] TO_MCU:[0x%02x->0x%02x]"
                    , prev_cmd, static_cast<uint8_t>(cmd), temp_tofnbatt_command, g_transmission_data.tofnBatt_flags);
        }else{
            RCLCPP_INFO(this->get_logger(), "SET_BATT_CMD UNDEFINED CMD:[%d->%d] TO_MCU:[0x%02x->0x%02x]"
                    , prev_cmd, static_cast<uint8_t>(cmd), temp_tofnbatt_command, g_transmission_data.tofnBatt_flags);
        }        
    }
}

// ToF 상태를 설정하는 함수 (하위 4 비트에 ToF On/Off 설정)
void UARTCommunication::setToF(ToFnBatt cmd)
{
    // std::lock_guard<std::mutex> lock(tof_mutex);
    uint8_t prev_cmd = temp_tofnbatt_command & 0x0F;

    if(cmd == ToFnBatt::ON || cmd == ToFnBatt::OFF){
        // 하위 4 비트 설정 (ToF On/Off)
        g_transmission_data.tofnBatt_flags &= 0xF0;  // 기존 하위 4 비트 초기화 (상위 4 비트만 남기기)
        g_transmission_data.tofnBatt_flags |= static_cast<uint8_t>(cmd); // 하위 4 비트에 새로운 ToF 상태 설정
    }

    if(prev_cmd != static_cast<uint8_t>(cmd)){
        if (cmd == ToFnBatt::ON){
            RCLCPP_INFO(this->get_logger(), "SET_TOF_CMD ON CMD:[%d->%d] TO_MCU:[0x%02x->0x%02x]"
                    , prev_cmd, static_cast<uint8_t>(cmd), temp_tofnbatt_command, g_transmission_data.tofnBatt_flags);
        }else if(cmd == ToFnBatt::OFF){
            RCLCPP_INFO(this->get_logger(), "SET_TOF_CMD OFF CMD:[%d->%d] TO_MCU:[0x%02x->0x%02x]"
                    , prev_cmd, static_cast<uint8_t>(cmd), temp_tofnbatt_command, g_transmission_data.tofnBatt_flags);
        }else{
            RCLCPP_INFO(this->get_logger(), "SET_TOF_CMD UNDEFINED CMD:[%d->%d] TO_MCU:[0x%02x->0x%02x]"
                    , prev_cmd, static_cast<uint8_t>(cmd), temp_tofnbatt_command, g_transmission_data.tofnBatt_flags);
        }        
    }    
}


// 배터리 명령어를 읽어오는 함수 (상위 4 비트)
ToFnBatt UARTCommunication::getBattMode() const
{
    // std::lock_guard<std::mutex> lock(tof_mutex);
    return static_cast<ToFnBatt>(g_transmission_data.tofnBatt_flags >> 4); // 상위 4 비트 추출
}


// ToF 상태 값을 읽어오는 함수 (하위 4 비트)
ToFnBatt UARTCommunication::getToFStatus() const
{
    // std::lock_guard<std::mutex> lock(tof_mutex);
    return static_cast<ToFnBatt>(g_transmission_data.tofnBatt_flags & 0x0F); // 하위 4 비트 추출
}

// 배터리 상태가 설정되었는지 확인하는 함수 (상위 4 비트)
bool UARTCommunication::isBattMode(ToFnBatt mode) const
{
    return getBattMode() == mode;
}


// ToF 상태가 설정되었는지 확인하는 함수 (하위 4 비트)
bool UARTCommunication::isToFStatus(ToFnBatt status) const
{
    return getToFStatus() == status;
}

/**************************************************************************************/
// Motor Control
/**************************************************************************************/
// std::mutex linear_velocity_mutex;
// std::mutex angular_velocity_mutex;
// std::mutex left_rpm_mutex;
// std::mutex right_rpm_mutex;


void UARTCommunication::setLinearVelocity(double velocity) {
    // std::lock_guard<std::mutex> lock(linear_velocity_mutex);
    g_transmission_data.linear_velocity = velocity;
}

double UARTCommunication::getLinearVelocity() const {
    // std::lock_guard<std::mutex> lock(linear_velocity_mutex);
    return g_transmission_data.linear_velocity;
}

void UARTCommunication::setAngularVelocity(double velocity) {
    // std::lock_guard<std::mutex> lock(angular_velocity_mutex);
    g_transmission_data.angular_velocity = velocity;
}

double UARTCommunication::getAngularVelocity() const {
    // std::lock_guard<std::mutex> lock(angular_velocity_mutex);
    return g_transmission_data.angular_velocity;
}

void UARTCommunication::setLeftMotorRpm(int16_t rpm) {
    // std::lock_guard<std::mutex> lock(left_rpm_mutex);
    g_transmission_data.left_mt_rpm = rpm;
}

int16_t UARTCommunication::getLeftMotorRpm() const {
    // std::lock_guard<std::mutex> lock(left_rpm_mutex);
    return g_transmission_data.left_mt_rpm;
}

void UARTCommunication::setRightMotorRpm(int16_t rpm) {
    // std::lock_guard<std::mutex> lock(right_rpm_mutex);
    g_transmission_data.right_mt_rpm = rpm;
}

int16_t UARTCommunication::getRightMotorRpm() const {
    // std::lock_guard<std::mutex> lock(right_rpm_mutex);
    return g_transmission_data.right_mt_rpm;
}

/**************************************************************************************/
#if true
void UARTCommunication::sendProtocolV1Data(TransmissionData& gData)
{
    TransmissionData data; //**** 함수가 호출될 때만 새로 쓰기 때문에 lock 필요 없음.
    //{
        // std::lock_guard<std::mutex> lock(linear_velocity_mutex);
        data.linear_velocity = gData.linear_velocity;
    //}

    //{
        // std::lock_guard<std::mutex> lock(angular_velocity_mutex);
        data.angular_velocity = gData.angular_velocity;
    //}

    //{
        // std::lock_guard<std::mutex> lock(left_rpm_mutex);
        data.left_mt_rpm = gData.left_mt_rpm;
    //}

    //{
        // std::lock_guard<std::mutex> lock(right_rpm_mutex);
        data.right_mt_rpm = gData.right_mt_rpm;
    //}

    //{
        // std::lock_guard<std::mutex> lock(reset_mutex);
        data.reset_flags = gData.reset_flags;
    //}

    //{
        // std::lock_guard<std::mutex> lock(motor_mutex);
        data.motor_flags = gData.motor_flags;
    //}

    //{
        // std::lock_guard<std::mutex> lock(docking_mutex);
        data.docking_flags = gData.docking_flags;
    //}

    //{
        // std::lock_guard<std::mutex> lock(imu_mutex);
        data.imu_flags = gData.imu_flags;
    //}

    //{
        // std::lock_guard<std::mutex> lock(tof_mutex);
        data.tofnBatt_flags = gData.tofnBatt_flags;
    //}
    
    // {
    //     std::lock_guard<std::mutex> lock(index_mutex);
        data.index = gData.index;
    // }
    // <-- 여기서 자동으로 mutex unlock 됨

    // 전송 메시지 구성 시작
    std::vector<uint8_t> message(31, 0);
    message[0] = 0xAA;
    message[1] = 0x55;
    message[2] = 28;
    message[3] = 0x51;

    static bool isPreventAccelation = false;
    if(preventUnintendAccelationChecker(data.linear_velocity,data.angular_velocity,isPreventAccelation)){
        if(!isPreventAccelation){
            RCLCPP_INFO(this->get_logger(), "start PreventAccelation");
        }
        data.linear_velocity = 0.0;
        data.angular_velocity = 0.0;
        isPreventAccelation = true;
    }else{
        if(isPreventAccelation){
            RCLCPP_INFO(this->get_logger(), "clear preventAccelation");
        }
        isPreventAccelation = false;
    }

    // linear velocity (double → mm/s → 8 bytes)
    double linear_vel_mm = data.linear_velocity * 1000.0;
    uint64_t linear_velocity_bits;
    std::memcpy(&linear_velocity_bits, &linear_vel_mm, sizeof(uint64_t));
    for (int i = 0; i < 8; ++i)
        message[4 + i] = (linear_velocity_bits >> (56 - 8 * i)) & 0xFF;

    // angular velocity (double → 8 bytes)
    uint64_t angular_velocity_bits;
    std::memcpy(&angular_velocity_bits, &data.angular_velocity, sizeof(uint64_t));
    for (int i = 0; i < 8; ++i)
        message[12 + i] = (angular_velocity_bits >> (56 - 8 * i)) & 0xFF;

    // RPMs
    message[20] = (data.left_mt_rpm >> 8) & 0xFF;
    message[21] = data.left_mt_rpm & 0xFF;
    message[22] = (data.right_mt_rpm >> 8) & 0xFF;
    message[23] = data.right_mt_rpm & 0xFF;
    
    // Flags
    message[24] = data.reset_flags;
    if(temp_reset_command != message[24]){
        RCLCPP_INFO(this->get_logger(), "[SEND_TO_MCU] RESET COMMAND SEND TO MCU HEX : 0x%02X",message[24]);
        temp_reset_command = message[24];
    }

    message[25] = data.motor_flags;
    if(temp_motor_command != message[25]){
        RCLCPP_INFO(this->get_logger(), "[SEND_TO_MCU] MOTOR COMMAND SEND TO MCU HEX : 0x%02X",message[25]);
        temp_motor_command = message[25];
    }
    
    message[26] = data.docking_flags;
    if(temp_docking_command != message[26]){
        RCLCPP_INFO(this->get_logger(), "[SEND_TO_MCU] DOCKING&CHARGE COMMAND SEND TO MCU HEX : 0x%02X",message[26]);
        temp_docking_command = message[26];
    }

    message[27] = data.imu_flags;
    if(temp_imu_calib_command != message[27]){
        RCLCPP_INFO(this->get_logger(), "[SEND_TO_MCU] IMU COMMAND SEND TO MCU HEX : 0x%02X",message[27]);
        temp_imu_calib_command = message[27];
    }
    
    message[28] = data.tofnBatt_flags;
    if(temp_tofnbatt_command != message[28]){
        RCLCPP_INFO(this->get_logger(), "[SEND_TO_MCU] TOF & BATT COMMAND SEND TO MCU HEX : 0x%02X",message[28]);
        temp_tofnbatt_command = message[28];
    }

    message[29] = data.index;

    // Checksum
    message[30] = calculateChecksum(message, 3, 29);

    // 전송
    try{
        uart_.sendData(message);
    }
    catch (const serial::IOException &e){
        RCLCPP_ERROR(this->get_logger(), "[sendProtocolV1Data] serial::IOEXCEPTION sending data: %s", e.what());
    }
    catch (const serial::SerialException &e){
        RCLCPP_ERROR(this->get_logger(), "[sendProtocolV1Data] serial::SerialException sending data: %s", e.what());
    }
    catch (const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "[sendProtocolV1Data] std::exception sending data: %s", e.what());
    }  
    catch (...){
        RCLCPP_ERROR(this->get_logger(), "[sendProtocolV1Data] Unknown error in Serial::write");
    }
    
#if false
    /* ----------------------------------------------------
     *  MCU_RESET → NORMAL_OPERATION 자동 클리어
     * --------------------------------------------------- */
    if (data.reset_flags == static_cast<uint8_t>(ResetCommand::MCU_RESET))
    // {
    //     std::lock_guard<std::mutex> lock(reset_mutex);
        gData.reset_flags = static_cast<uint8_t>(ResetCommand::NORMAL_OPERATION);
    // }
#endif
}
#else
void UARTCommunication::sendProtocolV1Data(TransmissionData& data)
{
    // 데이터 크기 계산: 헤더 4바이트 + 데이터 27바이트 = 31바이트
    std::vector<uint8_t> message(31, 0);

    message[0] = 0xAA; // PRE_AMBLE_0
    message[1] = 0x55; // PRE_AMBLE_1
    message[2] = 28;   // Data_Size (28바이트, 헤더 및 데이터 크기를 포함)
    message[3] = 0x51; // Header (Command Type)

    double linear_vel_mm = data.linear_velocity * 1000.0; // m/s를 mm/s로 변환

    uint64_t linear_velocity_bits;
    std::memcpy(&linear_velocity_bits, &linear_vel_mm, sizeof(uint64_t));

    for (int i = 0; i < 8; ++i) {
        uint8_t linear_velocity = (linear_velocity_bits >> (56 - 8 * i)) & 0xFF;
        int idx = 4 + i;
        message[idx] = linear_velocity;
        //RCLCPP_INFO(this->get_logger(), "indx: %d, linear_velocity_byte: %d",idx,linear_velocity);
    }

    uint64_t angular_velocity_bits;
    std::memcpy(&angular_velocity_bits, &data.angular_velocity, sizeof(uint64_t));

    for (int i = 0; i < 8; ++i) {
        uint8_t angular_velocity = (angular_velocity_bits >> (56 - 8 * i)) & 0xFF;
        int idx = 12 + i;
        message[idx] = angular_velocity;
        //RCLCPP_INFO(this->get_logger(), "indx: %d, angular_velocity: %d",idx,angular_velocity);
    }

    // Left Motor RPM (int16_t, Big-Endian 저장)
    message[20] = (data.left_mt_rpm >> 8) & 0xFF; // MSB
    message[21] = data.left_mt_rpm & 0xFF;        // LSB

    // Right Motor RPM (int16_t, Big-Endian 저장)
    message[22] = (data.right_mt_rpm >> 8) & 0xFF; // MSB
    message[23] = data.right_mt_rpm & 0xFF;        // LSB

    if( (data.linear_velocity != 0 || data.angular_velocity != 0) && (data.left_mt_rpm != 0 || data.right_mt_rpm != 0)){
        RCLCPP_INFO(this->get_logger(), "[WARNING - SEND TO MCU] linear_velocity and angular_velocity is not zero, but left_mt_rpm or right_mt_rpm is not zero");
    }
    // Reset Flags
    message[24] = data.reset_flags;
    if(temp_reset_command != message[24]){
        RCLCPP_INFO(this->get_logger(), "[SEND_TO_MCU] RESET COMMAND SEND TO MCU HEX : 0x%02X",message[24]);
        temp_reset_command = message[24];
    }

    // Motor Enable Flags
    message[25] = data.motor_flags;
    if(temp_motor_command != message[25]){
        RCLCPP_INFO(this->get_logger(), "[SEND_TO_MCU] MOTOR COMMAND SEND TO MCU HEX : 0x%02X",message[25]);
        temp_motor_command = message[25];
    }

    // DOCKING&CHARGE Flags
    message[26] = data.docking_flags;
    if(temp_docking_command != message[26]){
        RCLCPP_INFO(this->get_logger(), "[SEND_TO_MCU] DOCKING&CHARGE COMMAND SEND TO MCU HEX : 0x%02X",message[26]);
        temp_docking_command = message[26];
    }

    // IMU Calibration Flags
    message[27] = data.imu_flags;
    if(temp_imu_calib_command != message[27]){
        RCLCPP_INFO(this->get_logger(), "[SEND_TO_MCU] IMU COMMAND SEND TO MCU HEX : 0x%02X",message[27]);
        temp_imu_calib_command = message[27];
    }

    // tof Flags
    message[28] = data.tofnBatt_flags;
    if(temp_tof_command != message[28]){
        RCLCPP_INFO(this->get_logger(), "[SEND_TO_MCU] TOF & BATT COMMAND SEND TO MCU HEX : 0x%02X",message[28]);
        temp_tof_command = message[28];
    }
    // Index
    message[29] = data.index; // Index (0~255)

    // 체크섬 계산 (3번째 바이트부터 29번째 바이트까지)
    message[30] = calculateChecksum(message, 3, 29); // 체크섬 값은 예시로 설정, 실제로는 데이터 기반으로 계산해야 합니다.

    uart_.sendData(message);
    // UART로 데이터 전송 (디버그용으로 출력하는 예시)
    // for (auto byte : message) {
    //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    // }
    // std::cout << std::endl;
}
#endif

void UARTCommunication::timerCallback()
{
	auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = now - transmit_time;
    double milisec_interval = elapsed_seconds.count()*1000.0;
    static size_t call_count = 0;
    static size_t delayed_count = 0;

    if(call_count > 86400000){
        RCLCPP_INFO(this->get_logger(), "[timerCallback] reset check_cnt (%zu/%zu)", delayed_count, call_count);
        call_count = 0; 
        delayed_count = 0; 
    }
    ++call_count;

    if(now > transmit_time){
        if(milisec_interval >= 100.0 && call_count > 0){
            ++delayed_count;
            double delay_rate = static_cast<double>(delayed_count) / static_cast<double>(call_count) * 100.0;
            RCLCPP_INFO(this->get_logger(), "[timerCallback]trans to mcu interval : %.3f ms", milisec_interval);
            RCLCPP_INFO(this->get_logger(), "[timerCallback]trans Delay rate (>=100ms): %.2f%% (%zu/%zu)",delay_rate, delayed_count, call_count);
        }
    }else{
        RCLCPP_INFO(this->get_logger(), "[timerCallback]time-stamp Error!! interval is revert : %.3f ms", milisec_interval);
    }

    // 마지막 명령이 Ship 모드 이면서, 충전 단자가 인식되는 경우 Ship 모드를 해제 한다.
    if((static_cast<uint8_t>((temp_tofnbatt_command >> 4) & 0x0F) == static_cast<uint8_t>(ToFnBatt::BATT_SLEEP)) // 마지막 명령이 Ship 모드인 경우
        && (station_msg.docking_status & 0x10)) // 충전 단자 인식이 확인되는 경우
    {
        RCLCPP_INFO(this->get_logger(), "SET_BATT_CMD NORMAL : Docking Connect True");
        setBattMode(ToFnBatt::BATT_NORMAL);
    }

    sendProtocolV1Data(g_transmission_data);
	transmit_time = now;
}

void UARTCommunication::defaultTransmissionData(void) {
    memset(&g_transmission_data, 0, sizeof(TransmissionData));
}


uint8_t UARTCommunication::getUpperBits(uint8_t byte)
{
    return (byte >> 4);
}

uint8_t UARTCommunication::getLowerBits(uint8_t byte)
{
    return (byte & 0x0F);
}

uint16_t UARTCommunication::combineBytesToUint16(const std::vector<uint8_t> &data, int index)
{
    return (static_cast<uint16_t>(data[index]) << 8) | data[index + 1];
}

int16_t UARTCommunication::combineBytesToInt16(const std::vector<uint8_t> &data, int index)
{
    return (static_cast<int16_t>(data[index]) << 8) | data[index + 1];
}

uint32_t combineBytesToUint32(const std::vector<uint8_t> &data, size_t offset)
{
    return (data[offset] << 24) | (data[offset + 1] << 16) | (data[offset + 2] << 8) | data[offset + 3];
}

uint64_t UARTCommunication::combineBytesToUint64(const std::vector<uint8_t> &data, int startIndex)
{
    uint64_t value = 0;
    for (int i = 0; i < 8; i++)
    {
        value = (value << 8) | data[startIndex + i];
    }
    return value;
}

double UARTCommunication::combineBytesToDouble(const std::vector<uint8_t> &data, int index)
{
    double result;

    uint8_t byteArray[] = {
        data[index + 7],
        data[index + 6],
        data[index + 5],
        data[index + 4],
        data[index + 3],
        data[index + 2],
        data[index + 1],
        data[index + 0]};

    // std::memcpy(&result, &data[index], sizeof(double));
    std::memcpy(&result, byteArray, sizeof(double));

    return result;
}

/*V1DataPacket*/
template <typename PacketType>
void UARTCommunication::setOdomMsg(const PacketType &packet)
{
    // std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = (packet.x_position) / 1000.0;
    odom_msg.pose.pose.position.y = (packet.y_position) / 1000.0;
    odom_msg.pose.pose.position.z = 0.0;

    // ***USE IMU Orientation
    odom_msg.pose.pose.orientation = imu_orientation;
}

template <typename PacketType>
void UARTCommunication::setImuMsg(const PacketType &packet)
{
    // std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";

    double yaw = -packet.imu_yaw * DEG_TO_RAD_0_01;
    double pitch = packet.imu_pitch * DEG_TO_RAD_0_01;
    double roll = packet.imu_roll * DEG_TO_RAD_0_01;

    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = current_time - previous_time;
    double dt = elapsed_seconds.count();
    
    if(dt > 0.005 && dt < 0.015)
    {
        double roll_dot = (roll - previous_roll) / dt;
        double pitch_dot = (pitch - previous_pitch) / dt;
        double yaw_dot = (yaw - previous_yaw) / dt;    

        std::vector<double> angular_velocity = computeAngularVelocity(roll, pitch, roll_dot, pitch_dot, yaw_dot);
        if(angular_velocity.size() >= 3)
        {
            imu_msg.angular_velocity.x = angular_velocity[0];
            imu_msg.angular_velocity.y = angular_velocity[1];
            imu_msg.angular_velocity.z = angular_velocity[2];
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "[setImuMsg] angular_velocity size is %zu",angular_velocity.size());
        }
    }

    previous_roll = roll;
    previous_pitch = pitch;
    previous_yaw = yaw;
    previous_time = current_time;

    tf2::Quaternion imu_q;
    imu_q.setRPY(roll, pitch, yaw);
    imu_msg.orientation = tf2::toMsg(imu_q);
    imu_orientation = imu_msg.orientation;

    int16_t linear_acceleration_x = packet.imu_x_acc;
    int16_t linear_acceleration_y = packet.imu_y_acc;
    int16_t linear_acceleration_z = packet.imu_z_acc;
    imu_msg.linear_acceleration.x = linear_acceleration_x * ACCEL_SCALE;
    imu_msg.linear_acceleration.y = linear_acceleration_y * ACCEL_SCALE;
    imu_msg.linear_acceleration.z = linear_acceleration_z * ACCEL_SCALE;
}
template <typename PacketType>
void UARTCommunication::setBatteryMsg(const PacketType &packet)
{
    // std::lock_guard<std::mutex> lock(battery_mutex_);
    battery_msg.battery_voltage = static_cast<double>(packet.battery_voltage);
    battery_msg.battery_current = static_cast<double>(packet.battery_current);

    battery_msg.battery_percent = packet.battery_percent;
    battery_msg.cell_voltage1 = packet.cell_voltage1;
    battery_msg.cell_voltage2 = packet.cell_voltage2;
    battery_msg.cell_voltage3 = packet.cell_voltage3;
    battery_msg.cell_voltage4 = packet.cell_voltage4;
    battery_msg.cell_voltage5 = packet.cell_voltage5;
    battery_msg.total_capacity = packet.total_capacity;
    battery_msg.remaining_capacity = packet.remaining_capacity;
    battery_msg.battery_manufacturer = packet.battery_manufacturer;
    battery_msg.battery_temperature1 = packet.battery_temperature1;
    battery_msg.battery_temperature2 = packet.battery_temperature2;
    battery_msg.design_capacity = packet.design_capacity;
    battery_msg.number_of_cycles = packet.number_of_cycles;

    battery_msg.charge_status = static_cast<int>(packet.charging_mode);
    battery_msg.charging_mode = packet.charging_mode;

    if constexpr (std::is_same<PacketType, V2DataPacket>::value)
    {
        battery_msg.shipping_mode = packet.shipping_mode; // 1 : shipping ON(battery off) / 2 : shipping OFF (battery on) / 0 : Unknown

        // Precharge 상태 (0~1비트)
        uint8_t cur_precharge_state = packet.charge_control_flags & 0b00000011;
        // CC/CV 충전 상태 (2~3비트)
        uint8_t cur_charge_mode = (packet.charge_control_flags >> 2) & 0b00000011;
        // FET 상태 (4~5비트)
        uint8_t cur_fet_state = (packet.charge_control_flags >> 4) & 0b00000011;

        battery_msg.precharge_state = cur_precharge_state;
        battery_msg.charge_mode = cur_charge_mode;
        battery_msg.fet_state = cur_fet_state;
        battery_msg.battery_version = packet.battery_version;

        if (cur_precharge_state != prev_precharge_state) {
            RCLCPP_INFO(this->get_logger(),
                "Precharge State Changed: [%02u] ==> [%02u]  (00: Uncharged, 01: ON, 02: OFF)",
                prev_precharge_state, cur_precharge_state
            );
        }
        if (cur_charge_mode != prev_charge_mode) {
            RCLCPP_INFO(this->get_logger(),
                "Charge Mode Changed: [%02u] ==> [%02u]  (00: Uncharged, 01: CC, 02: CV)",
                prev_charge_mode, cur_charge_mode
            );
        }
        if (cur_fet_state != prev_fet_state) {
            RCLCPP_INFO(this->get_logger(),
                "FET Mode Changed: [%02u] ==> [%02u]  (00: Undefined, 01: OFF, 02: ON)",
                prev_fet_state, cur_fet_state
            );
        }

        prev_precharge_state = cur_precharge_state;
        prev_charge_mode = cur_charge_mode;
        prev_fet_state = cur_fet_state;

#if false
        const char *precharge_str[] = {"Not charging", "Precharge ON", "Precharge OFF", "Reserved"};
        const char *charge_mode_str[] = {"Not charging", "Constant Current (CC)", "Constant Voltage (CV)", "Reserved"};
        const char *fet_str[] = {"-", "FET OFF", "FET ON", "Reserved"};

        RCLCPP_INFO(this->get_logger(),
                    "Battery Status - Shipping: %u | Precharge: %s | Charge Mode: %s | FET: %s",
                    battery_msg.shipping_mode,
                    precharge_str[battery_msg.precharge_state],
                    charge_mode_str[battery_msg.charge_mode],
                    fet_str[battery_msg.fet_state]);

#endif
    }
    else
    {
        battery_msg.shipping_mode = 0;
        battery_msg.precharge_state = 0;
        battery_msg.charge_mode = 0;
        battery_msg.fet_state = 0;
    }

}
template <typename PacketType>
void UARTCommunication::setButtomIRMsg(const PacketType &packet)
{
    // std::lock_guard<std::mutex> lock(bottom_mutex_);

    bottom_ir_msg.adc_ff = packet.bottom_ir_1;
    bottom_ir_msg.adc_fl = packet.bottom_ir_2;
    bottom_ir_msg.adc_bl = packet.bottom_ir_3;
    bottom_ir_msg.adc_bb = packet.bottom_ir_4;
    bottom_ir_msg.adc_br = packet.bottom_ir_5;
    bottom_ir_msg.adc_fr = packet.bottom_ir_6;

}
template <typename PacketType>
float calculateAverage(const uint16_t *array, size_t size)
{
    uint32_t sum = 0;

    for (size_t i = 0; i < size; i++)
    {
        sum += array[i];
    }

    return static_cast<float>(sum) / size;
}
template <typename PacketType>
void UARTCommunication::setTofMsg(const PacketType &packet)
{
    double temp_amcl_x,temp_amcl_y, temp_amcl_angle;
    
    {
        // std::lock_guard<std::mutex> amcl_lock(amcl_pose_);
        temp_amcl_x = amcl_pose_x_;
        temp_amcl_y = amcl_pose_y_;
        temp_amcl_angle = amcl_pose_angle_;
    }

    // std::lock_guard<std::mutex> lock(tof_mutex_);
    tof_msg.timestamp = this->get_clock()->now();
    tof_msg.top = static_cast<double>(packet.top_tof) / 1000.0f;

    // Left ToF Remapping
    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            // [25.04.28 hyjoe: mtof 인덱스 수정 - column 반대 오류]
            // int sensor_index = row * 4 + (3 - col);
            int sensor_index = row * 4 + col;
            int topic_index = row * 4 + col;
            tof_msg.bot_left[topic_index] = static_cast<double>(packet.lower_left_tof[sensor_index]) / 1000.0f;
        }
    }

    // Right ToF Remapping
    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            // [25.04.28 hyjoe: mtof 인덱스 수정 - column 반대 오류]
            // int sensor_index = (3 - row) * 4 + col;
            int sensor_index = (3 - row) * 4 + (3 - col);
            int topic_index = row * 4 + col;
            tof_msg.bot_right[topic_index] = static_cast<double>(packet.lower_right_tof[sensor_index]) / 1000.0f;
        }
    }

    // status
    tof_msg.top_status = packet.top_1d_tof_status;
    tof_msg.bot_status = packet.lower_tof_status;

    // amcl pose
    tof_msg.robot_x = temp_amcl_x;
    tof_msg.robot_y = temp_amcl_y;
    tof_msg.robot_angle = temp_amcl_angle;
}
template <typename PacketType>
void UARTCommunication::setMotorMsg(const PacketType &packet)
{
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_sec = now-motor_communication_debug_time_;
    double runtime = elapsed_sec.count();

    // std::lock_guard<std::mutex> lock(motor_mutex_);
    motor_msg.left_motor_encoder = packet.left_motor_encoder;
    motor_msg.right_motor_encoder = packet.right_motor_encoder;

    motor_msg.left_motor_current = packet.left_motor_current;
    motor_msg.right_motor_current = packet.right_motor_current;
    
    motor_msg.left_motor_rpm = packet.left_motor_rpm;
    motor_msg.right_motor_rpm = packet.right_motor_rpm;

    motor_msg.left_motor_temperature = packet.left_motor_temperature;
    motor_msg.right_motor_temperature = packet.right_motor_temperature;
 
    motor_msg.left_motor_status = packet.left_motor_status;
    motor_msg.right_motor_status = packet.right_motor_status;

    motor_msg.left_motor_type = packet.left_motor_type;
    motor_msg.right_motor_type = packet.right_motor_type;
 
    motor_msg.motor_mode = packet.motor_mode;

    if(runtime >= 60.0){
        if(motor_msg.left_motor_status == 0x00 || motor_msg.right_motor_status == 0x00){
            if(motor_msg.right_motor_status == 0x00){
                RCLCPP_INFO(this->get_logger(), "right motor communication error");
            }else{
                RCLCPP_INFO(this->get_logger(), "left motor communication error");
            }
            motor_communication_debug_time_ = std::chrono::steady_clock::now();
        }
    }
}
template <typename PacketType>
void UARTCommunication::setDockingMsg(const PacketType &packet)
{
    static uint8_t prev_docking_state = 0;
    uint8_t prev_all_docking_status = 0, new_all_docking_status = 0,prev_charging_status = 0,prev_docking_status = 0,new_charging_status = 0, new_docking_status = 0; 
    uint8_t prev_short_sig = 0, prev_long_sig = 0, new_short_sig = 0, new_long_sig = 0;
    uint8_t new_docking_state = 0;
    bool isDocking = false, isOnstation = false, wasDocking = false, wasOnstation = false;
    {
        // std::lock_guard<std::mutex> lock(station_mutex_);
        prev_all_docking_status = station_msg.docking_status;
        new_all_docking_status = packet.docking_status;
        prev_docking_status = (prev_all_docking_status & 0x0F);
        prev_charging_status = (prev_all_docking_status & 0xF0);
        prev_short_sig = station_msg.sig_short;
        prev_long_sig = station_msg.sig_long;
        new_docking_status = (new_all_docking_status & 0x0F);
        new_charging_status = (new_all_docking_status & 0xF0);
        new_short_sig = packet.dock_short_ir_position;
        new_long_sig = packet.dock_long_ir_position;
        station_msg.sig_short = new_short_sig;
        station_msg.sig_long = new_long_sig;
        station_msg.receiver_status = packet.dock_sig_status;
        station_msg.docking_status = new_all_docking_status;
        station_msg.left_terminal = packet.cradle_adc0;
        station_msg.right_terminal = packet.cradle_adc1;
        isDocking = (new_docking_status & 0x01);
        isOnstation = (new_charging_status & 0x70);
        wasDocking = (prev_docking_status & 0x01);
        wasOnstation = (prev_charging_status & 0x70);
        if constexpr (std::is_same<PacketType, V2DataPacket>::value){
            new_docking_state = packet.docking_state;
        }
    }

    if(prev_docking_state != new_docking_state){
        RCLCPP_INFO(this->get_logger(), "[setDockingMsg] mcu-docking step prev[%d]-->new[%d]", prev_docking_state,new_docking_state);
        prev_docking_state = new_docking_state;
    }

    if(wasDocking != isDocking){
        if(isDocking){
            RCLCPP_INFO(this->get_logger(), "[setDockingMsg] mcu-docking start docking status[0x%02X]", new_all_docking_status);
        }else{
            RCLCPP_INFO(this->get_logger(), "[setDockingMsg] mcu-docking stop docking status[0x%02X]",new_all_docking_status);
        }
    }

    if(wasOnstation != isOnstation){
        if(isOnstation){
            RCLCPP_INFO(this->get_logger(), "[setDockingMsg] on-station docking status[0x%02X]", new_all_docking_status);
        }else{
            RCLCPP_INFO(this->get_logger(), "[setDockingMsg] off-station docking status[0x%02X]",new_all_docking_status);
        }
    }

    if(prev_short_sig != new_short_sig || prev_long_sig != new_long_sig ){
        RCLCPP_INFO(this->get_logger(), "[setDockingMsg] sig short : 0x%02X -> 0x%02X, sig long : 0x%02X -> 0x%02X", prev_short_sig, new_short_sig, prev_long_sig, new_long_sig);
    }

    if(isDocking && isOnstation)
    {
        RCLCPP_INFO(this->get_logger(), "[setDockingMsg] docking on station --> set Docking stop docking status[0x%02x] ",new_all_docking_status);
        setDocking(static_cast<DockingChargeCommand>(0)); // 도킹 signal 감지시 docking stop 패킷 설정.
    }
}
template <typename PacketType>
void UARTCommunication::setBottomMsg(const PacketType &packet)
{
    double temp_amcl_x,temp_amcl_y, temp_amcl_angle;
    
    {
        // std::lock_guard<std::mutex> amcl_lock(amcl_pose_);
        temp_amcl_x = amcl_pose_x_;
        temp_amcl_y = amcl_pose_y_;
        temp_amcl_angle = amcl_pose_angle_;
    }
    
    // std::lock_guard<std::mutex> lock(bottom_ir_mutex_);
    bottom_status_msg.data = packet.bottom_wheel_lifting_sensor;

    bottom_ir_msg.timestamp = this->get_clock()->now();

    bottom_ir_msg.ff = (packet.bottom_wheel_lifting_sensor & (1 << 0)) != 0;
    bottom_ir_msg.fl = (packet.bottom_wheel_lifting_sensor & (1 << 1)) != 0;
    bottom_ir_msg.bl = (packet.bottom_wheel_lifting_sensor & (1 << 2)) != 0;
    bottom_ir_msg.bb = (packet.bottom_wheel_lifting_sensor & (1 << 3)) != 0;
    bottom_ir_msg.br = (packet.bottom_wheel_lifting_sensor & (1 << 4)) != 0;
    bottom_ir_msg.fr = (packet.bottom_wheel_lifting_sensor & (1 << 5)) != 0;

    if constexpr (std::is_same<PacketType, V2DataPacket>::value)
    {
        bottom_ir_msg.adc_ff = packet.bottom_ir_1;
        bottom_ir_msg.adc_fl = packet.bottom_ir_2;
        bottom_ir_msg.adc_bl = packet.bottom_ir_3;
        bottom_ir_msg.adc_bb = packet.bottom_ir_4;
        bottom_ir_msg.adc_br = packet.bottom_ir_5;
        bottom_ir_msg.adc_fr = packet.bottom_ir_6;
#if false
        RCLCPP_INFO(this->get_logger(), 
        "Bottom IR ADC values - FF: %u, FL: %u, BL: %u, BB: %u, BR: %u, FR: %u",
        bottom_ir_msg.adc_ff,
        bottom_ir_msg.adc_fl,
        bottom_ir_msg.adc_bl,
        bottom_ir_msg.adc_bb,
        bottom_ir_msg.adc_br,
        bottom_ir_msg.adc_fr);
#endif
    }
    else
    {
        bottom_ir_msg.adc_ff = 0;
        bottom_ir_msg.adc_fl = 0;
        bottom_ir_msg.adc_bl = 0;
        bottom_ir_msg.adc_bb = 0;
        bottom_ir_msg.adc_br = 0;
        bottom_ir_msg.adc_fr = 0;
    }
    // amcl pose
    bottom_ir_msg.robot_x = temp_amcl_x;
    bottom_ir_msg.robot_y = temp_amcl_y;
    bottom_ir_msg.robot_angle = temp_amcl_angle;
}
template <typename PacketType>
void UARTCommunication::setOdomStatusMsg(const PacketType &packet)
{
    static uint8_t prev_odom_status = 0;
    static uint8_t prev_imu_status = 0;

    // std::lock_guard<std::mutex> lock(imu_odom_status_mutex_);
    uint8_t odom_status = getUpperBits(packet.imu_odometry_status);
    uint8_t imu_status = getLowerBits(packet.imu_odometry_status);
    if(imu_odom_status_msg.data != packet.imu_odometry_status){
        if(imu_status != prev_imu_status){
            RCLCPP_INFO(this->get_logger(), "imu status is changed 0x%02X -> 0x%02X", prev_imu_status, imu_status);
            prev_imu_status = imu_status;
        }
        if(odom_status != prev_odom_status){
            RCLCPP_INFO(this->get_logger(), "odom status is changed 0x%02X -> 0x%02X", prev_odom_status, odom_status);
            prev_odom_status = odom_status;
        }
    }
    imu_odom_status_msg.data = packet.imu_odometry_status;
   
}
template <typename PacketType>
void UARTCommunication::setFwVersionMsg(const PacketType &packet)
{
    char buffer[16]; // 넉넉하게
    snprintf(buffer, sizeof(buffer), "%u.%02u", packet.firmware_ver_msb, packet.firmware_ver_lsb);

    std::string tmp(buffer);

    if (fwVersion_msg.data != tmp)
    {
        RCLCPP_INFO(this->get_logger(), "packet.firmware Major version : %u", packet.firmware_ver_msb);
        RCLCPP_INFO(this->get_logger(), "packet.firmware Minor version : %02u", packet.firmware_ver_lsb);
        fwVersion_msg.data = tmp;
        RCLCPP_INFO(this->get_logger(), "fwVersion_msg.data : %s", fwVersion_msg.data.c_str());
        // fw_version_pub_->publish(fwVersion_msg);
    }
}
template <typename PacketType>
void UARTCommunication::setMcuErrorMsg(const PacketType &packet)
{
    // std::lock_guard<std::mutex> lock(error_mutex_);
    //Packet Error code 1
    updateErrorStatus(packet.error_code_1, ErrorCode1::ERROR_LEFT_MT_UART, " ERROR_LEFT_MT_UART", error_left_mt_uart_msg,1);
    updateErrorStatus(packet.error_code_1, ErrorCode1::ERROR_LEFT_MT_STALL|ErrorCode1::ERROR_LEFT_MT_CURRENT, " ERROR_LEFT_MT_OVERCURRENT", error_left_mt_stall_or_overcurrent_msg,1);
    updateErrorStatus(packet.error_code_1, ErrorCode1::ERROR_LEFT_MT_OVERHEAT, " ERROR_LEFT_MT_OVERHEAT", error_left_mt_overheat_msg,1);
    //updateErrorStatus(packet.error_code_1, ErrorCode1::ERROR_LEFT_MT_CURRENT, " ERROR_LEFT_MT_OVERCURRENT", error_left_mt_stall_or_overcurrent_msg,1);
    updateErrorStatus(packet.error_code_1, ErrorCode1::ERROR_RIGHT_MT_UART, " ERROR_RIGHT_MT_UART", error_right_mt_uart_msg,1);
    updateErrorStatus(packet.error_code_1, ErrorCode1::ERROR_RIGHT_MT_STALL|ErrorCode1::ERROR_RIGHT_MT_CURRENT, " ERROR_RIGHT_MT_OVERCURRENT", error_right_mt_stall_or_overcurrent_msg,1);
    updateErrorStatus(packet.error_code_1, ErrorCode1::ERROR_RIGHT_MT_OVERHEAT, " ERROR_RIGHT_MT_OVERHEAT", error_right_mt_overheat_msg,1);
    //updateErrorStatus(packet.error_code_1, ErrorCode1::ERROR_RIGHT_MT_CURRENT, " ERROR_RIGHT_MT_OVERCURRENT", error_right_mt_stall_or_overcurrent_msg,1);

    //Packet Error code 2
    updateErrorStatus(packet.error_code_2, ErrorCode2::ERROR_Left_TOF_I2C, " ERROR_Left_TOF_I2C", error_left_tof_i2c_msg,2);
    updateErrorStatus(packet.error_code_2, ErrorCode2::ERROR_Right_TOF_I2C, " ERROR_Right_TOF_I2C", error_right_tof_i2c_msg,2);
    updateErrorStatus(packet.error_code_2, ErrorCode2::ERROR_1D_TOF_UART, " ERROR_1D_TOF_UART", error_1d_tof_uart_msg,2);
    updateErrorStatus(packet.error_code_2, ErrorCode2::ERROR_IMU_UART, " ERROR_IMU_UART", error_imu_uart_msg,2);
    // updateErrorStatus(packet.error_code_2, ErrorCode2::ERROR_AP_RX_UART, " ERROR_AP_RX_UART", error_ap_rx_msg,2);
    updateErrorStatus(packet.error_code_2, ErrorCode2::ERROR_BATTERY_COMMUNICATION, " ERROR_BATTERY_COMMUNICATION", error_battery_communication_msg,2);
    updateErrorStatus(packet.error_code_2, ErrorCode2::ERROR_CHARGE_OVERCURRENT, " ERROR_CHARGE_OVERCURRENT", error_charge_overcurrent_msg,2);
    updateErrorStatus(packet.error_code_2, ErrorCode2::ERROR_DISCHARGE_OVERCURRENT, " ERROR_DISCHARGE_OVERCURRENT", error_discharge_overcurrent_msg,2);

    //Packet Error code 3
    updateErrorStatus(packet.error_code_3, ErrorCode3::ERROR_BATTERY_CHARGE_OVER_HEAT, " ERROR_BATTERY_CHARGE_OVER_HEAT", error_battery_charge_overheat_msg,3);
    updateErrorStatus(packet.error_code_3, ErrorCode3::ERROR_BATTERY_DISCHAR_OVER_HEAT, " ERROR_BATTERY_DISCHARGE_OVER_HEAT", error_battery_discharge_overheat_msg,3);
    updateErrorStatus(packet.error_code_3, ErrorCode3::ERROR_POGO_PIN_OVER_HEAT, " ERROR_POGO_PIN_OVER_HEAT", error_pogo_pin_overheat_msg,3);
    updateErrorStatus(packet.error_code_3, ErrorCode3::ERROR_DOCKING_SIG_NOT_FOUND, " ERROR_DOCKING_SIG_NOT_FOUND", error_docking_sig_msg,3);
}

void UARTCommunication::updateErrorStatus(uint8_t errorCode, uint8_t mask, const char *errorMessage, std_msgs::msg::Bool &error_msg, uint8_t errorCategory) {
    if (errorCategory < 1 || errorCategory > 3){
        RCLCPP_INFO(this->get_logger(), "Invalid error category: %d", errorCategory);
        return;
    }
    
    if (mask == 0) {
        RCLCPP_WARN(this->get_logger(), "Invalid error mask: 0 [category: %d, message: %s]", errorCategory, errorMessage ? errorMessage : "unknown");
        return;
    }
    
    int bitPosition = __builtin_ctz(mask);  // Get bit index (0-7)
    if (bitPosition < 0 || bitPosition >= 8){
        RCLCPP_INFO(this->get_logger(), "Invalid mask bit position: %d", bitPosition);
        return;
    }

    int categoryIndex = errorCategory - 1;  // Convert to 0-based index
    bool errorOccurred = (errorCode & mask);
    bool &previousState = errorStates[categoryIndex][bitPosition];
    

    error_msg.data = errorOccurred;

    if (errorOccurred && !previousState) {
        RCLCPP_INFO(this->get_logger(), "[%s]error occurred code[%02x] mask[%02x] category[%d] idx[%d] bitPosition[%d]"
        , errorMessage ? errorMessage : "unknown", errorCode, mask, errorCategory,categoryIndex,bitPosition);
        previousState = true;
    } else if (!errorOccurred && previousState) {
        RCLCPP_INFO(this->get_logger(), "[%s]error cleared code[%02x] mask[%02x] category[%d] idx[%d] bitPosition[%d]"
        , errorMessage ? errorMessage : "unknown", errorCode, mask, errorCategory,categoryIndex,bitPosition);
        previousState = false;
    }

    // motor 에러 발생 시 온도 로깅 추가 (hyjoe: 25.06.04)
    if (errorOccurred &&
        (std::strcmp(errorMessage, " ERROR_LEFT_MT_OVERHEAT") == 0 ||
         std::strcmp(errorMessage, " ERROR_RIGHT_MT_OVERHEAT") == 0)) {
        RCLCPP_INFO(this->get_logger(),
            "Left Motor(cur:%d, rpm:%d, temp:%d), Right Motor(cur:%d, rpm:%d, temp:%d)",
            motor_msg.left_motor_current, motor_msg.left_motor_rpm, motor_msg.left_motor_temperature, 
            motor_msg.right_motor_current, motor_msg.right_motor_rpm, motor_msg.right_motor_temperature 
        );
    }
}

template <typename PacketType>
void UARTCommunication::setCradleADCMsg(const PacketType &packet)
{
    // 미충전 -> 충전 || 충전 -> 미충전 상태로 변하는 시점에 cradle adc 데이터 로깅.
    static bool firstLogging = true;
    static bool prevCharging = false;    
    static bool detect_charge_moving = false;    

    bool currentCharging = packet.docking_status & 0x70; // true: charging
    if (firstLogging) {
        firstLogging = false;
        RCLCPP_INFO(this->get_logger(),
            "Initial docking status:[%02x] Charging State: [%s]\n"
            "Battery Manufacturer:[%d] / Percentage:[%d %%] / Current:[%d mA] / Voltage:[%d mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
            "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n"
            "Cradle Left ADC: %d, Cradle Right ADC: %d",
            packet.docking_status, currentCharging ? "charging" : "uncharged",
            packet.battery_manufacturer, packet.battery_percent, packet.battery_current, packet.battery_voltage, packet.battery_temperature1, packet.battery_temperature2,
            battery_msg.cell_voltage1, battery_msg.cell_voltage2, battery_msg.cell_voltage3, battery_msg.cell_voltage4, battery_msg.cell_voltage5,
            static_cast<int>(packet.cradle_adc0), static_cast<int>(packet.cradle_adc1));
    } else if (prevCharging != currentCharging) {
        RCLCPP_INFO(this->get_logger(),
            "docking status:[%02x] Charging State Changed [%s] -> [%s]\n"
            "Battery Manufacturer:[%d] / Percentage:[%d %%] / Current:[%d mA] / Voltage:[%d mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
            "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n"
            "Cradle Left ADC: %d, Cradle Right ADC: %d",
            packet.docking_status, prevCharging ? "charging" : "uncharged", currentCharging ? "charging" : "uncharged",
            packet.battery_manufacturer, packet.battery_percent, packet.battery_current, packet.battery_voltage,packet.battery_temperature1,packet.battery_temperature2,
            battery_msg.cell_voltage1, battery_msg.cell_voltage2, battery_msg.cell_voltage3, battery_msg.cell_voltage4, battery_msg.cell_voltage5,
            static_cast<int>(packet.cradle_adc0), static_cast<int>(packet.cradle_adc1));
    }

    if (currentCharging) {
        double linear_vel = getLinearVelocity();
        double angular_vel = getAngularVelocity();
        if(fabs(linear_vel) < 0.01f && fabs(angular_vel) < 0.03f) {
            if(abs(packet.left_motor_rpm) > 1 || abs(packet.right_motor_rpm) > 1) {
                if(!detect_charge_moving) {
                    RCLCPP_INFO(this->get_logger(),"[Charge_Moving](1) docking status:[%02x], Left Motor(cur:%d, rpm:%d, temp:%d), Right Motor(cur:%d, rpm:%d, temp:%d)",
                    packet.docking_status, 
                    packet.left_motor_current, packet.left_motor_rpm, packet.left_motor_temperature, 
                    packet.right_motor_current, packet.right_motor_rpm, packet.right_motor_temperature);
                    detect_charge_moving = true;
                }
            } else {
                if(detect_charge_moving && packet.left_motor_rpm == 0 && packet.right_motor_rpm == 0) {
                    RCLCPP_INFO(this->get_logger(),"[Charge_Moving](0) docking status:[%02x], Left Motor(cur:%d, rpm:%d, temp:%d), Right Motor(cur:%d, rpm:%d, temp:%d)",
                    packet.docking_status, 
                    packet.left_motor_current, packet.left_motor_rpm, packet.left_motor_temperature, 
                    packet.right_motor_current, packet.right_motor_rpm, packet.right_motor_temperature);
                    detect_charge_moving = false;
                }
            }
        } else {
            detect_charge_moving = false;
        }
    } else {
        detect_charge_moving = false;
    }

    prevCharging = currentCharging;
}

void UARTCommunication::ParseV2Data(const std::vector<uint8_t> &data, V2DataPacket &packet)
{

    packet.pre_amble_0 = data[0];  // 0xAA
    packet.pre_amble_1 = data[1];  // 0x55
    packet.data_size = data[2];    // Data Size: 166
    packet.command = data[3];      // Command: 0x11

    // Odometry data
    packet.x_position = combineBytesToDouble(data, 4);  // X Position (double)
    packet.y_position = combineBytesToDouble(data, 12); // Y Position (double)

    // Motor status and types
    packet.left_motor_status = data[20];   // Left Motor Status
    packet.right_motor_status = data[21];  // Right Motor Status
    packet.motor_mode = data[22];          // Motor Mode
    packet.left_motor_type = data[23];     // Left Motor Type (0x01 or 0x02)
    packet.right_motor_type = data[24];    // Right Motor Type (0x01 or 0x02)

    // Error codes
    packet.error_code_1 = data[27];        // Error Code 3 (0x00: No Error, 0x01: Error)
    packet.error_code_2 = data[26];        // Error Code 2
    packet.error_code_3 = data[25];             // Erro Code 1

    // IMU data (Index, Yaw, Pitch, Roll, Acceleration)
    packet.imu_index = data[28];           // IMU Index (0-255)
    packet.imu_yaw = combineBytesToInt16(data, 29);   // IMU Yaw (16-bit)
    packet.imu_pitch = combineBytesToInt16(data, 31); // IMU Pitch (16-bit)
    packet.imu_roll = combineBytesToInt16(data, 33);  // IMU Roll (16-bit)

    packet.imu_x_acc = combineBytesToInt16(data, 35); // IMU X-axis Acceleration
    packet.imu_y_acc = combineBytesToInt16(data, 37); // IMU Y-axis Acceleration
    packet.imu_z_acc = combineBytesToInt16(data, 39); // IMU Z-axis Acceleration

    // Battery voltage and capacity data
    packet.cell_voltage1 = combineBytesToInt16(data, 41);  // Cell Voltage 1 (mV)
    packet.cell_voltage2 = combineBytesToInt16(data, 43);  // Cell Voltage 2 (mV)
    packet.cell_voltage3 = combineBytesToInt16(data, 45);  // Cell Voltage 3 (mV)
    packet.cell_voltage4 = combineBytesToInt16(data, 47);  // Cell Voltage 4 (mV)
    packet.cell_voltage5 = combineBytesToInt16(data, 49);  // Cell Voltage 5 (mV)

    packet.total_capacity = combineBytesToInt16(data, 51);       // Total Capacity (mAh)
    packet.remaining_capacity = combineBytesToInt16(data, 53);   // Remaining Capacity (mAh)
    packet.battery_manufacturer = data[55];  // Battery Manufacturer (0x00, 0x01, or 0x02)

    // Charging status and other sensor data
    packet.charge_status = data[56];          // Charge Status
    packet.imu_odometry_status = data[57];    // IMU and Odometry Status
    packet.bottom_wheel_lifting_sensor = data[58]; // Bottom Wheel Lifting Sensor
    packet.dock_sig_status = data[59];        // Dock Signal Status
    packet.dock_short_ir_position = data[60]; // Dock Short IR Position
    packet.docking_status = data[61];         // Docking Status

    packet.dock_long_ir_position = data[62];  // Dock Long IR Position
    packet.battery_percent = data[63];        // Battery Percentage (0-100%)
    packet.battery_voltage = combineBytesToInt16(data, 64); // Battery Voltage (mV)
    packet.battery_current = combineBytesToInt16(data, 66); // Battery Current (mA)
    packet.battery_temperature1 = data[68];   // Battery Temperature Sensor 1
    packet.battery_temperature2 = data[69];   // Battery Temperature Sensor 2
    packet.design_capacity = combineBytesToInt16(data, 70); // Design Capacity (mAh)
    packet.number_of_cycles = combineBytesToInt16(data, 72); // Number of Cycles (uint16_t)

    // TOF and IMU calibration data
    packet.top_tof = combineBytesToInt16(data, 74);       // Top TOF (mm)
    packet.top_1d_tof_status = data[76];   // Top 1D TOF Status

    // Lower Left TOF (mm) - Populate the array using a loop
    for (int i = 0; i < 16; ++i)
    {
        packet.lower_left_tof[i] = combineBytesToInt16(data, 77 + (i * 2)); // Lower Left TOF (mm)
    }

    // Lower Right TOF (mm) - Populate the array using a loop
    for (int i = 0; i < 16; ++i)
    {
        packet.lower_right_tof[i] = combineBytesToInt16(data, 109 + (i * 2)); // Lower Right TOF (mm)
    }

    packet.lower_tof_status = data[141];    // Left/Right TOF Status
    if(temp_tof_status_from_mcu != packet.lower_tof_status){
        RCLCPP_INFO(this->get_logger(), "[RECV_FROM_MCU] TOF_STATUS CHANGED [0x%02X->0x%02X]", temp_tof_status_from_mcu, data[141]);
        temp_tof_status_from_mcu = data[141];
    }

    packet.imu_calibration_status = data[142];  // IMU Calibration Status

    // Motor encoder and current data
    packet.left_motor_encoder = combineBytesToUint32(data, 143);  // Left Motor Encoder (int32_t)
    packet.right_motor_encoder = combineBytesToUint32(data, 147); // Right Motor Encoder (int32_t)

    packet.left_motor_current = combineBytesToInt16(data, 151);  // Left Motor Current (10mA)
    packet.right_motor_current = combineBytesToInt16(data, 153); // Right Motor Current (10mA)
    packet.left_motor_rpm = combineBytesToInt16(data, 155);     // Left Motor RPM
    packet.right_motor_rpm = combineBytesToInt16(data, 157);    // Right Motor RPM

    // Cradle data (Temperature and Firmware version)
    packet.cradle_adc0 = data[159];  // Cradle ADC (Temperature in °C)
    packet.cradle_adc1 = data[160];  // Cradle ADC (Temperature in °C)
    packet.cradle_fw_ver_msb = data[161]; // Cradle FW Major Version
    packet.cradle_fw_ver_lsb = data[162]; // Cradle FW Minor Version

    packet.charging_mode = data[163]; // Charging Mode

    // Motor temperatures and firmware version
    packet.left_motor_temperature = data[164];  // Left Motor Temperature (°C)
    packet.right_motor_temperature = data[165]; // Right Motor Temperature (°C)

    // BOTTOM IR sensors (2 bytes each: LSB first, MSB second)
    // packet.bottom_ir_1 = (static_cast<uint16_t>(data[167]) << 8) | data[166];
    packet.bottom_ir_1 = (static_cast<uint16_t>(data[166]) << 8) | data[167];

    packet.bottom_ir_2 = (static_cast<uint16_t>(data[168]) << 8) | data[169];
    packet.bottom_ir_3 = (static_cast<uint16_t>(data[170]) << 8) | data[171];
    packet.bottom_ir_4 = (static_cast<uint16_t>(data[172]) << 8) | data[173];
    packet.bottom_ir_5 = (static_cast<uint16_t>(data[174]) << 8) | data[175];
    packet.bottom_ir_6 = (static_cast<uint16_t>(data[176]) << 8) | data[177];

    
    packet.shipping_mode = data[178];
    // Reserve fields
    // packet.reserve_1 = data[179];
    packet.charge_control_flags = data[179];

    packet.docking_state = data[180];
    packet.battery_version = data[181];

    packet.firmware_ver_msb = data[182]; // Firmware Major Version
    packet.firmware_ver_lsb = data[183]; // Firmware Minor Version
    packet.checksum = data[184];         // Checksum (from byte 3 to byte 167)
}

void UARTCommunication::ParseV1Data(const std::vector<uint8_t> &data, V1DataPacket &packet)
{

    packet.pre_amble_0 = data[0];  // 0xAA
    packet.pre_amble_1 = data[1];  // 0x55
    packet.data_size = data[2];    // Data Size: 166
    packet.command = data[3];      // Command: 0x11

    // Odometry data
    packet.x_position = combineBytesToDouble(data, 4);  // X Position (double)
    packet.y_position = combineBytesToDouble(data, 12); // Y Position (double)

    // Motor status and types
    packet.left_motor_status = data[20];   // Left Motor Status
    packet.right_motor_status = data[21];  // Right Motor Status
    packet.motor_mode = data[22];          // Motor Mode
    packet.left_motor_type = data[23];     // Left Motor Type (0x01 or 0x02)
    packet.right_motor_type = data[24];    // Right Motor Type (0x01 or 0x02)

    // Error codes
    packet.error_code_1 = data[27];        // Error Code 3 (0x00: No Error, 0x01: Error)
    packet.error_code_2 = data[26];        // Error Code 2
    packet.error_code_3 = data[25];             // Erro Code 1

    // IMU data (Index, Yaw, Pitch, Roll, Acceleration)
    packet.imu_index = data[28];           // IMU Index (0-255)
    packet.imu_yaw = combineBytesToInt16(data, 29);   // IMU Yaw (16-bit)
    packet.imu_pitch = combineBytesToInt16(data, 31); // IMU Pitch (16-bit)
    packet.imu_roll = combineBytesToInt16(data, 33);  // IMU Roll (16-bit)

    packet.imu_x_acc = combineBytesToInt16(data, 35); // IMU X-axis Acceleration
    packet.imu_y_acc = combineBytesToInt16(data, 37); // IMU Y-axis Acceleration
    packet.imu_z_acc = combineBytesToInt16(data, 39); // IMU Z-axis Acceleration

    // Battery voltage and capacity data
    packet.cell_voltage1 = combineBytesToInt16(data, 41);  // Cell Voltage 1 (mV)
    packet.cell_voltage2 = combineBytesToInt16(data, 43);  // Cell Voltage 2 (mV)
    packet.cell_voltage3 = combineBytesToInt16(data, 45);  // Cell Voltage 3 (mV)
    packet.cell_voltage4 = combineBytesToInt16(data, 47);  // Cell Voltage 4 (mV)
    packet.cell_voltage5 = combineBytesToInt16(data, 49);  // Cell Voltage 5 (mV)

    packet.total_capacity = combineBytesToInt16(data, 51);       // Total Capacity (mAh)
    packet.remaining_capacity = combineBytesToInt16(data, 53);   // Remaining Capacity (mAh)
    packet.battery_manufacturer = data[55];  // Battery Manufacturer (0x00, 0x01, or 0x02)

    // Charging status and other sensor data
    packet.charge_status = data[56];          // Charge Status
    packet.imu_odometry_status = data[57];    // IMU and Odometry Status
    packet.bottom_wheel_lifting_sensor = data[58]; // Bottom Wheel Lifting Sensor
    packet.dock_sig_status = data[59];        // Dock Signal Status
    packet.dock_short_ir_position = data[60]; // Dock Short IR Position
    packet.docking_status = data[61];         // Docking Status

    packet.dock_long_ir_position = data[62];  // Dock Long IR Position
    packet.battery_percent = data[63];        // Battery Percentage (0-100%)
    packet.battery_voltage = combineBytesToInt16(data, 64); // Battery Voltage (mV)
    packet.battery_current = combineBytesToInt16(data, 66); // Battery Current (mA)
    packet.battery_temperature1 = data[68];   // Battery Temperature Sensor 1
    packet.battery_temperature2 = data[69];   // Battery Temperature Sensor 2
    packet.design_capacity = combineBytesToInt16(data, 70); // Design Capacity (mAh)
    packet.number_of_cycles = combineBytesToInt16(data, 72); // Number of Cycles (uint16_t)

    // TOF and IMU calibration data
    packet.top_tof = combineBytesToInt16(data, 74);       // Top TOF (mm)
    packet.top_1d_tof_status = data[76];   // Top 1D TOF Status

    // Lower Left TOF (mm) - Populate the array using a loop
    for (int i = 0; i < 16; ++i)
    {
        packet.lower_left_tof[i] = combineBytesToInt16(data, 77 + (i * 2)); // Lower Left TOF (mm)
    }

    // Lower Right TOF (mm) - Populate the array using a loop
    for (int i = 0; i < 16; ++i)
    {
        packet.lower_right_tof[i] = combineBytesToInt16(data, 109 + (i * 2)); // Lower Right TOF (mm)
    }

    packet.lower_tof_status = data[141];    // Left/Right TOF Status
    packet.imu_calibration_status = data[142];  // IMU Calibration Status

    // Motor encoder and current data
    packet.left_motor_encoder = combineBytesToUint32(data, 143);  // Left Motor Encoder (int32_t)
    packet.right_motor_encoder = combineBytesToUint32(data, 147); // Right Motor Encoder (int32_t)

    packet.left_motor_current = combineBytesToInt16(data, 151);  // Left Motor Current (10mA)
    packet.right_motor_current = combineBytesToInt16(data, 153); // Right Motor Current (10mA)
    packet.left_motor_rpm = combineBytesToInt16(data, 155);     // Left Motor RPM
    packet.right_motor_rpm = combineBytesToInt16(data, 157);    // Right Motor RPM

    // Cradle data (Temperature and Firmware version)
    packet.cradle_adc0 = data[159];  // Cradle ADC (Temperature in °C)
    packet.cradle_adc1 = data[160];  // Cradle ADC (Temperature in °C)
    packet.cradle_fw_ver_msb = data[161]; // Cradle FW Major Version
    packet.cradle_fw_ver_lsb = data[162]; // Cradle FW Minor Version

    packet.charging_mode = data[163]; // Charging Mode

    // Motor temperatures and firmware version
    packet.left_motor_temperature = data[164];  // Left Motor Temperature (°C)
    packet.right_motor_temperature = data[165]; // Right Motor Temperature (°C)

    packet.firmware_ver_msb = data[166]; // Firmware Major Version
    packet.firmware_ver_lsb = data[167]; // Firmware Minor Version
    packet.checksum = data[168];         // Checksum (from byte 3 to byte 167)
}

void UARTCommunication::printV1DataPacket(const V1DataPacket &packet)
{
 // Print basic information
    RCLCPP_INFO(this->get_logger(), "Pre_amble_0: 0x%02x", static_cast<int>(packet.pre_amble_0));
    RCLCPP_INFO(this->get_logger(), "Pre_amble_1: 0x%02x", static_cast<int>(packet.pre_amble_1));
    RCLCPP_INFO(this->get_logger(), "Data_Size: %d", static_cast<int>(packet.data_size));
    RCLCPP_INFO(this->get_logger(), "Command: 0x%02x", static_cast<int>(packet.command));

    // Print odometry data
    RCLCPP_INFO(this->get_logger(), "X Position: %f mm", packet.x_position);
    RCLCPP_INFO(this->get_logger(), "Y Position: %f mm", packet.y_position);

    // Print motor status and types
    RCLCPP_INFO(this->get_logger(), "Left Motor Status: 0x%02x", static_cast<int>(packet.left_motor_status));
    RCLCPP_INFO(this->get_logger(), "Right Motor Status: 0x%02x", static_cast<int>(packet.right_motor_status));

    RCLCPP_INFO(this->get_logger(), "Motor Mode: 0x%02x", static_cast<int>(packet.motor_mode));
    RCLCPP_INFO(this->get_logger(), "Left Motor Type: 0x%02x", static_cast<int>(packet.left_motor_type));
    RCLCPP_INFO(this->get_logger(), "Right Motor Type: 0x%02x", static_cast<int>(packet.right_motor_type));

    // Print error codes
    RCLCPP_INFO(this->get_logger(), "Error Code 1: 0x%02x", static_cast<int>(packet.error_code_1));
    RCLCPP_INFO(this->get_logger(), "Error Code 2: 0x%02x", static_cast<int>(packet.error_code_2));
    RCLCPP_INFO(this->get_logger(), "Error Code 3:: 0x%02x", static_cast<int>(packet.error_code_3));

    // Print IMU data
    RCLCPP_INFO(this->get_logger(), "IMU Index: %d", static_cast<int>(packet.imu_index));
    RCLCPP_INFO(this->get_logger(), "IMU Yaw: %d degrees", packet.imu_yaw);
    RCLCPP_INFO(this->get_logger(), "IMU Pitch: %d degrees", packet.imu_pitch);
    RCLCPP_INFO(this->get_logger(), "IMU Roll: %d degrees", packet.imu_roll);
    RCLCPP_INFO(this->get_logger(), "IMU X Acceleration: %d mg", packet.imu_x_acc);
    RCLCPP_INFO(this->get_logger(), "IMU Y Acceleration: %d mg", packet.imu_y_acc);
    RCLCPP_INFO(this->get_logger(), "IMU Z Acceleration: %d mg", packet.imu_z_acc);

    // Print cell voltages
    RCLCPP_INFO(this->get_logger(), "Cell Voltage 1: %d mV", packet.cell_voltage1);
    RCLCPP_INFO(this->get_logger(), "Cell Voltage 2: %d mV", packet.cell_voltage2);
    RCLCPP_INFO(this->get_logger(), "Cell Voltage 3: %d mV", packet.cell_voltage3);
    RCLCPP_INFO(this->get_logger(), "Cell Voltage 4: %d mV", packet.cell_voltage4);
    RCLCPP_INFO(this->get_logger(), "Cell Voltage 5: %d mV", packet.cell_voltage5);

    // Print battery info
    RCLCPP_INFO(this->get_logger(), "Total Capacity: %d mAh", packet.total_capacity);
    RCLCPP_INFO(this->get_logger(), "Remaining Capacity: %d mAh", packet.remaining_capacity);
    RCLCPP_INFO(this->get_logger(), "Battery Manufacturer: 0x%02x", static_cast<int>(packet.battery_manufacturer));

    // Print charge status
    RCLCPP_INFO(this->get_logger(), "Charge Status: 0x%02x", static_cast<int>(packet.charge_status));

    RCLCPP_INFO(this->get_logger(), "IMU and Odometry Status: 0x%02x", static_cast<int>(packet.imu_odometry_status));
    RCLCPP_INFO(this->get_logger(), "Bottom Wheel Lifting Sensor: 0x%02x", static_cast<int>(packet.bottom_wheel_lifting_sensor));
    RCLCPP_INFO(this->get_logger(), "Dock Signal Status: 0x%02x", static_cast<int>(packet.dock_sig_status));
    RCLCPP_INFO(this->get_logger(), "Dock Short IR Position: 0x%02x", static_cast<int>(packet.dock_short_ir_position));
    RCLCPP_INFO(this->get_logger(), "Docking Status: 0x%02x", static_cast<int>(packet.docking_status));
    RCLCPP_INFO(this->get_logger(), "Dock Long IR Position: 0x%02x", static_cast<int>(packet.dock_long_ir_position));
    RCLCPP_INFO(this->get_logger(), "Battery Percentage: %d%%", packet.battery_percent);
    RCLCPP_INFO(this->get_logger(), "Battery Voltage: %d mV", packet.battery_voltage);
    RCLCPP_INFO(this->get_logger(), "Battery Current: %d mA", packet.battery_current);
    RCLCPP_INFO(this->get_logger(), "Battery Temperature 1: %d°C", packet.battery_temperature1);
    RCLCPP_INFO(this->get_logger(), "Battery Temperature 2: %d°C", packet.battery_temperature2);
    RCLCPP_INFO(this->get_logger(), "Design Capacity: %d mAh", packet.design_capacity);
    RCLCPP_INFO(this->get_logger(), "Number of Cycles: %d", packet.number_of_cycles);

    RCLCPP_INFO(this->get_logger(), "Top TOF: %d mm", packet.top_tof);  // Top TOF (mm)
    RCLCPP_INFO(this->get_logger(), "Top 1D TOF Status: 0x%02x", static_cast<int>(packet.top_1d_tof_status));  // Top 1D TOF Status

    // Print TOF values (arrays)
    for (int i = 0; i < 16; ++i)
    {
        RCLCPP_INFO(this->get_logger(), "Lower Left TOF[%d]: %d mm", i, packet.lower_left_tof[i]);
    }

    for (int i = 0; i < 16; ++i)
    {
        RCLCPP_INFO(this->get_logger(), "Lower Right TOF[%d]: %d mm", i, packet.lower_right_tof[i]);
    }
    RCLCPP_INFO(this->get_logger(), "Lower TOF Status: 0x%02x", static_cast<int>(packet.lower_tof_status));


    // Print additional fields as needed
    RCLCPP_INFO(this->get_logger(), "IMU Calibration Status: 0x%02x", static_cast<int>(packet.imu_calibration_status));
    RCLCPP_INFO(this->get_logger(), "Left Motor Encoder: %d", packet.left_motor_encoder);
    RCLCPP_INFO(this->get_logger(), "Right Motor Encoder: %d", packet.right_motor_encoder);

    // Motor current and RPM
    RCLCPP_INFO(this->get_logger(), "Left Motor Current: %d (10mA)", packet.left_motor_current);
    RCLCPP_INFO(this->get_logger(), "Right Motor Current: %d (10mA)", packet.right_motor_current);
    RCLCPP_INFO(this->get_logger(), "Left Motor RPM: %d", packet.left_motor_rpm);
    RCLCPP_INFO(this->get_logger(), "Right Motor RPM: %d", packet.right_motor_rpm);

    // Print cradle and firmware versions
    RCLCPP_INFO(this->get_logger(), "Cradle ADC0: 0x%02x", static_cast<int>(packet.cradle_adc0));
    RCLCPP_INFO(this->get_logger(), "Cradle ADC1: 0x%02x", static_cast<int>(packet.cradle_adc1));

    RCLCPP_INFO(this->get_logger(), "Cradle FW Major Version: 0x%02x", static_cast<int>(packet.cradle_fw_ver_msb));
    RCLCPP_INFO(this->get_logger(), "Cradle FW Minor Version: 0x%02x", static_cast<int>(packet.cradle_fw_ver_lsb));
    
    RCLCPP_INFO(this->get_logger(), "Charging Mode: 0x%02x", static_cast<int>(packet.charging_mode));
    
    // Print motor temperatures
    RCLCPP_INFO(this->get_logger(), "Left Motor Temperature: %d°C", packet.left_motor_temperature);
    RCLCPP_INFO(this->get_logger(), "Right Motor Temperature: %d°C", packet.right_motor_temperature);

    RCLCPP_INFO(this->get_logger(), "Firmware Major Version: 0x%02x", static_cast<int>(packet.firmware_ver_msb));
    RCLCPP_INFO(this->get_logger(), "Firmware Minor Version: 0x%02x", static_cast<int>(packet.firmware_ver_lsb));
 
    // Print checksum
    RCLCPP_INFO(this->get_logger(), "Checksum: 0x%02x", static_cast<int>(packet.checksum));
}

template <typename PacketType>
void UARTCommunication::setJigCalibrationMsg(const PacketType &packet)
{
    // std::lock_guard<std::mutex> lock(jig_imu_calibration_mutex_);
    jig_imu_calibration_msg.calibration_status = packet.imu_calibration_status;
    jig_imu_calibration_msg.roll = packet.imu_roll;
    jig_imu_calibration_msg.pitch = packet.imu_pitch;
    jig_imu_calibration_msg.yaw = packet.imu_yaw;
}

void UARTCommunication::parseDataFields(const std::vector<uint8_t> &data)
{
    if (data.size() < 185)
    {
        RCLCPP_ERROR(this->get_logger(), "Data size too small for parsing.");
        return;
    }

    CommandType command = static_cast<CommandType>(data[3]);
    //RCLCPP_INFO(this->get_logger(), "[command : %x]", command);

    switch (command)
    {
    case CommandType::RX_V1PROTOCOL_:
    {
        V1DataPacket packet;
        //RCLCPP_INFO(this->get_logger(), "[RX_V1PROTOCOL_]");

        ParseV1Data(data, packet);
        setImuMsg(packet);
        setOdomMsg(packet);
        setBatteryMsg(packet);
        setTofMsg(packet);
        setMotorMsg(packet);
        setDockingMsg(packet);
        setBottomMsg(packet);
        setOdomStatusMsg(packet);
        setFwVersionMsg(packet);
        setJigCalibrationMsg(packet);
        setMcuErrorMsg(packet);
        setCradleADCMsg(packet);
        // TODO Error 
        // 차주 FW 협의 필요
    }
    break;
    case CommandType::RX_V2PROTOCOL_:
    {
        V2DataPacket packet;
        //RCLCPP_INFO(this->get_logger(), "[RX_V2PROTOCOL_]");

        ParseV2Data(data, packet);
        setImuMsg(packet);
        setOdomMsg(packet);
        setBatteryMsg(packet);
        setTofMsg(packet);
        setMotorMsg(packet);
        setDockingMsg(packet);
        setBottomMsg(packet);
        setOdomStatusMsg(packet);
        setFwVersionMsg(packet);
        setJigCalibrationMsg(packet);
        setMcuErrorMsg(packet);
        setCradleADCMsg(packet);
        // setButtomIRMsg(packet);
        // TODO Error 
        // 차주 FW 협의 필요
    }
    break;
    
    default:
        RCLCPP_WARN(this->get_logger(), "Unknown command: 0x%02x", data[3]);
        break;
    }
}

void UARTCommunication::searchAndParseData(std::vector<uint8_t> &buffer)
{
    const std::array<uint8_t, 2> preamble = {0xAA, 0x55}; // 패킷의 프리엠블

    while (buffer.size() >= 5)
    {
        auto preamble_position = std::search(buffer.begin(), buffer.end(), preamble.begin(), preamble.end());

        if (preamble_position == buffer.end())
        {
            buffer.erase(buffer.begin());
            continue;
        }

        size_t preamble_index = std::distance(buffer.begin(), preamble_position);

        if (buffer.size() - preamble_index < 5)
        {
            RCLCPP_WARN(this->get_logger(), "Data size too small to contain all fields, waiting for more data.");
            return;
        }

        size_t data_size = static_cast<size_t>(buffer[preamble_index + 2]);  // 🔹 uint8_t → size_t 변환

        if (buffer.size() - preamble_index < data_size + 3)  // 🔹 size_t로 변환했으므로 안전한 비교
        {
            return;
        }

        uint8_t checksum = 0;
        for (size_t i = preamble_index + 3; i < preamble_index + 3 + data_size - 1; ++i)
        {
            checksum += buffer[i];
        }

        if (checksum == buffer[preamble_index + 3 + data_size - 1])
        {
            std::vector<uint8_t> packet_data(buffer.begin() + preamble_index, buffer.begin() + preamble_index + 3 + data_size);
            parseDataFields(packet_data);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Checksum mismatch, discarding packet.");
        }

        buffer.erase(buffer.begin(), buffer.begin() + preamble_index + 3 + data_size);  // 🔹 중복 코드 제거하여 하나의 `erase`로 통일
    }
}

void UARTCommunication::receiveData()
{
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_sec = now-received_time_;
    double milisec_interval = elapsed_sec.count()*1000.0;

    static size_t call_count = 0;
    static size_t delayed_count = 0;
    static size_t continuous_no_data_count = 0;

    //250526 KKS : prevent overflow at 24hour
    if(call_count > 86400000){
        RCLCPP_INFO(this->get_logger(), "[receiveData] reset check_cnt (%zu/%zu)", delayed_count, call_count);
        call_count = 0; 
        delayed_count = 0; 
        continuous_no_data_count = 0;
    }

    ++call_count;

    if (milisec_interval >= 100.0 && call_count > 0) {
        ++delayed_count;
        double delay_rate = static_cast<double>(delayed_count) / static_cast<double>(call_count) * 100.0;
        RCLCPP_INFO(this->get_logger(), "[receiveData] interval : %.3f ms",milisec_interval);
        RCLCPP_INFO(this->get_logger(),"[receiveData] Delay rate (>=100ms): %.2f%% (%zu/%zu)",delay_rate, delayed_count, call_count);
    }    

    static std::vector<uint8_t> buffer;
    const int num_bytes_to_read = 1024;
    std::vector<uint8_t> new_data(num_bytes_to_read);
    bool parsing_flag = false;
    size_t available_bytes = getUartAvailableData();

    if (available_bytes > 0)
    {
        // RCLCPP_INFO(this->get_logger(), "receive Data : ");
        size_t bytes_to_read = std::min(available_bytes, static_cast<size_t>(num_bytes_to_read));
        int bytes_read = getUartReadData(new_data.data(), bytes_to_read);

        if (bytes_read > 0)
        {
            new_data.resize(bytes_read);
            buffer.insert(buffer.end(), new_data.begin(), new_data.end());

            searchAndParseData(buffer);
            parsing_flag = true;
            if(continuous_no_data_count > 0){
                RCLCPP_INFO(this->get_logger(), "[receiveData] received ok interval:%.3fms, count:%zu", milisec_interval, continuous_no_data_count);
                continuous_no_data_count = 0;
            }      
        }
    }

    if(!parsing_flag && milisec_interval > 10.0){
        continuous_no_data_count++;
        if(continuous_no_data_count < 100){
            RCLCPP_INFO(this->get_logger(), "[receiveData] No data received or read error interval:%.3fms, count:%zu", milisec_interval, continuous_no_data_count);
        }else if(continuous_no_data_count == 100){
            RCLCPP_INFO(this->get_logger(), "[receiveData] No data received or read error interval:%.3fms, count:%zu, no longer output", milisec_interval, continuous_no_data_count);
        }
    }
	received_time_ = now;
    
    pubTopic();
}

std::vector<double> UARTCommunication::computeAngularVelocity(double roll, double pitch, double roll_dot, double pitch_dot, double yaw_dot)
{
    std::vector<double> angular_velocity(3);
    angular_velocity[0] = roll_dot + yaw_dot * std::sin(pitch);
    angular_velocity[1] = pitch_dot * std::cos(roll) - yaw_dot * std::sin(roll) * std::cos(pitch);
    angular_velocity[2] = pitch_dot * std::sin(roll) + yaw_dot * std::cos(roll) * std::cos(pitch);
    return angular_velocity;
}

void UARTCommunication::batterySleepCallback(const std_msgs::msg::Empty::SharedPtr)
{
    // 충전 단자 연결이 확인될시, Ship 모드 명령 무시
    if(station_msg.docking_status & 0x10)
    {
        RCLCPP_INFO(this->get_logger(), "BATTERY-SLEEP CMD Ignore : Docking Connect True");
        return;
    }
    setBattMode(ToFnBatt::BATT_SLEEP);
    RCLCPP_ERROR(this->get_logger(), "BATTERY-SLEEP"); 
}

// void UARTCommunication::motorModeCallback(const std_msgs::msg::UInt8::SharedPtr msg)
// {
//     uint8_t mode = getUpperBits(msg->data);
//     MotornRemoteMode mode_enum = static_cast<MotornRemoteMode>(mode);
//     RCLCPP_ERROR(this->get_logger(), "[motorModeCallback] msg : 0x%02x, mode(upper->low) : 0x%2x", msg->data, mode);
//     setMotorMode(mode_enum);
// }

void UARTCommunication::remoteBlockCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg) { // msg 가 nullptr 일 경우 죽을 수 있음
        RCLCPP_WARN(this->get_logger(), "[remoteBlockCallback] Received nullptr");
        return;
    }

    MotornRemoteMode mode = msg->data ? MotornRemoteMode::BLOCK_REMOTE : MotornRemoteMode::UNBLOCK_REMOTE;
    RCLCPP_ERROR(this->get_logger(), "[remoteBlockCallback] msg : %d, mode : %u", msg->data,static_cast<uint8_t>(mode));
    setRemoteMode(mode);
}

void UARTCommunication::tofCommandCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg) { // msg 가 nullptr 일 경우 죽을 수 있음
        RCLCPP_WARN(this->get_logger(), "[tofCommandCallback] Received nullptr");
        return;
    }

    ToFnBatt status = msg->data ? ToFnBatt::ON : ToFnBatt::OFF;
    setToF(status);
}

void UARTCommunication::dockingCommandCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    if (!msg) { // msg 가 nullptr 일 경우 죽을 수 있음
        RCLCPP_WARN(this->get_logger(), "[dockingCommandCallback] Received nullptr");
        return;
    }

    setDocking(static_cast<DockingChargeCommand>(msg->data)); 
}

void UARTCommunication::e_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg) { // msg 가 nullptr 일 경우 죽을 수 있음
        RCLCPP_WARN(this->get_logger(), "[e_stop_callback] Received nullptr");
        return;
    }

    if(msg->data){
        bEmergencyStop = true;
        //setMotorMode(MotornRemoteMode::BRAKE_MODE);
    }else{
        bEmergencyStop = false;
        //setMotorMode(MotornRemoteMode::VW_MODE);
    }
}

void UARTCommunication::charge_cmd_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    if (!msg) { // msg 가 nullptr 일 경우 죽을 수 있음
        RCLCPP_WARN(this->get_logger(), "[charge_cmd_callback] Received nullptr");
        return;
    }

    setCharge(static_cast<DockingChargeCommand>(msg->data)); 

}

void UARTCommunication::odom_imu_reset_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    if (!msg) { // msg 가 nullptr 일 경우 죽을 수 있음
        RCLCPP_WARN(this->get_logger(), "[odom_imu_reset_callback] Received nullptr");
        return;
    }

    ResetCommand reset_cmd = static_cast<ResetCommand>(msg->data);
    setReset(reset_cmd);
}

void UARTCommunication::onFWResetReceived(const std_msgs::msg::Empty::SharedPtr)
{
    setReset(ResetCommand::MCU_RESET);
}

void UARTCommunication::amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    if (!msg) { // msg 가 nullptr 일 경우 죽을 수 있음
        RCLCPP_WARN(this->get_logger(), "[amcl_pose_callback] Received nullptr");
        return;
    }

    // std::lock_guard<std::mutex> lock(amcl_pose_);
    const auto &pose = msg->pose.pose;
    amcl_pose_x_ = pose.position.x;
    amcl_pose_y_ = pose.position.y;
    amcl_pose_angle_ = quaternion_to_euler(pose.orientation);
}

double UARTCommunication::quaternion_to_euler(const geometry_msgs::msg::Quaternion &quat)
{
    tf2::Quaternion q;
    tf2::fromMsg(quat, q);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw; // Return yaw as theta
}

size_t UARTCommunication::getUartAvailableData()
{
    size_t ret = 0;
    try
    {
        ret = uart_.available();
    }
    catch (const serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[getUartAvailableData] serial::IOEXCEPTION available error : %s", e.what());
    }
    catch (const serial::SerialException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[getUartAvailableData] serial::SerialException available error : %s", e.what());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "[getUartAvailableData] std::exception uart available error : %s", e.what());
    }
    catch (...)
    {
        RCLCPP_ERROR(this->get_logger(), "[getUartAvailableData] Unknown error in uart available");
    }

    return ret;
    
}

int UARTCommunication::getUartReadData(unsigned char* buffer, size_t read_size)
{
    int ret = 0;
    try
    {
        ret = uart_.read(buffer,read_size);
    }
    catch (const serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[getUartReadData] serial::IOEXCEPTION read error : %s", e.what());
    }
    catch (const serial::SerialException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[getUartReadData] serial::SerialException read error : %s", e.what());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "[getUartReadData] std::exception uart read error : %s", e.what());
    }
    catch (...)
    {
        RCLCPP_ERROR(this->get_logger(), "[getUartReadData] Unknown error in uart read");
    }
    
    return ret;
}

bool UARTCommunication::isEqualbyEpsilon(double a, double b, double epsilon) {
    return std::fabs(a - b) < epsilon;
}

#if PREVENT_SAME_CMD_VEL > 0
void UARTCommunication::movingStateCallback(const everybot_custom_msgs::msg::NaviState::SharedPtr msg)
{
    if(!msg) { // msg 가 nullptr 일 경우 죽을 수 있음
        RCLCPP_WARN(this->get_logger(), "[movingStateCallback] Received nullptr");
        return;
    }
    NAVI_STATE covert_state_from_msg = static_cast<NAVI_STATE>(msg->state);
    
    if(navi_state != covert_state_from_msg){
        RCLCPP_INFO(this->get_logger(), "[movingStateCallback] state changed from %s to %s", enumToString(navi_state).c_str(), enumToString(covert_state_from_msg).c_str());
    }

    navi_state = covert_state_from_msg;
}
#endif

bool UARTCommunication::preventUnintendAccelationChecker(double current_linear_velocity, double current_angular_velocity, bool preventState)
{
    static uint8_t same_cmd_vel_count_ = 0;
    static double prev_linear_velocity = 0.0, prev_angular_velocity = 0.0;
    static double delayTimeLimitms = 1000.0; //1sec

    if(fabs(current_linear_velocity) <= 0.001){
        same_cmd_vel_count_ = 0;
        prev_linear_velocity = current_linear_velocity;
        prev_angular_velocity = current_angular_velocity;
        return false;
    } 

    bool bCmdVelSameValue = false, bCmdVelDelayed = false;
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> diff = now - last_cmd_vel_time_;
    double callback_deay_ms = diff.count();
    
    #if PREVENT_SAME_CMD_VEL > 0
    NAVI_STATE current_navi_state = static_cast<NAVI_STATE>(navi_state);
    //실제 로봇이 움직일수 있는 속도는 linear 약0.01(m/sec) 이상 / angular : 0.05(rad/sec) 2.86(deg/sec)
    //hjkim : 급발진 방어코드 cmd_vel_callback이 계속해서 같은 값으로 내려올 때, 비정상 명령으로 판단하여 정지시킴.
    if(current_navi_state == NAVI_STATE::MOVE_GOAL && isEqualbyEpsilon(current_linear_velocity, prev_linear_velocity, 0.0001) && isEqualbyEpsilon(current_angular_velocity, prev_angular_velocity, 0.0001))
    {
        if(same_cmd_vel_count_ >= 50){
            bCmdVelSameValue = true;
        }else{
            same_cmd_vel_count_++;
        }
    }else{
        same_cmd_vel_count_ = 0;
    }
    #endif

    if(callback_deay_ms > delayTimeLimitms){
        bCmdVelDelayed = true;
    }

    prev_linear_velocity = current_linear_velocity;
    prev_angular_velocity = current_angular_velocity;

    if(bCmdVelDelayed || bCmdVelSameValue){
        if(!preventState){
            RCLCPP_WARN(this->get_logger(), "[preventUnintendAccelationChecker] velocity[%s], callback[%s] time[%.3f] clear velocity(%.3f, %.3f)",
            bCmdVelSameValue?" same value ":" changed",bCmdVelDelayed?" delayed": " ok",callback_deay_ms,current_linear_velocity,current_angular_velocity);
        }
        return true;
    }
    return false;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<UARTCommunication>());
    } catch (...) {
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();

    return 0;
}
