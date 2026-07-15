#ifndef LIBNETWOKR_H
#define LIBNETWOKR_H

#include <vector>
#include <string>

#define BASE_VERSION "0.89.p"

#ifdef LOG_SETTING
    #if DEBUG_MODE
        #define VERSION BASE_VERSION ".2.1"
    #else
        #define VERSION BASE_VERSION ".2.0"
    #endif
#elif defined(OTA_SETTING)
    #if DEBUG_MODE
        #define VERSION BASE_VERSION ".1.1"
    #else
        #define VERSION BASE_VERSION ".1.0"
    #endif
#else // NORMAL 모드
    #if DEBUG_MODE
        #define VERSION BASE_VERSION ".0.1"
    #else
        #define VERSION BASE_VERSION ".0.0"
    #endif
#endif

#define getPositionIdx 1 
#define getLidarSensorStatusIdx 2
#define getMotorStatusIdx 3
#define getCameraStatusIdx 4
#define getLineLaserStatusIdx 5
#define getTofStatusIdx 6
#define getIRStatusIdx 7
#define getSonicStatusIdx 8
#define getBatteryStatusIdx 9
#define getSoftwareVersionIdx 10
#define getRobotInfoIdx 11
#define getErrorListIdx 12
#define getTargetPositionIdx 13
#define getMapStatusIdx 14
#define getMapDataIdx 15
#define getLidarDataIdx 16
#define getModifiedMapDataIdx 17 
#define getXXXStatusIdx 18
#define getRobotSpeedIdx 19
#define getTargetPositionCalculateIdx 20

#define getRobotStatusIdx 21
#define getActionStatusIdx 22
#define getNotificationIdx 23

#define getAICalibrationIdx 25
#define getLogDataIdx 26

#define getDockingStatusIdx 30 

#define getMotorStatusV2Idx 31
#define getRecvIRStatusIdx 32
#define getCliffIRStatusIdx 33
#define getCameraStatusV2Idx 34
#define getLineLaserStatusV2Idx 35
#define getTofStatusV2Idx 36
#define getBatteryStatusV2Idx 37
#define getTemperatureIdx 38
#define getFollowMeStatusIdx 39

#if false
#define getIntegrationMapDataIdx 31
#define getIntegrationModifiedMapDataIdx 32 
#endif

#define getAllStatusIdx 40
#define getAllMovingInfoIdx 41
#define getAllStatusV2Idx 42
#define getAllMovingInfoV2Idx 43
#define getEncMapDataIdx 44
#define getZipMapDataIdx 45

#define getTofCalibrationIdx 46
#define getFactoryDataIdx 47
#define getZipSenSorDataIdx 48

#define setReturnToChargingStationIdx 50
#define setEmergencyStopIdx 51 
#define setStartLidarIdx 52 
#define setStopLidarIdx 53
#define setStartChargingIdx 54
#define setStopChargingIdx 55
#define setSoftwareResetIdx 56
#define setMaxDrivingSpeedIdx 57
#define setMotorManual_RPMIdx 58
#define setMotorManual_VWIdx 59
#define setTargetPositionIdx 60
#define setDrivingIdx 61
#define setMappingIdx 62
#define setModifiedMapDataIdx 63
#define setHeartbeatIdx 64
#define setExcelStepsIdx 65
#define setRotationIdx 66
#define setTimeDataIdx 67
#define setMapCopyIdx 68

#define setIntegrationModifiedMapDataIdx 70
#define setByPassOneDataIdx 71
#define setByPassTwoDataIdx 72
#define setPGMMapDataIdx 73

#define setFactoryModeIdx 74
#define setBatterySleepModeIdx 75
#define setSensorInspectionModeIdx 76
#define setStationRepositioningIdx 77
#define setSelfDiagnosisMotorIdx 78
#define setFollowMeIdx 79
#define setInitMapIdx 86
#define setSaveMapIdx 87
#define setRecoveryReturnToChargingStationIdx 88

#define setOTAIdx 80
#define setOTADataIdx 81

#define setFactoryResetIdx 83
#define setRecoveryIdx 84
#define setOTAsuccessIdx 85

#define setDockingStatusIdx 90 

// Everybot First
#define emergencyMSGFanCmdAndTempsIdx 99
#define emergencyMSGERRORIdx 100
#define emergencyMSGOTAIdx 101

//
#define setSettingCommandIdx 251
#define setConfigCommandIdx 252

#define objectReq   0x01
#define objectRes   0x02
#define objectEmergency   0x0F
#define objectJig  0xC8
#define objectOTAData  0x0E
/******************************************** */

// #1
struct Positions_t {
    double x;
    double y;
    double theta;
};

// #2
struct LidarSensorStatus_t {
    int front;
    int rear;
};

// #3
struct MotorStatus_t {
    double leftSpeed;
    double  rightSpeed;
    double leftPower;
    double rightPower;
    int leftStatus;
    int rightStatus;
};

struct MotorStatusV2_t {
    double leftRPM;
    double  rightRPM;
    double leftCurrent;
    double rightCurrent;
    int leftType;
    int rightType;
};

// #4
struct CameraStatus_t {
    int status;
};

struct FollowMeStatus_t {
    int status;
};

// #5
struct LineLaserStatus_t {
    int left;
    int right;
};

// #6
struct TofStatus_t {
    int leftStatus;
    double leftDistance;
    int rightStatus; 
    double rightDistance;
    int topStatus;
    double topDistance;
};

struct TofStatusV2_t {
    int leftStatus;
    int rightStatus; 
    int topStatus;
};

// #7
struct IRStatus_t {
    bool sensor1;
    bool sensor2;
    bool sensor3;
    bool sensor4;
    bool sensor5;
    bool sensor6;
    bool stationIR1;
    bool stationIR2;
    bool stationIR3;
    bool stationIR4;
};

struct RecvIRStatus_t {
    bool RecvIR1;
    bool RecvIR2;
    bool RecvIR3;
    bool RecvIR4;
};

struct CliffIRStatus_t {
    bool CliffIR1;
    bool CliffIR2;
    bool CliffIR3;
    bool CliffIR4;
    bool CliffIR5;
    bool CliffIR6;
};

// #8
struct SonicStatus_t {
    int leftStatus;
    bool LeftSenseing;
    int rightStatus;
    bool rightSenseing;
};

// #9
struct BatteryStatus_t {
    double  voltage;
    double  current;
    double  temperature;
    double  chargeState;
    double  capacity;
    double  designCapacity;
    double  percentage;
    double  supplyStatus;
    double  supplyHealth;
    double  supplyTechnology;
    double  present;
    double  cellVoltage;
    double  cellTemperature;
    double  location;
    double  serialNumber;
};

struct BatteryStatusV2_t {
    int cell_voltage1;
    int cell_voltage2;
    int cell_voltage3;
    int cell_voltage4;
    int cell_voltage5;
    int total_capacity;
    int remaining_capacity;
    int battery_manufacturer;
    int battery_percent;
    double battery_voltage;
    double battery_current;
    int battery_temperature1;
    int battery_temperature2;
    int design_capacity;
    int number_of_cycles;
    int charge_status;
};

struct Temperature_t {
    double npu;
    double gpu;
    double center;
    double soc;
    double bigcore0;
    double bigcore1;
    double littlecore;
};


// #10
struct SoftwareVersion_t {
    std::string version;
};

// #11
struct RobotInfo_t {
    std::string serialNumber;
    int status;
};

// #12
struct ErrorList_t {
    int resolved;
    int rank;
    std::string errorCode;
};

// #13
struct TargetPosition_t {
    double x;
    double y;
    double theta;
};

struct TargetPositionType_t {
    double x;
    double y;
    double theta;
    int type;
};

struct StationRepositioning_t
{
    double x;
    double y;
    double theta;
};

struct SelfDiagnosisMotor_t
{
    bool enable;
    double mS;
    double Distance;
};

struct AICalibration_t {
    int result;
};

struct FollowMe_t {
    bool enable;
};


// #14 -
struct MapStatus_t {
    double Temp;
};

// #15
struct MapData_t {
    double width;
    double height;
    double resolution;
    double posX;
    double posY;
    std::string data;
};

struct IntegrationMapData_t {
    std::string data;
};
// #16
struct LidarData_t {
    double front;
    double rear;
};

// #17
struct ModifiedMapData_t {
    double width;
    double height;
    std::string	data;
};

struct ModifiedMapDataB_t {
    double width;
    double height;
    std::vector<unsigned char> data;
};

struct PGMMapData_t {
    std::string	 name;
    std::string	 sha256;
    std::string  data;
};

struct PGMMapDataB_t {
    std::string	 name;
    bool	      sha256;
    std::vector<unsigned char> data;
};

struct MapCopy_t {
    std::string	 sha256;
    std::string  data;
};

struct MapCopyB_t {
    bool	      sha256;
    std::vector<unsigned char> data;
};

struct RobotSpeed_t {
    double mS;
    double radS;
};

struct RobotStatus_t {
    int status;
};

struct ActionStatus_t {
    int status;
};

struct Notification_t {
    int code;
    std::string	description;
};

struct IntegrationModifiedMapData_t {
    std::string	data;
};

struct LogData_t {
    std::string	data;
};

struct ByPassData_t {
    std::string	data;
};

struct Position {
    double x;
    double y;
};

struct ChargingPosition {
    double x;
    double y;
    double theta = 0.0; // theta도 존재함
};

struct Room {
    std::string id;
    std::string name;
    std::string color;
    std::string desc;
    std::vector<Position> robot_path;
    std::vector<Position> image_path;
};

struct BlockArea {
    std::string id;
    std::vector<Position> robot_path;
    std::vector<Position> image_path;
};

struct ChargingStation {
    // std::string id;
    bool has_data = false;  // 데이터 유무 플래그
    ChargingPosition robot_position;
    ChargingPosition image_position;
};

struct InitPosition {
    std::string id;
    Position robot_position;
    Position image_position;
};

struct ByPassOne_t {
    std::string uid;
    std::string version;
    std::string modified;
    std::vector<Room> room_list;
    std::vector<BlockArea> block_area;
    std::vector<BlockArea> block_wall;
    ChargingStation charging_station;
    std::vector<InitPosition> init_position;
};

struct JsonData_t {
    std::string uid;
    std::string version;
    std::string modified;
    std::vector<Room> room_list;
    std::vector<BlockArea> block_area;
    std::vector<BlockArea> block_wall;
    ChargingStation charging_station;
    std::vector<InitPosition> init_position;
};


struct IntegrationModifiedMapDataB_t {
    std::vector<unsigned char>	data;
};

struct AllStatus_t {
    std::string	data;
};

struct AllStatusV2_t {
    std::string	data;
};

struct AllMovingInfo_t {
    std::string	data;
};

struct AllMovingInfoV2_t {
    std::string	data;
};

// #Debug Docking
struct DockingStatus_t {
    int dock;
};


/******************************************** */


// #50
struct ReturnToChargingStation_t {
    bool Return;
};

// #51
struct EmergencyStop_t {
    bool Return;
};

// #52
struct StartLidar_t {
    bool Return;

};

// #53
struct StopLidar_t {
    bool Return;

};

// #54
struct StartCharging_t {
    bool Return;
};

// #55  
struct StopCharging_t {
    bool Return;

};

// #56
struct SoftwareReset_t {
    bool Return;

};

struct FactoryMode_t {
    bool start;
};

struct BatterySleepMode_t {
    bool start;
};

struct InspectionMode_t {
    bool start;
};

struct FactoryActive_t {
    bool active;
};

// #57
struct MaxDrivingSpeed_t {
    double mS;

};

// #58
struct MotorManual_RPM_t {
    int leftRPM;
    int rightRPM;
};

// # 59
struct MotorManual_VW_t {
    double mS;
    double radS;
};

// # 61
struct Driving_t {
    int set;

};

// # 62
struct Mapping_t {
    int set;
};

#if 0
struct ModifiedMapData_t {
    double x;
    double y;
    double theta;
};
#endif

// # 63
struct heartbeat_t {
    bool Return;
};

struct ExcelSteps_t {
    int set;
};

struct Rotation_t {
    int type;
    double radian;
};

struct TimeData_t {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
};

struct Status_t {
    int status;
};

struct ota_t {
    std::string	checksum;
    int size;
};

struct otadata_t {
    std::string	checksum;
    std::string	data;
};

struct otadataB_t {
    std::string	checksum;
    std::vector<unsigned char>	data;
     std::string	filename;
};

struct SettingCommand_t {
    int set;
};

struct ConfigCommand_t {
    int set;
};

struct version_t {
    std::string	version;
};

struct ObjectDataV2
{
    uint8_t class_id;      // 객체의 클래스 ID
    uint8_t confidence;    // 신뢰도 (0~100)
    int16_t x;             // X 좌표
    int16_t y;             // Y 좌표
    int16_t theta;         // 각도 (Theta)
    int16_t width;         // 너비 (Width)
    int16_t height;        // 높이 (Height)
    uint16_t distance;     // 거리 (Distance)
};

struct LLDataV2
{
    int16_t x;
    int16_t y;
    int16_t theta;
    // int16_t width;
    int8_t direction; // Added field for Direction
    // int8_t reserved;  // Added field for Reserved
    int16_t height;
    uint16_t distance;
};

struct KeyFileStatus {
    bool kek_exists;
    bool dek_enc_exists;
    bool dek_tag_exists;
    bool dek_iv_exists;
};

enum OTASDatastep {
    DStep0 = 0, // 초기 단계
    DStep1 = 1, // 단계 1
    DStep2 = 2, // 단계 2
    DStepP = 0xA, // 단계 Pass
    DStepF0 = 0xF0,  // 단계 Fail
    DStepF1 = 0xF1,  // 단계 Fail
    DStepF2 = 0xF2  // 단계 Fail
};

enum class ServerState {
    waiting,
    connected,
};

enum class OTADownloadState {
    ready,
    downloading,
    done,
    fail,
};

struct OrderStats {
    int min;
    int max;
    int median;
};

struct Metric {
    std::string name; // "min", "minRef", "max", "maxRef", "median", ...
    double value;
};

struct Aggregate {
    int tofIndex;                   // 예: 14, 7
    std::vector<Metric> metrics;    // ["min", 12.3], ["minRef", 12.3], ...
};

struct SideData {
    std::vector<std::pair<int,double>> raw; // [index, value] 쌍들
    std::vector<Aggregate> aggregates;      // [tofIndex, [[name,value],...]] 튜플들
    int result = 0;                          //  1=Pass 10 =Fail,
};

#ifdef __cplusplus
extern "C" {
#endif

// 현재 LibNetwork 버전 정보를 가져오는 API
// - data: 버전 정보가 저장될 문자열 참조
void APIGetVersion(std::string &data);

// Heartbeat용 전체 버전 문자열을 반환하는 API
// - return: 설정된 전체 버전 문자열
std::string APIGetTotalVersion();
// Heartbeat용 전체 버전 문자열을 설정하는 API
// - data: 저장할 전체 버전 문자열
void APISetTotalVersion(std::string data);

// 통신 암호화 사용 여부를 설정하는 API
// - enable: true 시 암호화 사용, false 시 평문 사용
void APISetEnc(bool enable);

// 입력 파일을 AES-128-CBC 방식으로 암호화하여 출력 파일로 저장
// - inputPath: 원본 파일 경로
// - outputPath: 암호화 파일 저장 경로
// - removeOriginal: true 시 암호화 후 원본 파일 삭제
// - return: 성공 시 true, 실패 시 false
bool APIencryptToFile(const std::string& inputPath, const std::string& outputPath, bool removeOriginal = true);
// 암호화된 파일을 복호화하여 출력 파일로 저장
// - inputPath: 암호화 파일 경로
// - outputPath: 복호화된 파일 저장 경로
// - removeEncrypted: true 시 복호화 후 암호화 파일 삭제
// - return: 성공 시 true, 실패 시 false
bool APIdecryptToFile(const std::string& inputPath, const std::string& outputPath, bool removeEncrypted = true);
// PGM 이미지 데이터를 저장하면서 동시에 암호화
// - dir: 저장할 디렉토리 경로
// - pgmName: 저장할 PGM 파일명
// - map_data: PGM 픽셀 데이터
// - width, height: 이미지 크기
// - removePlain: true 시 평문 파일 삭제
// - return: 성공 시 true, 실패 시 false
bool APIsaveAndEncryptPGM(const std::string& dir, const std::string& pgmName,const unsigned char* map_data, int width, int height,bool removePlain = true);
// YAML 메타데이터를 저장하면서 동시에 암호화
// - dir: 저장할 디렉토리 경로
// - yamlName: 저장할 YAML 파일명
// - image: 참조할 이미지 파일명 (ex: map.pgm)
// - resolution: 지도 해상도 (m/pixel)
// - origin_x, origin_y, origin_theta: 지도 원점 정보
// - negate, occupied_thresh, free_thresh: 지도 파라미터
// - removePlain: true 시 평문 파일 삭제
// - return: 성공 시 true, 실패 시 false
bool APIsaveAndEncryptYAML(const std::string& dir, const std::string& yamlName, const std::string& image,double resolution, double origin_x, double origin_y, double origin_theta,int negate, double occupied_thresh, double free_thresh, bool removePlain = true);
// 암호화된 파일을 메모리 상에서 복호화하여 반환
// - inputPath: 암호화된 파일 경로
// - return: 복호화된 데이터 (PGM 또는 기타 바이너리), 실패 시 빈 벡터
std::vector<unsigned char>  APIdecryptToMemory(const std::string& inputPath);

bool APIsaveAndPGM(const std::string &dir, const std::string &pgmName,
                   const unsigned char *map_data, int width, int height);
bool APIsaveAndYAML(const std::string &dir, const std::string &yamlName, 
                    const std::string &image, double resolution, double origin_x, double origin_y, double origin_theta, int negate, double occupied_thresh, double free_thresh);
bool API_init_keys_with_DEK();
bool API_encrypt_file_with_DEK(const std::string &input_file,
                               const std::string &output_file,
                               bool deleteOriginal = false);
bool API_decrypt_file_with_DEK(const std::string &input_file,
                               const std::string &output_file,
                               bool deleteOriginal = false);
std::vector<uint8_t> API_decrypt_file_to_memory_with_DEK(const std::string &encrypted_file);
bool API_check_key_files_status();
std::vector<uint8_t> API_read_kek();
std::vector<uint8_t> API_read_kek_key();
std::vector<uint8_t> API_read_dek_enc();
std::vector<uint8_t> API_read_dek_tag();
std::vector<uint8_t> API_read_dek_iv();
std::vector<uint8_t> API_read_plain_dek();
bool API_reset_keys_with_DEK();

void start_server();
void stop_server();
ServerState getServerState();
OTADownloadState getDownloadState();

/*********************************************************************/
bool reqGetPosition(void);
bool reqGetLidarSensorStatus(void);
bool reqGetMotorStatus(void);
bool reqGetMotorStatusV2(void);
bool reqGetCameraStatus(void);
bool reqGetFollowMeStatus(void);
bool reqGetCameraStatusV2(void);
bool reqGetLineLaserStatus(void);
bool reqGetLineLaserStatusV2(void);
bool reqGetTofStatus(void);
bool reqGetTofStatusV2(void);
bool reqGetToFCalibrationData(void);
bool reqGetFactoryData(FactoryActive_t &active);
bool reqGetIRStatus(void);
bool reqGetRecvIRStatus(void);
bool reqGetCliffIRStatus(void);

bool reqGetSonicStatus(void);
bool reqGetBatteryStatus(void);
bool reqGetBatteryStatusV2(void);
bool reqGetTemperature(void);
bool reqGetSoftwareVersion(void);
bool reqGetRobotInfo(void);
bool reqGetErrorList(void);
bool reqGetTargetPosition(void);
bool reqGetMapStatus(void);
bool reqGetMapData(void);
bool reqGetEncMapData(void);
bool reqGetZipMapData(void);
bool reqGetZipSensorData(void);
bool reqGetLidarData(void);
bool reqGetModifiedMapData(void);
bool reqGetAllStatus(void);
bool reqGetAllMovingInfo(void);
bool reqGetAllStatusV2(void);
bool reqGetAllMovingInfoV2(void);
bool reqGetDockingStatus(void);
bool reqGetRobotSpeed(void);
bool reqGetRobotStatus(void);
bool reqGetActionStatus(void);
bool reqGetNotification(void);

bool reqGetTargetPositionCalculate(TargetPosition_t &settings);
bool reqGetAICalibration(AICalibration_t &settings);

bool reqGetLogData(void);

// void resGetPosition(double x, double y, double theta);
void resGetjson2(const std::string& mode, const std::string& waterLevel, const std::string& soundVolume, bool tumbleDryerFlag, int dryLevel, const std::string& dryPower, const std::string& language, const std::string& country, const std::string& utcOffset);
void resGetjson_with_base64(const std::string& base64Data);

void resGetPosition(double x, double y, double theta);
void resGetLidarSensorStatus(int front, int rear);
void resGetMotorStatus(double MotorStatusleftSpeed, double MotorStatusrightSpeed,double MotorStatusleftPower, double MotorStatusrightPower,int MotorStatusleftStatus, int MotorStatusrightStatus);
void resGetMotorStatusV2(double MotorStatusleftRPM, double MotorStatusrightRPM,double MotorStatusleftCurrent, double MotorStatusrightCurrent,int MotorStatusleftType, int MotorStatusrightType);

void resGetCameraStatus(int status);
void resGetFollowMeStatus(int status);
// void resGetCameraStatusV2(int status, const std::vector<ObjectDataV2> &objects);
void resGetCameraStatusV2(int status, int type, const std::vector<ObjectDataV2> &objects);

void resGetLineLaserStatus(int left, int right);
void resGetLineLaserStatusV2(int status, const std::vector<LLDataV2> &llDataList);
void resGetTofStatus(int leftStatus, double leftDistance, int rightStatus, double rightDistance, int topStatus, double topDistance);

#if true
void resGetTofStatusV2(int leftStatus, const std::vector<int> &leftDistances,
                       int rightStatus, const std::vector<int> &rightDistances,
                       int topStatus, int topDistance);
#else
void resGetTofStatusV2(int leftStatus, const std::vector<int> &leftDistances, const std::vector<double> &leftCalibrationData,
                       int rightStatus, const std::vector<int> &rightDistances, const std::vector<double> &rightCalibrationData,
                       int topStatus, int topDistance);
#endif

#if true
void resGetToFCalibrationData(const SideData &left, const SideData &right);
#else
void resGetToFCalibrationData(const std::vector<std::pair<int, float>> &leftData,
                              const std::vector<std::pair<int, float>> &rightData);
#endif
void resGetFactoryData(const std::vector<OrderStats> &adc,
                       const OrderStats &oneD,
                       const OrderStats &lidarFront,
                       const OrderStats &lidarRear,
                       const std::vector<OrderStats> &multiTofLeft,
                       const std::vector<OrderStats> &multiTofRight);
void resGetFactoryAck(bool Result);

void resGetIRStatus(bool sensor1, bool sensor2, bool sensor3, bool sensor4, bool sensor5, bool sensor6, bool stationIR1, bool stationIR2, bool stationIR3, bool stationIR4);

void resGetRecvIRStatus(bool RecvIR1, bool RecvIR2, bool RecvIR3, bool RecvIR4);
void resGetCliffIRStatus(int CliffIR1, int CliffIR2, int CliffIR3, int CliffIR4, int CliffIR5, int CliffIR6);

void resGetSonicStatus(int leftStatus, bool LeftSenseing, int rightStatus, bool rightSenseing);
void resGetBatteryStatus(double voltage, double current, double temperature, double chargeState, double capacity, double designCapacity, double percentage, double supplyStatus, double supplyHealth, double supplyTechnology, double present, double cellVoltagem, double cellTemperature, double location, const std::string &serialNumber);
void resGetBatteryStatusV2(int cell_voltage1, int cell_voltage2, int cell_voltage3,
                           int cell_voltage4, int cell_voltage5, int total_capacity,
                           int remaining_capacity, int battery_manufacturer, int battery_percent,
                           double battery_voltage, double battery_current, int battery_temperature1,
                           int battery_temperature2, int design_capacity, int number_of_cycles, int charge_status);
void resGetTemperature(double SWnpu, double SWgpu, double SWcenter, double SWsoc, double SWbigcore0, double SWbigcore1, double SWlittlecore,
                       double AInpu, double AIgpu, double AIcenter, double AIsoc, double AIbigcore0, double AIbigcore1, double AIlittlecore,
                       int leftTerminal, int RightTerminal);
void resGetSoftwareVersion(const std::string &version);
void resGetRobotInfo(const std::string &serialNumber, int status);
void resGetErrorList(const std::vector<ErrorList_t> &ErrorList);
void resGetTargetPosition(double x, double y, double theta);
void resGetMapStatus(bool mapping, const std::string &data);

void resGetMapDataS(double width, double height, double resolution, double posX, double posY, const std::string &data);
void resGetMapDataB(double width, double height, double resolution, double posX, double posY, const unsigned char *data, size_t length);
void resGetLidarData(double front, double rear);
void resGetNotification(int code, std::string Notification);

// 암호화된 YAML 및 PGM 파일을 압축하고 Base64 인코딩 후 전송합니다.
// - dirPath: YAML 및 PGM 암호화 파일이 있는 디렉토리 경로
// - yamlFileName: YAML 파일 이름
// - pgmFileName:  PGM 파일 이름
// - return: 성공 시 true, 실패 시 false
bool resGetEncryptedMapData(const std::string &dirPath, const std::string &yamlFileName, const std::string &pgmFileName);
bool resGetOriginMapData(const std::string &dirPath,
                         const std::string &yamlFileName,
                         const std::string &pgmFileName);
bool resGetZipMapData(const std::string &dirPath,
                      const std::string &yamlFileName,
                      const std::string &pgmFileName,
                      bool deleteAfterSend = false);
bool resDecryptAndSendZipMapData(const std::string &dirPath,
                      const std::string &yamlFileName,
                      const std::string &pgmFileName,
                      bool deleteAfterSend = false);
bool resGetZipSensorData(const std::string &folder);
void resGetModifiedMapData(const std::string &data);
void resGetModifiedMapDataB(double width, double height, const unsigned char *data, size_t length);
void resGetRobotSpeed(double mS, double radS);
void resGetTargetPositionCalculate(double Time, double Distance, double x, double y, double theta);
void resGetRobotStatus(int status);
void resGetActionStatus(int status);

void resGetAICalibration(int result);

bool resGetLogData(const std::string &folderPath);

void resGetAllStatus(
    int LidarSensorleft, int LidarSensorright,
    double MotorStatusleftSpeed, double MotorStatusrightSpeed, double MotorStatusleftPower, double MotorStatusrightPower, int MotorStatusleftStatus, int MotorStatusrightStatus,
    int Camerastatus,
    int LineLaserleft, int LineLaserright,
    int TofStatusleftStatus, double TofStatusleftDistance, int TofStatusrightStatus, double TofStatusrightDistance, int TofStatustopStatus, double TofStatustopDistance,
    bool IRsensor1, bool IRsensor2, bool IRsensor3, bool IRsensor4, bool IRsensor5, bool IRsensor6, bool stationIR1, bool stationIR2, bool stationIR3, bool stationIR4,
    int SonicleftStatus, int SonicleftSenseing, int SonicrightStatus, int SonicrightSenseing,
    double Batteryvoltage, double Batterycurrent, double Batterytemperature, double BatterychargeState, double Batterycapacity,
    double BatterydesignCapacity, double Batterypercentage, double BatterysupplyStatus, double BatterysupplyHealth,
    double BatterysupplyTechnology, double Batterypresent, double BatterycellVoltage, double BatterycellTemperature,
    double Batterylocation, std::string BatteryserialNumber);

void resGetAllStatusV2(
    int LidarSensorfront, int LidarSensorrear,
    double MotorStatusleftRPM, double MotorStatusrightRPM, double MotorStatusleftCurrent, double MotorStatusrightCurrent, int MotorStatusleftType, int MotorStatusrightType,
    int Camerastatus, int CameraType, const std::vector<ObjectDataV2> &objects,
    int LineLaserStatus, const std::vector<LLDataV2> &llDataList,
    int TofStatusleftStatus, int TofStatusrightStatus, int TofStatustopStatus,
    const std::vector<int> &leftDistances, const std::vector<int> &rightDistances, int TofStatustopDistance,
    // const std::vector<double> &TofStatusleftCalibrationData, const std::vector<double> &TofStatusrightCalibrationData,
    int CliffIR1, int CliffIR2, int CliffIR3, int CliffIR4, int CliffIR5, int CliffIR6,
    bool RecvIR1, bool RecvIR2, bool RecvIR3, bool RecvIR4,
    double cellVoltage1, double cellVoltage2, double cellVoltage3, double cellVoltage4, double cellVoltage5,
    double totalCapacity, double remainingCapacity, int batteryManufacturer, double batteryPercent,
    double batteryVoltage, double batteryCurrent, double batteryTemperature1, double batteryTemperature2,
    double designCapacity, int numberOfCycles, int chargeStatus,
    double accX, double accY, double accZ, double roll, double pitch, double yaw);

void resGetAllMovingInfo(
    int movingState,
    double x, double y, double theta,
    double Targetx, double Targety, double Targettheta);

void resGetAllMovingInfoV2(
        int movingState,
        bool validPosition,
        double x, double y, double theta,
        bool validTargetPosition,
        double Targetx, double Targety, double Targettheta);

void resGetIntegrationMapDataB(const unsigned char *data, size_t length);
void resGetIntegrationModifiedMapDataB(const unsigned char *data, size_t length);

void resGetDockingStatus(int State);

/*********************************************************************/

bool reqSetReturnToChargingStation(void);
bool reqSetRecoveryReturnToChargingStation(void);
bool reqSetEmergencyStop(void);
bool reqSetStartLidar(void);
bool reqSetStopLidar(void);
bool reqSetStartCharging(void);
bool reqSetStopCharging(void);
bool reqSetSoftwareReset(void);
bool reqSetInitMap(void);
bool reqSetSaveMap(void);
bool reqSetMaxDrivingSpeed(MaxDrivingSpeed_t& settings);
bool reqSetMotorManual_RPM(MotorManual_RPM_t& settings);
bool reqSetMotorManual_VW(MotorManual_VW_t& settings);
bool reqSetTargetPosition(TargetPosition_t& settings);
bool reqSetTargetPositionType(TargetPositionType_t &settings);
bool reqSetStationRepositioning(StationRepositioning_t &settings);
bool reqSetSelfDiagnosisMotor(SelfDiagnosisMotor_t &settings);
bool reqSetDriving(Driving_t& settings);
bool reqSetMapping(Mapping_t& settings);
bool reqSetModifiedMapDataS(std::string&  data);
//bool reqSetModifiedMapDataB(std::vector<unsigned char> &data) ;
bool reqSetModifiedMapDataB(ModifiedMapDataB_t &data);
bool reqSetByPassOneData(ByPassOne_t &data);
bool reqSetMapCopyData(ByPassOne_t &data);
bool reqSetPGMMapData(PGMMapDataB_t &data);
bool reqSetMapCopy(MapCopyB_t &data);
bool MapCopyAlarm(void);

bool reqSetIntegrationModifiedMapDataB(IntegrationModifiedMapDataB_t &data);

bool reqSetheartbeat(void);
bool reqSetExcelSteps(ExcelSteps_t &data);
bool reqSetRotation(Rotation_t &data);
bool reqSetTimeData(TimeData_t &data);

bool reqSetOTA(ota_t &data);
bool reqSetOTAdata(otadataB_t &data);
bool reqSetFactoryReset(void);
bool reqSetRecovery(void);
bool reqSetOTAsuccess(void);

bool reqSetFactoryMode(FactoryMode_t &data);
bool reqSetBatterySleepMode(BatterySleepMode_t &data);
bool reqSetSensorInspectionMode(InspectionMode_t &data);
bool reqSetFollowMe(FollowMe_t &settings);

bool reqSetDockingState(DockingStatus_t &data);

void resSetReturnToChargingStation(bool Result);
void resSetRecoveryReturnToChargingStation(bool Result);
void resSetEmergencyStop(bool Result);
void resSetStartLidar(bool Result);
void resSetStopLidar(bool Result);
void resSetStartCharging(bool Result);
void resSetStopCharging(bool Result);
void resSetSoftwareReset(bool Result);
void resSetInitMap(bool Result);
void resSetSaveMap(bool Result);
void resSetMaxDrivingSpeed(bool Result);
void resSetMotorManual_RPM(bool Result);
void resSetMotorManual_VW(bool Result);
void resSetTargetPosition(bool Result);
void resSetTargetPositionType(bool Result);

void resSetStationRepositioning(bool Result);
void resSetSelfDiagnosisMotor(bool Result);

void resSetDriving(bool Result);
void resSetMapping(bool Result);
void resSetModifiedMapData(bool Result);
void resSetPGMMapData(bool Result);
void resSetMapCopy(bool Result);

void resSetIntegrationModifiedMapData(bool Result);
void resSetByPassOneData(bool Result);
void resSetByPassTwoData(bool Result);

void resSetheartbeat(bool Result);
void resSetExcelSteps(bool Result);
void resSetRotation(bool Result);
void resSetTimeData(bool Result);

void resSetOTA(bool Result);
// void resSetOTAdata(bool Result);
void resSetOTAdata(int progress, int state);
void resSetFactoryReset(bool Result);
void resSetRecovery(bool Result);
void resSetOTAsuccess(bool Result);

void resSetFactoryMode(bool Result);
void resSetBatterySleepMode(bool Result);
void resSetSensorInspectionMode(bool Result);
void resSetFollowMe(bool Result);

void resSetDockingState(bool Result);

bool RecvErrorRes(void);

/*********************************************************************/
void EmergencyMSG_OTA(int state,int progress,std::string version);
void EmergencyMSG_ERROR(const std::vector<ErrorList_t> &ErrorList);
void EmergencyMSG_FanCmdAndTemps(bool fanOn, double SWnpu, double SWgpu, double SWcenter, double SWsoc, double SWbigcore0, double SWbigcore1, double SWlittlecore,
                                              double temperature1, double temperature2);

bool reqSetSettingCommand(SettingCommand_t &data);
void resSetSettingCommand(bool Result);
bool reqSetConfigCommand(ConfigCommand_t &data);
void resSetConfigCommand(bool Result);
/*********************************************************************/
bool API_RecvJigData(int command, std::vector<uint8_t>& data);
void API_SendJigData(int command, const std::vector<uint8_t> &data);


/*********************************************************************/
std::string API_FileRead(const std::string &filename, unsigned char *key);
void API_FileSave(const std::string& plaintext, const std::string& filename, const unsigned char* key, const unsigned char* iv);
bool API_FileExist(const std::string &filename);
void API_FileDelete(const std::string &filename);
// start_server 없이 사용을 위해서는 API_GenerateKey 필수
void API_GenerateKey();
bool API_FileDataRead(ByPassOne_t &data);
bool API_JsonFileReader(const std::string &filepath,
                        ByPassOne_t &data,
                        bool encData = true);
bool API_FileDataApply();
bool API_MapCopyDataApply();

#ifdef __cplusplus
}
#endif

#endif // LIBNETWOKR_H
