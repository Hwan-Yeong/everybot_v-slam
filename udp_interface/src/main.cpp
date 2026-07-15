#include <iostream>
#include <cstdlib>
#include <dlfcn.h>
#include <rclcpp/rclcpp.hpp>
#include "udp_communication.hpp"

void *g_handle = nullptr;
void (*g_stop_server)() = nullptr;

void cleanup() {
    if (g_stop_server) {
        std::string log = "[UDP-MAIN] Stopping UDP server...";
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), log.c_str());
        g_stop_server();
    }
    if (g_handle) {
        std::string log = "[UDP-MAIN] Dynamic library closed.";
        dlclose(g_handle);
        g_handle = nullptr;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), log.c_str());
    }
    rclcpp::shutdown();
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    //setup_signal_handler();
    
    g_handle = dlopen("/home/airbot/vslam_ws/install/udp_interface/include/udp_interface/libNetwork.so", RTLD_LAZY);
    if (!g_handle) {
        std::string log = "[UDP-MAIN] Cannot open library: " + std::string(dlerror());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), log.c_str());
        cleanup();
        return -1;
    }

    auto start_server = (void(*)())dlsym(g_handle, "start_server");
    g_stop_server = (void(*)())dlsym(g_handle, "stop_server");
    auto APIGetVersion = (void(*)(std::string&))dlsym(g_handle, "APIGetVersion");
    auto APISetEnc = (void(*)(bool))dlsym(g_handle, "APISetEnc");

    if (!start_server || !g_stop_server || !APIGetVersion || !APISetEnc) {
        std::string log = "[UDP-MAIN] Cannot load functions from library";
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), log.c_str());
        cleanup();
        return -1;
    }

    start_server();
    std::string version;
    APIGetVersion(version);
    APISetEnc(false);
    
    std::string log = "[UDP-MAIN] API Version: " + version;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), log.c_str());

    std::atexit(cleanup);
    auto udp_node = std::make_shared<UdpCommunication>();
    rclcpp::spin(udp_node);

    return 0;
}