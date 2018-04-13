#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#define NOMINMAX

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cstdint>

#include <Windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>
#include <stdio.h>

#include <string>

#pragma comment(lib, "Ws2_32.lib")

const std::string HANDSHAKE_MSG = "hello";
const std::string CLOSE_MSG = "close";


class NetClient {
public:
    NetClient(const std::string& server_url, uint32_t server_port);
    bool connect_client();
    bool send_pointcloud(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr pointcloud);
    bool close_client();
    bool is_open();
    ~NetClient();
private:
    bool setup_socket();
    bool handshake();

    void serialize(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr pointcloud, std::vector<uint8_t>& buffer);

    std::string server_url;
    uint32_t server_port;
    bool open;
    SOCKET connect_socket = INVALID_SOCKET;
};