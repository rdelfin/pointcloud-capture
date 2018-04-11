#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cstdint>

#pragma comment(lib, "Ws2_32.lib")

const std::string HANDSHAKE_MSG = "hello";
const std::string CLOSE_MSG = "close";


class NetClient {
public:
    NetClient(const std::string& server_url, uint32_t server_port);
    bool connect_client();
    bool send_pointcloud(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr pointcloud);
    bool close_client();
    ~NetClient();
private:
    bool setup_socket();
    bool handshake();

    bool serialize(pcl::PointCloud<pcl::PointXYZRgBA>::ConstPtr pointcloud, std::vector<uint8_t>& buffer);

    std::string server_url;
    uint32_t server_port;
    bool open;
    SOCKET connect_socket = INVALID_SOCKET;
};