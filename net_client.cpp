#include "net_client.hpp"

#include <cstring>

#define BUF_SIZE 255

#define COPY_DATA(pointer, data) {         \
    memcpy(pointer, &data, sizeof(data));  \
    pointer += sizeof(data);               \
}

NetClient::NetClient(const std::string& server_url, uint32_t server_port)
    : server_url(server_url), server_port(server_port), open(false) {

}

bool NetClient::is_open() { return this->open; }

bool NetClient::setup_socket() {
    int error;
    std::string port_string = std::to_string(this->server_port);
    struct addrinfo *result = NULL,
                    *ptr = NULL,
                    hints;

    ZeroMemory( &hints, sizeof(hints) );
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    error = getaddrinfo(server_url.c_str(), port_string.c_str(), &hints, &result);
    if (error != 0) {
        printf("getaddrinfo failed: %d\n", error);
        WSACleanup();
        return false;
    }

    connect_socket = INVALID_SOCKET;
    ptr = result;

    // Create a SOCKET for connecting to server
    this->connect_socket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);

    if (this->connect_socket == INVALID_SOCKET) {
        printf("Error at socket(): %ld\n", WSAGetLastError());
        freeaddrinfo(result);
        WSACleanup();
        return false;
    }

    error = connect(this->connect_socket, ptr->ai_addr, (int)ptr->ai_addrlen);
    if (error == SOCKET_ERROR) {
        closesocket(this->connect_socket);
        this->connect_socket = INVALID_SOCKET;
    }

    // Should really try the next address returned by getaddrinfo
    // if the connect call failed
    // But for this simple example we just free the resources
    // returned by getaddrinfo and print an error message

    freeaddrinfo(result);

    if (this->connect_socket == INVALID_SOCKET) {
        printf("Unable to connect to server!\n");
        WSACleanup();
        return false;
    }

    return true;
}

bool NetClient::handshake() {
    int error;
    char recv_buf[BUF_SIZE];

    error = send(this->connect_socket, HANDSHAKE_MSG.c_str(), HANDSHAKE_MSG.length(), 0);
    
    if (error == SOCKET_ERROR) {
        printf("Handshake send failed: %d\n", WSAGetLastError());
        closesocket(this->connect_socket);
        WSACleanup();
        return false;
    }

    printf("Bytes Sent: %ld\n", error);

    error = recv(this->connect_socket, recv_buf, BUF_SIZE, 0);
    if (error > 0) {
        printf("Bytes received: %d\n", error);
        if(strcmp(recv_buf, HANDSHAKE_MSG.c_str()) == 0)
            return true;
        else
            printf("Handshake not received.\n");
    }
    else if (error == 0)
        printf("Connection closed\n");
    else
        printf("recv failed: %d\n", WSAGetLastError());

    return false;
}

bool NetClient::connect_client() {
    if(!setup_socket())
        return false;
    if(!handshake())
        return false;
    return true;
}

bool NetClient::send_pointcloud(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr pointcloud) {
    std::vector<uint8_t> serial_data_buffer;
    this->serialize(pointcloud, serial_data_buffer);

    std::string header_string = "snd ";
    uint64_t bytes_length_net = htonl(serial_data_buffer.size());
    std::vector<uint8_t> header_buffer;
    //                   v-- 4 for 'snd ' plus 8 for byte count
    header_buffer.resize(12);

    memcpy(&header_buffer[0], header_string.c_str(), header_string.size());
    memcpy(&header_buffer[4], &bytes_length_net, sizeof(bytes_length_net));

    // Attempt to send the header
	int error = send(this->connect_socket, (char*)&header_buffer[0], header_buffer.size(), 0);
    if (error == SOCKET_ERROR) {
        printf("Header for pointcloud send failed: %d\n", WSAGetLastError());
        closesocket(this->connect_socket);
        WSACleanup();
        return false;
    }

    // Attempt to send the header
    error = send(this->connect_socket, (char*)&serial_data_buffer[0], serial_data_buffer.size(), 0);
    if (error == SOCKET_ERROR) {
        printf("Pointcloud data send failed: %d\n", WSAGetLastError());
        closesocket(this->connect_socket);
        WSACleanup();
        return false;
    }

	return true;
}

bool NetClient::close_client() {
    if(open) {
        int error;
		char* sendbuf = "end";
        error = send(this->connect_socket, sendbuf, (int)strlen(sendbuf), 0);
        if (error == SOCKET_ERROR) {
            printf("Close mesage send failed: %d\n", WSAGetLastError());
            printf("Skipping to shutdown.\n");
            closesocket(this->connect_socket);
            WSACleanup();
        }

        error = shutdown(this->connect_socket, SD_SEND);
        if (error == SOCKET_ERROR) {
            printf("Shutdown failed: %d\n", WSAGetLastError());
            closesocket(this->connect_socket);
            WSACleanup();
            return false;
        }
    }

    return true;
}

void NetClient::serialize(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr pointcloud, std::vector<uint8_t>& buffer) {
    buffer.clear();

    // Get header data and convert to network order
    // uint32_t seq       - 32
    // uint64_t stamp     - 96
    // uint32_t width     - 128
    // uint32_t height    - 160
    // uint8_t  is_dense  - 168
    uint32_t seq_net = htonl(pointcloud->header.seq);
    uint64_t stamp_net = htonll(pointcloud->header.stamp);
    uint32_t width_net = htonl(pointcloud->width);
    uint32_t height_net = htonl(pointcloud->height);
    uint8_t is_dense_net = pointcloud->is_dense; // Network order says the same for 1-byte data

    // string   frame_id  - ??? (32 + length of string, skipping)
    std::vector<uint8_t> frame_id;
    uint64_t frame_id_len_net = htonll(pointcloud->header.frame_id.length());
    frame_id.resize(8 + pointcloud->header.frame_id.length());
    uint8_t* frame_id_ptr =&frame_id[0];
    memcpy(frame_id_ptr, &frame_id_len_net, 8);
    frame_id_ptr += 8;
    memcpy(frame_id_ptr, pointcloud->header.frame_id.c_str(), pointcloud->header.frame_id.length());

    // Calculate bytes taken up by points              v-- xyz values    v-- RGBA values
    uint64_t point_bytes = pointcloud->points.size() * (3*sizeof(float) + 4*sizeof(uint8_t));
    
    // Resize the data buffer to the size needed by the pointcloud
    //            v-- Header data  v-- Frame id      v-- pointcloud
    buffer.resize(168 +            frame_id.size() + point_bytes);

    uint8_t* buf_ptr = &buffer[0];

    // Copy header data
    COPY_DATA(buf_ptr, seq_net);
    COPY_DATA(buf_ptr, stamp_net);
    memcpy(buf_ptr, &frame_id[0], frame_id.size());
    buf_ptr += frame_id.size();
    COPY_DATA(buf_ptr, width_net);
    COPY_DATA(buf_ptr, height_net);
    COPY_DATA(buf_ptr, is_dense_net);

    // Copy pointcloud
    for(pcl::PointXYZRGBA point : pointcloud->points) {
        // Convert data to network ordering
        uint32_t x_net = htonf(point.x);
        uint32_t y_net = htonf(point.y);
        uint32_t z_net = htonf(point.z);
        uint8_t r_net = point.r;
        uint8_t g_net = point.g;
        uint8_t b_net = point.b;
        uint8_t a_net = point.a;

        // Copy data to buffer
        COPY_DATA(buf_ptr, x_net);
        COPY_DATA(buf_ptr, y_net);
        COPY_DATA(buf_ptr, z_net);
        COPY_DATA(buf_ptr, r_net);
        COPY_DATA(buf_ptr, g_net);
        COPY_DATA(buf_ptr, b_net);
        COPY_DATA(buf_ptr, a_net);
    }
}

NetClient::~NetClient() {
    close_client();
}