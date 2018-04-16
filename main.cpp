// Shamelessly solen from https://gist.github.com/UnaNancyOwen/fc7b62cc069fef74e0e096db3dd0a856

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <sstream>
#include <stdexcept>

#define NOMINMAX

#include "net_client.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Kinect.h>
#include <wrl/client.h>
using namespace Microsoft::WRL;

// Error Check
#define ERROR_CHECK( ret )                                        \
    if( FAILED( ret ) ){                                          \
        std::stringstream ss;                                     \
        ss << "failed " #ret " " << std::hex << ret << std::endl; \
        throw std::runtime_error( ss.str().c_str() );             \
    }

class Kinect
{
private:
    // Sensor
    ComPtr<IKinectSensor> kinect;

    // Coordinate Mapper
    ComPtr<ICoordinateMapper> coordinateMapper;

    // Reader
    ComPtr<IColorFrameReader> colorFrameReader;
    ComPtr<IDepthFrameReader> depthFrameReader;

    // Color Buffer
    std::vector<BYTE> colorBuffer;
    int colorWidth;
    int colorHeight;
    unsigned int colorBytesPerPixel;

    // Depth Buffer
    std::vector<UINT16> depthBuffer;
    int depthWidth;
    int depthHeight;
    unsigned int depthBytesPerPixel;

    // PCL
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

    // Network client
    NetClient client;

public:
    // Constructor
    Kinect(const std::string& server_ip, uint32_t port)
    {
        // Initialize
        initialize(server_ip, port);
    }

    // Destructor
    ~Kinect()
    {
        // Finalize
        finalize();
    }

    // Processing
    void run(){
        while( !viewer->wasStopped() ){
            // Update Data
            update();

            // Draw Data
            draw();

            // Show Data
            show();

            // Send data over network
            send();
        }
    }

private:
    // Initialize
    void initialize(const std::string& server_ip, uint32_t port)
    {
        // Initialize network connection
        initializeNetworkClient(server_ip, port);
        
        // Initialize Sensor
        initializeSensor();

        // Initialize Color
        initializeColor();

        // Initialize Depth
        initializeDepth();

        // Initialize Point Cloud
        initializePointCloud();
    }

    // Initialize Sensor
    inline void initializeSensor()
    {
        // Open Sensor
        ERROR_CHECK( GetDefaultKinectSensor( &kinect ) );

        ERROR_CHECK( kinect->Open() );

        // Check Open
        BOOLEAN isOpen = FALSE;
        ERROR_CHECK( kinect->get_IsOpen( &isOpen ) );
        if( !isOpen ){
            throw std::runtime_error( "failed IKinectSensor::get_IsOpen( &isOpen )" );
        }

        // Retrieve Coordinate Mapper
        ERROR_CHECK( kinect->get_CoordinateMapper( &coordinateMapper ) );
    }

    // Initialize Color
    inline void initializeColor()
    {
        // Open Color Reader
        ComPtr<IColorFrameSource> colorFrameSource;
        ERROR_CHECK( kinect->get_ColorFrameSource( &colorFrameSource ) );
        ERROR_CHECK( colorFrameSource->OpenReader( &colorFrameReader ) );

        // Retrieve Color Description
        ComPtr<IFrameDescription> colorFrameDescription;
        ERROR_CHECK( colorFrameSource->CreateFrameDescription( ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription ) );
        ERROR_CHECK( colorFrameDescription->get_Width( &colorWidth ) ); // 1920
        ERROR_CHECK( colorFrameDescription->get_Height( &colorHeight ) ); // 1080
        ERROR_CHECK( colorFrameDescription->get_BytesPerPixel( &colorBytesPerPixel ) ); // 4

        // Allocation Color Buffer
        colorBuffer.resize( colorWidth * colorHeight * colorBytesPerPixel );
    }

    // Initialize Depth
    inline void initializeDepth()
    {
        // Open Depth Reader
        ComPtr<IDepthFrameSource> depthFrameSource;
        ERROR_CHECK( kinect->get_DepthFrameSource( &depthFrameSource ) );
        ERROR_CHECK( depthFrameSource->OpenReader( &depthFrameReader ) );

        // Retrieve Depth Description
        ComPtr<IFrameDescription> depthFrameDescription;
        ERROR_CHECK( depthFrameSource->get_FrameDescription( &depthFrameDescription ) );
        ERROR_CHECK( depthFrameDescription->get_Width( &depthWidth ) ); // 512
        ERROR_CHECK( depthFrameDescription->get_Height( &depthHeight ) ); // 424
        ERROR_CHECK( depthFrameDescription->get_BytesPerPixel( &depthBytesPerPixel ) ); // 2

        // Allocation Depth Buffer
        depthBuffer.resize( depthWidth * depthHeight );
    }

    // Initialize Point Cloud
    inline void initializePointCloud()
    {
        // Create Point Cloud
        cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
        cloud->width = static_cast<uint32_t>( depthWidth );
        cloud->height = static_cast<uint32_t>( depthHeight );
        cloud->points.resize( cloud->height * cloud->width );
        cloud->is_dense = false;

        // Create PCLVisualizer
        viewer = boost::make_shared<pcl::visualization::PCLVisualizer>( "Point Cloud Viewer" );

        // Initialize camera position
        viewer->setCameraPosition( 0.0, 0.0, -2.5, 0.0, 0.0, 0.0 );

        // Add Coordinate System
        viewer->addCoordinateSystem( 0.1 );
    }

    inline void initializeNetworkClient(const std::string& server_ip, uint32_t port) {
        this->client = NetClient(server_ip, port);
        if(!this->client.connect_client())
            std::cerr << "ERROR: There was an issue connecting to the client." << std::endl;
        else
            std::cerr << "Connected to client successfully." << std::endl;
    }

    // Finalize
    void finalize()
    {
        // Close Sensor
        if( kinect != nullptr ){
            kinect->Close();
        }
    }

    // Update Data
    void update()
    {
        // Update Color
        updateColor();

        // Update Depth
        updateDepth();

        // Update Point Cloud
        updatePointCloud();
    }

    // Update Color
    inline void updateColor()
    {
        // Retrieve Color Frame
        ComPtr<IColorFrame> colorFrame;
        const HRESULT ret = colorFrameReader->AcquireLatestFrame( &colorFrame );
        if( FAILED( ret ) ){
            return;
        }

        // Convert Format ( YUY2 -> BGRA )
        ERROR_CHECK( colorFrame->CopyConvertedFrameDataToArray( static_cast<UINT>( colorBuffer.size() ), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra ) );
    }

    // Update Depth
    inline void updateDepth()
    {
        // Retrieve Depth Frame
        ComPtr<IDepthFrame> depthFrame;
        const HRESULT ret = depthFrameReader->AcquireLatestFrame( &depthFrame );
        if( FAILED( ret ) ){
            return;
        }

        // Retrieve Depth Data
        ERROR_CHECK( depthFrame->CopyFrameDataToArray( static_cast<UINT>( depthBuffer.size() ), &depthBuffer[0] ) );
    }

    // Update Point Cloud
    inline void updatePointCloud()
    {
        // Reset Point Cloud
        cloud->clear();

        // Convert to Point Cloud
        for( int depthY = 0; depthY < depthHeight; depthY++ ){
            for( int depthX = 0; depthX < depthWidth; depthX++ ){
                pcl::PointXYZRGBA point;

                // Retrieve Mapped Coordinates
                DepthSpacePoint depthSpacePoint = { static_cast<float>( depthX ), static_cast<float>( depthY ) };
                UINT16 depth = depthBuffer[depthY * depthWidth + depthX];
                ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
                ERROR_CHECK( coordinateMapper->MapDepthPointToColorSpace( depthSpacePoint, depth, &colorSpacePoint ) );

                // Set Color to Point
                int colorX = static_cast<int>( colorSpacePoint.X + 0.5f );
                int colorY = static_cast<int>( colorSpacePoint.Y + 0.5f );
                if( ( 0 <= colorX ) && ( colorX < colorWidth ) && ( 0 <= colorY ) && ( colorY < colorHeight ) ){
                    unsigned int colorIndex = ( colorY * colorWidth + colorX ) * colorBytesPerPixel;
                    point.b = colorBuffer[colorIndex + 0];
                    point.g = colorBuffer[colorIndex + 1];
                    point.r = colorBuffer[colorIndex + 2];
                    point.a = colorBuffer[colorIndex + 3];
                }

                // Retrieve Mapped Coordinates
                CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
                ERROR_CHECK( coordinateMapper->MapDepthPointToCameraSpace( depthSpacePoint, depth, &cameraSpacePoint ) );

                // Set Depth to Point
                if( ( 0 <= colorX ) && ( colorX < colorWidth ) && ( 0 <= colorY ) && ( colorY < colorHeight ) ){
                    point.x = cameraSpacePoint.X;
                    point.y = cameraSpacePoint.Y;
                    point.z = cameraSpacePoint.Z;
                }

                // Set Point to Point Cloud
                cloud->push_back( point );
            }
        }
    }

    // Draw Data
    void draw()
    {
        // Draw Point Cloud
        drawPointCloud();
    }

    // Draw Point Cloud
    inline void drawPointCloud()
    {
        // Update Point Cloud
        if( !viewer->updatePointCloud( cloud, "cloud" ) ){
            viewer->addPointCloud( cloud, "cloud" );
        }
    }

    // Show Data
    void show()
    {
        // Show Point Cloud
        showPointCloud();
    }

    void send() {
        sendPointCloud();
    }

    // Show Point Cloud
    inline void showPointCloud()
    {
        // Update Viwer
        viewer->spinOnce();
    }

    inline void sendPointCloud() {
        if(this->client.is_open()) {
            if(!this->client.send_pointcloud(this->cloud)) {
                std::cerr << "ERROR: Could no send pointcloud." << std::endl;
            }

        } else {
            std::cerr << "WARNING: Client is not open. Skipping send..." << std::endl;
        }
    }
};

void initializeWinsock() {
    WSADATA wsaData;
    int error;

    // Initialize Winsock
    error = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (error != 0) {
        printf("WSAStartup failed: %d\n", error);
        exit(1);
    }
}

int main( int argc, char* argv[] )
{
    std::string ip_address;
    if(argc > 1)
        ip_address = std::string(argv[1]);
    else
        ip_address = "10.0.0.5";

    initializeWinsock();

    try{
        Kinect kinect(ip_address, 5525);
        kinect.run();
    } catch( std::exception& ex ){
        std::cout << ex.what() << std::endl;
    }

    return 0;
}
