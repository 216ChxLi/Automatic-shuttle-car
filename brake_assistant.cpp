#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <communication/multi_socket.h>
#include <models/tronis/ImageFrame.h>
#include <models/tronis/PointCloud.h>

#include <grabber/opencv_tools.hpp>
#include <geometry/coord_sys_adapter_tronis_model.h>

using namespace std;

class BrakeAssistant
{
public:
    BrakeAssistant() : brake( false ) //初始化为false
    {
    }

    bool processData( tronis::ModelDataWrapper data_model,
                      tronis::CircularMultiQueuedSocket& socket )
    {
        if( data_model->GetModelType() == tronis::ModelType::Tronis )
        {
            std::cout << "Id: " << data_model->GetTypeId() << ", Name: " << data_model->GetName()
                      << ", Time: " << data_model->GetTime() << std::endl;

            // if data is sensor output, process data
            switch( static_cast<tronis::TronisDataType>( data_model->GetDataTypeId() ) )
            {
                case tronis::TronisDataType::Image:
                {
                    processImage( data_model->GetName(),
                                  data_model.get_typed<tronis::ImageSub>()->Image );
                    processImageBrake( data_model.get_typed<tronis::ImageSub>()->Image, socket );
                    break;
                }
                case tronis::TronisDataType::ImageFrame:
                {
                    const tronis::ImageFrame& frames(
                        data_model.get_typed<tronis::ImageFrameSub>()->Images );
                    for( size_t i = 0; i != frames.numImages(); ++i )
                    {
                        std::ostringstream os;
                        os << data_model->GetName() << "_" << i + 1;

                        processImage( os.str(), frames.image( i ) );
                        processImageBrake( frames.image( i ), socket );
                    }
                    break;
                }
                case tronis::TronisDataType::ImageFramePose:
                {
                    const tronis::ImageFrame& frames(
                        data_model.get_typed<tronis::ImageFramePoseSub>()->Images );
                    for( size_t i = 0; i != frames.numImages(); ++i )
                    {
                        std::ostringstream os;
                        os << data_model->GetName() << "_" << i + 1;

                        processImage( os.str(), frames.image( i ) );
                        processImageBrake( frames.image( i ), socket );
                    }
                    break;
                }
                case tronis::TronisDataType::PointCloud:
                {
                    const tronis::PointCloudStruct* point_cloud =
                        data_model.get_typed<tronis::PointCloudStruct>();
                    if( point_cloud->Points.imageType() == tronis::ImageType::IT_LIDAR_D ||
                        point_cloud->Points.imageType() == tronis::ImageType::IT_LIDAR_DS )
                    {
                        processImage( data_model->GetName(), point_cloud->Points );
                        break;
                    }
                }
                default:
                {
                    std::cout << data_model->ToString() << std::endl;
                    break;
                }
            }
            return true;
        }
        else
        {
            std::cout << data_model->ToString() << std::endl;
            return false;
        }
    }

    // Function to show an openCV image in a separate window
    void showImage( std::string image_name, cv::Mat image )
    {
        cv::Mat out = image;
        if( image.type() == CV_32F || image.type() == CV_64F )
        {
            cv::normalize( image, out, 0.0, 1.0, cv::NORM_MINMAX, image.type() );
        }
        cv::namedWindow( image_name.c_str(), cv::WINDOW_NORMAL );
        cv::imshow( image_name.c_str(), out );
    }

protected:
    bool brake;

    // Function to convert tronis image to openCV image
    bool processImage( const std::string& base_name, const tronis::Image& image )
    {
        if( image.empty() )
        {
            return false;
        }

        switch( image.imageType() )
        {
            case tronis::ImageType::IT_LIDAR_DS:
            {
                cv::Mat channels[2];
                cv::split( tronis::image2MatRef( image ), channels );
                showImage( base_name, channels[0] );
            }
            break;
            case tronis::ImageType::IT_LIDAR_D:
            {
                showImage( base_name, tronis::image2MatRef( image ) );
            }
            break;
            default:
            {
                showImage( base_name, tronis::image2MatRef( image ) );
            }
        }
        return true;
    }

    void processImageBrake( const tronis::Image& image, tronis::CircularMultiQueuedSocket& socket )
    {
        cv::Mat data = tronis::image2MatRef( image );
        if( data.channels() == 1 && data.type() == CV_8U )
        {
            cv::Mat binary_image = data == 24; // 不懂

            double cpix = cv::countNonZero( binary_image ); // 返回灰度值不为0的像素数，可用来判断图像是否全黑。
            cpix /= image.elementSize(); // 返回每个元素的字节大小
            double vmin, vmax;
            cv::minMaxLoc( data, &vmin, &vmax ); // minMaxLoc寻找矩阵(一维数组当作向量,用Mat定义) 中最小值和最大值的位置.  
            std::cout << cpix << " " << vmin << " " << vmax << std::endl;
            if( cpix > 0.001 )
            {
                if( brake == false )
                {
                    socket.send( tronis::SocketData( "brake" ) );
                    brake = true;
                    std::cout << "Brake! " << vmax << std::endl;
                }
            }
            else
            {
                if( brake == true )
                {
                    socket.send( tronis::SocketData( "drive" ) );
                    brake = false;
                }
            }
        }
    }
};

// main loop opens socket and listens for incoming data
int main( int argc, char** argv ) //听一下课
{
    std::cout << "Welcome to brake assistant" << std::endl;

    // specify socket parameters
    std::string socket_type = "TcpSocket";
    std::string socket_ip = "127.0.0.1";
    std::string socket_port = "7778";

    std::ostringstream socket_params;
    socket_params << "{Socket:\"" << socket_type << "\", IpBind:\"" << socket_ip
                  << "\", PortBind:" << socket_port << "}";

    int key_press = 0;  // close app on key press 'q'
    tronis::CircularMultiQueuedSocket msg_grabber;
    uint32_t timeout_ms = 500;  // close grabber, if last received msg is older than this param

    BrakeAssistant brake_assistant;

    while( key_press != 'q' )
    {
        std::cout << "Wait for connection..." << std::endl;
        msg_grabber.open_str( socket_params.str() ); // 打开grabber 括号里是相关的参数

        if( !msg_grabber.isOpen() )
        {
            printf( "Failed to open grabber, retry...!\n" );
            continue;
        }

        std::cout << "Start grabbing" << std::endl;

        tronis::SocketData received_data;
        uint32_t time_ms = 0;

        while( key_press != 'q' )
        {
            // wait for data, close after timeout_ms without new data
            if( msg_grabber.tryPop( received_data, true ) ) // 数据存储给received_data
            {
                // data received! reset timer
                time_ms = 0;

                // convert socket data to tronis model data
                tronis::SocketDataStream data_stream( received_data ); //数据转换  创建datamodel
                tronis::ModelDataWrapper data_model(
                    tronis::Models::Create( data_stream, tronis::MessageFormat::raw ) );
                if( !data_model.is_valid() )
                {
                    std::cout << "received invalid data, continue..." << std::endl;
                    continue;
                }
                // identify data type
                brake_assistant.processData( data_model, msg_grabber );
            }
            else
            {
                // no data received, update timer
                ++time_ms;
                if( time_ms > timeout_ms )
                {
                    std::cout << "Timeout, no data" << std::endl;
                    msg_grabber.close();
                    break;
                }
                else
                {
                    std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
                    key_press = cv::waitKey( 1 );
                }
            }
        }
        msg_grabber.close();
    }
    return 0;
}
