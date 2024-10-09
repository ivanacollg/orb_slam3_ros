/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono.cc
*
*/

#include "common.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(){};

    void GrabImage(const sensor_msgs::CompressedImageConstPtr& msg);
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(5.0, cv::Size(501, 501));
    ros::Publisher pub_img;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono_compressed_field");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    std::string node_name = ros::this_node::getName();

    ros::NodeHandle node_handler;
    image_transport::ImageTransport image_transport(node_handler);

    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
        ros::shutdown();
        return 1;
    }

    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "map");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");

    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::MONOCULAR;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);
    ImageGrabber igb;

    ros::Subscriber sub_img = node_handler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);
    igb.pub_img = node_handler.advertise<sensor_msgs::Image>("/processed_img", 1);

    setup_publishers(node_handler, image_transport, node_name);
    setup_services(node_handler, node_name);


    ros::spin();

    // Stop all threads
    pSLAM->Shutdown();
    ros::shutdown();

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabImage(const sensor_msgs::CompressedImageConstPtr &img_msg)
{
    // Convert the compressed ROS image message to a cv::Mat.
    cv::Mat image;
    try
    {
        // Decode the compressed image from the message.
        image = cv::imdecode(cv::Mat(img_msg->data), cv::IMREAD_GRAYSCALE);
    }
    catch (cv::Exception& e)
    {
        ROS_ERROR("cv::Exception while decoding image: %s", e.what());
        return ; // Return an empty matrix on failure.
    }
    
    // Check if the image was successfully decoded
    if (image.empty())
    {
        ROS_ERROR("Failed to decode compressed image");
        return ; // Return an empty matrix if decoding failed.
    }

    
    // Filter image
    cv::Mat gray;
    //cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // Smooth the image
    cv::GaussianBlur(image,gray,cv::Size(5,5),0);
    //mClahe->apply(gray,gray);

    // Apply thresholding 551
    cv::adaptiveThreshold(gray, gray, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 501, 0);
    
    // noise removal
    
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::morphologyEx(gray, gray,
                    cv::MORPH_OPEN,
                    kernel,
                    cv::Point(-1, -1),
                    3);

    // ORB-SLAM3 runs in TrackMonocular()
    Sophus::SE3f Tcw = pSLAM->TrackMonocular(gray, img_msg->header.stamp.toSec());

    ros::Time msg_time = img_msg->header.stamp;

    publish_topics(msg_time);

    // Convert image from cv::Mat (OpenCV) type to sensor_msgs/Image (ROS) type and publish
    sensor_msgs::ImagePtr msg;
    msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray).toImageMsg();
    pub_img.publish(msg);
    
}