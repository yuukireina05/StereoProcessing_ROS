#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <blackmagic_camera_driver/decklink_interface.hpp>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// XIAOFENG
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/transform_broadcaster.h>
// Multi-threading
#include <thread>
#include <iostream>
#include <blackmagic_camera_driver/barrier.hpp>

#define DIVISOR 4

namespace blackmagic_camera_driver
{
namespace
{

Barrier b(2);
ros::Duration oneStep(1.0);
ros::Time timeStep;

// void process_left(const BMDCompatibleVideoFrame& video_frame, image_transport::Publisher image_pub_left, 
//                     ros::Publisher cam_pub_left, sensor_msgs::CameraInfo ros_cameraInfo_left){
//       //////////////////////////////////////////////////
//       //  Important NOTE:
//       //  Both cameras shold be set to 1080P60hz, or you
//       //  will get a wired format of YUV and abnormal 
//       //  colorspace conversion!!
//       ///////////////////////////////////////////////////
//       // XIAOFENG
//       // Resize the ROS output image
//       cv::Mat cvImgResized;
//       cv::Mat cv_image_bgra;
//       uint8_t* buffer = video_frame.Data();
//       cv_image_bgra = cv::Mat(int(video_frame.Height()), int(video_frame.Width()), CV_8UC4, buffer, video_frame.Step());
//       cv::Mat cv_image_bgr;
//       cv::cvtColor(cv_image_bgra, cv_image_bgr, cv::COLOR_BGRA2BGR);
//       // Write the frame into the file
//       // videoRecord_left << cv_image_bgr;
//       cv::resize(cv_image_bgra,cvImgResized,cv::Size(960, 540));
//       ros_image_resized_left = cv_bridge::CvImage(std_msgs::Header(), "bgra8", cvImgResized).toImageMsg();
//       b.arrive_and_wait();
//       ros_image_resized_left->header.stamp = ros::Time::now();
//       ros_cameraInfo_left.header.stamp = ros::Time::now();
//       // ros_image_resized_left->header.stamp = nowTime;
//       // ros_cameraInfo_left.header.stamp = nowTime;
//       image_pub_left.publish(ros_image_resized_left);
//       cam_pub_left.publish(ros_cameraInfo_left);
//       cvImgResized.release();
//       cv_image_bgra.release();
//       cv_image_bgr.release();
// }

// void process_right(const BMDCompatibleVideoFrame& video_frame, image_transport::Publisher image_pub_right, 
//                     ros::Publisher cam_pub_right, sensor_msgs::CameraInfo ros_cameraInfo_right, ros::Time nowTime){
//       //////////////////////////////////////////////////
//       //  Important NOTE:
//       //  Both cameras shold be set to 1080P60hz, or you
//       //  will get a wired format of YUV and abnormal 
//       //  colorspace conversion!!
//       ///////////////////////////////////////////////////
//       // XIAOFENG
//       // Resize the ROS output image
//       cv::Mat cvImgResized;
//       cv::Mat cv_image_bgra;
      
//       uint8_t* buffer = video_frame.Data();

//       cv_image_bgra = cv::Mat(int(video_frame.Height()), int(video_frame.Width()), CV_8UC4, buffer, video_frame.Step());

//       cv::Mat cv_image_bgr;
//       cv::cvtColor(cv_image_bgra, cv_image_bgr, cv::COLOR_BGRA2BGR);
//       // Write the frame into the file
//       // videoRecord_right << cv_image_bgr;

//       cv::resize(cv_image_bgra,cvImgResized,cv::Size(960, 540));
//       ros_image_resized_right = cv_bridge::CvImage(std_msgs::Header(), "bgra8", cvImgResized).toImageMsg();

//       b.arrive_and_wait();
//       ros_image_resized_right->header.stamp = ros::Time::now();
//       ros_cameraInfo_right.header.stamp = ros::Time::now();
//       // ros_image_resized_right->header.stamp = nowTime;
//       // ros_cameraInfo_right.header.stamp = nowTime;
//       image_pub_right.publish(ros_image_resized_right);
//       cam_pub_right.publish(ros_cameraInfo_right);

//       nowTime = ros::Time::now();
// }

void tf_broadcasting(ros::Time nowTime){
  ///////////////////////////
  // tf_frame
  ///////////////////////////
  tf::TransformBroadcaster br;
  tf::Transform transform;

  // tf brocasting
  transform.setOrigin( tf::Vector3(0.0, 0.0, 3) );
  transform.setRotation( tf::Quaternion(-1, 0, 0, 0) );
  br.sendTransform(tf::StampedTransform(transform, nowTime, "map", "video_input_left[1]"));
}

ros::console::levels::Level ConvertLogLevels(const LogLevel level)
{
  if (level == LogLevel::DEBUG)
  {
    return ros::console::levels::Debug;
  }
  else if (level == LogLevel::INFO)
  {
    return ros::console::levels::Info;
  }
  else if (level == LogLevel::WARN)
  {
    return ros::console::levels::Warn;
  }
  else if (level == LogLevel::ERROR)
  {
    return ros::console::levels::Error;
  }
  else
  {
    throw std::runtime_error("Invalid LogLevel value");
  }
}

void RosLoggingFunction(
    const LogLevel level, const std::string& message, const bool throttle)
{
  const auto ros_level = ConvertLogLevels(level);
  if (throttle)
  {
    ROS_LOG_THROTTLE(10, ros_level, ROSCONSOLE_DEFAULT_NAME, message.c_str());
  }
  else
  {
    ROS_LOG(ros_level, ROSCONSOLE_DEFAULT_NAME, message.c_str());
  }
}

/// @brief Left camera thread
/// @param nhp 
/// @param decklink_devices 
void leftCamera_thread(ros::NodeHandle& nh, ros::NodeHandle& nhp, std::vector<DeckLinkHandle>& decklink_devices) 
{
  ros::NodeHandle nh_left(nh, "left");

  const int32_t decklink_device_indexL // Index for the LEFT camera (device[1])
  = nhp.param(std::string("decklink_device_indexL"), 0);
  const std::string image_topic_left
  = nhp.param(std::string("image_topic_left"), std::string("image_raw"));
  const std::string image_frame_left
  = nhp.param(std::string("image_frame_left"), std::string("video_input_left[1]"));

    ROS_INFO("Selecting DeckLink device [%d] for LEFT", decklink_device_indexL);

  ///////////////////////////
  // Setup ROS interface
  ///////////////////////////
  image_transport::ImageTransport it_left(nh_left);
  image_transport::Publisher image_pub_left = it_left.advertise(image_topic_left, 1, false);
  ros::Publisher cam_pub_left = nh_left.advertise<sensor_msgs::CameraInfo>("camera_info", 1000);
  sensor_msgs::Image ros_image_left;
  sensor_msgs::ImagePtr ros_image_resized_left;

  ///////////////////////////
  // CameraInfo loading
  ///////////////////////////
  std::string url_leftInfo; 
  nhp.getParam(ros::this_node::getName()+"/camera_info_url_left", url_leftInfo);
  camera_info_manager::CameraInfoManager* m_cameraInfoManagerL = new camera_info_manager::CameraInfoManager(nh_left);

  if(m_cameraInfoManagerL->validateURL(url_leftInfo))
    {
      m_cameraInfoManagerL->loadCameraInfo(url_leftInfo);
      m_cameraInfoManagerL->setCameraName(image_frame_left);
    }
  sensor_msgs::CameraInfo ros_cameraInfo_left = m_cameraInfoManagerL->getCameraInfo();
  ros_cameraInfo_left.header.frame_id = image_frame_left;

  // Create callback functions for left cameras
  const auto frame_size_change_fn_left = [&](
  const int64_t width, const int64_t height, const int64_t step)
  {
      ros_image_left.header.frame_id = image_frame_left;
      ros_image_left.width = static_cast<uint32_t>(width);
      ros_image_left.height = static_cast<uint32_t>(height);
      ros_image_left.encoding = "bgra8";
      ros_image_left.is_bigendian = false;
      ros_image_left.step = static_cast<uint32_t>(step);
      ros_image_left.data.clear();
      ros_image_left.data.resize(ros_image_left.step * ros_image_left.height, 0x00);
  };

  const auto frame_received_fn_left = [&](const BMDCompatibleVideoFrame& video_frame)
  {

      if (video_frame.DataSize() != static_cast<int64_t>(ros_image_left.data.size()))
      {
      throw std::runtime_error("Video frame and ROS image are different sizes");
      }
      //////////////////////////////////////////////////
      //  Important NOTE:
      //  Both cameras shold be set to 1080P60hz, or you
      //  will get a wired format of YUV and abnormal 
      //  colorspace conversion!!
      ///////////////////////////////////////////////////
      // XIAOFENG
      // Resize the ROS output image
      cv::Mat cvImgResized;
      cv::Mat cv_image_bgra;

      uint8_t* buffer = video_frame.Data();

      cv_image_bgra = cv::Mat(int(video_frame.Height()), int(video_frame.Width()), CV_8UC4, buffer, video_frame.Step());

      cv::Mat cv_image_bgr;
      cv::cvtColor(cv_image_bgra, cv_image_bgr, cv::COLOR_BGRA2BGR);

      // Write the frame into the file
      // videoRecord_left << cv_image_bgr;

      cv::resize(cv_image_bgra,cvImgResized,cv::Size(960, 540));
      ros_image_resized_left = cv_bridge::CvImage(std_msgs::Header(), "bgra8", cvImgResized).toImageMsg();

      b.Wait();

      // now = ros::Time::now();
      timeStep = timeStep + oneStep;
      ros_image_resized_left->header.stamp = timeStep;
      ros_cameraInfo_left.header.stamp = timeStep;
      // ros_image_resized_left->header.stamp = nowTime;
      // ros_cameraInfo_left.header.stamp = nowTime;
      image_pub_left.publish(ros_image_resized_left);
      cam_pub_left.publish(ros_cameraInfo_left);

      cvImgResized.release();
      cv_image_bgra.release();
      cv_image_bgr.release();

  };

  DeckLinkInputDevice input_deviceL(
      RosLoggingFunction, frame_size_change_fn_left, frame_received_fn_left,
      std::move(decklink_devices.at(decklink_device_indexL)));
      
        // Start video input
  ROS_INFO("Starting video input...");
  input_deviceL.Start();

  while (ros::ok())
  {}
    // Stop video input
  ROS_INFO("...stopping video input");
  input_deviceL.Stop();
}

/// @brief Right camera thread
/// @param nhp 
/// @param decklink_devices 
void rightCamera_thread(ros::NodeHandle& nh, ros::NodeHandle& nhp, std::vector<DeckLinkHandle>& decklink_devices) {
        // Load parameters

  const int32_t decklink_device_indexR // Index for the RIGHT camera (device[2])
      = nhp.param(std::string("decklink_device_indexR"), 1);
  const std::string image_topic_right
      = nhp.param(std::string("image_topic_right"), std::string("image_raw"));
  const std::string image_frame_right
      = nhp.param(std::string("image_frame_right"), std::string("video_input_left[2]"));
      
  // Get the selected device

  ROS_INFO("Selecting DeckLink device [%d] for RIGHT", decklink_device_indexR);

  ///////////////////////////
  // Setup ROS interface
  ///////////////////////////
  ros::NodeHandle nh_right(nh, "right");
  image_transport::ImageTransport it_right(nh_right);
  image_transport::Publisher image_pub_right = it_right.advertise(image_topic_right, 1, false);
  ros::Publisher cam_pub_right = nh_right.advertise<sensor_msgs::CameraInfo>("camera_info", 1000);

  sensor_msgs::Image ros_image_right;
  sensor_msgs::ImagePtr ros_image_resized_right;

  ///////////////////////////
  // CameraInfo loading
  ///////////////////////////
  std::string url_rightInfo; 
  nhp.getParam(ros::this_node::getName()+"/camera_info_url_right", url_rightInfo);

  camera_info_manager::CameraInfoManager* m_cameraInfoManagerR = new camera_info_manager::CameraInfoManager(nh_right);
  // ROS_INFO_STREAM(m_cameraInfoManagerL->validateURL(url_leftInfo));

  if(m_cameraInfoManagerR->validateURL(url_rightInfo))
    {
      m_cameraInfoManagerR->loadCameraInfo(url_rightInfo);
      m_cameraInfoManagerR->setCameraName(image_frame_right);
    }

  sensor_msgs::CameraInfo ros_cameraInfo_right = m_cameraInfoManagerR->getCameraInfo();


  ros_cameraInfo_right.header.frame_id = image_frame_right;
  // ROS_INFO_STREAM(ros_cameraInfo_right.K[0]);
  // ros::Time nowTime = ros::Time::now();
  // ros::Time nowTime;
  
  // Create callback functions for right cameras
  const auto frame_size_change_fn_right = [&](
      const int64_t width, const int64_t height, const int64_t step)
  {
    ros_image_right.header.frame_id = image_frame_right;
    ros_image_right.width = static_cast<uint32_t>(width);
    ros_image_right.height = static_cast<uint32_t>(height);
    ros_image_right.encoding = "bgra8";
    ros_image_right.is_bigendian = false;
    ros_image_right.step = static_cast<uint32_t>(step);
    ros_image_right.data.clear();
    ros_image_right.data.resize(ros_image_right.step * ros_image_right.height, 0x00);
  };

  const auto frame_received_fn_right = [&](const BMDCompatibleVideoFrame& video_frame)
  {
    if (video_frame.DataSize() != static_cast<int64_t>(ros_image_right.data.size()))
    {
      throw std::runtime_error("Video frame and ROS image are different sizes");
    }

        //////////////////////////////////////////////////
    //  Important NOTE:
    //  Both cameras shold be set to 1080P60hz, or you
    //  will get a wired format of YUV and abnormal 
    //  colorspace conversion!!
    ///////////////////////////////////////////////////
    // XIAOFENG
    // Resize the ROS output image
    cv::Mat cvImgResized;
    cv::Mat cv_image_bgra;
    
    uint8_t* buffer = video_frame.Data();

    cv_image_bgra = cv::Mat(int(video_frame.Height()), int(video_frame.Width()), CV_8UC4, buffer, video_frame.Step());

    cv::Mat cv_image_bgr;
    cv::cvtColor(cv_image_bgra, cv_image_bgr, cv::COLOR_BGRA2BGR);
    // Write the frame into the file
    // videoRecord_right << cv_image_bgr;

    cv::resize(cv_image_bgra,cvImgResized,cv::Size(960, 540));
    ros_image_resized_right = cv_bridge::CvImage(std_msgs::Header(), "bgra8", cvImgResized).toImageMsg();

    b.Wait();
    // ros_image_resized_right->header.stamp = ros::Time::now();
    // ros_cameraInfo_right.header.stamp = ros::Time::now();
    ros_image_resized_right->header.stamp = timeStep;
    ros_cameraInfo_right.header.stamp = timeStep;
    image_pub_right.publish(ros_image_resized_right);
    cam_pub_right.publish(ros_cameraInfo_right);

  };

  DeckLinkInputDevice input_deviceR(
    RosLoggingFunction, frame_size_change_fn_right, frame_received_fn_right,
    std::move(decklink_devices.at(decklink_device_indexR)));

  // Start video input
  input_deviceR.Start();
  tf_broadcasting(timeStep);

  while (ros::ok())
  {
  }
    // Stop video input
  ROS_INFO("...stopping video input");
  input_deviceR.Stop();
}
}  // namespace
}  // namespace blackmagic_camera_driver

/// @brief Main function
/// @param argc 
/// @param argv 
/// @return 
int main(int argc, char** argv)
{


  ros::init(argc, argv, "video_input_node");
  // return blackmagic_camera_driver::DoMain();

    // Get node handles
  // Calibration of the cameras needs two node handles 
  // (https://answers.ros.org/question/318068/how-to-configure-camerainfomanager-set_camera_info-service-name/)
  ros::NodeHandle nh("MyStereo");
  ros::NodeHandle nhp("~");

  blackmagic_camera_driver::timeStep = ros::Time::now();

  // Discover DeckLink devices
  std::vector<blackmagic_camera_driver::DeckLinkHandle> decklink_devices = blackmagic_camera_driver::GetDeckLinkHardwareDevices();
  if (decklink_devices.size() > 0)
  {
    ROS_INFO("Found [%zu] DeckLink device(s)", decklink_devices.size());
  }
  else
  {
    throw std::runtime_error("No DeckLink device(s) found");
  }

  // Create two threads
  std::thread threadL(blackmagic_camera_driver::leftCamera_thread, std::ref(nh), std::ref(nhp), std::ref(decklink_devices));
  std::thread threadR(blackmagic_camera_driver::rightCamera_thread, std::ref(nh), std::ref(nhp), std::ref(decklink_devices));

  // Wait for the threads to finish
  threadL.join();
  threadR.join();

  // Spin while video callbacks run
  ros::Rate spin_rate(30.0);
  while (ros::ok())
  {
    ros::spinOnce();
    spin_rate.sleep();
  }

}
