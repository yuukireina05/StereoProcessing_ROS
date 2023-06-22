#include <ros/ros.h>

// OpenCV
#include <opencv2/opencv.hpp>

// ROS Message
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cv_bridge/cv_bridge.h>

// tf frame
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

// #include <image_proc/processor.h>
#include <image_geometry/stereo_camera_model.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud.h>

class replayPointCloud
{
    public:
    // Get disparity pixel value by snapshot
    bool dispFlag = false;
    bool leftImgFlag = false;

    // void setDisparityMsg(const stereo_msgs::DisparityImage& msg){disparity = msg;}
    void setDisparityMsg(const stereo_msgs::DisparityImagePtr& msg){disp_msg = msg;}
    void setDisparityFlag(){dispFlag = true;}

    void setLeftImgMsg(const sensor_msgs::ImageConstPtr& msg){l_image_msg = msg;}
    void setLeftImgFlag(){leftImgFlag = true;}

    void convertLeftImg();
    void processPoints2(ros::Publisher pub);
    bool isValidPoint(const cv::Vec3f& pt, const double MISSING_Z);

    void projectDisparityImageTo3d(const cv::Mat& disparity, cv::Mat& out3D);

    void tfbroadcaster();

    private:
    const double MISSING_Z = 10000;
    cv::Mat left_image_;

    // stereo_msgs::DisparityImage disparity; 
    sensor_msgs::ImageConstPtr l_image_msg = boost::make_shared<sensor_msgs::Image>();
    stereo_msgs::DisparityImagePtr disp_msg = boost::make_shared<stereo_msgs::DisparityImage>();
    // cv::Mat_<float> dispMat;

};

bool replayPointCloud::isValidPoint(const cv::Vec3f& pt, const double MISSING_Z)
{
  // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
  // and zero disparities (point mapped to infinity).
  return pt[2] != MISSING_Z && !std::isinf(pt[2]);
}

// Reproject image to 3D
void replayPointCloud::projectDisparityImageTo3d(const cv::Mat& disparity, cv::Mat& out3D)
{
    // std::cout <<  "Converting from disparity to point cloud"<< std::endl;

	CV_Assert(disparity.type() == CV_32F && !disparity.empty());

	// 3-channel matrix for containing the reprojected 3D world coordinates
	out3D = cv::Mat::zeros(disparity.size(), CV_32FC3);

    // find the invalid value in disparity image by finding the max&min value in it
    double minVal; 
    double maxVal; 
    cv::minMaxLoc(disparity, &minVal, &maxVal);

	// Transforming a single-channel disparity map to a 3-channel image representing a 3D surface
	for (int i = 0; i < disparity.rows; i++)
	{
		const float* disp_ptr = disparity.ptr<float>(i);
		cv::Vec3f* out3D_ptr = out3D.ptr<cv::Vec3f>(i);

		for (int j = 0; j < disparity.cols; j++)
		{
            // Define scalling coefficient as 0.1
            float scaling = 0.1;

			cv::Vec3f& point = out3D_ptr[j];
			// point[0] = (static_cast<float>(j)+Q03)*0.03846*scaling;
			// point[1] = (static_cast<float>(i)+Q13)*0.03846*scaling;
            point[0] = (static_cast<float>(j)-960/2)*0.03846*scaling;
			point[1] = (static_cast<float>(i)-540/2)*0.03846*scaling;

            // a simple way to eliminate the invalid value by finding max&min value. 
            if((disp_ptr[j] == float(maxVal))||(disp_ptr[j] == float(minVal)))
                point[2] = 10000; 
            else
            point[2] = (1/6.4 * (disp_ptr[j]+67.5))*scaling;
		}
	}

    // std::cout <<  "Converted from disparity to point cloud"<< std::endl;
}

void replayPointCloud::convertLeftImg()
{
    // std::cout << "convertLeftImg is running."<< std::endl;

    if(replayPointCloud::leftImgFlag)
    {
        // std::cout <<  "The leftImgFlag is true" << std::endl;
        replayPointCloud::leftImgFlag = false;
        replayPointCloud::left_image_ = cv_bridge::toCvShare(replayPointCloud::l_image_msg, "bgra8")->image;
    }
    
}

void replayPointCloud::processPoints2(ros::Publisher pub)
{
    
    if(replayPointCloud::leftImgFlag)
    {
    if(replayPointCloud::dispFlag)
    {
    // Initializing variables
    stereo_msgs::DisparityImagePtr disp_msg = replayPointCloud::disp_msg;
    const double& MISSING_Z = replayPointCloud::MISSING_Z;
    sensor_msgs::ImageConstPtr& l_image_msg = replayPointCloud::l_image_msg;
    sensor_msgs::PointCloud2Ptr points_msg = boost::make_shared<sensor_msgs::PointCloud2>(); 

    // Calculate point cloud
    const sensor_msgs::Image& dimage = disp_msg->image;
    const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
    cv::Mat_<cv::Vec3f> points_mat_; 
    projectDisparityImageTo3d(dmat, points_mat_);
    cv::Mat_<cv::Vec3f> mat = points_mat_;

    
    // Fill in new PointCloud2 message (2D image-like layout)
    points_msg->header = disp_msg->header;
    points_msg->height = mat.rows;
    points_msg->width  = mat.cols;
    points_msg->is_bigendian = false;
    points_msg->is_dense = false; // there may be invalid points

    sensor_msgs::PointCloud2Modifier pcd_modifier(*points_msg);
    pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    sensor_msgs::PointCloud2Iterator<float> iter_x(*points_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*points_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*points_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*points_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*points_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*points_msg, "b");

    float bad_point = std::numeric_limits<float>::quiet_NaN ();
    for (int v = 0; v < mat.rows; ++v)
    {
        for (int u = 0; u < mat.cols; ++u, ++iter_x, ++iter_y, ++iter_z)
        {
        if (isValidPoint(mat(v,u), MISSING_Z))
        {
            // x,y,z
            *iter_x = mat(v, u)[0];
            *iter_y = mat(v, u)[1];
            *iter_z = mat(v, u)[2];
        }
        else
        {
            *iter_x = *iter_y = *iter_z = bad_point;
        }
        }
    }

    // Fill in color
    namespace enc = sensor_msgs::image_encodings;
    const std::string& encoding = l_image_msg->encoding;
    if (encoding == enc::MONO8)
    {
        const cv::Mat_<uint8_t> color(l_image_msg->height, l_image_msg->width,
                                    (uint8_t*)&l_image_msg->data[0],
                                    l_image_msg->step);
        for (int v = 0; v < mat.rows; ++v)
        {
        for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b)
        {
            uint8_t g = color(v,u);
            *iter_r = *iter_g = *iter_b = g;
        }
        }
    }
    else if (encoding == enc::RGB8)
    {
        const cv::Mat_<cv::Vec3b> color(l_image_msg->height, l_image_msg->width,
                                        (cv::Vec3b*)&l_image_msg->data[0],
                                        l_image_msg->step);
        for (int v = 0; v < mat.rows; ++v)
        {
        for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b)
        {
            const cv::Vec3b& rgb = color(v,u);
            *iter_r = rgb[0];
            *iter_g = rgb[1];
            *iter_b = rgb[2];
        }
        }
    }
    else if (encoding == enc::RGBA8)
    {
        const cv::Mat_<cv::Vec4b> color(l_image_msg->height, l_image_msg->width,
                                        (cv::Vec4b*)&l_image_msg->data[0],
                                        l_image_msg->step);
        for (int v = 0; v < mat.rows; ++v)
        {
        for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b)
        {
            const cv::Vec4b& rgba = color(v,u);
            *iter_r = rgba[0];
            *iter_g = rgba[1];
            *iter_b = rgba[2];
        }
        }
    }
    else if (encoding == enc::BGR8)
    {
        const cv::Mat_<cv::Vec3b> color(l_image_msg->height, l_image_msg->width,
                                        (cv::Vec3b*)&l_image_msg->data[0],
                                        l_image_msg->step);
        for (int v = 0; v < mat.rows; ++v)
        {
        for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b)
        {
            const cv::Vec3b& bgr = color(v,u);
            *iter_r = bgr[2];
            *iter_g = bgr[1];
            *iter_b = bgr[0];
        }
        }
    }
    else if (encoding == enc::BGRA8)
    {
        const cv::Mat_<cv::Vec4b> color(l_image_msg->height, l_image_msg->width,
                                        (cv::Vec4b*)&l_image_msg->data[0],
                                        l_image_msg->step);
        for (int v = 0; v < mat.rows; ++v)
        {
        for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b)
        {
            const cv::Vec4b& bgra = color(v,u);
            *iter_r = bgra[2];
            *iter_g = bgra[1];
            *iter_b = bgra[0];
        }
        }
    }
    else
    {
        ROS_WARN_THROTTLE(30, "Could not fill color channel of the point cloud, "
                            "unsupported encoding '%s'", encoding.c_str());
    }
    
    // Reset the flag of the disparity
    replayPointCloud::dispFlag = false;
    replayPointCloud::leftImgFlag = false;
    pub.publish (points_msg);

    points_msg.reset();
    disp_msg.reset();
    l_image_msg.reset();
    }
    }
}


void replayPointCloud::tfbroadcaster()
{
    // ///////////////////////////
    // // tf_frame
    // ///////////////////////////
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "map";
    static_transformStamped.child_frame_id = "video_input_left[1]";
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 3;
    tf2::Quaternion quat;
    // quat.setRPY(0, 0, 0);
    static_transformStamped.transform.rotation.x = -1;
    static_transformStamped.transform.rotation.y = 0;
    static_transformStamped.transform.rotation.z = 0;
    static_transformStamped.transform.rotation.w = 0;
    static_broadcaster.sendTransform(static_transformStamped);
}