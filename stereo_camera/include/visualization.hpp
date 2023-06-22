// OpenCV
#include <opencv2/opencv.hpp>

// ROS Message
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

// boost
#include <boost/optional.hpp>

// math
#include <math.h>

namespace enc = sensor_msgs::image_encodings;

static unsigned char colormap[768] =  //R,G,B
{ 150, 150, 150,
107, 0, 12,
106, 0, 18,
105, 0, 24,
103, 0, 30,
102, 0, 36,
101, 0, 42,
99, 0, 48,
98, 0, 54,
97, 0, 60,
96, 0, 66,
94, 0, 72,
93, 0, 78,
92, 0, 84,
91, 0, 90,
89, 0, 96,
88, 0, 102,
87, 0, 108,
85, 0, 114,
84, 0, 120,
83, 0, 126,
82, 0, 131,
80, 0, 137,
79, 0, 143,
78, 0, 149,
77, 0, 155,
75, 0, 161,
74, 0, 167,
73, 0, 173,
71, 0, 179,
70, 0, 185,
69, 0, 191,
68, 0, 197,
66, 0, 203,
65, 0, 209,
64, 0, 215,
62, 0, 221,
61, 0, 227,
60, 0, 233,
59, 0, 239,
57, 0, 245,
56, 0, 251,
55, 0, 255,
54, 0, 255,
52, 0, 255,
51, 0, 255,
50, 0, 255,
48, 0, 255,
47, 0, 255,
46, 0, 255,
45, 0, 255,
43, 0, 255,
42, 0, 255,
41, 0, 255,
40, 0, 255,
38, 0, 255,
37, 0, 255,
36, 0, 255,
34, 0, 255,
33, 0, 255,
32, 0, 255,
31, 0, 255,
29, 0, 255,
28, 0, 255,
27, 0, 255,
26, 0, 255,
24, 0, 255,
23, 0, 255,
22, 0, 255,
20, 0, 255,
19, 0, 255,
18, 0, 255,
17, 0, 255,
15, 0, 255,
14, 0, 255,
13, 0, 255,
11, 0, 255,
10, 0, 255,
9, 0, 255,
8, 0, 255,
6, 0, 255,
5, 0, 255,
4, 0, 255,
3, 0, 255,
1, 0, 255,
0, 4, 255,
0, 10, 255,
0, 16, 255,
0, 22, 255,
0, 28, 255,
0, 34, 255,
0, 40, 255,
0, 46, 255,
0, 52, 255,
0, 58, 255,
0, 64, 255,
0, 70, 255,
0, 76, 255,
0, 82, 255,
0, 88, 255,
0, 94, 255,
0, 100, 255,
0, 106, 255,
0, 112, 255,
0, 118, 255,
0, 124, 255,
0, 129, 255,
0, 135, 255,
0, 141, 255,
0, 147, 255,
0, 153, 255,
0, 159, 255,
0, 165, 255,
0, 171, 255,
0, 177, 255,
0, 183, 255,
0, 189, 255,
0, 195, 255,
0, 201, 255,
0, 207, 255,
0, 213, 255,
0, 219, 255,
0, 225, 255,
0, 231, 255,
0, 237, 255,
0, 243, 255,
0, 249, 255,
0, 255, 255,
0, 255, 249,
0, 255, 243,
0, 255, 237,
0, 255, 231,
0, 255, 225,
0, 255, 219,
0, 255, 213,
0, 255, 207,
0, 255, 201,
0, 255, 195,
0, 255, 189,
0, 255, 183,
0, 255, 177,
0, 255, 171,
0, 255, 165,
0, 255, 159,
0, 255, 153,
0, 255, 147,
0, 255, 141,
0, 255, 135,
0, 255, 129,
0, 255, 124,
0, 255, 118,
0, 255, 112,
0, 255, 106,
0, 255, 100,
0, 255, 94,
0, 255, 88,
0, 255, 82,
0, 255, 76,
0, 255, 70,
0, 255, 64,
0, 255, 58,
0, 255, 52,
0, 255, 46,
0, 255, 40,
0, 255, 34,
0, 255, 28,
0, 255, 22,
0, 255, 16,
0, 255, 10,
0, 255, 4,
2, 255, 0,
8, 255, 0,
14, 255, 0,
20, 255, 0,
26, 255, 0,
32, 255, 0,
38, 255, 0,
44, 255, 0,
50, 255, 0,
56, 255, 0,
62, 255, 0,
68, 255, 0,
74, 255, 0,
80, 255, 0,
86, 255, 0,
92, 255, 0,
98, 255, 0,
104, 255, 0,
110, 255, 0,
116, 255, 0,
122, 255, 0,
128, 255, 0,
133, 255, 0,
139, 255, 0,
145, 255, 0,
151, 255, 0,
157, 255, 0,
163, 255, 0,
169, 255, 0,
175, 255, 0,
181, 255, 0,
187, 255, 0,
193, 255, 0,
199, 255, 0,
205, 255, 0,
211, 255, 0,
217, 255, 0,
223, 255, 0,
229, 255, 0,
235, 255, 0,
241, 255, 0,
247, 255, 0,
253, 255, 0,
255, 251, 0,
255, 245, 0,
255, 239, 0,
255, 233, 0,
255, 227, 0,
255, 221, 0,
255, 215, 0,
255, 209, 0,
255, 203, 0,
255, 197, 0,
255, 191, 0,
255, 185, 0,
255, 179, 0,
255, 173, 0,
255, 167, 0,
255, 161, 0,
255, 155, 0,
255, 149, 0,
255, 143, 0,
255, 137, 0,
255, 131, 0,
255, 126, 0,
255, 120, 0,
255, 114, 0,
255, 108, 0,
255, 102, 0,
255, 96, 0,
255, 90, 0,
255, 84, 0,
255, 78, 0,
255, 72, 0,
255, 66, 0,
255, 60, 0,
255, 54, 0,
255, 48, 0,
255, 42, 0,
255, 36, 0,
255, 30, 0,
255, 24, 0,
255, 18, 0,
255, 12, 0,
255,  6, 0,
255,  0, 0,
};

static void dispMouseClick(int event, int x, int y, int flags, void* ptr) 
{
    if (event == cv::EVENT_LBUTTONDOWN) {
        cv::Mat* dispMat = (cv::Mat*)ptr;
        // According to https://stackoverflow.com/questions/25642532/opencv-pointx-y-represent-column-row-or-row-column
        // mat.at<type>(row,column) or mat.at<type>(cv::Point(x,y)) to access the same point if x=column and y=row
		float disp_at_pixel = dispMat->at<float>(cv::Point(x,y));
        ROS_INFO("The value of diparity is %f\n", disp_at_pixel);
	}
}

cv::Point firstPoint = cv::Point(0,0);
static void leftMouseClick(int event, int x, int y, int flags, void* ptr) 
{
    if (event == cv::EVENT_LBUTTONDOWN) {
        cv::Point* point = (cv::Point*)ptr;
        if(point->x == 0){
            ROS_INFO("The pixel is (%d,%d), this is the first point\n", x, y);
            point->x = x;
            point->y = y;
        }
        else{
            float distanceInPixel = sqrt(pow(point->x-x , 2) + pow(point->y-y , 2));
            ROS_INFO("The pixel and the distance are (%d,%d), %f. \n", x, y, distanceInPixel);
            *point = cv::Point(0,0);
        }
	}
}

static cv::Mat_<cv::Vec3b> disp2Color(cv::Mat_<float> dmat,float min_disparity, float multiplier, int height, int width)
{
    cv::Mat_<cv::Vec3b> disparity_color_avg;
    disparity_color_avg.create(height, width);
    for (int row = 0; row < disparity_color_avg.rows; ++row) 
    {
        const float* d_cout = dmat[row];
        for (int col = 0; col < disparity_color_avg.cols; ++col) 
        {
            int index = (d_cout[col] - min_disparity) * multiplier + 0.5;
            index = std::min(255, std::max(0, index));
            // Fill as BGR
            disparity_color_avg(row, col)[2] = colormap[3*index + 0]; //R
            disparity_color_avg(row, col)[1] = colormap[3*index + 1]; //G
            disparity_color_avg(row, col)[0] = colormap[3*index + 2]; //B
        }
    }

    return disparity_color_avg;
}

static cv::Mat filterMasking(cv::Mat dmat, cv::Mat pure_bg, int th = 5)
{
    cv::Mat mask, diff; 
	cv::absdiff(dmat, pure_bg, diff);
	cv::threshold(diff, mask, th, 1, cv::THRESH_BINARY);
    
    return mask;
}

class Visualization
{
    public:
    sensor_msgs::ImageConstPtr last_left_msg__;
    cv::Mat last_left_image_;

    cv::Mat_<cv::Vec3b> disparity_color_, disparity_color_avg; //hoga.at<cv::Vec3b>(y, x)[0] = xx; //B ..[1] G ..[2] R
    stereo_msgs::DisparityImageConstPtr disparity_msg; 
    cv::Mat_<float> dispMat, disp_cout, dispMat_avg;

    // Get disparity pixel value by snapshot
    int dispCaptureEnable = 0;
    cv::Mat snapshot;

    void setDisparityMsg(const stereo_msgs::DisparityImageConstPtr& msg){disparity_msg = msg;}

//    Visualization(){disp_cout = cv::Mat(disparity_msg->image.height, disparity_msg->image.width, CV_32FC1, cv::Scalar(0));}
    ~Visualization(){cv::destroyAllWindows();}

    void showDisparity();

    void showStreamLeft(const sensor_msgs::ImageConstPtr& msg);

    float checkPixelValue(int row, int col);

    void enableDispCapture();

    private:
    int keyVal;
    int img_counter = -1; 
    float min_disparity,max_disparity,multiplier,bg_value;
    cv::Mat pure_bg, mask_cout;
};

void Visualization::showDisparity()
{
    // Colormap and display the disparity image
    min_disparity = disparity_msg->min_disparity;
    max_disparity = disparity_msg->max_disparity;
    multiplier = 255.0f / (max_disparity - min_disparity);

    assert(disparity_msg->image.encoding == enc::TYPE_32FC1);

    // Create a disparity mat and copy it
    const cv::Mat_<float> dmat(disparity_msg->image.height, disparity_msg->image.width,
                                (float*)&disparity_msg->image.data[0], disparity_msg->image.step);
    dispMat = dmat;
    bg_value = dmat(0,disparity_msg->image.width-1); // (ROW, COLUMN)
	pure_bg = cv::Mat(disparity_msg->image.height, disparity_msg->image.width,CV_32FC1, cv::Scalar(bg_value));

    disparity_color_ = disp2Color(dmat, min_disparity, multiplier, disparity_msg->image.height, disparity_msg->image.width);

    // Show disparity
    cv::imshow("disparity", disparity_color_);

}

float Visualization::checkPixelValue(int row, int col)
{
    const float d = dispMat(row, col);
    return d; 
}

void Visualization::showStreamLeft(const sensor_msgs::ImageConstPtr& msg)
{
    last_left_msg__ = msg; 
    last_left_image_ = cv_bridge::toCvShare(last_left_msg__, "bgr8")->image;
    cv::imshow("streamLeft", last_left_image_);
    cv::waitKey(1);
    cv::setMouseCallback("streamLeft", leftMouseClick, &firstPoint);
}

void Visualization::enableDispCapture()
{
    keyVal = cv::waitKey(1) & 0xFF;
    // If Z is pushed then start accumulating the img
    if (keyVal == 90){ // the key is 'z'
        img_counter = 0;
        disp_cout = cv::Mat(disparity_msg->image.height, disparity_msg->image.width,
                            CV_32FC1, cv::Scalar(0));
        mask_cout = cv::Mat(disparity_msg->image.height, disparity_msg->image.width,
                            CV_32FC1, cv::Scalar(0));
        }
    if (img_counter!=-1)
    {
        if (img_counter<10){
            // ROS_INFO("Bug is here!");
            mask_cout = mask_cout + filterMasking(dispMat, pure_bg);
            disp_cout = disp_cout + ( dispMat - bg_value );
            img_counter = img_counter + 1;
        }
        else{
            // enable the checking of disprity_avg and show the figure
            // Explanation: compute the average of the pixel value by (sum of the value in 100 picture)/(the times it appeared)
            cv::threshold(mask_cout, mask_cout, 2, 0, cv::THRESH_TOZERO); // If the value of mask lower than 10 then set 0
            cv::divide(disp_cout, mask_cout, disp_cout);
            disp_cout = disp_cout + bg_value; 
            dispMat_avg = disp_cout.clone();
            disparity_color_avg = disp2Color(dispMat_avg, min_disparity, multiplier, disparity_msg->image.height, disparity_msg->image.width);
            cv::imshow("Snapshot", disparity_color_avg);

            // initialize the counter and dispMat
            img_counter = -1;
            disp_cout = cv::Mat(disparity_msg->image.height, disparity_msg->image.width,
                            CV_32FC1, cv::Scalar(0));
        }
    }
    cv::setMouseCallback("Snapshot", dispMouseClick, &dispMat_avg);
}