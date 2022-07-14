#include <ros/ros.h>
// sensor msgs
// ros image message from simulator
#include <sensor_msgs/Image.h>
// opencv bridge for convert Image to cv::Mat
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>

// opencv include
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


// custom ros srv
#include <krti2022/Activate.h>
#include <krti2022/ActivateResponse.h>

// custom ros msgs
#include <krti2022/DResult.h>
#include <krti2022/QRResult.h>

//math
#include <cmath>
#include <random>
#include <iostream>

#include <nav_msgs/Odometry.h>
auto VERBOSE = false;

class Vision{
    ros::NodeHandle nh;
    ros::ServiceServer activate_qr = nh.advertiseService("/vision/activate/qr", Vision::activate_QR);
    bool sim = false;
    int Fcamera_index = 0;
    int Dcamera_index = 1;
    
public:
    Vision(){
        ros::param::get("/vision/verbose", VERBOSE);
        ros::param::get("/vision/use_sim", sim);

        if(VERBOSE)
            ROS_INFO("Vision node is ready");
        


    }
    ~Vision(){

    }
    void imageCallback(const sensor_msgs::ImageConstPtr& msg){

    }
    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg){

    }
    void odomCallback(const nav_msgs::OdometryConstPtr& msg){

    }
    void activate_QR(krti2022::ActivateRequest &req, krti2022::ActivateResponse &res){

    }

};

int main(int argc, char** argv){
    ros::init(argc, argv, "vision");
    Vision vision;
    ros::Time last;
    last =  ros::Time::now();
    while(ros::ok()){

        ros::spinOnce();
    }

    ros::spin();

}
