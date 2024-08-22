#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <string.h>
#include "std_msgs/String.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/video/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/calib3d.hpp>

#include <cmath> 

using namespace std;

static const std::string OPENCV_WINDOW = "Camera";

class Aruco_set
{
public:
    
    ros::NodeHandle nh_;
 
    ros::Subscriber sensor_data_sub;
    ros::Subscriber sub;
    cv_bridge::CvImagePtr cv_ptr;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    
    
    tf::TransformBroadcaster my_transformBroadcaster;
    tf::TransformListener listener_;


  

    Aruco_set() : it_(nh_)
    {
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &Aruco_set::imageCb, this);
      cv::namedWindow(OPENCV_WINDOW);
        
        
    }
    ~Aruco_set()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    

       void imageCb(const sensor_msgs::ImageConstPtr& msg)
        {
            try
            {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat im = cv_ptr->image;
            }
            catch (cv_bridge::Exception& e)
            {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
            }

            // Update GUI Window
           cv::imshow(OPENCV_WINDOW, cv_ptr->image);
            cv::waitKey(3);

        }

    
    void aruco_get() {
        
        float aruco_x, aruco_y, aruco_z;
        float rot_x, rot_y, rot_z;
        float markerSize;
        ros::Rate loop_rate(10);
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

        double m[3][3] = {{274.139508945831, 0, 141.184472810944}, {0, 275.741846757374, 106.693773654172}, {0, 0, 1}};
        double k[5] = {-0.0870160932911717, 0.128210165050533, 0.003379500659424, -0.00106205540818586, 0};
        cv::Mat cameraMatrix(3, 3, CV_64F, m);
        cv::Mat distCoeffs(1, 5, CV_64F, k);
        cv::Mat imageCopy;
        cv::Mat image;
       


         ROS_INFO("Enter p to start aruco detection");
        
        char response;
              cin >> response;
        if ((response == 'p' || response == 'P')) {
           while (ros::ok()) {
            
            ros::spinOnce(); 
             
              
            
            cv::Mat im = cv_ptr->image;
            im.copyTo(imageCopy);
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f> > corners, rejectedCandidates;
            cv::aruco::detectMarkers(im, dictionary, corners, ids, parameters, rejectedCandidates);
            tf :: Transform my_transform ;
            tf :: Quaternion q ;
            
            if (ids.size() > 0) {
                cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

                std::vector<cv::Vec3d> rvecs, tvecs, objPoints;
                cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs, objPoints);
                
                for(int i=0; i<ids.size(); i++) {
                    cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
                    ROS_INFO("3D position of marker %d is: %f, %f, %f\n", i + 1, tvecs[i][0], tvecs[i][1], tvecs[i][2]);
                }
                /*cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[0], tvecs[0], 0.1);
                    ROS_INFO("3D position of marker %d is: %f, %f, %f\n", 0 + 1, tvecs[0][0], tvecs[0][1], tvecs[0][2]);*/
                aruco_x = 2*tvecs[0][0];
                aruco_y = 2*tvecs[0][1];
                aruco_z = 4*tvecs[0][1];

                my_transform.setOrigin(tf::Vector3(aruco_x, aruco_y, aruco_z));
                my_transformBroadcaster.sendTransform(tf::StampedTransform(my_transform, ros::Time::now() ,"world","map"));

        }
            ROS_INFO_STREAM("aruco_x ="<<aruco_x);
            ROS_INFO_STREAM("aruco_y ="<<aruco_y);
            ROS_INFO_STREAM("aruco_z ="<<aruco_z);


            cv::imshow("3D marker position", imageCopy);

            cv::waitKey(30);
            }
    } 
    
}

};
int main(int argc, char** argv)
{ 
    ros::init(argc, argv, "aruco_marker");
    
    Aruco_set ic;

    ros::Rate loop_rate(10);
    
    while (ros::ok() ) {
        
        

        ros::spinOnce();
        ic.aruco_get();
        loop_rate.sleep();
    }
    
 
    return 0;

}
