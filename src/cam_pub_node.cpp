#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "vcap/vcap.hpp"
#include "vcap/device.hpp"

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;

class CamCap{
    public:
        CamCap(){
            vcap::device vd;
            int w, h;
            unsigned char *rgb = NULL;
            //VideoCapture cap(0);
 
            if (!vd.open_device("/dev/video0")) {
                std::cout << "Device 0 not openned...\n";
            }

            if( !vd.set_frame_size(1280,480) ) {
                printf("Frame size [1280x480] is not avaliable!\n");
            }

            vd.get_image_dimension(&w, &h);
            rgb = (unsigned char *)malloc(sizeof(unsigned char) * w * h * 3);

            /*
            if(!cap.isOpened())
            {
                cout << "Error opening video stream or file" << endl;
            }
            */

            //Mat frame;
            int rate = 30;

            //create a node handle: it is reference assigned to a new node
            //every node created must has a reference to create publishers and subscibers
            ros::NodeHandle n;
            //create a image_transport publisher with topic "raw_image"
            image_transport::ImageTransport it_(n);
            //advertise are used to create Publisher topic name and queue size
            image_transport::Publisher image_pub_ = it_.advertise("/raw_image", 1);

            cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
            //Define the frequency that message are send. Could be the frame rate of the camera
            ros::Rate loop_rate(rate); //10 hz

            vd.start();

            while(ros::ok()){//keep spinning loop until user presses Ctrl+c

                vd.grabRGB(rgb);

                Mat frame(h, w, CV_8UC3, rgb);

                //cap >> frame;

                if(frame.empty())
                    break;

                ros::Time time = ros::Time::now();
                cv_ptr->encoding = "rgb8";
                cv_ptr->header.stamp = time;
                cv_ptr->header.frame_id = "/raw_image";

                cv_ptr->image = frame;
                //Publish the message
                image_pub_.publish(cv_ptr->toImageMsg());

                ROS_INFO("ImageMsg Sent.");
                ROS_INFO("Subscribers: %d", image_pub_.getNumSubscribers());

                //Need to call this function to allow ROS to process incoming messages
                //Handles the events and returns immediately
                ros::spinOnce();
                //Sleep for the rest of the cycle, to enforce the loop rate
                loop_rate.sleep();
            }
            vd.stop();

            vd.close_device();

            std::cout << "\n";

            free(rgb);
            //cap.release();

        }

        ~CamCap(){
            
            destroyAllWindows();
        }

};

int main(int argc, char **argv)
{
    //Initialize new ROS node named "cam_node"
    ros::init(argc, argv, "cam_node");
    CamCap camcap;
    //Main loop for ROS that repeatedly calls SpinOnce until the node is shut down
    ros::spin();
    return 0;

}
