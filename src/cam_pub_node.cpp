#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "vcap/vcap.hpp"
#include "vcap/device.hpp"
#include <cstdio>
#include "json/json.hpp"

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;
using json = nlohmann::json;




class CamCap{
    private:
        vcap::device vd;
        json configs;

    
    public:
        CamCap(){
            
            int w, h;
            unsigned char * rgb = NULL;
            int rate = 30;
            
            CamConfig();

            vd.get_image_dimension(&w, &h);
            rgb = (unsigned char *)malloc(sizeof(unsigned char) * w * h * 3);

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
            
            std::cout << "\n";
            free(rgb);

        }

        ~CamCap(){
            vd.stop();
            vd.close_device();
        }

        void CamConfig(){

            // enum control default
            struct v4l2_queryctrl qctrl;

            if (!vd.open_device("/dev/video1")) {
                std::cout << "Device not openned...\n";
            }

            if( !vd.set_frame_size(320,240) ) {
                std::cout << "Frame size is not avaliable!\n";
            }
    
            if(!loadFIle()){
                std::cout << "Can not open file\n";
            }else{
                auto configArray = configs["config"];
                //std::cout << "Size" << configArray.size();
                for(const auto &prop : configArray){
                    //std::cout << "Name" << prop["name"];
                    int type = prop["type"];
                    int id = prop["id"];
                    int value = prop["value"];
         
                    qctrl.id = id;                    
                    qctrl.type = type;

                    if(vcap::set_control(vd.get_fd(), qctrl.id, value) < 0)
                        std::cout << "ERROR seting "<< prop["name"]<<"\n";
                    /*
                    switch(type){
                        case V4L2_CTRL_TYPE_INTEGER:
                            break;
                        case V4L2_CTRL_TYPE_BOOLEAN:
                            break;
                        case V4L2_CTRL_TYPE_MENU:
                            break;
                        default:
				            break;
                    }
                    */

                }

            }

        }

        bool loadFIle(){
            try {
                string file_path =  ros::package::getPath("cam_pub");
                file_path = file_path+"/src/cam_config.json";
                ifstream file(file_path);
                if (file.is_open()) {
                    file >> configs;
                    file.close();
                } else return false;
            } catch (exception &e) {
                string error = e.what();
                if (error.find("parse_error") != string::npos) return 2;
            }
            return true;

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
