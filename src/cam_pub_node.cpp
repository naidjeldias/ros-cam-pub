#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include "vcap/vcap.hpp"
#include "vcap/device.hpp"
#include <cstdio>
#include "json/json.hpp"

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;
using json = nlohmann::json;
const char * devname = "/dev/video0";
const char * devType = "mono";

class CamCap{
    private:
        vcap::device vd;
        json configs;
        unsigned char * rgb = NULL;
        cv::Mat M1l,M2l,M1r,M2r;
        
    public:

        //ROS node Handle
        ros::NodeHandle node_;

        //shared image message
        sensor_msgs::Image img_;

        //parameters
        std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_, devtype_;
        int image_width_, image_height_, framerate_, exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
            white_balance_, gain_;
        bool autofocus_, autoexposure_, auto_white_balance_;
        boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;



        CamCap(): node_("~")
        {

             // grab the parameters
            //node_.param("video_device", video_device_name_, std::string(devname));
            node_.getParam("/cam_pub_node/video_device", video_device_name_);
            node_.getParam("/cam_pub_node/devType", devtype_);
            node_.getParam("/cam_pub_node/image_width", image_width_);
            node_.getParam("/cam_pub_node/image_height", image_height_);
            node_.getParam("/cam_pub_node/autofocus", autofocus_);
            node_.getParam("/cam_pub_node/focus", focus_);
            node_.getParam("/cam_pub_node/autoexposure", autoexposure_);
            node_.getParam("/cam_pub_node/exposure", exposure_);
            node_.getParam("/cam_pub_node/auto_white_balance", auto_white_balance_);
            node_.getParam("/cam_pub_node/white_balance", white_balance_);


            // node_.param("brightness", brightness_, -1); //0-255, -1 "leave alone"
            // node_.param("contrast", contrast_, -1); //0-255, -1 "leave alone"
            // node_.param("saturation", saturation_, -1); //0-255, -1 "leave alone"
            // node_.param("sharpness", sharpness_, -1); //0-255, -1 "leave alone"
            // //node_.param("devType", devtype_, std::string(devType));
           
            
            // // possible values: mmap, read, userptr
            // // node_.param("io_method", io_method_name_, std::string("mmap"));
            // node_.param("image_width", image_width_, 640);
            // node_.param("image_height", image_height_, 480);
            // node_.param("framerate", framerate_, 30);
            // // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
            // node_.param("pixel_format", pixel_format_name_, std::string("rgb"));
            // // enable/disable autofocus
            // node_.param("autofocus", autofocus_, false);
            // node_.param("focus", focus_, -1); //0-255, -1 "leave alone"
            // // enable/disable autoexposure
            // node_.param("autoexposure", autoexposure_, false);
            // node_.param("exposure", exposure_, 100);
            // node_.param("gain", gain_, -1); //0-100?, -1 "leave alone"
            // // enable/disable auto white balance temperature
            // node_.param("auto_white_balance", auto_white_balance_, false);
            // node_.param("white_balance", white_balance_, 4000);

            // load the camera info
            node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
            node_.param("camera_name", camera_name_, std::string("head_camera"));
            node_.param("camera_info_url", camera_info_url_, std::string(""));
            cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));

            // check for default camera info
            if (!cinfo_->isCalibrated())
            {
            cinfo_->setCameraName(video_device_name_);
                sensor_msgs::CameraInfo camera_info;
                camera_info.header.frame_id = img_.header.frame_id;
                camera_info.width = image_width_;
                camera_info.height = image_height_;
                cinfo_->setCameraInfo(camera_info);
            }

            std::string file_path = "/src/stereo.yaml"; 
            string ros_path =  ros::package::getPath("cam_pub");

            cv::FileStorage fsSettings(ros_path+file_path, cv::FileStorage::READ);
            if(!fsSettings.isOpened())
            {
                cerr << "ERROR: Wrong path to settings" << endl;
            }else{

                cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
                fsSettings["LEFT.K"] >> K_l;
                fsSettings["RIGHT.K"] >> K_r;

                fsSettings["LEFT.P"] >> P_l;
                fsSettings["RIGHT.P"] >> P_r;

                fsSettings["LEFT.R"] >> R_l;
                fsSettings["RIGHT.R"] >> R_r;

                fsSettings["LEFT.D"] >> D_l;
                fsSettings["RIGHT.D"] >> D_r;

                int rows_l = fsSettings["LEFT.height"];
                int cols_l = fsSettings["LEFT.width"];
                int rows_r = fsSettings["RIGHT.height"];
                int cols_r = fsSettings["RIGHT.width"];

                // cout << "K_l = "<< endl << " "  << K_l << endl << endl;
                // cout << "K_r = "<< endl << " "  << K_r << endl << endl;

                if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                        rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
                {
                    cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
                    
                }else{
                     //obtendo as novas matrizes de projeção das imagens retificadas
                    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
                    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
                }
                
            }

            if (!vd.open_device(video_device_name_.c_str())) {
                std::cout << "Device not openned...\n";
            }else{
                CamConfig();
            }
            
            vd.get_image_dimension(&image_width_, &image_height_);
            rgb = (unsigned char *)malloc(sizeof(unsigned char) * image_width_ * image_height_ * 3);
        }

        ~CamCap(){
            vd.stop();
            vd.close_device();
        }

        void CamConfig(){

            // enum control default
            struct v4l2_queryctrl qctrl;

            if( vcap::enum_control_default( vd.get_fd(), &qctrl, true) ) {
                // printf("\n# Supported controls\n");
                // printf("----------------------------------\n");
                do {
                    // printf("id: %15d; Name: %30s; Type: %5d \n", qctrl.id, qctrl.name, qctrl.type);
                    std::string sName(reinterpret_cast<char*>(qctrl.name));
                    if(sName == "Focus, Auto" && autofocus_){
                        if(vcap::set_control(vd.get_fd(), qctrl.id, autofocus_) < 0)
                            std::cout << "ERROR seting "<< sName <<"\n";
                    }
                    if(sName == "Focus (absolute)" && focus_ >= 0){
                        if(vcap::set_control(vd.get_fd(), qctrl.id, focus_) < 0)
                            std::cout << "ERROR seting "<< sName <<"\n";
                    }
                    if(sName == "Exposure, Auto" && autoexposure_ ){
                        if(vcap::set_control(vd.get_fd(), qctrl.id, autoexposure_) < 0)
                            std::cout << "ERROR seting "<< sName <<"\n";
                    }
                    if(sName == "Exposure (Absolute)" && exposure_>=0){
                        if(vcap::set_control(vd.get_fd(), qctrl.id, exposure_) < 0)
                            std::cout << "ERROR seting "<< sName <<"\n";
                    }
                    if(sName == "White Balance Temperature, Auto" && auto_white_balance_){
                        if(vcap::set_control(vd.get_fd(), qctrl.id, auto_white_balance_) < 0)
                            std::cout << "ERROR seting "<< sName <<"\n";
                    }

                    if(sName == "White Balance Temperature" && white_balance_>=0){
                        if(vcap::set_control(vd.get_fd(), qctrl.id, white_balance_) < 0)
                            std::cout << "ERROR seting "<< sName <<"\n";
                    }
                } while (vcap::enum_control_default( vd.get_fd(), &qctrl, false) );
            }
    
            // if(!loadFIle(file_path)){
            //     std::cout << "Can not open file\n";
            // }else{
            //     auto configArray = configs["config"];
            //     //std::cout << "Size" << configArray.size();
            //     for(const auto &prop : configArray){
            //         //std::cout << "Name" << prop["name"];
            //         int type = prop["type"];
            //         int id = prop["id"];
            //         int value = prop["value"];
         
            //         qctrl.id = id;                    
            //         qctrl.type = type;

            //         if(vcap::set_control(vd.get_fd(), qctrl.id, value) < 0)
            //             std::cout << "ERROR seting "<< prop["name"]<<"\n"; 
            //     }
            // }

        }

        bool loadFIle(std::string file_dir){
            try {
                string ros_path =  ros::package::getPath("cam_pub");
                // ros_path = ros_path+"/src/cam_config.json";
                ros_path = ros_path+file_dir;
                ifstream file(ros_path);
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

        void grabMonoImage(){

            if( !vd.set_frame_size(640,480)) {
                std::cout << "Frame size is not avaliable!\n";
            }

            //CamConfig("/src/cam_config.json");

            int rate = framerate_;

            std::string img_raw_topic   = "image_raw_color";
            string left_topic           = "left/" + img_raw_topic;

            //create a image_transport publisher with topic "raw_image"
            image_transport::ImageTransport it_(node_);
            //advertise are used to create Publisher topic name and queue size
            image_transport::CameraPublisher image_pub_ = it_.advertiseCamera(left_topic, 1);

            cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
            //Define the frequency that message are send. Could be the frame rate of the camera
            ros::Rate loop_rate(rate); 
            vd.start();

            while(ros::ok()){//keep spinning loop until user presses Ctrl+c

                vd.grabRGB(rgb);

                Mat frame(image_height_, image_width_, CV_8UC3, rgb);

                if(frame.empty())
                    break;

                ros::Time time = ros::Time::now();
                //Publish the message
                publishImage(frame, image_pub_, left_topic, time);

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


        void grabStereoImage(){

            if( !vd.set_frame_size(1280,480)) {
                std::cout << "Frame size is not avaliable!\n";
            }

            //CamConfig("/src/cam_config_stereo.json");

            int rate = framerate_;

            string left_topic           = "/camera/left/";
            string right_topic          = "/camera/right/";

            std::string img_raw_topic   = "image_raw";
            std::string img_rect_topic   = "image_rect";

            string left_raw_topic_           = left_topic + img_raw_topic;
            string right_raw_topic_          = right_topic + img_raw_topic;

            string left_rect_topic_           = left_topic + img_rect_topic;
            string right_rect_topic_          = right_topic + img_rect_topic;

            //create a image_transport publisher with topic "raw_image"
            image_transport::ImageTransport it_(node_);
            //advertise are used to create Publisher topic name and queue size
            image_transport::CameraPublisher left_raw_topic   = it_.advertiseCamera(left_raw_topic_, 1);
            image_transport::CameraPublisher right_raw_topic  = it_.advertiseCamera(right_raw_topic_, 1);

            image_transport::CameraPublisher left_rect_topic   = it_.advertiseCamera(left_rect_topic_, 1);
            image_transport::CameraPublisher right_rect_topic  = it_.advertiseCamera(right_rect_topic_, 1);

                
            //Define the frequency that message are send. Could be the frame rate of the camera
            ros::Rate loop_rate(rate); 
            vd.start();

            while(ros::ok()){//keep spinning loop until user presses Ctrl+c

                ros::Time time;
                vd.grabRGB(rgb);

                Mat frame(image_height_, image_width_, CV_8UC3, rgb);

                if(frame.empty())
                    break;

                Mat left_raw_image  = frame(Rect(0, 0, image_width_/2, image_height_)).clone();
                Mat right_raw_image = frame(Rect(image_width_/2, 0, image_width_/2, image_height_)).clone();

                Mat left_rect_image, right_rect_image;

                cv::remap(left_raw_image, left_rect_image, M1l, M2l, cv::INTER_LINEAR);
                cv::remap(right_raw_image, right_rect_image, M1r, M2r, cv::INTER_LINEAR);


                time = ros::Time::now();
                publishImage(left_raw_image, left_raw_topic, left_topic, time);
                time = ros::Time::now();
                publishImage(right_raw_image, right_raw_topic, right_topic, time);

                time = ros::Time::now();
                publishImage(left_rect_image, left_rect_topic, left_topic, time);
                time = ros::Time::now();
                publishImage(right_rect_image, right_rect_topic, right_topic, time);
                

                ROS_INFO("ImageMsg Sent.");

                //Need to call this function to allow ROS to process incoming messages
                //Handles the events and returns immediately
                ros::spinOnce();
                //Sleep for the rest of the cycle, to enforce the loop rate
                loop_rate.sleep();
            }
            std::cout << "\n";
            free(rgb);
        }

        void publishImage(cv::Mat img, image_transport::CameraPublisher &pub_img, string img_frame_id, ros::Time t) {
            img_.header.frame_id = img_frame_id;
            sensor_msgs::CameraInfoPtr ci (new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
            ci -> header.frame_id = img_frame_id;
            ci -> header.stamp    = t;

            pub_img.publish(imageToROSmsg(img, sensor_msgs::image_encodings::RGB8, img_frame_id, t), ci);
        }

        sensor_msgs::ImagePtr imageToROSmsg(cv::Mat im, const std::string encodingType, std::string frameId, ros::Time t){
            cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

            cv_ptr->encoding = encodingType;
            cv_ptr->header.stamp = t;
            cv_ptr->header.frame_id = frameId;
            cv_ptr->image = im;

            return cv_ptr->toImageMsg();
        }

};

int main(int argc, char **argv)
{   

    //Initialize new ROS node named "cam_node"
    ros::init(argc, argv, "cam_node");

    CamCap camcap;

    std::cout << "DevType: " << camcap.devtype_ << std::endl;

    if(camcap.devtype_ == "mono"){
        camcap.grabMonoImage();
    }else if (camcap.devtype_ == "stereo")
    {
        camcap.grabStereoImage();
        
    }else{
        cerr  << "Usage: rosrun cam_pub cam_pub_node <device_name> <device_type> (mono/stereo)" << endl;        
    }

    //Main loop for ROS that repeatedly calls SpinOnce until the node is shut down
    ros::spin();
    return 0;

}
