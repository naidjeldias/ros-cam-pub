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
#include <sensor_msgs/distortion_models.h>
#include <vcap.hpp>
#include <device.hpp>
#include <cstdio>
#include "json/json.hpp"

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;
using json = nlohmann::json;
// const char * devname = "/dev/video0";
// const char * devType = "mono";

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
        std::string io_method_name_, pixel_format_name_, camera_name_, camera_info_url_, devtype_;
        int image_width_, image_height_, framerate_, brightness_, contrast_, saturation_, sharpness_, focus_,
             gain_;
        double white_balance_, exposure_; 
        bool autofocus_, autoexposure_, auto_white_balance_;
        boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
        boost::shared_ptr<camera_info_manager::CameraInfoManager> lcinfo_, rcinfo_;

        sensor_msgs::CameraInfoPtr mRightCamInfoMsg, mLeftCamInfoMsg;
        string mRightCamId, mLeftCamId; 

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;


        CamCap(const string &strSettingsFile, const string &strDevName): node_("~")
        {
            if (!vd.open_device(strDevName.c_str())) {
                std::cout << "Device not openned...\n";
            }else{                
                // std::string file_path = "/src/stereo.yaml"; 
                // string ros_path =  ros::package::getPath("cam_pub");

                // grab the parameters
                node_.getParam("/cam_pub_node/image_width", image_width_);
                node_.getParam("/cam_pub_node/image_height", image_height_);
                node_.getParam("/cam_pub_node/autofocus", autofocus_);
                node_.getParam("/cam_pub_node/focus", focus_);
                node_.getParam("/cam_pub_node/autoexposure", autoexposure_);
                node_.getParam("/cam_pub_node/exposure", exposure_);
                node_.getParam("/cam_pub_node/auto_white_balance", auto_white_balance_);
                node_.getParam("/cam_pub_node/white_balance", white_balance_);
                node_.getParam("/cam_pub_node/framerate", framerate_);


                cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
                if(!fsSettings.isOpened())
                {
                    cerr << "ERROR: Wrong path to settings" << endl;
                }else{                
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

                    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
                    {
                        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
                        
                    }else{
                        
                        mLeftCamInfoMsg.reset(new sensor_msgs::CameraInfo());
                        mRightCamInfoMsg.reset(new sensor_msgs::CameraInfo());

                        mLeftCamId = "leftCamera";
                        mRightCamId = "rightCamera";

                        fillCameraInfo(mLeftCamInfoMsg, mRightCamInfoMsg, mLeftCamId, mRightCamId);
                        //obtendo as novas matrizes de projeção das imagens retificadas
                        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
                        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
                    }

                    
                }
            }

        }

        ~CamCap(){
            vd.stop();
            vd.close_device();
        }

        void fillCameraInfo(sensor_msgs::CameraInfoPtr leftCamInfoMsg,sensor_msgs::CameraInfoPtr rightCamInfoMsg, 
                            string leftFrameId, string rightFrameId){

            leftCamInfoMsg->distortion_model =
            sensor_msgs::distortion_models::PLUMB_BOB;
            rightCamInfoMsg->distortion_model =
            sensor_msgs::distortion_models::PLUMB_BOB;

            leftCamInfoMsg->D.resize(5);
            rightCamInfoMsg->D.resize(5);

            leftCamInfoMsg->D[0] = D_l.at<double>(0);   // k1
            leftCamInfoMsg->D[1] = D_l.at<double>(1);   // k2
            leftCamInfoMsg->D[2] = D_l.at<double>(2);   // k3
            leftCamInfoMsg->D[3] = D_l.at<double>(3);   // p1
            leftCamInfoMsg->D[4] = D_l.at<double>(4);   // p2

            rightCamInfoMsg->D[0] = D_r.at<double>(0); // k1
            rightCamInfoMsg->D[1] = D_r.at<double>(1); // k2
            rightCamInfoMsg->D[2] = D_r.at<double>(2); // k3
            rightCamInfoMsg->D[3] = D_r.at<double>(3); // p1
            rightCamInfoMsg->D[4] = D_r.at<double>(4); // p2

            leftCamInfoMsg->K.fill(0.0);
            rightCamInfoMsg->K.fill(0.0);

            leftCamInfoMsg->K[0] = K_l.at<double>(0,0);
            leftCamInfoMsg->K[2] = K_l.at<double>(0,2);
            leftCamInfoMsg->K[4] = K_l.at<double>(1,1);
            leftCamInfoMsg->K[5] = K_l.at<double>(1,2);
            leftCamInfoMsg->K[8] = 1.0;

            rightCamInfoMsg->K[0] = K_r.at<double>(0,0);
            rightCamInfoMsg->K[2] = K_r.at<double>(0,2);
            rightCamInfoMsg->K[4] = K_r.at<double>(1,1);
            rightCamInfoMsg->K[5] = K_r.at<double>(1,2);
            rightCamInfoMsg->K[8] = 1.0;


            leftCamInfoMsg->R.fill(0.0);
            leftCamInfoMsg->R[0] = R_l.at<double>(0);
            leftCamInfoMsg->R[1] = R_l.at<double>(1);
            leftCamInfoMsg->R[2] = R_l.at<double>(2);
            leftCamInfoMsg->R[3] = R_l.at<double>(3);
            leftCamInfoMsg->R[4] = R_l.at<double>(4);
            leftCamInfoMsg->R[5] = R_l.at<double>(5);
            leftCamInfoMsg->R[6] = R_l.at<double>(6);
            leftCamInfoMsg->R[7] = R_l.at<double>(7);
            leftCamInfoMsg->R[8] = R_l.at<double>(8);


            rightCamInfoMsg->R.fill(0.0);
            rightCamInfoMsg->R[0] = R_r.at<double>(0);
            rightCamInfoMsg->R[1] = R_r.at<double>(1);
            rightCamInfoMsg->R[2] = R_r.at<double>(2);
            rightCamInfoMsg->R[3] = R_r.at<double>(3);
            rightCamInfoMsg->R[4] = R_r.at<double>(4);
            rightCamInfoMsg->R[5] = R_r.at<double>(5);
            rightCamInfoMsg->R[6] = R_r.at<double>(6);
            rightCamInfoMsg->R[7] = R_r.at<double>(7);
            rightCamInfoMsg->R[8] = R_r.at<double>(8);


            leftCamInfoMsg->P.fill(0.0);
            rightCamInfoMsg->P.fill(0.0);

            leftCamInfoMsg->P[0] = P_l.at<double>(0,0);
            leftCamInfoMsg->P[2] = P_l.at<double>(0,2);
            leftCamInfoMsg->P[5] = P_l.at<double>(1,1);
            leftCamInfoMsg->P[6] = P_l.at<double>(1,2);
            leftCamInfoMsg->P[10] = 1.0;
            
            
            rightCamInfoMsg->P[0] = P_r.at<double>(0,0);
            rightCamInfoMsg->P[2] = P_r.at<double>(0,2);
            rightCamInfoMsg->P[3] = P_r.at<double>(0,3);
            rightCamInfoMsg->P[5] = P_r.at<double>(1,1);
            rightCamInfoMsg->P[6] = P_l.at<double>(1,2);
            rightCamInfoMsg->P[10] = 1.0;
            leftCamInfoMsg->width   = rightCamInfoMsg->width = image_width_;
            leftCamInfoMsg->height  = rightCamInfoMsg->height = image_height_;
            leftCamInfoMsg->header.frame_id = leftFrameId;
            rightCamInfoMsg->header.frame_id = rightFrameId;


        }

        void CamConfig(){

            // enum control default
            struct v4l2_queryctrl qctrl;

            if(!vd.set_frame_size(image_width_, image_height_)) {
                std::cout << "Frame size is not avaliable!\n";
            }

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
                    if(sName == "Focus (absolute)" && focus_){
                        if(vcap::set_control(vd.get_fd(), qctrl.id, focus_) < 0)
                            std::cout << "ERROR seting "<< sName <<"\n";
                    }
                    if(sName == "Exposure, Auto" && autoexposure_ ){
                        if(vcap::set_control(vd.get_fd(), qctrl.id, autoexposure_) < 0)
                            std::cout << "ERROR seting "<< sName <<"\n";
                    }
                    if(sName == "Exposure (Absolute)" && exposure_){
                        if(vcap::set_control(vd.get_fd(), qctrl.id, exposure_) < 0)
                            std::cout << "ERROR seting "<< sName <<"\n";
                    }
                    if(sName == "White Balance Temperature, Auto" && auto_white_balance_){
                        if(vcap::set_control(vd.get_fd(), qctrl.id, auto_white_balance_) < 0)
                            std::cout << "ERROR seting "<< sName <<"\n";
                    }

                    if(sName == "White Balance Temperature" && white_balance_){
                        if(vcap::set_control(vd.get_fd(), qctrl.id, white_balance_) < 0)
                            std::cout << "ERROR seting "<< sName <<"\n";
                    }
                } while (vcap::enum_control_default( vd.get_fd(), &qctrl, false) );
            }

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

        // void grabMonoImage(){

        //     if( !vd.set_frame_size(image_width_,image_height_)) {
        //         std::cout << "Frame size is not avaliable!\n";
        //     }

        //     //CamConfig("/src/cam_config.json");

        //     int rate = framerate_;

        //     std::string img_raw_topic   = "image_raw_color";
        //     string left_topic           = "left/" + img_raw_topic;

        //     //create a image_transport publisher with topic "raw_image"
        //     image_transport::ImageTransport it_(node_);
        //     //advertise are used to create Publisher topic name and queue size
        //     image_transport::CameraPublisher image_pub_ = it_.advertiseCamera(left_topic, 1);

        //     cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        //     //Define the frequency that message are send. Could be the frame rate of the camera
        //     ros::Rate loop_rate(rate); 
        //     vd.start();

        //     while(ros::ok()){//keep spinning loop until user presses Ctrl+c

        //         vd.grabRGB(rgb);

        //         Mat frame(image_height_, image_width_, CV_8UC3, rgb);

        //         if(frame.empty())
        //             break;

        //         ros::Time time = ros::Time::now();
        //         //Publish the message
        //         publishImage(frame, image_pub_, left_topic, time, lcinfo_);

        //         ROS_INFO("ImageMsg Sent.");
        //         ROS_INFO("Subscribers: %d", image_pub_.getNumSubscribers());

        //         //Need to call this function to allow ROS to process incoming messages
        //         //Handles the events and returns immediately
        //         ros::spinOnce();
        //         //Sleep for the rest of the cycle, to enforce the loop rate
        //         loop_rate.sleep();
        //     }
            
        //     std::cout << "\n";
        //     free(rgb);

        // }


        void grabStereoImage(){

            CamConfig();

            int image_width, image_height;
            vd.get_image_dimension(&image_width, &image_height);
            rgb = (unsigned char *)malloc(sizeof(unsigned char) * image_width * image_height * 3);

            // std::cout << "Image size: (" <<image_width << " , " << image_height << ")" << std::endl;

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
            ros::Rate loop_rate(framerate_); 
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
                publishImage(left_raw_image, left_raw_topic, left_topic, time, mLeftCamInfoMsg);
                publishImage(right_raw_image, right_raw_topic, right_topic, time, mRightCamInfoMsg);

                publishImage(left_rect_image, left_rect_topic, left_topic, time, mLeftCamInfoMsg);
                publishImage(right_rect_image, right_rect_topic, right_topic, time, mRightCamInfoMsg);

                //Need to call this function to allow ROS to process incoming messages
                //Handles the events and returns immediately
                ROS_INFO("ImageMsg Sent.");
                ros::spinOnce();
                //Sleep for the rest of the cycle, to enforce the loop rate
                loop_rate.sleep();
            }
            std::cout << "\n";
            // vd.stop();
            // vd.close_device();
            free(rgb);
        }

        void publishImage(cv::Mat img, image_transport::CameraPublisher &pub_img, string img_frame_id, ros::Time t, 
                                    sensor_msgs::CameraInfoPtr cinfo) {

            pub_img.publish(imageToROSmsg(img, sensor_msgs::image_encodings::RGB8, img_frame_id, t), cinfo);
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
    ros::init(argc, argv, "pub_node");
    ros::start();

    if(argc != 4)
    {
        ROS_ERROR ("Device name device type and Path to settings need to be set.");
        ROS_ERROR ("Usage: rosrun cam_pub cam_pub_node <device_name> <device_type> (mono/stereo) <path_settings>");
        ros::shutdown();
        return 1;
    }

    CamCap camcap(argv[3], argv[1]);

    string devType = argv[2];

    if(devType == "mono"){
        // camcap.grabMonoImage();
    }else if (devType == "stereo")
    {
        camcap.grabStereoImage();
    }else{
        cerr  << "Usage: rosrun cam_pub cam_pub_node <device_name> <device_type> (mono/stereo) <path_settings>" << endl;        
    }

    //Main loop for ROS that repeatedly calls SpinOnce until the node is shut down
    ros::spin();
    return 0;

}
