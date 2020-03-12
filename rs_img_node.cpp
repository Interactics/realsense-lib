#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include <iostream>                 // for cout
#include <sstream>                  // for converting the command line parameter to integer

class ImgPublish{
public:
    ImgPublish() : it_(nh_){
        // Subscrive to input video feed and publish output video feed
        color_pub_ = it_.advertise("/rs_img/color", 1);
        depth_pub_ = it_.advertise("/rs_img/depth", 1);
        cv::namedWindow(OPENCV_WINDOW);
    }
    ~ImageConverter(){
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imagePub(const cv::Mat& Img){
        sensor_msgs::ImagePtr msg;

        if(Img.empty()){
            ROS_INF("Frame is empty!");
            return;
        }
        
        try{
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Img).toImageMsg();
        }
        catch (cv_bridge::Exception &e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        pub.publish(msg);
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher color_pub_;
    image_transport::Publisher depth_pub_;

    const static auto OPENCV_WINDOW = "rs_img";

};

class RsCam{
public:

    RsCam(){
        cfg.enable_stream(RS2_STREAM_DEPTH);
        cfg.enable_stream(RS2_STREAM_COLOR);
        pipe.start(cfg);
    }
    ~RsCam(){}

    void rsImg(){

        Frame = pipe.wait_for_frames();

        depthData = align_to_depth.process(Frame)
        depthData = depthData.get_depth_frame().apply_filter(color_map);

        colorData = align_to_color.process(Frame);

        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        Mat image(Size(w, h), CV_8UC3, (void*)depthData.get_data(), Mat::AUTO_STEP).copyto(C_);
        Mat image(Size(w, h), CV_8UC1, (void*)colorData.get_data(), Mat::AUTO_STEP).copyto(D_);

        imshow(window_name, image);
    }

    cv::Mat& colorImg(){
        return C_;
    }
    cv::Mat& depthImg(){
        return D_;
    }

private:
    rs2::pipeline pipe;
    rs2::colorizer color_map;
    rs2::frameset Frame;
    rs2::frameset depthData;
    rs2::frameset colorData;

    rs2::frame depth;
    rs2::config cfg;

    cv::Mat C_;
    cv::Mat D_;

    const static rs2::align align_to_color(RS2_STREAM_COLOR);
};


int main(int argc, char** argv){
  // Check if video source has been passed as a parameter

  ros::init(argc, argv, "rs_img_node");
  ros::Rate loop_rate(5);

  ImgPublish rsc;

  while (ros::ok()){
      rsc.imagePub();
      loop_rate.sleep();
  }
}
