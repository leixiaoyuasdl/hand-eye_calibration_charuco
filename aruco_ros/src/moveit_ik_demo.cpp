/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>


using namespace aruco;


class ArucoSimple
{
private:
    cv::Mat image;
    cv::Mat imageCopy;
    aruco::CameraParameters camParam;
    tf::StampedTransform rightToLeft;
    bool useRectifiedImages;
    Marker boarder;
    ros::Subscriber cam_info_sub;
    bool cam_info_received;

    image_transport::Publisher image_pub;

    std::string camera_frame;
    std::string reference_frame;

    cv::Vec3d rvec, tvec;
    cv::Mat cameraMatrix, distCoeffs;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::CharucoBoard> board ;
    cv::Ptr<cv::aruco::DetectorParameters> params ;

    bool rotate_marker_axis_;

    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;

    tf::TransformListener _tfListener;

    tf::Transform transform;

    ros::Time curr_stamp;
    bool isok;


public:
    ArucoSimple()
            : cam_info_received(false),
              nh("~"),
              it(nh)
    {
        isok==false;
        nh.param<std::string>("reference_frame", reference_frame, "base_link");
        nh.param<std::string>("camera_frame", camera_frame, "camera_color_frame");
        nh.param<bool>("image_is_rectified", useRectifiedImages, true);
        nh.param<bool>("rotate_marker_axis", rotate_marker_axis_, false);

        ROS_ASSERT(camera_frame != "" );
        image_pub = it.advertise("result", 1);
        if ( reference_frame.empty() )
            reference_frame = camera_frame;


//        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
//        board = cv::aruco::CharucoBoard::create(5, 7, 0.03f, 0.015f, dictionary);
        dictionary = cv::aruco::generateCustomDictionary(16,4);
        board = cv::aruco::CharucoBoard::create(4, 8, 0.03f, 0.015f, dictionary);
        params = cv::aruco::DetectorParameters::create();


        image_sub = it.subscribe("/camera/color/image_raw", 1, &ArucoSimple::image_callback, this);
        cam_info_sub = nh.subscribe("/camera/color/camera_info", 1, &ArucoSimple::cam_info_callback, this);
        ros::Rate r(10);
       while(isok==false)
       {
           ros::spinOnce();
           r.sleep();
       }

    }

    bool getTransform(const std::string& refFrame,
                      const std::string& childFrame,
                      tf::StampedTransform& transform)
    {
        std::string errMsg;

        if ( !_tfListener.waitForTransform(refFrame,
                                           childFrame,
                                           ros::Time(0),
                                           ros::Duration(0.5),
                                           ros::Duration(0.01),
                                           &errMsg)
                )
        {
            ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
            return false;
        }
        else
        {
            try
            {
                _tfListener.lookupTransform( refFrame, childFrame,
                                             ros::Time(0),                  //get latest available
                                             transform);
            }
            catch ( const tf::TransformException& e)
            {
                ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
                return false;
            }

        }
        return true;
    }
    bool detectCharucoBoardWithCalibrationPose()
    {
        image.copyTo(imageCopy);
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f> > markerCorners;
        cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);
        // if at least one marker detected
        if (markerIds.size() > 0) {
            cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
            std::vector<cv::Point2f> charucoCorners;
            std::vector<int> charucoIds;
            cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);
            // if at least one charuco corner detected
            if (charucoIds.size() > 0) {
                cv::Scalar color = cv::Scalar(255, 0, 0);
                cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
                // cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
                bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
                // if charuco pose is valid
                if (valid)
                {
                   // cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1f);

                    boarder.ssize=0.01;
                    boarder.id=1;
//                    boarder.Rvec.at<float>(0,0)=rvec[0];
//                    boarder.Rvec.at<float>(1,0)=rvec[1];
//                    boarder.Rvec.at<float>(2,0)=rvec[2];
//
//                    boarder.Tvec.at<float>(0,0)=tvec[0];
//                    boarder.Tvec.at<float>(1,0)=tvec[1];
//                    boarder.Tvec.at<float>(2,0)=tvec[2];
                    boarder.Rvec.at<double>(0,0)=rvec[0];
                    boarder.Rvec.at<double>(1,0)=rvec[1];
                    boarder.Rvec.at<double>(2,0)=rvec[2];

                  //  std::cout<<"rvec xyz  : "<<rvec[0]<<" "<<rvec[1]<<" "<<rvec[2]<<endl;
                   // std::cout<<"tvec xyz  : "<<tvec[0]<<" "<<tvec[1]<<" "<<tvec[2]<<endl;

                    boarder.Tvec.at<double>(0,0)=tvec[0];
                    boarder.Tvec.at<double>(1,0)=tvec[1];
                    boarder.Tvec.at<double>(2,0)=tvec[2];
//
//                    cv::Mat Rvec64;
//                    boarder.Rvec.convertTo(Rvec64, CV_64FC1);
//                    cv::Mat tran64;
//                    boarder.Tvec.convertTo(tran64, CV_64FC1);
//
//                    boarder.Rvec=Rvec64;
//                    boarder.Tvec=tran64;


                    CvDrawingUtils::draw3dAxis(imageCopy, boarder, camParam);
                    if(image_pub.getNumSubscribers() > 0)
                    {
                        //show input with augmented information
                        cv_bridge::CvImage out_msg;
                        out_msg.header.stamp = curr_stamp;
                        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
                        out_msg.image = imageCopy;
                        image_pub.publish(out_msg.toImageMsg());
                    }

                    char aa;
                    std::cout<<"start y or n"<<endl;
                    std::cin>>aa;
                    if(aa!='y') return false;
                    isok=true;
                    image_sub.shutdown();
                    return true;
                }

            }
        }
        return false;
    }

    void image_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        if(cam_info_received==true)
        {
            curr_stamp = msg->header.stamp;
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
                image = cv_ptr->image;
                if(detectCharucoBoardWithCalibrationPose()==false) return;

                transform = aruco_ros::arucoMarker2Tf(boarder, false);
                tf::StampedTransform cameraToReference;
                cameraToReference.setIdentity();


                if ( reference_frame != camera_frame )
                {
                    getTransform(reference_frame,
                                 camera_frame,
                                 cameraToReference);
                }

              //  std::cout<<"transform1 xyz  : "<<transform.getOrigin().getX()<<" "<<transform.getOrigin().getY()<<" "<<transform.getOrigin().getZ()<<endl;

                transform =
                        static_cast<tf::Transform>(cameraToReference)
                        * static_cast<tf::Transform>(rightToLeft)
                        * transform;
              //  std::cout<<"transform2 xyz  : "<<transform.getOrigin().getX()<<" "<<transform.getOrigin().getY()<<" "<<transform.getOrigin().getZ()<<endl;
                moveit_demo();

            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
        }
    }

    // wait for one camerainfo, then shut down that subscriber
    void cam_info_callback(const sensor_msgs::CameraInfo &msg)
    {
        camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

        cameraMatrix=camParam.CameraMatrix;
        distCoeffs=camParam.Distorsion;

        // handle cartesian offset between stereo pairs
        // see the sensor_msgs/CamaraInfo documentation for details
        rightToLeft.setIdentity();
        rightToLeft.setOrigin(
                tf::Vector3(
                        -msg.P[3]/msg.P[0],
                        -msg.P[7]/msg.P[5],
                        0.0));

        cam_info_received = true;
        cam_info_sub.shutdown();
    }


void moveit_demo()
{


        moveit::planning_interface::MoveGroupInterface arm("manipulator");

    //获取终端link的名称
    std::string end_effector_link = arm.getEndEffectorLink();
    std::cout<<" end_effector_link  : "<<end_effector_link<<endl;
    //设置目标位置所使用的参考坐标系
    arm.setPoseReferenceFrame(reference_frame);

    //当运动规划失败后，允许重新规划
    arm.allowReplanning(true);

    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.001);

    //设置允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);


    tf::Transform tfff;
    tfff.setIdentity();
    tf::Matrix3x3 tf_r(1, 0, 0,
                       0, -1, 0,
                       0, 0, -1);

    tfff.setBasis(tf_r);



    transform=transform*tfff;

    tf::Transform tfff1;
    tfff1.setIdentity();

    tf::Vector3 orr1(0,0,-0.081
    );
    tfff1.setOrigin(orr1);

    transform=transform*tfff1;
    // 设置机器人终端的目标位置
    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = transform.getRotation().getX();
    target_pose.orientation.y = transform.getRotation().getY();
    target_pose.orientation.z = transform.getRotation().getZ();
    target_pose.orientation.w = transform.getRotation().getW();

    target_pose.position.x = transform.getOrigin().getX();
    target_pose.position.y = transform.getOrigin().getY();
    target_pose.position.z = transform.getOrigin().getZ();

    std::cout<<"xy  : "<<target_pose.position.x<<" "<<target_pose.position.y<<" "<<target_pose.position.z<<endl;
    std::vector<geometry_msgs::Pose> way_points_msg;
    way_points_msg.push_back(target_pose);
    moveit_msgs::ExecuteKnownTrajectory srv;
    srv.request.wait_for_execution = true;
    ros::ServiceClient executeKnownTrajectoryServiceClient = nh.serviceClient < moveit_msgs::ExecuteKnownTrajectory
    > ("/execute_kinematic_path");

    double percentage = arm.computeCartesianPath(way_points_msg, 0.01, 0.0, srv.request.trajectory);
std::cout<<"percentage "<<percentage<<endl;


    std::cout<<"continue to executeKnownTrajectory   y or n ?"<<endl;
    char aa;
    std::cout<<"start y or n"<<endl;
    std::cin>>aa;
    if(aa!='y') {
        isok=false;
        return;
    }

     executeKnownTrajectoryServiceClient.call(srv);
}
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_ik_demo");

     ArucoSimple node;

     return 0;
}
