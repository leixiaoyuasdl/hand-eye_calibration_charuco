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

#include <fstream>

#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
using namespace std;
using namespace KDL;
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
    image_transport::Publisher debug_pub;
    ros::Publisher pose_pub;
    ros::Publisher transform_pub;
    ros::Publisher position_pub;
    ros::Publisher pixel_pub;


    ros::Publisher marker_pub; //rviz visualization marker


    std::string marker_frame;
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

    int sum_pitures;

public:
    ArucoSimple()
            : cam_info_received(false),
              nh("~"),
              it(nh)
    {
        sum_pitures=0;
        //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
      //  board = cv::aruco::CharucoBoard::create(10, 10, 0.015f, 0.01f, dictionary);
       cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
       board = cv::aruco::CharucoBoard::create(5, 5, 0.02f, 0.015f, dictionary);
       params = cv::aruco::DetectorParameters::create();

        std::string refinementMethod;
        nh.param<std::string>("corner_refinement", refinementMethod, "LINES");

        image_sub = it.subscribe("/camera/color/image_raw", 1, &ArucoSimple::image_callback, this);
        cam_info_sub = nh.subscribe("/camera/color/camera_info", 1, &ArucoSimple::cam_info_callback, this);

        image_pub = it.advertise("result", 1);
        debug_pub = it.advertise("debug", 1);
        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
        transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
        position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);
        pixel_pub = nh.advertise<geometry_msgs::PointStamped>("pixel", 10);

        marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 10);


        nh.param<std::string>("reference_frame", reference_frame, "camera_color_frame");
        nh.param<std::string>("camera_frame", camera_frame, "camera_color_frame");
        nh.param<std::string>("marker_frame", marker_frame, "camera_marker");
        nh.param<bool>("image_is_rectified", useRectifiedImages, true);
        nh.param<bool>("rotate_marker_axis", rotate_marker_axis_, false);

        ROS_ASSERT(camera_frame != "" && marker_frame != "");

        if ( reference_frame.empty() )
            reference_frame = camera_frame;

        ROS_INFO("Aruco node will publish pose to TF with %s as parent and %s as child.",
                 reference_frame.c_str(), marker_frame.c_str());

    }

    bool getTransform(const std::string& refFrame,
                      const std::string& childFrame,
                      tf::StampedTransform& transform)
    {
        std::string errMsg;

        if ( !_tfListener.waitForTransform(refFrame,
                                           childFrame,
                                           ros::Time(0),
                                           ros::Duration(5),
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
    void detectCharucoBoardWithCalibrationPose()
    {
                  image.copyTo(imageCopy);
                std::vector<int> markerIds;
                std::vector<std::vector<cv::Point2f> > markerCorners;
                cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);
                // if at least one marker detected
                if (markerIds.size() > 0) {
                    //cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
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
//                        if (valid)
//                            cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1f);
                    }

                }
    }
    void detectCharucoBoardWithoutCalibration()
    {
            image.copyTo(imageCopy);
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f> > markerCorners;
            cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);
            //or
            //cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, params);
            // if at least one marker detected
            if (markerIds.size() > 0) {
                //cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
                std::vector<cv::Point2f> charucoCorners;
                std::vector<int> charucoIds;
                cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds);
                // if at least one charuco corner detected
                if (charucoIds.size() > 0)
                    cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
                for(int i=0;i<markerIds.size();i++)
                {
                    cv::circle(imageCopy, charucoCorners.at(i), 5, cv::Scalar(0, 0, 255), -1);
                    cout<<"id "<<charucoCorners.at(i)<<endl;
                }
            }

    }

    void image_callback(const sensor_msgs::ImageConstPtr& msg)
    {



        static tf::TransformBroadcaster br;
        if(cam_info_received)
        {
            ros::Time curr_stamp = msg->header.stamp;
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
                image = cv_ptr->image;
                detectCharucoBoardWithCalibrationPose();
//                detectCharucoBoardWithoutCalibration();

                boarder.ssize=0.01;
                boarder.id=1;
                boarder.Rvec.at<double>(0,0)=rvec[0];
                boarder.Rvec.at<double>(1,0)=rvec[1];
                boarder.Rvec.at<double>(2,0)=rvec[2];

                boarder.Tvec.at<double>(0,0)=tvec[0];
                boarder.Tvec.at<double>(1,0)=tvec[1];
                boarder.Tvec.at<double>(2,0)=tvec[2];


                        tf::Transform transform = aruco_ros::arucoMarker2Tf(boarder, rotate_marker_axis_);
                        tf::StampedTransform cameraToReference;
                        cameraToReference.setIdentity();

                        if ( reference_frame != camera_frame )
                        {
                            getTransform(reference_frame,
                                         camera_frame,
                                         cameraToReference);
                        }
                        transform =
                                static_cast<tf::Transform>(cameraToReference)
                                * static_cast<tf::Transform>(rightToLeft)
                                * transform;

                        tf::StampedTransform stampedTransform(transform, curr_stamp,
                                                              reference_frame, marker_frame);
                        br.sendTransform(stampedTransform);
                        geometry_msgs::PoseStamped poseMsg;
                        tf::poseTFToMsg(transform, poseMsg.pose);
                        poseMsg.header.frame_id = reference_frame;
                        poseMsg.header.stamp = curr_stamp;
                        pose_pub.publish(poseMsg);

                        geometry_msgs::TransformStamped transformMsg;
                        tf::transformStampedTFToMsg(stampedTransform, transformMsg);
                        transform_pub.publish(transformMsg);

                        geometry_msgs::Vector3Stamped positionMsg;
                        positionMsg.header = transformMsg.header;
                        positionMsg.vector = transformMsg.transform.translation;
                        position_pub.publish(positionMsg);


                //Publish rviz marker representing the ArUco marker patch
                visualization_msgs::Marker visMarker;
                visMarker.header = transformMsg.header;
                visMarker.id = 1;
                visMarker.type   = visualization_msgs::Marker::CUBE;
                visMarker.action = visualization_msgs::Marker::ADD;
                visMarker.pose = poseMsg.pose;
                visMarker.scale.x = 0.1;
                visMarker.scale.y = 0.001;
                visMarker.scale.z = 0.1;
                visMarker.color.r = 1.0;
                visMarker.color.g = 0;
                visMarker.color.b = 0;
                visMarker.color.a = 1.0;
                visMarker.lifetime = ros::Duration(3.0);
                marker_pub.publish(visMarker);


                if(image_pub.getNumSubscribers() > 0)
                {
                    //show input with augmented information
                    cv_bridge::CvImage out_msg;
                    out_msg.header.stamp = curr_stamp;
                    out_msg.encoding = sensor_msgs::image_encodings::RGB8;
                    out_msg.image = imageCopy;
                    image_pub.publish(out_msg.toImageMsg());
                }


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



};

void createBoard()
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(3, 4, 0.03f, 0.015f, dictionary);
    cv::Mat boardImage;
    board->draw(cv::Size(900, 1800), boardImage, 0, 1);
    cv::imwrite("/home/fzt/BoardImage.jpg", boardImage);
}
int main(int argc,char **argv)
{

    ros::init(argc, argv, "charuco");

    ArucoSimple node;

    //createBoard();

    ros::spin();
}
