#ifndef _OPENPOSE_ROS_IO
#define _OPENPOSE_ROS_IO

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Header.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>  // Video write

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>


#include <openpose_ros_msgs/BoundingBox.h>
#include <openpose_ros_msgs/OpenPoseHuman.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/PointWithProb.h>
#include <openpose_ros_msgs/OpenPoseHuman3D.h>
#include <openpose_ros_msgs/OpenPoseHumanList3D.h>
#include <openpose_ros_msgs/PointWithProb3D.h>

#include <openpose.h>
#include <openpose_flags.h>

// OpenPose dependencies
#include <openpose/headers.hpp>

namespace openpose_ros {

    class OpenPoseROSIO
    {
        private:
            ros::NodeHandle nh_;
            ros::Publisher openpose_human_list_pub_;
            ros::Publisher openpose_human_list_pub_3D_;
			ros::Publisher marker_pub;
			ros::Publisher skeleton_pub;
            image_transport::ImageTransport it_;
            image_transport::Subscriber image_sub_;
            image_transport::Subscriber depth_sub_;

            cv_bridge::CvImagePtr cv_img_ptr_;
            cv_bridge::CvImagePtr cv_depth_ptr_;
            std_msgs::Header image_header_;
            std_msgs::Header depth_header_;

            OpenPose* openpose_;

            bool display_output_flag_;
            bool print_keypoints_flag_;

            bool save_original_video_flag_;
            std::string original_video_file_name_;
            bool original_video_writer_initialized_;
            cv::VideoWriter original_video_writer_;

            bool save_openpose_video_flag_;
            std::string openpose_video_file_name_;
            bool openpose_video_writer_initialized_;
            cv::VideoWriter openpose_video_writer_;

			bool VIS_right, VIS_left, VIS_face, VIS_body;

            int video_fps_;

        public:
            OpenPoseROSIO(OpenPose &openPose);

            ~OpenPoseROSIO(){}

            void processImage(const sensor_msgs::ImageConstPtr& msg);

            void convertImage(const sensor_msgs::ImageConstPtr& msg);

            void convertDepth(const sensor_msgs::ImageConstPtr& msg);

            std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> createDatum();

            bool display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);

            bool saveOriginalVideo(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);

            bool saveOpenPoseVideo(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);

            cv_bridge::CvImagePtr& getCvImagePtr();

            void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);

            void publish(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);

            void publish3D(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);
		
			void visualize(const std::vector<openpose_ros_msgs::OpenPoseHuman3D> humans);

			geometry_msgs::Point AddPoint(const openpose_ros_msgs::PointWithProb3D bodypart);

			bool PointISValid(const openpose_ros_msgs::PointWithProb3D& bodypart);

			openpose_ros_msgs::PointWithProb3D get3D(float, float, float);

			double Average(std::vector<double> v);

            void stop();
    };
}

#endif
