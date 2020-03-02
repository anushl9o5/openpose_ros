#include <openpose_ros_io.h>

// #define width 672
// #define height 376

using namespace openpose_ros;

OpenPoseROSIO::OpenPoseROSIO(OpenPose &openPose): nh_("/openpose_ros_node"), it_(nh_)
{
    // Subscribe to input video feed and publish human lists as output
    std::string image_topic;
    std::string depth_topic;
    std::string info_topic;
    std::string output_topic;
    std::string output_topic3D;
    std::string input_image_transport_type;

    nh_.param("image_topic", image_topic, std::string("/camera/image_raw"));
    nh_.param("depth_topic", depth_topic, std::string("/camera/depth/image_raw"));
    nh_.param("info_topic", info_topic, std::string("/camera/depth/camera_info"));
    nh_.param("input_image_transport_type", input_image_transport_type, std::string("raw"));
    nh_.param("output_topic", output_topic, std::string("/openpose_ros/human_list"));
    nh_.param("output_topic3D", output_topic3D, std::string("/openpose_ros/human_list3D"));
    nh_.param("display_output", display_output_flag_, false);
    nh_.param("print_keypoints", print_keypoints_flag_, false);
    nh_.param("save_original_video", save_original_video_flag_, false);
    nh_.param("save_openpose_video", save_openpose_video_flag_, false);
    nh_.param("original_video_file_name", original_video_file_name_, std::string(""));
    nh_.param("openpose_video_file_name", openpose_video_file_name_, std::string(""));
    nh_.param("video_fps", video_fps_, 10);

    image_sub_ = it_.subscribe(image_topic, 1, &OpenPoseROSIO::processImage, this, image_transport::TransportHints(input_image_transport_type));
    depth_sub_ = it_.subscribe(depth_topic, 1, &OpenPoseROSIO::convertDepth, this, image_transport::TransportHints(input_image_transport_type));
	info_sub = nh_.subscribe(info_topic, 1, &OpenPoseROSIO::get_CamInfo, this);

    openpose_human_list_pub_ = nh_.advertise<openpose_ros_msgs::OpenPoseHumanList>(output_topic, 10);
    openpose_human_list_pub_3D_ = nh_.advertise<openpose_ros_msgs::OpenPoseHumanList3D>(output_topic3D, 10);

    marker_pub = nh_.advertise<visualization_msgs::Marker>("/openpose_ros/skeleton_3d/visualization_markers", 10);
    skeleton_pub = nh_.advertise<visualization_msgs::Marker>("/openpose_ros/skeleton_3d/visualization_skeleton", 10);

    cv_img_ptr_ = nullptr;
    cv_depth_ptr_ = nullptr;

	VIS_body = true;
	VIS_face = false;
	VIS_right = false;
	VIS_left = true;

	//Default params is according to RS-D415
	fx = 608.007;
	fy = 608.007;	
	cx = 309.006;
	cy = 251.755;

    openpose_ = &openPose;

    if(save_original_video_flag_)
    {
        if(original_video_file_name_.empty())
        {
            std::cout << "No original video filename was provided. Not saving original video." << std::endl; 
            save_original_video_flag_ = false;
        }
        else
        {
            original_video_writer_initialized_ = false;
        }
    }
    if(save_openpose_video_flag_)
    {
        if(openpose_video_file_name_.empty())
        {
            std::cout << "No openpose video filename was provided. Not saving openpose video." << std::endl; 
            save_openpose_video_flag_ = false;
        }
        else
        {
            openpose_video_writer_initialized_ = false;
        }
    }
}

void OpenPoseROSIO::processImage(const sensor_msgs::ImageConstPtr& msg)
{
    convertImage(msg);
    std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> datumToProcess = createDatum();

    bool successfullyEmplaced = openpose_->waitAndEmplace(datumToProcess);
    
    // Pop frame
    std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> datumProcessed;
    if (successfullyEmplaced && openpose_->waitAndPop(datumProcessed))
    {
        if(display_output_flag_)
        {
            display(datumProcessed);
        }
        if(print_keypoints_flag_)
        {
            printKeypoints(datumProcessed);
        }
        if(save_original_video_flag_)
        {
            saveOriginalVideo(datumToProcess);
        }
        if(save_openpose_video_flag_)
        {
            saveOpenPoseVideo(datumProcessed);
        }
	if(FLAGS_depth)
	{
            publish3D(datumProcessed);
	}
	else
	{
            publish(datumProcessed);
	}	
    }
    else
    {
        op::opLog("Processed datum could not be emplaced.", op::Priority::High,
                __LINE__, __FUNCTION__, __FILE__);
    }
}

void OpenPoseROSIO::convertImage(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_img_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        image_header_ = msg->header;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void OpenPoseROSIO::convertDepth(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_depth_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1); //TYPE_32FC1
        depth_header_ = msg->header;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void OpenPoseROSIO::get_CamInfo(const sensor_msgs::CameraInfo& msg)
{
	fx = msg.K[0];
	fy = msg.K[4];
	cx = msg.K[2];
	cy = msg.K[5];
}


std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> OpenPoseROSIO::createDatum()
{
    // Close program when empty frame
    if (cv_img_ptr_ == nullptr)
    {
        return nullptr;
    }
    else // if (cv_img_ptr_ == nullptr)
    {
        // Create new datum
        auto datumsPtr = std::make_shared<std::vector<std::shared_ptr<op::Datum>>>();
        datumsPtr->emplace_back();
        auto& datumPtr = datumsPtr->at(0);
        datumPtr = std::make_shared<op::Datum>();

        // Fill datum
        datumPtr->cvInputData = OP_CV2OPCONSTMAT(cv_img_ptr_->image);

        return datumsPtr;
    }
}

bool OpenPoseROSIO::display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    // User's displaying/saving/other processing here
        // datum.cvOutputData: rendered frame with pose or heatmaps
        // datum.poseKeypoints: Array<float> with the estimated pose
    char key = ' ';
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        cv::imshow("User worker GUI", OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvOutputData));
        // Display image and sleeps at least 1 ms (it usually sleeps ~5-10 msec to display the image)
        key = (char)cv::waitKey(1);
    }
    else
        op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
    return (key == 27);
}

bool OpenPoseROSIO::saveOriginalVideo(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    char key = ' ';
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        cv::Mat current_image = OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvInputData);
        if(!current_image.empty())
        {
            if(!original_video_writer_initialized_)
            {
                original_video_writer_ = cv::VideoWriter(original_video_file_name_, CV_FOURCC('M','J','P','G'), video_fps_, current_image.size());
                original_video_writer_initialized_ = true;
            }   
            original_video_writer_.write(current_image);
        }
    }
    else
        op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
    return (key == 27);
}

bool OpenPoseROSIO::saveOpenPoseVideo(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    char key = ' ';
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        cv::Mat current_image = OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvOutputData);
        if(!current_image.empty())
        {
            if(!openpose_video_writer_initialized_)
            {
                openpose_video_writer_ = cv::VideoWriter(openpose_video_file_name_, CV_FOURCC('M','J','P','G'), video_fps_, current_image.size());
                openpose_video_writer_initialized_ = true;
            }   
            openpose_video_writer_.write(current_image);
        }
    }
    else
        op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
    return (key == 27);
}

cv_bridge::CvImagePtr& OpenPoseROSIO::getCvImagePtr()
{
    return cv_img_ptr_;
}

void OpenPoseROSIO::printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    // Example: How to use the pose keypoints
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        op::opLog("\nKeypoints:");
        // Accesing each element of the keypoints
        const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
        op::opLog("Person pose keypoints:");
        for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
        {
            op::opLog("Person " + std::to_string(person) + " (x, y, score):");
            for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
            {
                std::string valueToPrint;
                for (auto xyscore = 0 ; xyscore < poseKeypoints.getSize(2) ; xyscore++)
                {
                    valueToPrint += std::to_string(   poseKeypoints[{person, bodyPart, xyscore}]   ) + " ";
                }
                op::opLog(valueToPrint);
            }
        }
        op::opLog(" ");
        // Alternative: just getting std::string equivalent
        if(FLAGS_face)
        {
            op::opLog("Face keypoints: " + datumsPtr->at(0)->faceKeypoints.toString(), op::Priority::High);
        }
        if(FLAGS_hand)
        {
            op::opLog("Left hand keypoints: " + datumsPtr->at(0)->handKeypoints[0].toString(), op::Priority::High);
            op::opLog("Right hand keypoints: " + datumsPtr->at(0)->handKeypoints[1].toString(), op::Priority::High);
        }
        // Heatmaps
        const auto& poseHeatMaps = datumsPtr->at(0)->poseHeatMaps;
        if (!poseHeatMaps.empty())
        {
            op::opLog("Pose heatmaps size: [" + std::to_string(poseHeatMaps.getSize(0)) + ", "
                    + std::to_string(poseHeatMaps.getSize(1)) + ", "
                    + std::to_string(poseHeatMaps.getSize(2)) + "]");
            const auto& faceHeatMaps = datumsPtr->at(0)->faceHeatMaps;
            op::opLog("Face heatmaps size: [" + std::to_string(faceHeatMaps.getSize(0)) + ", "
                    + std::to_string(faceHeatMaps.getSize(1)) + ", "
                    + std::to_string(faceHeatMaps.getSize(2)) + ", "
                    + std::to_string(faceHeatMaps.getSize(3)) + "]");
            const auto& handHeatMaps = datumsPtr->at(0)->handHeatMaps;
            op::opLog("Left hand heatmaps size: [" + std::to_string(handHeatMaps[0].getSize(0)) + ", "
                    + std::to_string(handHeatMaps[0].getSize(1)) + ", "
                    + std::to_string(handHeatMaps[0].getSize(2)) + ", "
                    + std::to_string(handHeatMaps[0].getSize(3)) + "]");
            op::opLog("Right hand heatmaps size: [" + std::to_string(handHeatMaps[1].getSize(0)) + ", "
                    + std::to_string(handHeatMaps[1].getSize(1)) + ", "
                    + std::to_string(handHeatMaps[1].getSize(2)) + ", "
                    + std::to_string(handHeatMaps[1].getSize(3)) + "]");
        }
    }
    else
        op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
}

bool OpenPoseROSIO::PointISValid(const openpose_ros_msgs::PointWithProb3D& bodypart)
{
  if (std::isnan(bodypart.z) || std::isinf(bodypart.z) || std::isnan(bodypart.x) || std::isinf(bodypart.x) || std::isnan(bodypart.y) || std::isinf(bodypart.y)){return false;}
  else{return true;}
}

geometry_msgs::Point OpenPoseROSIO::AddPoint(const openpose_ros_msgs::PointWithProb3D bodypart)
{
  geometry_msgs::Point p;
  p.x = (bodypart.x - cx) * (bodypart.z / (fx*1000.0));
  p.y = (bodypart.y - cy) * (bodypart.z / (fy*1000.0));
  p.z = bodypart.z/1000.0;
   
  return p;
}

void OpenPoseROSIO::visualize(const std::vector<openpose_ros_msgs::OpenPoseHuman3D> humans)
{
	for(auto human = 0 ; human < humans.size() ; human++)
	{
		visualization_msgs::Marker marker;

		// Set boyjoints markers
		marker.header.frame_id = "/camera_color_optical_frame";
		marker.id = human;
		marker.ns = "joints";
		marker.header.stamp = ros::Time();
		// Markers will be spheres
		marker.type = visualization_msgs::Marker::SPHERE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		marker.scale.x = 0.05;
		marker.scale.y = 0.05;
		marker.scale.z = 0.05;
		// Joints are red
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;

		// Set marker duration in 150ms
		marker.lifetime = ros::Duration(0);
		
		visualization_msgs::Marker skeleton, hand_skeleton;

		skeleton.id  = human;
		skeleton.header.frame_id = "/camera_color_optical_frame";//zed_left_camera_optical_frame
		skeleton.ns = "skeleton";
		skeleton.header.stamp = ros::Time();
		// Skeleton will be lines
		skeleton.type = visualization_msgs::Marker::LINE_LIST;
		skeleton.scale.x = 0.03;
		skeleton.scale.y = 0.03;
		skeleton.scale.z = 0.03;
		// Skeleton is blue
		skeleton.color.a = 1.0;
		skeleton.color.r = 0.0;
		skeleton.color.g = 0.0;
		skeleton.color.b = 1.0;

		hand_skeleton.id  = human;
		hand_skeleton.header.frame_id = "/camera_color_optical_frame";//zed_left_camera_optical_frame
		hand_skeleton.ns = "hand_skeleton";
		hand_skeleton.header.stamp = ros::Time();
		// Skeleton will be lines
		hand_skeleton.type = visualization_msgs::Marker::LINE_LIST;
		hand_skeleton.scale.x = 0.03;
		hand_skeleton.scale.y = 0.03;
		hand_skeleton.scale.z = 0.03;
		// Skeleton is blue
		hand_skeleton.color.a = 1.0;
		hand_skeleton.color.r = 0.0;
		hand_skeleton.color.g = 1.0;
		hand_skeleton.color.b = 0.0;


		// Set skeleton lifetime
		skeleton.lifetime = ros::Duration(0);

		if(VIS_body)
		{
			for (auto handkeys=0 ; handkeys < humans.at(human).body_key_points_with_prob.size() ; handkeys++)
			{
				if(PointISValid(humans.at(human).body_key_points_with_prob.at(handkeys)))
				{
					marker.points.push_back(AddPoint(humans.at(human).body_key_points_with_prob.at(handkeys)));
			        if(handkeys > 0)
					{
						if(handkeys == 1 || handkeys == 14 || handkeys == 15)
						{
							if(PointISValid(humans.at(human).body_key_points_with_prob.at(0)))
							{
								skeleton.points.push_back(AddPoint(humans.at(human).body_key_points_with_prob.at(0)));
								skeleton.points.push_back(AddPoint(humans.at(human).body_key_points_with_prob.at(handkeys)));
							}
					
						}

						else if(handkeys == 2 || handkeys == 5 || handkeys == 8 || handkeys == 11)
						{
							if(PointISValid(humans.at(human).body_key_points_with_prob.at(1)))
							{
								skeleton.points.push_back(AddPoint(humans.at(human).body_key_points_with_prob.at(1)));
								skeleton.points.push_back(AddPoint(humans.at(human).body_key_points_with_prob.at(handkeys)));
							}
						}	
						else
						{
							if(PointISValid(humans.at(human).body_key_points_with_prob.at(handkeys-1)))
							{
								skeleton.points.push_back(AddPoint(humans.at(human).body_key_points_with_prob.at(handkeys-1)));
								skeleton.points.push_back(AddPoint(humans.at(human).body_key_points_with_prob.at(handkeys)));
							}
						}
					}
				}
			}	
		}

		if(FLAGS_face)
		{
			if(VIS_face)
			{
				for (auto handkeys=0 ; handkeys < humans.at(human).face_key_points_with_prob.size() ; handkeys++)
				{
					if(PointISValid(humans.at(human).face_key_points_with_prob.at(handkeys)))
					{
						marker.points.push_back(AddPoint(humans.at(human).face_key_points_with_prob.at(handkeys)));
					    if(handkeys > 0)
						{
							if(handkeys == 1 || handkeys == 5 || handkeys == 9 || handkeys == 13 || handkeys == 17)
							{
								if(PointISValid(humans.at(human).face_key_points_with_prob.at(0)))
								{
									skeleton.points.push_back(AddPoint(humans.at(human).face_key_points_with_prob.at(0)));
									skeleton.points.push_back(AddPoint(humans.at(human).face_key_points_with_prob.at(handkeys)));
								}
							}	
							else
							{
								if(PointISValid(humans.at(human).face_key_points_with_prob.at(handkeys-1)))
								{
									skeleton.points.push_back(AddPoint(humans.at(human).face_key_points_with_prob.at(handkeys-1)));
									skeleton.points.push_back(AddPoint(humans.at(human).face_key_points_with_prob.at(handkeys)));
								}
							}
						}
					}
				}	
			}
		}

		if(FLAGS_hand)
		{
			// 3D skeleton for Right-Hand Keypoints
			if(VIS_right)
			{
				for (auto handkeys=0 ; handkeys < humans.at(human).right_hand_key_points_with_prob.size() ; handkeys++)
				{
					if(PointISValid(humans.at(human).right_hand_key_points_with_prob.at(handkeys)))
					{
						marker.points.push_back(AddPoint(humans.at(human).right_hand_key_points_with_prob.at(handkeys)));
				        if(handkeys > 0)
						{
							if(handkeys == 1 || handkeys == 5 || handkeys == 9 || handkeys == 13 || handkeys == 17)
							{
								if(PointISValid(humans.at(human).right_hand_key_points_with_prob.at(0)))
								{
									hand_skeleton.points.push_back(AddPoint(humans.at(human).right_hand_key_points_with_prob.at(0)));
									hand_skeleton.points.push_back(AddPoint(humans.at(human).right_hand_key_points_with_prob.at(handkeys)));
								}
							}	
							/*else
							{
								if(PointISValid(humans.at(human).right_hand_key_points_with_prob.at(handkeys-1)))
								{
									skeleton.points.push_back(AddPoint(humans.at(human).right_hand_key_points_with_prob.at(handkeys-1)));
									skeleton.points.push_back(AddPoint(humans.at(human).right_hand_key_points_with_prob.at(handkeys)));
								}
							}*/
						}
					}
				}
			}
		
			// 3D skeleton for Left-Hand Keypoints
			if(VIS_left)
			{ 
				for (auto handkeys=0 ; handkeys < humans.at(human).left_hand_key_points_with_prob.size() ; handkeys++)
				{
					if (PointISValid(humans.at(human).left_hand_key_points_with_prob.at(handkeys)))
					{
						marker.points.push_back(AddPoint(humans.at(human).left_hand_key_points_with_prob.at(handkeys)));
				        if(handkeys > 0)
						{
							if(handkeys == 1 || handkeys == 5 || handkeys == 9 || handkeys == 13 || handkeys == 17)
							{
								if(PointISValid(humans.at(human).left_hand_key_points_with_prob.at(0)))
								{
									hand_skeleton.points.push_back(AddPoint(humans.at(human).left_hand_key_points_with_prob.at(0)));
									hand_skeleton.points.push_back(AddPoint(humans.at(human).left_hand_key_points_with_prob.at(handkeys)));
								}
							}	
							/*else
							{ 
								if(PointISValid(humans.at(human).left_hand_key_points_with_prob.at(handkeys-1)))
								{
									skeleton.points.push_back(AddPoint(humans.at(human).left_hand_key_points_with_prob.at(handkeys-1)));
									skeleton.points.push_back(AddPoint(humans.at(human).left_hand_key_points_with_prob.at(handkeys)));
								}
							}*/
						}
					}
				}
			}
		}
		
		marker_pub.publish(marker);
		skeleton_pub.publish(skeleton);
		skeleton_pub.publish(hand_skeleton);

	}
}

double OpenPoseROSIO::Average(std::vector<double> v)
{
	double total = 0.0;
	double size  = 0.0;
	for (int n = 0; n < v.size(); n++)
	{
		total += v[n];size++;
	}

	return total/size;
}

openpose_ros_msgs::PointWithProb3D OpenPoseROSIO::get3D(float x_pixel, float y_pixel, float prob)
{
    openpose_ros_msgs::PointWithProb3D bodypart_depth;
    auto pt_z = cv_depth_ptr_->image.at<float>(cv::Point(x_pixel, y_pixel));

  	if (std::isnan(pt_z) || std::isinf(pt_z))
  	{
  		bodypart_depth.x = NAN;
  		bodypart_depth.y = NAN;
  		bodypart_depth.z = NAN;
		bodypart_depth.prob = 0;
  	}

	else
	{
        bodypart_depth.x = x_pixel;
        bodypart_depth.y = y_pixel;
        bodypart_depth.z = pt_z;
		bodypart_depth.prob = prob;

        /*std::cerr << " real world coordinates (x,y,z): " << std::endl;
        std::cerr << "( " << bodypart_depth.x << ", " 
           	          << bodypart_depth.y << ", "
                          << bodypart_depth.z << ")" << std::endl;	*/
	}

	return bodypart_depth;

}

void OpenPoseROSIO::publish3D(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    if (datumsPtr != nullptr && !datumsPtr->empty() && cv_depth_ptr_ != nullptr)
    {
        const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
        const auto& faceKeypoints = datumsPtr->at(0)->faceKeypoints;
        const auto& leftHandKeypoints = datumsPtr->at(0)->handKeypoints[0];
        const auto& rightHandKeypoints = datumsPtr->at(0)->handKeypoints[1];
        std::vector<op::Rectangle<float>>& face_rectangles = datumsPtr->at(0)->faceRectangles;

        openpose_ros_msgs::OpenPoseHumanList3D human_list_msg;
        human_list_msg.header.stamp = ros::Time::now();
        human_list_msg.image_header = image_header_;
        human_list_msg.num_humans = poseKeypoints.getSize(0);
        
        std::vector<openpose_ros_msgs::OpenPoseHuman3D> human_list(poseKeypoints.getSize(0));

        for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
        {
            openpose_ros_msgs::OpenPoseHuman3D human;

            double body_min_x = -1;
            double body_max_x = -1;
            double body_min_y = -1;
            double body_max_y = -1;
            double body_min_z = -1;
            double body_max_z = -1;

            int num_body_key_points_with_non_zero_prob = 0;
            for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
            {
                openpose_ros_msgs::PointWithProb3D body_point_with_prob;

                if(poseKeypoints[{person, bodyPart, 2}] < 0.1)
                {
					body_point_with_prob.x = NAN;
		            body_point_with_prob.y = NAN;
					body_point_with_prob.z = NAN;
		            body_point_with_prob.prob = 0;
					
                }
				else
				{
					body_point_with_prob = get3D(poseKeypoints[{person, bodyPart, 0}], poseKeypoints[{person, bodyPart, 1}], poseKeypoints[{person, bodyPart, 2}]);
                    num_body_key_points_with_non_zero_prob++;

                    if(body_min_x == -1 || body_point_with_prob.x < body_min_x)
                    {
                        body_min_x = body_point_with_prob.x;
                    }
                    if(body_point_with_prob.x > body_max_x)
                    {
                        body_max_x = body_point_with_prob.x;
                    }

                    if(body_min_y == -1 || body_point_with_prob.y < body_min_y)
                    {
                        body_min_y = body_point_with_prob.y;
                    }
                    if(body_point_with_prob.y > body_max_y)
                    {
                        body_max_y = body_point_with_prob.y;
                    }

				}
                human.body_key_points_with_prob.at(bodyPart) = body_point_with_prob;
            }

            human.num_body_key_points_with_non_zero_prob = num_body_key_points_with_non_zero_prob;
            human.body_bounding_box.x = body_min_x;
            human.body_bounding_box.y = body_min_y;
            human.body_bounding_box.width = body_max_x - body_min_x;
            human.body_bounding_box.height = body_max_y - body_min_y;

            if(FLAGS_face)
            {
                int num_face_key_points_with_non_zero_prob = 0;

                for (auto facePart = 0 ; facePart < faceKeypoints.getSize(1) ; facePart++)
                {
					openpose_ros_msgs::PointWithProb3D face_point_with_prob;

					if(faceKeypoints[{person, facePart, 2}] < 0.1)
					{
		                face_point_with_prob.x = NAN;
		                face_point_with_prob.y = NAN;
				        face_point_with_prob.z = NAN;
		                face_point_with_prob.prob = 0;
		                human.face_key_points_with_prob.at(facePart) = face_point_with_prob;
					}
					else
					{ 
		                face_point_with_prob = get3D(faceKeypoints[{person, facePart, 0}], faceKeypoints[{person, facePart, 1}], faceKeypoints[{person, facePart, 2}]);
		                num_face_key_points_with_non_zero_prob++;
		                human.face_key_points_with_prob.at(facePart) = face_point_with_prob;
					}
                }  
                human.num_face_key_points_with_non_zero_prob = num_face_key_points_with_non_zero_prob;

                openpose_ros_msgs::BoundingBox face_bounding_box;
                face_bounding_box.x = face_rectangles.at(person).x;
                face_bounding_box.y = face_rectangles.at(person).y;
                face_bounding_box.width = face_rectangles.at(person).width;
                face_bounding_box.height = face_rectangles.at(person).height;
                human.face_bounding_box = face_bounding_box;
            }
            
            if(FLAGS_hand)
            {

                int num_right_hand_key_points_with_non_zero_prob = 0;
                int num_left_hand_key_points_with_non_zero_prob = 0;

                for (auto handPart = 0 ; handPart < rightHandKeypoints.getSize(1) ; handPart++)
                {
                    openpose_ros_msgs::PointWithProb3D right_hand_point_with_prob;

					if(rightHandKeypoints[{person, handPart, 2}] < 0.1)
					{
		                right_hand_point_with_prob.x = NAN;
		                right_hand_point_with_prob.y = NAN;
		                right_hand_point_with_prob.z = NAN;
		                right_hand_point_with_prob.prob = 0;
		                human.right_hand_key_points_with_prob.at(handPart) = right_hand_point_with_prob;						
					}

					else
					{ 
						right_hand_point_with_prob = get3D(rightHandKeypoints[{person, handPart, 0}], rightHandKeypoints[{person, handPart, 1}], rightHandKeypoints[{person, handPart, 2}]);
		                num_right_hand_key_points_with_non_zero_prob++;
						human.right_hand_key_points_with_prob.at(handPart) = right_hand_point_with_prob;
					}
				}
                human.num_right_hand_key_points_with_non_zero_prob = num_right_hand_key_points_with_non_zero_prob;

                for (auto handPart = 0 ; handPart < leftHandKeypoints.getSize(1) ; handPart++)
                {
                    openpose_ros_msgs::PointWithProb3D left_hand_point_with_prob;

					if(leftHandKeypoints[{person, handPart, 2}] < 0.1)
					{
		                left_hand_point_with_prob.x = NAN;
		                left_hand_point_with_prob.y = NAN;
		                left_hand_point_with_prob.z = NAN;
		                left_hand_point_with_prob.prob = 0;
		                human.left_hand_key_points_with_prob.at(handPart) = left_hand_point_with_prob;						
					}

					else
					{
						left_hand_point_with_prob = get3D(leftHandKeypoints[{person, handPart, 0}], leftHandKeypoints[{person, handPart, 1}], leftHandKeypoints[{person, handPart, 2}]);
		                num_left_hand_key_points_with_non_zero_prob++;
		                human.left_hand_key_points_with_prob.at(handPart) = left_hand_point_with_prob;
					}
                }
                human.num_left_hand_key_points_with_non_zero_prob = num_left_hand_key_points_with_non_zero_prob;
            }

            human_list.at(person) = human;
        }

        human_list_msg.human_list = human_list;

        openpose_human_list_pub_3D_.publish(human_list_msg);
		visualize(human_list);

    }
    else
        op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
}

void OpenPoseROSIO::publish(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
        const auto& faceKeypoints = datumsPtr->at(0)->faceKeypoints;
        const auto& leftHandKeypoints = datumsPtr->at(0)->handKeypoints[0];
        const auto& rightHandKeypoints = datumsPtr->at(0)->handKeypoints[1];
        std::vector<op::Rectangle<float>>& face_rectangles = datumsPtr->at(0)->faceRectangles;

        openpose_ros_msgs::OpenPoseHumanList human_list_msg;
        human_list_msg.header.stamp = ros::Time::now();
        human_list_msg.image_header = image_header_;
        human_list_msg.num_humans = poseKeypoints.getSize(0);
        
        std::vector<openpose_ros_msgs::OpenPoseHuman> human_list(poseKeypoints.getSize(0));

        for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
        {
            openpose_ros_msgs::OpenPoseHuman human;

            double body_min_x = -1;
            double body_max_x = -1;
            double body_min_y = -1;
            double body_max_y = -1;

            int num_body_key_points_with_non_zero_prob = 0;
            for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
            {
                openpose_ros_msgs::PointWithProb body_point_with_prob;
                openpose_ros_msgs::PointWithProb3D body_point_with_prob3D;
                body_point_with_prob.x = poseKeypoints[{person, bodyPart, 0}];
                body_point_with_prob.y = poseKeypoints[{person, bodyPart, 1}];
                body_point_with_prob.prob = poseKeypoints[{person, bodyPart, 2}];

                if(body_point_with_prob.prob > 0)
                {
                    num_body_key_points_with_non_zero_prob++;

                    if(body_min_x == -1 || body_point_with_prob.x < body_min_x)
                    {
                        body_min_x = body_point_with_prob.x;
                    }
                    if(body_point_with_prob.x > body_max_x)
                    {
                        body_max_x = body_point_with_prob.x;
                    }

                    if(body_min_y == -1 || body_point_with_prob.y < body_min_y)
                    {
                        body_min_y = body_point_with_prob.y;
                    }
                    if(body_point_with_prob.y > body_max_y)
                    {
                        body_max_y = body_point_with_prob.y;
                    }
                }
                human.body_key_points_with_prob.at(bodyPart) = body_point_with_prob;
            }
            human.num_body_key_points_with_non_zero_prob = num_body_key_points_with_non_zero_prob;
            human.body_bounding_box.x = body_min_x;
            human.body_bounding_box.y = body_min_y;
            human.body_bounding_box.width = body_max_x - body_min_x;
            human.body_bounding_box.height = body_max_y - body_min_y;

            if(FLAGS_face)
            {
                int num_face_key_points_with_non_zero_prob = 0;

                for (auto facePart = 0 ; facePart < faceKeypoints.getSize(1) ; facePart++)
                {
                    openpose_ros_msgs::PointWithProb face_point_with_prob;
                    face_point_with_prob.x = faceKeypoints[{person, facePart, 0}];
                    face_point_with_prob.y = faceKeypoints[{person, facePart, 1}];
                    face_point_with_prob.prob = faceKeypoints[{person, facePart, 2}];
                    if(face_point_with_prob.prob > 0)
                    {
                        num_face_key_points_with_non_zero_prob++;
                    }
                    human.face_key_points_with_prob.at(facePart) = face_point_with_prob;
                }  
                human.num_face_key_points_with_non_zero_prob = num_face_key_points_with_non_zero_prob;

                openpose_ros_msgs::BoundingBox face_bounding_box;
                face_bounding_box.x = face_rectangles.at(person).x;
                face_bounding_box.y = face_rectangles.at(person).y;
                face_bounding_box.width = face_rectangles.at(person).width;
                face_bounding_box.height = face_rectangles.at(person).height;
                human.face_bounding_box = face_bounding_box;
            }
            
            if(FLAGS_hand)
            {

                int num_right_hand_key_points_with_non_zero_prob = 0;
                int num_left_hand_key_points_with_non_zero_prob = 0;

                for (auto handPart = 0 ; handPart < rightHandKeypoints.getSize(1) ; handPart++)
                {
                    openpose_ros_msgs::PointWithProb right_hand_point_with_prob;
                    openpose_ros_msgs::PointWithProb left_hand_point_with_prob;
                    right_hand_point_with_prob.x = rightHandKeypoints[{person, handPart, 0}];
                    right_hand_point_with_prob.y = rightHandKeypoints[{person, handPart, 1}];
                    right_hand_point_with_prob.prob = rightHandKeypoints[{person, handPart, 2}];
                    if(right_hand_point_with_prob.prob > 0)
                    {
                        num_right_hand_key_points_with_non_zero_prob++;
                    }
                    left_hand_point_with_prob.x = leftHandKeypoints[{person, handPart, 0}];
                    left_hand_point_with_prob.y = leftHandKeypoints[{person, handPart, 1}];
                    left_hand_point_with_prob.prob = leftHandKeypoints[{person, handPart, 2}];
                    if(left_hand_point_with_prob.prob > 0)
                    {
                        num_left_hand_key_points_with_non_zero_prob++;
                    }
                    human.right_hand_key_points_with_prob.at(handPart) = right_hand_point_with_prob;
                    human.left_hand_key_points_with_prob.at(handPart) = left_hand_point_with_prob;
                }
                human.num_right_hand_key_points_with_non_zero_prob = num_right_hand_key_points_with_non_zero_prob;
                human.num_left_hand_key_points_with_non_zero_prob = num_left_hand_key_points_with_non_zero_prob;
            }

            human_list.at(person) = human;
        }

        human_list_msg.human_list = human_list;
        openpose_human_list_pub_.publish(human_list_msg);

    }
    else
        op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
}

void OpenPoseROSIO::stop()
{
    if(save_original_video_flag_)
    {
        original_video_writer_.release();
    }
    if(save_openpose_video_flag_)
    {
        openpose_video_writer_.release();
    }
}
