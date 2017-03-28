#include <manipulation.h>

Manipulation::Manipulation(std::string& robot)
{
    robot_name_ = robot;
    yInfo() << " Manipulation Constructor";
    
    bool port_connection;
    if(!yarp::os::Network::initialized())
        yarp::os::Network::init();
    
    image_input_port_ = new yarp::os::BufferedPort<yarp::sig::ImageOf< yarp::sig::PixelBgr >>;
    if(image_input_port_->open("/manipulation/image/input:i"))
    {
        if(yarp::os::Network::checkNetwork())
        {
            camera_port_name_ = "/" + robot_name_ + "/gazebo_yarp_plugin/camera:o";
            port_connection = yarp::os::Network::connect(camera_port_name_,image_input_port_->getName());
            if(!port_connection) yError() << "Cannot connect to the port  /gazebo_yarp_plugin/camera:o";
        }
        else yError() << "Check if YARP network is available!";

    }
    else yError() << "Failed to open the input image port!";
    
    left_wrench_input_port_ = new yarp::os::BufferedPort<yarp::sig::Vector>;
    if(left_wrench_input_port_->open("/manipulation/left_ft/input:i"))
    {
        if(yarp::os::Network::checkNetwork())
        {
            left_wrench_port_name_ = "/" + robot_name_ + "/first_link_handle/analog:o";
            port_connection = yarp::os::Network::connect(left_wrench_port_name_,left_wrench_input_port_->getName());
            if(!port_connection) yError() << "Cannot connect to the port  /floating_base_1R1P_2Link/first_link_handle/analog:o";
        }
        else yError() << "Check if YARP network is available!";
    }
    else yError() << "Failed to open the left ft input port!";
    
    right_wrench_input_port_ = new yarp::os::BufferedPort<yarp::sig::Vector>;
    if(right_wrench_input_port_->open("/manipulation/right_ft/input:i"))
    {
        if(yarp::os::Network::checkNetwork())
        {
            right_wrench_port_name_ = "/" + robot_name_ + "/second_link_handle/analog:o";
            port_connection = yarp::os::Network::connect(right_wrench_port_name_,right_wrench_input_port_->getName());
            if(!port_connection) yError() << "Cannot connect to the port  /floating_base_1R1P_2Link/second_link_handle/analog:o";
        }
        else yError() << "Check if YARP network is available!";
    }
    else yError() << "Failed to open the right ft input port!";
    
    /*external_wrench_output_port_ = new yarp::os::BufferedPort<yarp::os::Bottle>;
    if(external_wrench_output_port_->open("/manipulation/multiExternalWrench/output:o"))
    {
        if(yarp::os::Network::checkNetwork())
        {
            external_wrench_port_name_ = "/" + robot_name_ + "/applyMultiExternalWrench/rpc:i";
            port_connection = yarp::os::Network::connect(external_wrench_output_port_->getName(),external_wrench_port_name_);
            if(!port_connection) yError() << "Cannot connect to the port  /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i";
        }
        else yError() << "Check if YARP network is available!";
    }
    else yError() << "Failed to open the right extenal wrench output port!";*/
    
    if(port_connection)
    {
        //Initialization for ArUco markers
        marker_dictionary_ = cv::aruco::generateCustomDictionary(number_of_markers_,marker_dimension_);
        
        char cwd[1024];
        if(getcwd(cwd,sizeof(cwd)) != NULL)
        {
            current_directory_ = cwd;
            current_directory_.replace(current_directory_.find("build"),5,"data");
            data_directory_ = current_directory_;
            std::cout << "Data directory : " << data_directory_ << std::endl;
            
            current_directory_.replace(current_directory_.find("data"),5,"out_camera_data.yml");
            calibration_file_name_ = current_directory_;
            std::cout << "Calibration file : " << calibration_file_name_ << std::endl; 
            
        }
        else std::cerr << "getcwd() error!" << std::endl;
        
        log_data_ = false;
        loadCameraCalibParams();
        
        //Initializing time stamp
        time_stamp_ = yarp::os::Stamp();
        time_init_ = time_stamp_.getTime();
        duration = 0;
        yInfo() << "Initialization successful";
        
        //applyWrenches();
    }
    else yError() << "Failed to initialize manipulation, check if the model is available in gazebo";
}

void Manipulation::loadCameraCalibParams()
{
    //Camera Calibration of Gazebo camera plugin
    cv::Mat CM = (cv::Mat_<double>(3,3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
    camera_matrix_ = CM;
    
    cv::Mat DM = (cv::Mat_<double>(5,1) << -0.25, 0.12, 0.0, -0.00028, -0.00005);
    dist_coeffs_ = DM;
    
    //This uses calibration from laptop camera
    /*fs.open(calibration_file_name_,cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix_;
    fs["distortion_coefficients"] >> dist_coeffs_;
    fs.release();*/
    
    if(camera_matrix_.empty() || dist_coeffs_.empty())
    {
        std::cout << "Error in loading camera calibration parameters!" << std::endl;
        calib_success_ = 0;
    }
    else
    {
        std::cout << "Camera calibration parameters loaded Successfully" << std::endl;
        //std::cout << "Camera Matrix : " << camera_matrix_ << std::endl;
        //std::cout << "Distortion Coefficients : " << dist_coeffs_ << std::endl;
        calib_success_ = 1;
    }
}

void Manipulation::applyWrenches()
{
    yarp::os::Bottle& wrench = external_wrench_output_port_->prepare();
    wrench.clear();
    wrench.addString("first_link_handle");
    wrench.addDouble(0); 
    wrench.addDouble(0);
    wrench.addDouble(-10000);
    wrench.addDouble(0);
    wrench.addDouble(0);
    wrench.addDouble(0);
    wrench.addDouble(100);
    
    external_wrench_output_port_->write(true);
    
    yarp::os::Bottle& wrench1 = external_wrench_output_port_->prepare();
    wrench1.clear();
    wrench1.addString("second_link_handle");
    wrench1.addDouble(0); 
    wrench1.addDouble(0);
    wrench1.addDouble(-10000);
    wrench1.addDouble(0);
    wrench1.addDouble(0);
    wrench1.addDouble(0);
    wrench1.addDouble(100);
    
    external_wrench_output_port_->write(true);
}


bool Manipulation::detectMarkersAndComputePose()
{
    cv::aruco::detectMarkers(input_yarp_to_mat_image_,marker_dictionary_,marker_corners_,marker_ids_,detection_params_,rejected_candidates_);
    if(!marker_ids_.size() > 0)
    {
        //std::cout << "No markers detected!" << std::endl;
        marker_detect_success_=0;
        return false;
    }
    else 
    {
        //std::cout << "Markers detected" << std::endl;
        marker_detect_success_=1;
        
        //Detects all the identifiable markers
        cv::aruco::drawDetectedMarkers(input_yarp_to_mat_image_,marker_corners_,marker_ids_);
        if(calib_success_ == 1)
        {
            cv::aruco::estimatePoseSingleMarkers(marker_corners_,0.05,camera_matrix_,dist_coeffs_,rvecs,tvecs);
            for(int i=0; i < marker_ids_.size(); i++)
            {
                //Pose values of each marker
                cv::aruco::drawAxis(input_yarp_to_mat_image_,camera_matrix_,dist_coeffs_,rvecs[i],tvecs[i],0.1);
                //extractTrajectory(marker_ids_,rvecs,tvecs);
                getPoseInfo();
            }
        }
        else
        {
            std::cout << "Cannot compute the pose of detected Markers, Error in loading camera calibration parameters!" << std::endl;
            return false;
        }
    }
    return true;
}

void Manipulation::getPoseInfo()
{
    if(marker_ids_.size()-1 != 0)
    {
        for(int i=0; i < marker_ids_.size(); i++)
        { 
            //Sorting the marker ids
            if( !std::is_sorted(marker_ids_.begin(),marker_ids_.end()) )
            {
                std::cout << "Sorting marker pose vectors" << std::endl;
                //std::cout << "Actual Marker ids : " << marker_ids_ << "--->";
                //std::cout << rvecs << " , " << tvecs;
                
                sorted_rvecs.resize(rvecs.size());
                sorted_tvecs.resize(tvecs.size());
                
                //SORT the order of marker ids
                sorted_marker_ids=marker_ids_;
                std::sort(sorted_marker_ids.begin(),sorted_marker_ids.end());
                //std::cout << "Sorted Marker ids : " << sorted_marker_ids << "--->";
     
                for(int s=0; s < sorted_marker_ids.size(); s++)
                {
                    for(int p=0; p < marker_ids_.size(); p++)
                    {
                        if(sorted_marker_ids.at(s)==marker_ids_.at(p))
                        {
                            sorted_rvecs.at(s) = rvecs.at(p);
                            sorted_tvecs.at(s) = tvecs.at(p);
                        }
                    }
                }
                
                //std::cout << sorted_rvecs << " , " << sorted_tvecs;
                marker_ids_ = sorted_marker_ids;
                rvecs = sorted_rvecs;
                tvecs = sorted_tvecs;
                
                //Clearing the sorted vectors
                sorted_rvecs.clear();
                sorted_tvecs.clear();
                sorted_marker_ids.clear();
            }
            
            Eigen::Vector3d P;
            cv::cv2eigen(tvecs[i],P);
            
            //Rotational vectors in angels
            Eigen::Vector3d ang;
            cv::cv2eigen(rvecs[i],ang);
            
            if(log_data_ != true)
            {
                yInfo() <<  "[" << duration << "]" <<"Marker ID : " << marker_ids_.at(i) << " " << P(0) << " " << P(1) << " " << P(2) \
                        << " " << ang(0) << " " << ang(1) << " " << ang(2);
            }
            else
            {
                if(file_name_.is_open())
                {
                    file_name_ << " " << duration <<  " " << marker_ids_.at(i) << " " << P(0) << " " << P(1) << " " << P(2) \
                        << " " << ang(0) << " " << ang(1) << " " << ang(2);
                }
            }
        }
        getWrenchInfo();
    }
    else
    {
       std::cout << "Detected only single marker!" << std::endl;
    }
}


void Manipulation::extractTrajectory(std::vector<int>& marker_ids_,std::vector<cv::Vec3d>& rvecs,std::vector<cv::Vec3d>& tvecs)
{
    //std::cout << "Extracting trajectory" << std::endl;
    
    int dummy_track_length = marker_ids_.size()-1;
    if(dummy_track_length!=0)
    {
        //std::cout << "Detected more than one marker" << std::endl;
        //std::cout << "Track length : " << dummy_track_length << std::endl;
        
         //Track structure array variables
        boost::shared_ptr<Track> track_sptr {new Track[dummy_track_length],std::default_delete<Track[]>() };
        tracks.resize(dummy_track_length);
        
        //TODO Double check the index starting
        //This is fine because I need one less iteration than the markers id size
        for(int i=1; i < marker_ids_.size(); i++)
        { 
            
            //Sorting the marker ids
            if( !std::is_sorted(marker_ids_.begin(),marker_ids_.end()) )
            {
                //std::cout << "Actual Marker ids : " << marker_ids_ << "--->";
                //std::cout << rvecs << " , " << tvecs;
                
                sorted_rvecs.resize(rvecs.size());
                sorted_tvecs.resize(tvecs.size());
                
                //SORT the order of marker ids
                sorted_marker_ids=marker_ids_;
                std::sort(sorted_marker_ids.begin(),sorted_marker_ids.end());
                //std::cout << "Sorted Marker ids : " << sorted_marker_ids << "--->";
     
                for(int s=0; s < sorted_marker_ids.size(); s++)
                {
                    for(int p=0; p < marker_ids_.size(); p++)
                    {
                        if(sorted_marker_ids.at(s)==marker_ids_.at(p))
                        {
                            sorted_rvecs.at(s) = rvecs.at(p);
                            sorted_tvecs.at(s) = tvecs.at(p);
                        }
                    }
                }
                
                //std::cout << sorted_rvecs << " , " << sorted_tvecs;
                marker_ids_ = sorted_marker_ids;
                rvecs = sorted_rvecs;
                tvecs = sorted_tvecs;
                
                //Clearing the sorted vectors
                sorted_rvecs.clear();
                sorted_tvecs.clear();
                sorted_marker_ids.clear();
            }
            
            
            //Clear if only one time instance of observation is            
            //TODO update it with yarp time stamp
            observation_sptr->time = time_stamp_.getTime();
            //observation_sptr->links_rel_transformation.clear();

            
            //TODO Proper relative transformation implemented - Double check this
             //Translation vectors       
            Eigen::Vector3d P1,P2;
            cv::cv2eigen(tvecs[i-1],P1);
            cv::cv2eigen(tvecs[i],P2);
            
            //Rotational vectors in angels
            Eigen::Vector3d ang1,ang2;
            cv::cv2eigen(rvecs[i-1],ang1);
            cv::cv2eigen(rvecs[i],ang2);
            
            //This is differecen in orientation in RYP 
            //TODO Check this convention for ArUco markers
            Eigen::Matrix3f R1, R2;
            R1 = Eigen::AngleAxisf(ang1[0], Eigen::Vector3f::UnitZ())
                   * Eigen::AngleAxisf(ang1[1], Eigen::Vector3f::UnitY())
                   * Eigen::AngleAxisf(ang1[2], Eigen::Vector3f::UnitX());
           
            R2 = Eigen::AngleAxisf(ang2[0], Eigen::Vector3f::UnitZ())
                   * Eigen::AngleAxisf(ang2[1], Eigen::Vector3f::UnitY())
                   * Eigen::AngleAxisf(ang2[2], Eigen::Vector3f::UnitX());
           
            //Affine 3d Transformations
            Eigen::Affine3d T1,T2;
            T1.setIdentity();
            T1.translation() = P1;
            T1.rotate(Eigen::AngleAxisd(M_PI,ang1));
            
            T2.setIdentity();
            T2.translation() = P2;
            T2.rotate(Eigen::AngleAxisd(M_PI,ang2));
            
            //Computing relative transformation between marker poses
            Eigen::Affine3d rel_trans;
            rel_trans = T1.inverse()*T2;
            
            Eigen::Vector3d trans = rel_trans.translation();
            Eigen::Matrix3d rot = rel_trans.linear();
            Eigen::Quaterniond rot_q(rot);
            
            //Eigen::Vector3d euler_angels = rot.eulerAngles(0,1,2);
            
            //observation_sptr->links_rel_transformation.push_back(trans);
            //observation_sptr->links_rel_transformation.push_back(euler_angels);
            
            observation_sptr->transform.translation.x = trans[0];
            observation_sptr->transform.translation.y = trans[1];
            observation_sptr->transform.translation.z = trans[2];
            
            observation_sptr->transform.rotation.w = rot_q.w();
            observation_sptr->transform.rotation.x = rot_q.x();
            observation_sptr->transform.rotation.y = rot_q.y();
            observation_sptr->transform.rotation.z = rot_q.z();
        
            track_sptr.get()[i-1].obs.push_back(observation_sptr);
            std::string dummy_id = std::to_string(marker_ids_.at(i-1)) + std::to_string(marker_ids_.at(i));
            track_sptr.get()[i-1].id = dummy_id;
            //std::cout << "Track ID " << TRACK[i-1].id << std::endl;
            
            track_sptr.get()[i-1].modified = true;
        
            //This is of track length
            tracks.at(i-1) = track_sptr.get()[i-1];
            
            getTrajectoryInfo();
            
        }
    }
    else
    {
        std::cout << "Detected only single marker!" << std::endl;
        for(int i=0; i < tracks.size(); i++)
        {
            tracks.at(i).modified = false;
        }
    }
    
}

bool Manipulation::getTrajectoryInfo()
{
 
    //std::cout << "Getting trajectory information" << std::endl;
    //This should call extractTrajectory and display info
    if(marker_ids_.size()!=0)
    {        
        //This should be displayed only if new observations are added
        //std::cout << "track size : " << tracks.size() << std::endl; 
        for(int i=0; i < tracks.size(); i++)
        {
            if(tracks.at(i).modified==true)
            {
                for(int o = 0; o < tracks.at(i).obs.size(); o++)
                {
                    //std::string t = timeConversion(tracks.at(i).obs.at(o)->time);
                    if(log_data_!=true)
                    {
                        geometry_msgs::Transform dummy = tracks.at(i).obs.at(o)->transform;
                        
                        yInfo() << "Track ID : " << tracks.at(i).id << " ---> " << "[ " <<
                                               tracks.at(i).obs.at(o)->time << " ] "
                                               << dummy.translation.x << " " << dummy.translation.y << " " 
                                               << dummy.translation.z << " " << dummy.rotation.w << " " 
                                               << dummy.rotation.x << " " << dummy.rotation.y << " " <<
                                               dummy.rotation.z ;
                        
                    }
                    else
                    {
                        if(file_name_.is_open())
                        {
                            //std::cout << "file is open" << std::endl;
                            file_name_ << tracks.at(i).obs.at(o)->time << " ";
                            file_name_ << tracks.at(i).id << " ";
                            //Eigen::Vector3d dummy = tracks.at(i).obs.at(o)->links_rel_transformation.at(0);
                            geometry_msgs::Transform dummy = tracks.at(i).obs.at(o)->transform;
                            file_name_ << dummy.translation.x << " " << dummy.translation.y << " " << dummy.translation.z << " ";
                            file_name_ << dummy.rotation.w << " " << dummy.rotation.x << " " << dummy.rotation.y << " " 
                            << dummy.rotation.z;
                            //file_name_ << tracks.at(i).obs.at(o)->links_rel_transformation << std::endl;
                        }
                        else std::cerr << "file open error" << std::endl;
                    }
                }
            }
        }
        
        
    getWrenchInfo();
        
    }
    return true;
}

bool Manipulation::getWrenchInfo()
{
    if(log_data_ != true)
    {
        yInfo() << "Left wrench : " << left_wrench_->toString();
        yInfo() << "Right wrench : " << right_wrench_->toString();
        
    }
    else
    {
        //TODO Append it to the file in a same line a vision
        //file_name_ << " " << left_wrench_->toString()  << " " << right_wrench_->toString() << std::endl;
        
        double *left_data = left_wrench_->data();
        for(int i = 0; i < left_wrench_->size(); i++)
        {
            file_name_ << " " << *left_data;
            left_data++;
        }
        
        double *right_data = right_wrench_->data();
        for(int j = 0; j < right_wrench_->size(); j++)
        {
            file_name_ << " " << *right_data;
            right_data++;
        }
        file_name_ << std::endl;
    }
    
    
    return true;
}

bool Manipulation::displayImage(cv::Mat& image)
{
    cv::namedWindow("Input Image",CV_WINDOW_AUTOSIZE);
    cv::imshow("Input Image",image);
    char key = (char) cv::waitKey(1);
    if(key ==27)
    {
        yInfo() << "Esc key pressed!";
        return true;
    }
    else return false;
}

bool Manipulation::getSensoryInputs()
{
    
    //Sensory Inputs
    input_yarp_frame_ = image_input_port_->read();
    
    //TODO check which location will be better for a time stamp
    time_stamp_.update(); //Update the time stamp
    time_current_ = time_stamp_.getTime();
    duration = time_current_ - time_init_;
    time_init_ = time_current_;
    //yInfo() << "Duration : " << duration;
    
    //Reading new wrench values
    left_wrench_ = left_wrench_input_port_->read();
    right_wrench_ = right_wrench_input_port_->read();
    
    
    //conversion from yarp image to CV Mat
    IplImage *dummy = (IplImage*)(*input_yarp_frame_).getIplImage();
    input_yarp_to_mat_image_ = cv::cvarrToMat(dummy);
        
    //Getting time stamps
    
    //TODO THe timestamps obtained this way are not same, because of the current implementation
    //Try to  change this implementation later
    //image_input_port_->getEnvelope(image_ts_);
    //left_wrench_input_port_->getEnvelope(left_wrench_ts_);
    //right_wrench_input_port_->getEnvelope(right_wrench_ts_);
        
    return true;
}

Manipulation::~Manipulation()
{
    image_input_port_->close();
    delete image_input_port_;
    
    left_wrench_input_port_->close();
    delete left_wrench_input_port_;
    
    right_wrench_input_port_->close();
    delete right_wrench_input_port_;
    
}

