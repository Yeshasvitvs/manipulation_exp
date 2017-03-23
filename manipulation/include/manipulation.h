#ifndef MANIPULATION_H
#define MANIPULATION_H

#include <iostream>
#include <memory>
#include <fstream>
#include <string.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Time.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/format.hpp>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core/eigen.hpp>

#include <geometry_msgs/Transform.h>

class Manipulation
{
private:
    
    std::string robot_name_;
    double time_init_;
    double time_current_;
    yarp::os::Stamp time_stamp_;
    double duration;
    
    std::string camera_port_name_;
    yarp::os::BufferedPort<yarp::sig::ImageOf< yarp::sig::PixelBgr > > *image_input_port_;
    yarp::sig::ImageOf< yarp::sig::PixelBgr > *input_yarp_frame_;
    cv::Mat input_yarp_to_mat_image_;
    yarp::os::Stamp image_ts_;
    
    std::string left_wrench_port_name_;
    std::string right_wrench_port_name_;
    yarp::os::BufferedPort<yarp::sig::Vector> *left_wrench_input_port_;
    yarp::os::BufferedPort<yarp::sig::Vector> *right_wrench_input_port_;
    
    yarp::sig::Vector *left_wrench_;
    yarp::sig::Vector *right_wrench_;
    yarp::os::Stamp left_wrench_ts_;
    yarp::os::Stamp right_wrench_ts_;
    
    std::string external_wrench_port_name_;
    yarp::os::BufferedPort<yarp::os::Bottle> *external_wrench_output_port_;
    
    //ArUco Markers
     int number_of_markers_ = 6;
     int marker_dimension_ = 4;
     cv::aruco::Dictionary marker_dictionary_;
     
     std::vector<int> marker_ids_, sorted_marker_ids;
     std::vector<cv::Vec3d> rvecs, tvecs, sorted_rvecs, sorted_tvecs;
     std::vector<std::vector<cv::Point2f>> marker_corners_, rejected_candidates_;
     cv::aruco::DetectorParameters detection_params_;
     
     cv::Mat camera_matrix_, dist_coeffs_;
     std::string calibration_file_name_;
     cv::FileStorage fs;
     bool calib_success_;
     
public:
    
    bool log_data_;
    std::string current_directory_;
    std::string data_directory_;                        
    std::ofstream file_name_;
    
    //Containers for storing the trajectory information
    //Each observation contains a relative tranformation between two links
    struct Observation
    {
        //boost::posix_time::ptime time;
        double time;
        //std::vector<Eigen::Vector3d> links_rel_transformation;
        geometry_msgs::Transform transform;
    };
    
    boost::shared_ptr<Observation> observation_sptr {new Observation};
    
    //Each Track contains a sequence of observations between two links
    struct Track
    {
        std::vector< boost::shared_ptr<Observation> > obs;
        std::string id;
        bool modified;
    };
                        
    //This is vector of tracks between all the links
    std::vector<Track> tracks;
    
    bool marker_detect_success_;
    
    Manipulation(std::string&);
    ~Manipulation();
    
    cv::Mat getCVMat(){return input_yarp_to_mat_image_;}
    bool displayImage(cv::Mat&);
    bool getSensoryInputs();
    void loadCameraCalibParams();
    bool detectMarkersAndComputePose();
    
    void getPoseInfo();
    void applyWrenches(); 
    
    void extractTrajectory(std::vector<int>&,std::vector<cv::Vec3d>&,std::vector<cv::Vec3d>&);
    bool getTrajectoryInfo();
    bool getWrenchInfo();
    
};

#endif