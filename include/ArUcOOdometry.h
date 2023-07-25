//
// Created by lacie on 25/07/2023.
//

#ifndef JUNBOT_ODOMETRY_FUSION_ARUCOODOMETRY_H
#define JUNBOT_ODOMETRY_FUSION_ARUCOODOMETRY_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/aruco.hpp>
#include <tf/tf.h>

class MarkerObservation
{
public:
    MarkerObservation(){}
    MarkerObservation(const int& aruco_id, const double& r, const double& phi): aruco_id_(aruco_id), r_(r), phi_(phi){}
    int aruco_id_;
    double r_;
    double phi_;
}; // class MarkerObservation

class ArUcOOdometry {
public:
    ArUcOOdometry(const cv::Mat& K, const cv::Mat& dist,
                  const double& kl, const double kr, const double& b,
                  const Eigen::Matrix4d& T_r_c,
                  const double& k, const double& k_r, const double k_phi,
                  const int&  n_markers, const int& marker_size, const double& marker_length);

    void addEncoder(const double& enl, const double& enr);
    void addImage(const cv::Mat& img);

    visualization_msgs::MarkerArray toRosMarkers(double scale);
    geometry_msgs::PoseWithCovarianceStamped toRosPose();

    Eigen::MatrixXd& mu()
    {
        return mu_;
    }

    Eigen::MatrixXd& sigma()
    {
        return sigma_;
    }

    cv::Mat markedImg()
    {
        return marker_img_;
    }

private:
    int getObservations(const cv::Mat& img, std::vector<MarkerObservation>& obs);
    void normAngle(double& angle);
    bool checkLandmark(int aruco_id, int& landmark_idx);

    bool is_init_;

    cv::Mat K_, dist_;
    double kl_, kr_, b_;
    Eigen::Matrix4d T_r_c_;
    double k_;
    double k_r_;
    double k_phi_;

    int n_markers_;
    int marker_size_;
    double marker_length_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Mat marker_img_;

    double last_enl_, last_enr_;

    Eigen::MatrixXd mu_;
    Eigen::MatrixXd sigma_;
    std::vector<int> aruco_ids_;
};


#endif //JUNBOT_ODOMETRY_FUSION_ARUCOODOMETRY_H
