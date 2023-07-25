//
// Created by lacie on 25/07/2023.
//

#include "ArUcOOdometry.h"

ArUcOOdometry::ArUcOOdometry(const cv::Mat &K, const cv::Mat &dist, const double &kl, const double kr, const double &b,
                             const Eigen::Matrix4d &T_r_c, const double &k, const double &k_r, const double k_phi,
                             const int &n_markers, const int &marker_size, const double &marker_length)
                             : K_(K), dist_(dist), kl_(kl), kr_(kr), b_(b), T_r_c_(T_r_c), k_(k), k_r_(k_r), k_phi_(k_phi),
                               n_markers_(n_markers), marker_size_(marker_size), marker_length_(marker_length)
{
    is_init_ = false;

    mu_.resize(3, 1);
    mu_.setZero();
    sigma_.resize(3, 3);
    sigma_.setZero();

    dictionary_ = cv::aruco::generateCustomDictionary(n_markers_, marker_size_);
}

void ArUcOOdometry::addEncoder(const double &enl, const double &enr)
{
    if( is_init_ == false)
    {
        last_enl_  = enl;
        last_enr_ = enr;
        is_init_ = true;
        return;
    }

    /* Delta_l/r */
    double delta_enl = enl - last_enl_;
    double delta_enr = enr - last_enr_;
    double delta_sl = kl_ * delta_enl;
    double delta_sr = kr_ * delta_enr;

    /* Delta_theta, Delta_s */
    double delta_theta = (delta_sr - delta_sl) / b_;
    double delta_s = 0.5 * (delta_sr + delta_sl);

    double tmp_th = mu_(2,0) + 0.5 * delta_theta;
    double cos_tmp_th = cos( tmp_th );
    double sin_tmp_th = sin(tmp_th);

    mu_(0, 0) += delta_s * cos_tmp_th;
    mu_(1, 0) += delta_s * sin_tmp_th;
    mu_(2, 0) += delta_theta;
    normAngle(mu_(2, 0)); //norm

    /* G_xi */
    Eigen::Matrix3d G_xi;
    G_xi << 1.0, 0.0, -delta_s * sin_tmp_th,
            0.0, 1.0, delta_s * cos_tmp_th,
            0.0, 0.0, 1.0;

    /* G_u */
    Eigen::Matrix<double, 3, 2> Gup;
    Gup << 0.5  * (cos_tmp_th - delta_s * sin_tmp_th / b_), 0.5  * (cos_tmp_th + delta_s * sin_tmp_th / b_),
            0.5  * (sin_tmp_th + delta_s * cos_tmp_th /b_), 0.5  *(sin_tmp_th - delta_s * cos_tmp_th/b_),
            1.0/b_, -1.0/b_;

    int N = mu_.rows();
    Eigen::MatrixXd F(N, 3); F.setZero();
    F.block(0,0, 3, 3) = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd Gt = Eigen::MatrixXd::Identity(N, N);
    Gt.block(0, 0, 3, 3) = G_xi;

    Eigen::Matrix2d sigma_u;
    sigma_u << k_ * k_ * delta_sr * delta_sr, 0.0, 0.0, k_ * k_* delta_sl * delta_sl;

    sigma_ = Gt * sigma_ *Gt.transpose() + F * Gup * sigma_u * Gup.transpose() * F.transpose();

    last_enl_ = enl;
    last_enr_ = enr;
}

void ArUcOOdometry::addImage(const cv::Mat &img)
{
    if( is_init_ == false)
        return;
    std::vector<MarkerObservation> obs;
    getObservations(img, obs);

    for(MarkerObservation ob: obs)
    {
        Eigen::Matrix2d Q;
        Q << k_r_ * k_r_* fabs(ob.r_ * ob.r_), 0.0, 0.0, k_phi_ * k_phi_ * fabs(ob.phi_ * ob.phi_);

        int i;
        if(checkLandmark(ob.aruco_id_, i))
        {
            int N = mu_.rows();
            Eigen::MatrixXd F(5, N);
            F.setZero();
            F.block(0,0,3,3) = Eigen::Matrix3d::Identity();
            F(3, 3 + 2*i) = 1;
            F(4, 4 + 2*i) = 1;

            double& mx = mu_(3 + 2*i, 0);
            double& my = mu_(4 + 2*i, 0);
            double& x = mu_(0,0);
            double& y = mu_(1,0);
            double& theta = mu_(2,0);
            double delta_x = mx - x;
            double delta_y = my -y;
            double q = delta_x * delta_x + delta_y * delta_y;
            double sqrt_q = sqrt(q);

            Eigen::MatrixXd Hv(2, 5);
            Hv << -sqrt_q * delta_x, -sqrt_q* delta_y, 0, sqrt_q*delta_x, sqrt_q*delta_y,
                    delta_y, -delta_x, -q, -delta_y, delta_x;

            Hv = (1/q) * Hv;

            Eigen::MatrixXd Ht = Hv * F;

            Eigen::MatrixXd K = sigma_ * Ht.transpose()*( Ht * sigma_ * Ht.transpose() + Q ).inverse();

            double phi_hat = atan2(delta_y, delta_x)- theta;
            normAngle(phi_hat);
            Eigen::Vector2d z_hat(
                    sqrt_q, phi_hat
            );
            Eigen::Vector2d z(ob.r_, ob.phi_);
            mu_ = mu_ + K * (z - z_hat);
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(N, N);
            sigma_ = ( I - K * Ht) * sigma_;
        }
        else
        {
            double angle = mu_(2,0) + ob.phi_; normAngle(angle);
            double mx = ob.r_ * cos(angle) + mu_(0,0);
            double my = ob.r_ * sin(angle) + mu_(1,0);

            Eigen::Matrix3d sigma_xi = sigma_.block(0,0, 3, 3);

            Eigen::Matrix<double, 2, 3> Gp;
            Gp << 1, 0, -ob.r_ * sin(angle),
                    0, 1, ob.r_ * cos(angle);

            Eigen::Matrix2d Gz;
            Gz << cos(angle), -ob.r_ * sin(angle),
                    sin(angle), ob.r_ * cos(angle);

            Eigen::Matrix2d sigma_m = Gp * sigma_xi * Gp.transpose() + Gz * Q * Gz.transpose();

            Eigen::MatrixXd Gfx;
            Gfx.resize ( 2, mu_.rows() );
            Gfx.setZero();
            Gfx.block ( 0,0, 2, 3 ) = Gp;

            Eigen::MatrixXd sigma_mx;
            sigma_mx.resize ( 2, mu_.rows() );
            sigma_mx.setZero();
            sigma_mx = Gfx * sigma_;

            int N = mu_.rows();
            Eigen::MatrixXd tmp_mu ( N + 2, 1 );
            tmp_mu.setZero();
            tmp_mu << mu_ , mx, my;
            mu_.resize ( N+2, 1 );
            mu_ = tmp_mu;

            Eigen::MatrixXd tmp_sigma ( N+2, N+2 );
            tmp_sigma.setZero();
            tmp_sigma.block ( 0, 0, N, N ) = sigma_;
            tmp_sigma.block ( N, N, 2, 2 ) = sigma_m;
            tmp_sigma.block ( N, 0, 2, N ) = sigma_mx;
            tmp_sigma.block ( 0, N, N, 2 ) = sigma_mx.transpose();

            sigma_.resize ( N+2, N+2 );
            sigma_ = tmp_sigma;

            aruco_ids_.push_back ( ob.aruco_id_ );
        }// add new landmark
    }
}

int ArUcOOdometry::getObservations(const cv::Mat &img, std::vector<MarkerObservation> &obs)
{
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> IDs;
    std::vector<cv::Vec3d> rvs, tvs;
    cv::aruco::detectMarkers(img, dictionary_ , marker_corners, IDs);
    cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_length_, K_, dist_, rvs, tvs);

    /* draw all marks */
    marker_img_ = img.clone();
    cv::aruco::drawDetectedMarkers(marker_img_, marker_corners, IDs);

    marker_img_ = img.clone();
    cv::aruco::drawDetectedMarkers(marker_img_, marker_corners, IDs);
    for(size_t i=0; i<IDs.size(); i++)
    {
        cv::aruco::drawAxis(marker_img_, K_, dist_, rvs[i], tvs[i], 0.07);
    }

    const float DistTh = 3; //3 m
    for ( size_t i = 0; i < IDs.size(); i ++ )
    {
        float dist = cv::norm<double>(tvs[i]);
        if( dist > DistTh)
            continue;

        cv::Vec3d tvec = tvs[i];
        cv::Vec3d rvec = rvs[i];
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        Eigen::Matrix4d T_c_m;
        T_c_m <<
              R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), tvec[0],
                R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), tvec[1],
                R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), tvec[2],
                0.,0.,0.,1.;
        Eigen::Matrix4d T_r_m = T_r_c_ * T_c_m;

        double& x= T_r_m(0, 3);
        double& y = T_r_m(1, 3);

        double r = sqrt(x*x + y*y);
        double phi = atan2(y, x);
        int aruco_id = IDs[i];

        obs.push_back(MarkerObservation( aruco_id, r, phi));
    }//for all detected markers

    return obs.size();
}

visualization_msgs::MarkerArray ArUcOOdometry::toRosMarkers(double scale)
{

    visualization_msgs::MarkerArray markers;
    int N = 0;
    for(int i = 4; i < mu_.rows(); i+=2)
    {
        double& mx = mu_(i-1, 0);
        double& my = mu_(i, 0);

        Eigen::Matrix2d sigma_m = sigma_.block(i-1, i-1, 2, 2);
        cv::Mat cvsigma_m = (cv::Mat_<double>(2,2) <<
                                                   sigma_m(0,0), sigma_m(0,1), sigma_m(1,0), sigma_m(1,1));
        cv::Mat eigen_value, eigen_vector;
        cv::eigen(cvsigma_m, eigen_value, eigen_vector);
        double angle = atan2( eigen_vector.at<double>(0, 1),  eigen_vector.at<double>(0, 0));
        double x_len =  2 * sqrt(eigen_value.at<double>(0,0) * 5.991) ;
        double y_len = 2 * sqrt( eigen_value.at<double>(1,0)* 5.991);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = "ekf_slam";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = mx;
        marker.pose.position.y = my;
        marker.pose.position.z = 0;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
        marker.scale.x = scale * x_len;
        marker.scale.y = scale * y_len;
        marker.scale.z = 0.1 * scale * (x_len + y_len);
        marker.color.a = 0.8; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        markers.markers.push_back(marker);
    }// for all mpts

    return markers;
}

geometry_msgs::PoseWithCovarianceStamped ArUcOOdometry::toRosPose()
{
    geometry_msgs::PoseWithCovarianceStamped rpose;
    rpose.header.frame_id = "world";
    rpose.pose.pose.position.x = mu_(0, 0);
    rpose.pose.pose.position.y = mu_(1, 0);
    rpose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(mu_(2, 0));

    rpose.pose.covariance.at(0) = sigma_(0,0);
    rpose.pose.covariance.at(1) = sigma_(0,1);
    rpose.pose.covariance.at(6) = sigma_(1,0);
    rpose.pose.covariance.at(7) = sigma_(1,1);
    rpose.pose.covariance.at(5) = sigma_(0,2);
    rpose.pose.covariance.at(30) = sigma_(2,0);
    rpose.pose.covariance.at(35) = sigma_(2,2);

    return rpose;
}

void ArUcOOdometry::normAngle(double& angle)
{
    const static double PI = 3.1415926;
    static double Two_PI = 2.0 * PI;
    if( angle >= PI)
        angle -= Two_PI;
    if( angle < -PI)
        angle += Two_PI;
}

bool ArUcOOdometry::checkLandmark(int aruco_id, int& landmark_idx)
{
    for(size_t i = 0; i < aruco_ids_.size(); i ++)
    {
        if(aruco_id == aruco_ids_.at(i))
        {
            landmark_idx = i;
            return true;
        }
    }
    return false;
}

