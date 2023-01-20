#include "decision_node.h"

/*--------------------------------------------------------------------
 * DecisionNode()
 * Constructor.
 *------------------------------------------------------------------*/


DecisionNode::DecisionNode(ros::NodeHandle _nh)
    : _last_lsd_pose(new geometry_msgs::PoseStamped()), _last_rovioli_pose(new geometry_msgs::PoseStamped()), _last_orb_pose(new geometry_msgs::PoseStamped()), 
    _last_lsd_pred_pose(new slam_img_analyzer::StringStamped()), _last_rov_pred_pose(new slam_img_analyzer::StringStamped()),    
    _sub(_nh, cam_topic, 1000), _sub_lsd_slam(_nh, slam_1_topic, 1000), _sub_rovioli(_nh, slam_2_topic, 1000), _sub_orb(_nh, slam_3_topic, 1000), 
    _sub_lsd_prediciton(_nh, lsd_pred_topic, 1000), _sub_rov_prediciton(_nh, rov_pred_topic, 1000),
    _lsd_cache(_sub_lsd_slam, 100), _rovioli_cache(_sub_rovioli, 100), _orb_cache(_sub_orb, 100), 
    _lsd_pred_cache(_sub_lsd_prediciton, 100), _rov_pred_cache(_sub_rov_prediciton, 100),
    _sync(MySyncPolicy(100), _sub, _sub_lsd_slam, _sub_rovioli, _sub_orb, _sub_lsd_prediciton, _sub_rov_prediciton)
{
    _pub = _nh.advertise<geometry_msgs::PoseStamped>(publish_topic, 1000);
    _pub_slam = _nh.advertise<std_msgs::String>(publish_slam, 1000);
    
    _sub.registerCallback(
        boost::bind(&DecisionNode::messageCallback, this, _1, ref(_lsd_cache), ref(_rovioli_cache),
        ref(_orb_cache), ref(_lsd_pred_cache), ref(_rov_pred_cache))
    );

} // end DecisionNode()

/*--------------------------------------------------------------------
 * ~DecisionNode()
 * Destructor.
 *------------------------------------------------------------------*/

DecisionNode::~DecisionNode()
{
} // end ~DecisionNode()

/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for synchronizing three PoseStamped publisher and Two StringStamped publisher
 * (SLAM data, prdeiction data).
 *------------------------------------------------------------------*/

void DecisionNode::messageCallback(
    const sensor_msgs::ImageConstPtr& cam_img_msg,
    message_filters::Cache<geometry_msgs::PoseStamped> &lsd_cache,
    message_filters::Cache<geometry_msgs::PoseStamped> &rovioli_cache,
    message_filters::Cache<geometry_msgs::PoseStamped> &orb_cache,
    message_filters::Cache<slam_img_analyzer::StringStamped> &lsd_pred_cache,
    message_filters::Cache<slam_img_analyzer::StringStamped> &rov_pred_cache
){
    ROS_INFO("[ CAMERA TIMESTAMP: %u ms ]", cam_img_msg->header.stamp.sec);

    if(lsd_cache.getElemBeforeTime(cam_img_msg->header.stamp) != NULL) {
        _last_lsd_pose = lsd_cache.getElemBeforeTime(cam_img_msg->header.stamp);
    }

    if(orb_cache.getElemBeforeTime(cam_img_msg->header.stamp) != NULL) {
        _last_orb_pose = orb_cache.getElemBeforeTime(cam_img_msg->header.stamp);
    }

    if(rovioli_cache.getElemBeforeTime(cam_img_msg->header.stamp) != NULL) {
         _last_rovioli_pose = rovioli_cache.getElemBeforeTime(cam_img_msg->header.stamp);
    }

    if(lsd_pred_cache.getElemBeforeTime(cam_img_msg->header.stamp) != NULL) {
         _last_lsd_pred_pose = lsd_pred_cache.getElemBeforeTime(cam_img_msg->header.stamp);
    }

    if(rov_pred_cache.getElemBeforeTime(cam_img_msg->header.stamp) != NULL) {
         _last_rov_pred_pose = rov_pred_cache.getElemBeforeTime(cam_img_msg->header.stamp);
    }

    int rovioli_last_time = _last_rovioli_pose->header.stamp.sec;
    int lsd_last_time = _last_lsd_pose->header.stamp.sec;
    int orb_last_time = _last_orb_pose->header.stamp.sec;
    int pred_lsd_last_time = _last_lsd_pred_pose->header.stamp.sec;
    int pred_rov_last_time = _last_rov_pred_pose->header.stamp.sec;

    auto last_times = {lsd_last_time, rovioli_last_time, orb_last_time};
    auto max = std::max(last_times);

    ROS_INFO(
        "SLAM TIMES\n [MAX]: %u\n [ROVIOLI]: %u\n [ORB-SLAM]: %u\n [LSD-SLAM]: %u\n,  \
        [PREDICTION LSD]: %u, [PREDICTION ROVIOLI]: %u\n", \
        max, rovioli_last_time, orb_last_time, lsd_last_time, 
        pred_lsd_last_time, pred_rov_last_time
    );

    float prediction_lsd, prediction_rovioli;
    std::string unprocessed_lsd_pred = _last_lsd_pred_pose->prediction;
    std::string unprocessed_rov_pred = _last_rov_pred_pose->prediction;

    if( unprocessed_lsd_pred != "" && unprocessed_rov_pred != "" ) {
        prediction_lsd = std::stof(getPredFromString(unprocessed_lsd_pred));
        prediction_rovioli = std::stof(getPredFromString(unprocessed_rov_pred));
        ROS_INFO("[ PREDICTION EVALUATION ]: Prediction: %f , %f\n", prediction_lsd, prediction_rovioli);
    }
    
    // TODO N, M points calc 
    geometry_msgs::PoseStamped pose_to_publish;
   
    if( max == 0 ) {
        ROS_INFO("[ TIME EVALUATION ]: NO SLAM DATA \n");
    } else if( orb_last_time == lsd_last_time && rovioli_last_time == orb_last_time ) {
        ROS_INFO("[ TIME EVALUATION ]: OPTIMAL: ALL SLAM DATA \n");

    } else if ( max == orb_last_time && orb_last_time == lsd_last_time ) {
        ROS_INFO("[ TIME EVALUATION ]: ORBSLAM and LSD-SLAM data");
        
        if(prediction_lsd <= prediction_rovioli) {
            ROS_INFO("[ PREDICTION EVALUATION ]: LSD POSE PUBLISHED " );
            pose_to_publish.header = _last_lsd_pose->header;
            pose_to_publish.pose  = _last_lsd_pose->pose;
        } else {
            ROS_INFO("[ PREDICTION EVALUATION ]: ROVIOLI POSE PUBLISHED " );
            pose_to_publish.header = _last_rovioli_pose->header;
            pose_to_publish.pose  = _last_rovioli_pose->pose;
        }
    
    } else if ( max == orb_last_time && orb_last_time == rovioli_last_time) {
        ROS_INFO("[ TIME EVALUATION ]: ORBSLAM and ROVIOLI data");

    } else if ( max == lsd_last_time && lsd_last_time == rovioli_last_time) {
        ROS_INFO("[ TIME EVALUATION ]: LSD-SLAM and ROVIOLI data");
        
        if(prediction_lsd <= prediction_rovioli) {
            ROS_INFO("[ PREDICTION EVALUATION ]: LSD POSE PUBLISHED " );
            pose_to_publish.header = _last_lsd_pose->header;
            pose_to_publish.pose  = _last_lsd_pose->pose;
        } else {
            ROS_INFO("[ PREDICTION EVALUATION ]: ROVIOLI POSE PUBLISHED " );
            pose_to_publish.header = _last_rovioli_pose->header;
            pose_to_publish.pose  = _last_rovioli_pose->pose;
        }

    } else if ( max == lsd_last_time ) {
        ROS_INFO("[ TIME EVALUATION ]: Only LSD-SLAM data");

        pose_to_publish.header = _last_lsd_pose->header;
        pose_to_publish.pose  = _last_lsd_pose->pose;
    } else if ( max == rovioli_last_time ) {
        ROS_INFO("[ TIME EVALUATION ]: Only ROVIOLI data");
        
        pose_to_publish.header = _last_rovioli_pose->header;
        pose_to_publish.pose  = _last_rovioli_pose->pose;
    } else if ( max == orb_last_time ) {
        ROS_INFO("[ TIME EVALUATION ]: Only ORB-SLAM data");

        pose_to_publish.header = _last_orb_pose->header;
        pose_to_publish.pose  = _last_orb_pose->pose;
    }
    
    _pub.publish(pose_to_publish);       
}

boost::shared_ptr<geometry_msgs::PoseStamped const> DecisionNode::correct_lsd_pose(boost::shared_ptr<geometry_msgs::PoseStamped const>  lsd_pose){
    int lsd_scale = 6000;
    int lsd_corr_x = -250;
    int lsd_corr_y = -130;
    int lsd_corr_z = -600;

    boost::shared_ptr<geometry_msgs::PoseStamped> lsd_mod_pose(new geometry_msgs::PoseStamped());
    lsd_mod_pose->pose = lsd_pose->pose;
    lsd_mod_pose->header = lsd_pose->header;

    Eigen::Transform<double, 3, 1> t(Eigen::AngleAxis<double>(-1.97079633, Eigen::Vector3d::UnitY()));
    Eigen::Vector3d v(
       lsd_pose->pose.position.x,
       lsd_pose->pose.position.y,
       lsd_pose->pose.position.z 
    );
    v = t * v;

    lsd_mod_pose->pose.position.x = v.x() * lsd_scale + lsd_corr_x;
    lsd_mod_pose->pose.position.y = v.y() * lsd_scale + lsd_corr_y;
    lsd_mod_pose->pose.position.z = v.z() * lsd_scale + lsd_corr_z;

    return lsd_mod_pose;
}

boost::shared_ptr<geometry_msgs::PoseStamped const> DecisionNode::correct_rovioli_pose(boost::shared_ptr<geometry_msgs::PoseStamped const> rovioli_pose){
    int rovioli_scale = 800;
    int rovioli_corr_x = 450;
    int rovioli_corr_y = 330;
    int rovioli_corr_z = -150;

        ROS_INFO("FUNCTION WORKING");
    boost::shared_ptr<geometry_msgs::PoseStamped> rovioli_mod_pose(new geometry_msgs::PoseStamped());
    
        ROS_INFO("MOD POSE WORKING");
    rovioli_mod_pose->pose = rovioli_pose->pose;
    rovioli_mod_pose->header = rovioli_pose->header;
    
    Eigen::Transform<double, 3, 1> t(Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d v(
       rovioli_pose->pose.position.x,
       rovioli_pose->pose.position.y,
       rovioli_pose->pose.position.z 
    );
    v = t * v;

    rovioli_mod_pose->pose.position.x = v.x() * rovioli_scale + rovioli_corr_x;
    rovioli_mod_pose->pose.position.y = v.y() * rovioli_scale + rovioli_corr_y;
    rovioli_mod_pose->pose.position.z = v.z() * rovioli_scale + rovioli_corr_z;

    return rovioli_mod_pose;
}
  
boost::shared_ptr<geometry_msgs::PoseStamped const> DecisionNode::correct_orb_pose(boost::shared_ptr<geometry_msgs::PoseStamped const> orb_pose){
    int orb_scale = 1;

    boost::shared_ptr<geometry_msgs::PoseStamped> orb_mod_pose(new geometry_msgs::PoseStamped());
    orb_mod_pose->pose = orb_pose->pose;
    orb_mod_pose->header = orb_pose->header;

    Eigen::Transform<double, 3, 1> t(Eigen::AngleAxis<double>(-1.50079633, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d v(
       -(orb_pose->pose.position.y),
       orb_pose->pose.position.x,
       orb_pose->pose.position.z 
    );
    v = t * v;

    orb_mod_pose->pose.position.x = -(orb_pose->pose.position.y);
    orb_mod_pose->pose.position.y = orb_pose->pose.position.x;
    orb_mod_pose->pose.position.z = orb_pose->pose.position.z;

    return orb_mod_pose;
}

std::string DecisionNode::time_to_string(int sec, int nsec) {
    return std::to_string(sec)+std::to_string(nsec);
}

std::string DecisionNode::slam_to_string(std::string time, geometry_msgs::Pose pose, std::string slam) {
    return time + "," + \
    std::to_string(pose.position.x) + "," + std::to_string(pose.position.y) + "," + std::to_string(pose.position.z) + "," +\
    std::to_string(pose.orientation.x) + "," + std::to_string(pose.orientation.y) + "," + \
    std::to_string(pose.orientation.z) + "," + std::to_string(pose.orientation.w) + "," + \
    slam;
}

string DecisionNode::getPredFromString(const string unprocessed) {
    std::stringstream ss(unprocessed);
    std::vector<std::string> out;
    std::string s; 
    const char delim = ';';
    while (std::getline(ss, s, delim)) {
        out.push_back(s);
    }
    return out[1];
}

cv::Point3f DecisionNode::calculate_mean(const cv::Mat_<cv::Point3f> &points) {   
    cv::Mat_<cv::Point3f> result;
    cv::reduce(points, result, 0, CV_REDUCE_AVG);
    return result(0, 0);
}

cv::Mat DecisionNode::calculate_transform(const cv::Mat_<cv::Point3f> &points1, const cv::Mat_<cv::Point3f> &points2) {   
    // Calc means
    cv::Point3f p1 = -calculate_mean(points1);
    cv::Point3f p2 = -calculate_mean(points2);

    cv::Mat_<float> M1 = cv::Mat_<float>::eye(4, 4);
    M1(0, 3) = p1.x;
    M1(1, 3) = p1.y;
    M1(2, 3) = p1.z;

    cv::Mat_<float> M2 = cv::Mat_<float>::eye(4, 4);
    M2(0, 3) = -p2.x;
    M2(1, 3) = -p2.y;
    M2(2, 3) = -p2.z;

    // Calc covariance matrix, and root mean square deviation from center points
    cv::Mat_<float> C(3, 3, 0.0);
    float p1_rms = 0;
    float p2_rms = 0;

    for (int pt_idx = 0; pt_idx < points1.rows; pt_idx++) {
        cv::Vec3f dev1 = points1(pt_idx, 0) + p1;
        cv::Vec3f dev2 = points2(pt_idx, 0) + p2;
        p1_rms += dev1.dot(p1);
        p2_rms += dev2.dot(p2);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                C(i, j) += dev2[i] * dev1[j];
            }
        }
    }
    
    // Least-Square 
    cv::Mat_<float> u, s, vh;
    cv::SVD::compute(C, s, u, vh);
    cv::Mat_<float> R = u * vh;

    if (cv::determinant(R) < 0) {
        R -= u.col(2) * (vh.row(2) * 2.0);
    }

    float scale = sqrt(p2_rms / p1_rms);
    R *= scale;

    cv::Mat_<float> A = cv::Mat_<float>::eye(4, 4);
    R.copyTo(A.colRange(0, 3).rowRange(0, 3));

    cv::Mat_<float> rigid_transf = M2 * A * M1;
    rigid_transf /= rigid_transf(3, 3);
    return rigid_transf.rowRange(0, 3);
}

void DecisionNode::callback(geometry_msgs::PoseStampedConstPtr& ps) {
    ROS_INFO("[ SVO: %u ms ]", ps->header.stamp.sec);
}