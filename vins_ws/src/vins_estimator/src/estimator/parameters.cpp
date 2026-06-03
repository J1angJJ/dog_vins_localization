/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"
#include <algorithm>

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string OUTPUT_FOLDER;
std::string IMU_TOPIC;
int ROW, COL;
double TD;
int NUM_OF_CAM;
int STEREO;
int USE_IMU;
int MULTIPLE_THREAD;
map<int, Eigen::Vector3d> pts_gt;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;
int USE_LEG_ODOM;
std::string LEG_ODOM_TOPIC;
int LEG_ODOM_FACTOR_MODE;
double LEG_ODOM_MAX_TIME_DIFF;
double LEG_ODOM_POS_STD;
double LEG_ODOM_YAW_STD;
double LEG_ODOM_LOSS;
double LEG_ODOM_MIN_TRANSLATION;
double LEG_ODOM_MAX_TRANSLATION;
double LEG_ODOM_MAX_YAW;


template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;          
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];

    USE_LEG_ODOM = 0;
    LEG_ODOM_TOPIC = "/leg_odom2";
    LEG_ODOM_FACTOR_MODE = 0;
    LEG_ODOM_MAX_TIME_DIFF = 0.05;
    LEG_ODOM_POS_STD = 0.10;
    LEG_ODOM_YAW_STD = 0.10;
    LEG_ODOM_LOSS = 1.0;
    LEG_ODOM_MIN_TRANSLATION = 0.005;
    LEG_ODOM_MAX_TRANSLATION = 1.0;
    LEG_ODOM_MAX_YAW = 1.0;

    if (!fsSettings["use_leg_odom"].empty())
        USE_LEG_ODOM = static_cast<int>(fsSettings["use_leg_odom"]);
    if (!fsSettings["leg_odom_topic"].empty())
        fsSettings["leg_odom_topic"] >> LEG_ODOM_TOPIC;
    if (!fsSettings["leg_odom_factor_mode"].empty())
        LEG_ODOM_FACTOR_MODE = static_cast<int>(fsSettings["leg_odom_factor_mode"]);
    if (!fsSettings["leg_odom_max_time_diff"].empty())
        LEG_ODOM_MAX_TIME_DIFF = static_cast<double>(fsSettings["leg_odom_max_time_diff"]);
    if (!fsSettings["leg_odom_pos_std"].empty())
        LEG_ODOM_POS_STD = static_cast<double>(fsSettings["leg_odom_pos_std"]);
    if (!fsSettings["leg_odom_yaw_std"].empty())
        LEG_ODOM_YAW_STD = static_cast<double>(fsSettings["leg_odom_yaw_std"]);
    if (!fsSettings["leg_odom_loss"].empty())
        LEG_ODOM_LOSS = static_cast<double>(fsSettings["leg_odom_loss"]);
    if (!fsSettings["leg_odom_min_translation"].empty())
        LEG_ODOM_MIN_TRANSLATION = static_cast<double>(fsSettings["leg_odom_min_translation"]);
    if (!fsSettings["leg_odom_max_translation"].empty())
        LEG_ODOM_MAX_TRANSLATION = static_cast<double>(fsSettings["leg_odom_max_translation"]);
    if (!fsSettings["leg_odom_max_yaw"].empty())
        LEG_ODOM_MAX_YAW = static_cast<double>(fsSettings["leg_odom_max_yaw"]);

    LEG_ODOM_FACTOR_MODE = (LEG_ODOM_FACTOR_MODE == 1) ? 1 : 0;
    LEG_ODOM_MAX_TIME_DIFF = std::max(LEG_ODOM_MAX_TIME_DIFF, 0.001);
    LEG_ODOM_POS_STD = std::max(LEG_ODOM_POS_STD, 1e-4);
    LEG_ODOM_YAW_STD = std::max(LEG_ODOM_YAW_STD, 1e-4);
    LEG_ODOM_LOSS = std::max(LEG_ODOM_LOSS, 1e-4);
    LEG_ODOM_MIN_TRANSLATION = std::max(LEG_ODOM_MIN_TRANSLATION, 0.0);
    LEG_ODOM_MAX_TRANSLATION = std::max(LEG_ODOM_MAX_TRANSLATION, LEG_ODOM_MIN_TRANSLATION + 1e-4);
    LEG_ODOM_MAX_YAW = std::max(LEG_ODOM_MAX_YAW, 1e-4);

    if (USE_LEG_ODOM)
    {
        printf("USE_LEG_ODOM: %d\n", USE_LEG_ODOM);
        printf("LEG_ODOM_TOPIC: %s\n", LEG_ODOM_TOPIC.c_str());
        printf("LEG_ODOM_FACTOR_MODE: %d\n", LEG_ODOM_FACTOR_MODE);
    }

    MULTIPLE_THREAD = fsSettings["multiple_thread"];

    USE_IMU = fsSettings["imu"];
    printf("USE_IMU: %d\n", USE_IMU);
    if(USE_IMU)
    {
        fsSettings["imu_topic"] >> IMU_TOPIC;
        printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
        ACC_N = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_N = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        G.z() = fsSettings["g_norm"];
    }

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    fsSettings["output_path"] >> OUTPUT_FOLDER;
    VINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    } 
    
    NUM_OF_CAM = fsSettings["num_of_cam"];
    printf("camera number %d\n", NUM_OF_CAM);

    if(NUM_OF_CAM != 1 && NUM_OF_CAM != 2)
    {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);
    }


    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    CAM_NAMES.push_back(cam0Path);

    if(NUM_OF_CAM == 2)
    {
        STEREO = 1;
        std::string cam1Calib;
        fsSettings["cam1_calib"] >> cam1Calib;
        std::string cam1Path = configPath + "/" + cam1Calib; 
        //printf("%s cam1 path\n", cam1Path.c_str() );
        CAM_NAMES.push_back(cam1Path);
        
        cv::Mat cv_T;
        fsSettings["body_T_cam1"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %d COL: %d ", ROW, COL);

    if(!USE_IMU)
    {
        ESTIMATE_EXTRINSIC = 0;
        ESTIMATE_TD = 0;
        printf("no imu, fix extrinsic param; no time offset calibration\n");
    }

    fsSettings.release();
}
