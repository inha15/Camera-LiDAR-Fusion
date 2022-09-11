#ifndef FUSION_DECLARE
#define FUSION_DECLARE

//header
#include <iostream>
#include <cmath>
#include <vector>
#include <set>
#include <utility>
#include <algorithm>
#include <string>
#include <time.h>
#include <ros/ros.h>
#include <boost/format.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <Eigen/Dense>

//user define header
#include <Sensor_fusion/LiDAR_calibration.h>

//msg
#include "Sensor_fusion/obj_msg.h"
#include "Sensor_fusion/LiDAR_BB.h"
#include "Sensor_fusion/LiDAR_BB_arr.h"
#include "Sensor_fusion/Camera_BB.h"
#include "Sensor_fusion/Camera_BB_arr.h"
#include "Sensor_fusion/object_msg.h"
#include "Sensor_fusion/object_msg_arr.h"

#define PI 3.141592

using namespace std;


//typedef
typedef Sensor_fusion::LiDAR_BB LBB;
typedef Sensor_fusion::Camera_BB CBB;
typedef Sensor_fusion::LiDAR_BB_arr LBB_arr;
typedef Sensor_fusion::Camera_BB_arr CBB_arr;
typedef Sensor_fusion::object_msg_arr OBJ_MSG;

//user define structure
struct Bbox{
    float xmin = 0;
    float xmax = 0;
    float ymin = 0;
    float ymax = 0;
    int flag = 0;
};

struct finalbox{
    //cam Class
    string Class;
    //cam fixel
	float xmax = 0;
    float xmin = 0;
	float ymax = 0;
	float ymin = 0;
    //LiDAR center
	float xcenter = 0;
	float ycenter = 0;
    float zcenter = 0;
    //LiDAR minmax
    float Lxmin = 0;
    float Lxmax = 0;
    float Lymin = 0;
    float Lymax = 0;
    float Lzmin = 0;
    float Lzmax = 0;
};

//ros Pub
ros::Publisher fusionBB_pub;
ros::Publisher LiDARBB_pub;
ros::Publisher cameraBB_pub;
ros::Publisher trafficSign_pub;

//global parameter
Sensor_fusion::object_msg_arr LiDAR_DATA_comb;
std_msgs::String Camera_DATA_comb;
sensor_msgs::Image Camera_IMG_DATA_comb;
vector<int> used_LiDAR_DATA;     
vector<int> used_camera_DATA;    

//func & param
//sync func
void ONLY_DATA_CLEAR();
void whole_processing();
void RCVD_LiDAR_DATA(const Sensor_fusion::object_msg_arr);
void RCVD_Camera_DATA_BB(const std_msgs::String);
void RCVD_Camera_DATA_img(const sensor_msgs::Image);

//camera proc func
double s_to_f(string);
pair<pair<int, int>, pair<int, int>> s_to_minmax(string);
bool check_TFF_SIGN(string);
void trans_Camera_DATA(Sensor_fusion::Camera_BB_arr::Ptr, Sensor_fusion::Camera_BB_arr::Ptr);
float get_TFF_area(Sensor_fusion::Camera_BB);
bool TFF_comp(pair<string,float>, pair<string,float>);
void extract_traffic_SIGN(Sensor_fusion::Camera_BB_arr::Ptr, std_msgs::String&);

//LiDAR proc func
int check_project_case(vector<float>&);
void project_func(vector<float>&, Sensor_fusion::LiDAR_BB::Ptr, int);
void project_LiDAR_to_Img(vector<vector<float>>&, Sensor_fusion::LiDAR_BB_arr::Ptr);
void trans_LiDAR_DATA(Sensor_fusion::LiDAR_BB_arr::Ptr);
float get_TFF_area(Sensor_fusion::Camera_BB);

//fusion proc func
string f_to_s(float);
void print_LiDAR_BB(cv::Mat&, vector<Sensor_fusion::LiDAR_BB>&);
void print_Camera_BB(cv::Mat&, vector<Sensor_fusion::Camera_BB>&);
void print_Fusion_BB(cv::Mat&, finalbox&);
int check_class(std::string);
bool check_overlap(Sensor_fusion::LiDAR_BB&, Sensor_fusion::Camera_BB&, float, float);
float check_IoU(Sensor_fusion::LiDAR_BB&, Sensor_fusion::Camera_BB&);
finalbox make_fusionBox(Sensor_fusion::LiDAR_BB&, Sensor_fusion::Camera_BB&);
vector<finalbox> fusion_iter_basedOn_LiDAR(cv::Mat&, vector<Sensor_fusion::LiDAR_BB>&, vector<Sensor_fusion::Camera_BB>&);
vector<finalbox> fusion_iter_basedOn_camera(cv::Mat&, vector<Sensor_fusion::LiDAR_BB>&, vector<Sensor_fusion::Camera_BB>&);
void final_fusion(vector<finalbox>&, const sensor_msgs::Image&, Sensor_fusion::LiDAR_BB_arr&, Sensor_fusion::Camera_BB_arr&);

//MSG pub
bool check_overlap_idx(bool, int);
void only_LiDAR_obj_print(cv::Mat&, vector<Sensor_fusion::LiDAR_BB>&);
void only_camera_obj_print(cv::Mat&, vector<Sensor_fusion::Camera_BB>&);
void tff_pub(std_msgs::String);
void only_LiDAR_obj_pub(cv::Mat&, Sensor_fusion::LiDAR_BB_arr&,vector<Sensor_fusion::LiDAR_BB>&);
void only_camera_obj_pub(cv::Mat&, Sensor_fusion::Camera_BB_arr&,vector<Sensor_fusion::Camera_BB>&);
void fusion_obj_pub(vector<finalbox>&);

//etc
inline float cal_dist(float x, float y) { return sqrt(x * x + y * y); }
void SetCalibrationParam();

//param
bool switch_LiDAR_BB;
bool switch_Camera_BB;
bool switch_Fusion_BB;
bool camera_base_fusion;
bool LiDAR_base_fusion;
bool FBB_maked_CameraLiDARcenter;
bool FBB_maked_Cameracenter;
float IOU_limit;
bool fusionMSG_pub;
bool onlyLiDARMSG_pub;
bool onlyCameraMSG_pub;
bool tffMSG_pub;
bool switch_only_LiDAR_BB;
bool switch_only_Camera_BB;

int srt_sig = 0;
//------------------------------------------------------------------------------------------------------------


//delivery func & param
ros::Publisher deliv_pub;
bool comp_LiDAR_deliv(vector<float>, vector<float>);
void sort_LiDAR_deliv(vector<vector<float>>&);
vector<vector<float>> trans_LiDAR_DATA_deliv(Sensor_fusion::LiDAR_BB_arr::Ptr);
float get_area(Sensor_fusion::Camera_BB);
bool comp_camera_deliv(pair<float,Sensor_fusion::Camera_BB>, pair<float,Sensor_fusion::Camera_BB>);
string get_sorted_camera_deliv(Sensor_fusion::Camera_BB_arr::Ptr);
string tmp_msg_generator(vector<vector<float>>, string);
string ret_class(std::string);
void deliv_processing();
void msg_process(string);

string tmp_signboard = "A3B1B3B2";
string reset_tmp_signboard = tmp_signboard;
int lap_cnt = 1;
bool delete_flag = 0;
int roi_xmin = -0.3;
int tracking_iter = 3;
int tmp_tracking_iter = tracking_iter;
int reset_iter = 1350 - 50;
bool finish_flag = 0;
bool real_finish = 0;
int finish_iter = 10;

#endif
