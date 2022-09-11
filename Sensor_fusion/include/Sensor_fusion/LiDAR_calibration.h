#ifndef FUSION_CALIBRATION
#define FUSION_CALIBRATION

//user define header
#include <Sensor_fusion/fusion_declare.h>

#define PI 3.141592
#define PIXEL_YMIN 0
#define PIXEL_XMIN 0
#define PIXEL_YMAX 1080
#define PIXEL_XMAX 1920

bool set_CalibParam_flag = 0;

float intri_fx;
float intri_fy;
float intri_cx;
float intri_cy;
float intri_skew;

float rot_angle_x = 0;
float rot_angle_y = 0;
float rot_angle_z = 0;

float trans_x;
float trans_y;
float trans_z;

//라이다 & 카메라 calibration용 matrix, calibration matrix setting
cv::Mat intrinsic_mat = cv::Mat(3, 4, CV_32F, cv::Scalar::all(0));                // camera internal info
cv::Mat rotate_mat = cv::Mat(4, 4, CV_32F, cv::Scalar::all(0));                //rotation world coords to camera coords
cv::Mat translation_mat = cv::Mat(4, 4, CV_32F, cv::Scalar::all(0));            // translation world to camera
cv::Mat final_project_mat = cv::Mat(3, 4,CV_32F, cv::Scalar::all(0));   // project world to pixel

#endif



/*
tiny profile set (some boundary issue maybe from ROI setting)
1400.551,      0,          941.601,  0,
0,             1400.681,   500.747,  0,
0,             0,          1,        0

ordinary
1356.551,      0,        941.601, 0,
0,        1354.681, 597.747, 0,
0,        0,        1,       0 

** Added sample 65, p_x = 0.404, p_y = 0.196, p_size = 0.077, skew = 0.013
D = [-0.16550743987560654, 0.09230521344819245, -0.00046334922459399114, -0.0012772049555980257, 0.0]
K = [1375.6631594946225, 0.0, 947.6878775987998, 0.0, 1378.6421640769104, 615.956760952115, 0.0, 0.0, 1.0]
R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P = [1287.5865478515625, 0.0, 943.0413578667358, 0.0, 0.0, 1336.7132568359375, 615.9321615730187, 0.0, 0.0, 0.0, 1.0, 0.0]
None
# oST version 5.0 parameters


[image]

width
1920

height
1200

[narrow_stereo]

camera matrix
1375.663159 0.000000 947.687878
0.000000 1378.642164 615.956761
0.000000 0.000000 1.000000

distortion
-0.165507 0.092305 -0.000463 -0.001277 0.000000

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
1287.586548 0.000000 943.041358 0.000000
0.000000 1336.713257 615.932162 0.000000
0.000000 0.000000 1.000000 0.000000

*/