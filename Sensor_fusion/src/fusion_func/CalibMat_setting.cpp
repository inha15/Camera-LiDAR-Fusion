#include <Sensor_fusion/fusion_declare.h>
void SetCalibrationParam(){
    //intrinsic mat setting
    float intrinsic[] = {   intri_fx,    intri_skew,    intri_cx,   0,
                            0,           intri_fy,      intri_cy,   0,
                            0,           0,             1,          0};
    intrinsic_mat = cv::Mat(3, 4, CV_32F, intrinsic);

    //rotation mat setting
    float theta_x = rot_angle_x*(PI /180); //rotate radian angle for x_axis
    float theta_y = rot_angle_y*(PI /180); //rotate radian angle for y_axis
    float theta_z = rot_angle_z*(PI /180);; //rotate radian angle for z_axis

    float Rotate_x[] = { 1, 0,            0,
                        0, cos(theta_x), -sin(theta_x),
                        0, sin(theta_x), cos(theta_x) };


    float Rotate_y[] = { cos(theta_y),  0, sin(theta_y),
                        0,             1, 0,
                        -sin(theta_y), 0, cos(theta_y) };


    float Rotate_z[] = { cos(theta_z), -sin(theta_z), 0,
                        sin(theta_z), cos(theta_z),  0,
                        0,            0,             1 };

    cv::Mat rotate_x = cv::Mat(3, 3, CV_32F, Rotate_x);
    cv::Mat rotate_y = cv::Mat(3, 3, CV_32F, Rotate_y);
    cv::Mat rotate_z = cv::Mat(3, 3, CV_32F, Rotate_z);
    cv::Mat rotate_tmp = cv::Mat(3, 3, CV_32F, cv::Scalar(0));
    rotate_tmp = rotate_z * rotate_y * rotate_x;

    //float 0 is not accurate 0
    rotate_mat.at<float>(0, 0) = ((abs(rotate_tmp.at<float>(0, 0)) < 0.0001) ? 0 : rotate_tmp.at<float>(0, 0));
    rotate_mat.at<float>(0, 1) = ((abs(rotate_tmp.at<float>(0, 1)) < 0.0001) ? 0 : rotate_tmp.at<float>(0, 1));
    rotate_mat.at<float>(0, 2) = ((abs(rotate_tmp.at<float>(0, 2)) < 0.0001) ? 0 : rotate_tmp.at<float>(0, 2));
    rotate_mat.at<float>(0, 3) = 0; 
    rotate_mat.at<float>(1, 0) = ((abs(rotate_tmp.at<float>(1, 0)) < 0.0001) ? 0 : rotate_tmp.at<float>(1, 0));
    rotate_mat.at<float>(1, 1) = ((abs(rotate_tmp.at<float>(1, 1)) < 0.0001) ? 0 : rotate_tmp.at<float>(1, 1));
    rotate_mat.at<float>(1, 2) = ((abs(rotate_tmp.at<float>(1, 2)) < 0.0001) ? 0 : rotate_tmp.at<float>(1, 2));
    rotate_mat.at<float>(1, 3) = 0; 
    rotate_mat.at<float>(2, 0) = ((abs(rotate_tmp.at<float>(2, 0)) < 0.0001) ? 0 : rotate_tmp.at<float>(2, 0));
    rotate_mat.at<float>(2, 1) = ((abs(rotate_tmp.at<float>(2, 1)) < 0.0001) ? 0 : rotate_tmp.at<float>(2, 1));
    rotate_mat.at<float>(2, 2) = ((abs(rotate_tmp.at<float>(2, 2)) < 0.0001) ? 0 : rotate_tmp.at<float>(2, 2));
    rotate_mat.at<float>(2, 3) = 0; 
    rotate_mat.at<float>(3, 0) = 0;
    rotate_mat.at<float>(3, 1) = 0;
    rotate_mat.at<float>(3, 2) = 0;
    rotate_mat.at<float>(3, 3) = 1;

    //translation mat setting
    float translation[] = { 1,    0,    0,   trans_x,
                            0,    1,    0,   trans_y,
                            0,    0,    1,   trans_z, 
                            0,    0,    0,   1 };
    translation_mat = cv::Mat(4, 4, CV_32F, translation);

    //final mat setting ....  fin_mat = intrinsic * extrinsic     (extrinsic = rotate_mat * translation_mat)
    final_project_mat = intrinsic_mat * rotate_mat * translation_mat;  
}