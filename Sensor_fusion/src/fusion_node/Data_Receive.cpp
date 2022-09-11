#include <Sensor_fusion/fusion_declare.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "Receive_fusion_data"); //node name 
	ros::NodeHandle nh;         //nodehandle,LiDAR
   
    //FUSION param
    nh.getParam("Data_RCVD_node/switch_LiDAR_BB",               switch_LiDAR_BB);
    nh.getParam("Data_RCVD_node/switch_Camera_BB",              switch_Camera_BB);
    nh.getParam("Data_RCVD_node/switch_Fusion_BB",              switch_Fusion_BB);

    nh.getParam("Data_RCVD_node/camera_base_fusion",            camera_base_fusion);
    nh.getParam("Data_RCVD_node/LiDAR_base_fusion",             LiDAR_base_fusion);

    nh.getParam("Data_RCVD_node/FBB_maked_CameraLiDARcenter",   FBB_maked_CameraLiDARcenter);
    nh.getParam("Data_RCVD_node/FBB_maked_Cameracenter",        FBB_maked_Cameracenter);

    nh.getParam("Data_RCVD_node/IOU_limit",                     IOU_limit);

    nh.getParam("Data_RCVD_node/fusionMSG_pub",                 fusionMSG_pub);
    nh.getParam("Data_RCVD_node/onlyLiDARMSG_pub",              onlyLiDARMSG_pub);
    nh.getParam("Data_RCVD_node/onlyCameraMSG_pub",             onlyCameraMSG_pub);
    nh.getParam("Data_RCVD_node/tffMSG_pub",                    tffMSG_pub);
    nh.getParam("Data_RCVD_node/switch_only_LiDAR_BB",          switch_only_LiDAR_BB);
    nh.getParam("Data_RCVD_node/switch_only_Camera_BB",         switch_only_Camera_BB);

    //Calibration param
    //intrinsic parameter
    nh.getParam("Data_RCVD_node/intri_fx",                      intri_fx);
    nh.getParam("Data_RCVD_node/intri_fy",                      intri_fy);
    nh.getParam("Data_RCVD_node/intri_cx",                      intri_cx);
    nh.getParam("Data_RCVD_node/intri_cy",                      intri_cy);
    nh.getParam("Data_RCVD_node/intri_skew",                    intri_skew);


    //extrinsic parameter
    nh.getParam("Data_RCVD_node/trans_x",                       trans_x);
    nh.getParam("Data_RCVD_node/trans_y",                       trans_y);
    nh.getParam("Data_RCVD_node/trans_z",                       trans_z);

    nh.getParam("Data_RCVD_node/rot_angle_x",                   rot_angle_x);
    nh.getParam("Data_RCVD_node/rot_angle_y",                   rot_angle_y);
    nh.getParam("Data_RCVD_node/rot_angle_z",                   rot_angle_z);

    if(set_CalibParam_flag == 0){ //setting calibration Mat only once
        SetCalibrationParam();
        set_CalibParam_flag++;
    }


    //sub
    ros::Subscriber sub_LiDAR = nh.subscribe<Sensor_fusion::object_msg_arr>     ("/Lidar_object", 10, RCVD_LiDAR_DATA);
    ros::Subscriber sub_Camera_img = nh.subscribe<sensor_msgs::Image>           ("/yolov5/img", 10, RCVD_Camera_DATA_img); //카메라에 라이다 인풋 속도 맞추기로 가정
    ros::Subscriber sub_Camera_BB = nh.subscribe<std_msgs::String>              ("/data_sender/classes", 10, RCVD_Camera_DATA_BB);
    
    //delivery
    //ros::Subscriber sub_LiDAR = nh.subscribe<Sensor_fusion::obj_msg>          ("/Lidar_delivery_filtered_object", 10, RCVD_LiDAR_DATA);
    //deliv_pub = nh.advertise<std_msgs::String>                                ("/deliv_Fusion_msg", 10);
    
    //pub
    fusionBB_pub = nh.advertise<Sensor_fusion::object_msg_arr>                  ("/Fusion_msg", 10);
    LiDARBB_pub = nh.advertise<Sensor_fusion::object_msg_arr>                   ("/LiDAR_only_msg", 10);
    cameraBB_pub = nh.advertise<std_msgs::String>                               ("/Camera_only_msg", 10);
    trafficSign_pub = nh.advertise<std_msgs::String>                            ("/Trafficsign_msg", 10);

    ros::Rate r(10);
    while(ros::ok()){
        ros::spinOnce();
        if(srt_sig == 0) continue; //if rcvd img data, then start processing
        whole_processing();
        r.sleep();
    }

    ros::spin();
}