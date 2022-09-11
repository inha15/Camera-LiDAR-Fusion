#include <Sensor_fusion/fusion_declare.h>

void ONLY_DATA_CLEAR(){
    used_LiDAR_DATA.clear();
    used_camera_DATA.clear();
}

void whole_processing(){
    Sensor_fusion::LiDAR_BB_arr::Ptr LiDAR_BoundingBox_arr(new Sensor_fusion::LiDAR_BB_arr); 
    Sensor_fusion::Camera_BB_arr::Ptr Camera_BoundingBox_arr(new Sensor_fusion::Camera_BB_arr);
    Sensor_fusion::Camera_BB_arr::Ptr TFF_BoundingBox_arr(new Sensor_fusion::Camera_BB_arr);
    std_msgs::String Fin_Traffic_SIGN;
    vector<finalbox> fusion_BB;

    //LiDAR and camera decoding
    trans_LiDAR_DATA    (LiDAR_BoundingBox_arr); //라이다 기존 데이터가 보존된 2차원 벡터 OBJ 만들어줌
    trans_Camera_DATA   (Camera_BoundingBox_arr, TFF_BoundingBox_arr);

    if(TFF_BoundingBox_arr->Camera_BB_arr.empty()) Fin_Traffic_SIGN.data = "No_SIGN";
    else extract_traffic_SIGN(TFF_BoundingBox_arr, Fin_Traffic_SIGN);

    //fusion execute
    final_fusion        (fusion_BB,Camera_IMG_DATA_comb,*LiDAR_BoundingBox_arr,*Camera_BoundingBox_arr);
    
    // cout<<"============= Camera OBJ ==============="<<endl;
    // for(int i = 0; i < TFF_BoundingBox_arr->Camera_BB_arr.size(); i++){
    //     cout << TFF_BoundingBox_arr->Camera_BB_arr[i].Class << endl;
    //     cout << "xmin : "  << TFF_BoundingBox_arr->Camera_BB_arr[i].xmin_Camera << "      xmax : " << TFF_BoundingBox_arr->Camera_BB_arr[i].xmax_Camera << endl;
    //     cout << "ymin : "  << TFF_BoundingBox_arr->Camera_BB_arr[i].ymin_Camera << "      ymax : " << TFF_BoundingBox_arr->Camera_BB_arr[i].ymax_Camera << endl;
    // }
    cout<<"---------------------------------------------"<<endl;
    cout<<Fin_Traffic_SIGN.data<<endl;

    //MSG pub select
    if(fusionMSG_pub)       fusion_obj_pub(fusion_BB);
    if(tffMSG_pub)          tff_pub(Fin_Traffic_SIGN);
    ONLY_DATA_CLEAR();
}

void RCVD_LiDAR_DATA(const Sensor_fusion::object_msg_arr LiDAR_DATA){
    //cout << "LiDAR msg : " << LiDAR_DATA << endl;
    //for(int i=0;i<LiDAR_DATA.x.size();i++)
    //    cout << "LiDAR x : " << LiDAR_DATA.x[i] << "      LiDAR y : " << LiDAR_DATA.y[i] << "      LiDAR z : " << LiDAR_DATA.z[i] << endl;
    LiDAR_DATA_comb = LiDAR_DATA;
    //deliv_processing();
}

void RCVD_Camera_DATA_BB(const std_msgs::String Camera_DATA){
    //cout << "Camera_BB : " << Camera_DATA << endl;
    Camera_DATA_comb = Camera_DATA;
}

void RCVD_Camera_DATA_img(const sensor_msgs::Image Camera_DATA){
    Camera_IMG_DATA_comb = Camera_DATA; 
    //whole_processing(); //camera Img data rate로 sync 기준을 잡는다.  
    srt_sig = 1;
}