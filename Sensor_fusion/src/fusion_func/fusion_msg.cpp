#include <Sensor_fusion/fusion_declare.h>

bool check_overlap_idx(bool flag, int idx){ //flag = 0; => LiDAR | flag = 1; => camera
    if(flag == 0){ 
        for(int i = 0; i < used_LiDAR_DATA.size(); i++){
            if(used_LiDAR_DATA[i] == idx) return 1; 
        }
    }
    else {
        for(int i = 0; i < used_camera_DATA.size(); i++){ 
            if(used_camera_DATA[i] == idx) return 1;
        }
    }
    return 0;
}

void only_LiDAR_obj_print(cv::Mat& fin_Image, vector<Sensor_fusion::LiDAR_BB>& LiDAR_Bbox){
    for(int i=0;i<LiDAR_Bbox.size();i++){
        if(check_overlap_idx(0,i)) continue;
        float width = LiDAR_Bbox[i].xmax_LiDAR_px - LiDAR_Bbox[i].xmin_LiDAR_px;
        float height =  LiDAR_Bbox[i].ymax_LiDAR_px - LiDAR_Bbox[i].ymin_LiDAR_px;               
        cv::rectangle(fin_Image,cv::Rect(LiDAR_Bbox[i].xmin_LiDAR_px - 10,LiDAR_Bbox[i].ymin_LiDAR_px - 10,width + 20,height + 20),cv::Scalar(255,255,255),2,1,0);
    }
}

void only_camera_obj_print(cv::Mat& fin_Image, vector<Sensor_fusion::Camera_BB>& Camera_Bbox){
    for(int i=0;i<Camera_Bbox.size();i++){
        if(check_overlap_idx(1,i)) continue;
        float width = Camera_Bbox[i].xmax_Camera- Camera_Bbox[i].xmin_Camera;
        float height =  Camera_Bbox[i].ymax_Camera- Camera_Bbox[i].ymin_Camera;
        cv::rectangle(fin_Image,cv::Rect(Camera_Bbox[i].xmin_Camera - 10,Camera_Bbox[i].ymin_Camera - 10,width + 20,height + 20),cv::Scalar(255,255,255),2,1,0);    
    }
}

void tff_pub(std_msgs::String TFF_DATA){
    trafficSign_pub.publish(TFF_DATA);
}

void only_LiDAR_obj_pub(cv::Mat& fin_Image, Sensor_fusion::LiDAR_BB_arr& LiDAR_BoundingBox_arr, vector<Sensor_fusion::LiDAR_BB>& LiDAR_Bbox){
    Sensor_fusion::object_msg_arr::Ptr L_DATA_arr(new Sensor_fusion::object_msg_arr);
    Sensor_fusion::object_msg::Ptr L_DATA(new Sensor_fusion::object_msg);
    int onlyL_idx = 1;
    for(int i = 0;i < LiDAR_BoundingBox_arr.LiDAR_BB_arr.size(); i++){
        if(check_overlap_idx(0,i)) continue; //check only LiDAR DATA or fusioned DATA
        L_DATA -> classes = "UNKOWN";
        L_DATA -> idx = onlyL_idx++;
        L_DATA -> x = LiDAR_BoundingBox_arr.LiDAR_BB_arr[i].xcenter;
        L_DATA -> y = LiDAR_BoundingBox_arr.LiDAR_BB_arr[i].ycenter;
        L_DATA -> z = LiDAR_BoundingBox_arr.LiDAR_BB_arr[i].zcenter;
        L_DATA -> xMin = LiDAR_BoundingBox_arr.LiDAR_BB_arr[i].xmin;
        L_DATA -> xMax = LiDAR_BoundingBox_arr.LiDAR_BB_arr[i].xmax;
        L_DATA -> yMin = LiDAR_BoundingBox_arr.LiDAR_BB_arr[i].ymin;
        L_DATA -> yMax = LiDAR_BoundingBox_arr.LiDAR_BB_arr[i].ymax;
        L_DATA -> zMin = LiDAR_BoundingBox_arr.LiDAR_BB_arr[i].zmin;
        L_DATA -> zMax = LiDAR_BoundingBox_arr.LiDAR_BB_arr[i].zmax;
        L_DATA_arr -> object_msg_arr.push_back(*L_DATA);
    }
    if(switch_only_LiDAR_BB) only_LiDAR_obj_print(fin_Image, LiDAR_Bbox);
    LiDARBB_pub.publish(L_DATA_arr);
}

void only_camera_obj_pub(cv::Mat& fin_Image, Sensor_fusion::Camera_BB_arr& Camera_BoundingBox_arr, vector<Sensor_fusion::Camera_BB>& Camera_Bbox){
    std_msgs::String C_DATA_arr_st;
    for(int i = 0; i < Camera_BoundingBox_arr.Camera_BB_arr.size(); i++){
        if(check_overlap_idx(1,i)) continue;
        C_DATA_arr_st.data += Camera_BoundingBox_arr.Camera_BB_arr[i].Class;
        C_DATA_arr_st.data += "/";
    }
    if(switch_only_Camera_BB) only_camera_obj_print(fin_Image, Camera_Bbox);
    cameraBB_pub.publish(C_DATA_arr_st);
}

void fusion_obj_pub(vector<finalbox>& fusion_BB){
    Sensor_fusion::object_msg_arr::Ptr F_DATA_arr(new Sensor_fusion::object_msg_arr);
    Sensor_fusion::object_msg::Ptr F_DATA(new Sensor_fusion::object_msg);
    for(int i = 0; i < fusion_BB.size(); i++){
        F_DATA->classes = fusion_BB[i].Class;
        F_DATA->idx     = i+1;
        F_DATA->x       = fusion_BB[i].xcenter;
        F_DATA->y       = fusion_BB[i].ycenter;
        F_DATA->z       = fusion_BB[i].zcenter;
        F_DATA->xMin    = fusion_BB[i].Lxmin;
        F_DATA->xMax    = fusion_BB[i].Lxmax;
        F_DATA->yMin    = fusion_BB[i].Lymin;
        F_DATA->yMax    = fusion_BB[i].Lymax;
        F_DATA->zMin    = fusion_BB[i].Lzmin;
        F_DATA->zMax    = fusion_BB[i].Lzmax;
        F_DATA_arr->object_msg_arr.push_back(*F_DATA);
    }
    fusionBB_pub.publish(F_DATA_arr);
}

