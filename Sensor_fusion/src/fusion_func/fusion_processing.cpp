#include <Sensor_fusion/fusion_declare.h>

string f_to_s(float tmp){
    string st;
    int tmp1 = tmp; //정수부
    int tmp2 = (abs(tmp - tmp1)) * 100; //소수부
    if(tmp2 / 10 == 0) st = to_string(tmp1)+".0"+to_string(tmp2);
    else st = to_string(tmp1)+"."+to_string(tmp2);
    if(tmp1 == 0 && tmp - tmp1 < 0) st = "-" + st;
    return st;
}

void print_LiDAR_BB(cv::Mat& fin_Image,vector<Sensor_fusion::LiDAR_BB>& LiDAR_Bbox){
    for(int i=0;i<LiDAR_Bbox.size();++i){
        float width = LiDAR_Bbox[i].xmax_LiDAR_px - LiDAR_Bbox[i].xmin_LiDAR_px;
        float height =  LiDAR_Bbox[i].ymax_LiDAR_px - LiDAR_Bbox[i].ymin_LiDAR_px;               
        cv::rectangle(fin_Image,cv::Rect(LiDAR_Bbox[i].xmin_LiDAR_px,LiDAR_Bbox[i].ymin_LiDAR_px,width,height),cv::Scalar(255,0,0),2,1,0);
    }
}

void print_Camera_BB(cv::Mat& fin_Image,vector<Sensor_fusion::Camera_BB>& Camera_Bbox){
    for(int i=0;i<Camera_Bbox.size();++i){
        //if(Camera_Bbox[i].probability < 0.4) continue;
        float width = Camera_Bbox[i].xmax_Camera- Camera_Bbox[i].xmin_Camera;
        float height =  Camera_Bbox[i].ymax_Camera- Camera_Bbox[i].ymin_Camera;
        cv::rectangle(fin_Image,cv::Rect(Camera_Bbox[i].xmin_Camera,Camera_Bbox[i].ymin_Camera,width,height),cv::Scalar(0,0,255),2,1,0);    
    }
}

void print_Fusion_BB(cv::Mat& fin_Image, finalbox& fusionbox){
    float width_F = fusionbox.xmax - fusionbox.xmin;
    float height_F=  fusionbox.ymax - fusionbox.ymin;
    cv::rectangle(fin_Image,cv::Rect(fusionbox.xmin,fusionbox.ymin,width_F,height_F),cv::Scalar(0,255,255),3,1,0);

    string text = fusionbox.Class + " - " + "x : " + f_to_s(fusionbox.xcenter) + " " + "y : " + f_to_s(fusionbox.ycenter) + " " + "z : " + f_to_s(fusionbox.zcenter);
    //cout << "fusion Bbox : " << text << endl;
    int font_face = cv::FONT_HERSHEY_SIMPLEX; //폰트 종류
    double font_scale = 0.7; //폰트 크기
    int thickness = 2;
    cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, NULL); //baseline = NULL
    cv::Point origin;
    origin.x = fusionbox.xmin - 100; //offset = -100
    origin.y = fusionbox.ymin - text_size.height / 2;
    cv::putText(fin_Image, text, origin, font_face, font_scale, cv::Scalar(70, 255, 70), thickness, 8, 0); //BGR
}

int check_class(std::string BB_class) {
    if (BB_class == "person")  return 1;
    else if (BB_class == "car") return 2;
    else if (BB_class == "A1") return 3;
    else if (BB_class == "A2") return 3;
    else if (BB_class == "A3") return 3;
    else if (BB_class == "B1") return 3;
    else if (BB_class == "B2") return 3;
    else if (BB_class == "B3") return 3;
    else return 0;
}

bool check_overlap(Sensor_fusion::LiDAR_BB& LiDARBB,Sensor_fusion::Camera_BB& CameraBB, float width_C, float height_C){
    bool maxx = (CameraBB.xmax_Camera - LiDARBB.xmax_LiDAR_px < width_C / 1.3) && (CameraBB.xmax_Camera - LiDARBB.xmax_LiDAR_px > -width_C / 1.3);
    bool minx = (CameraBB.xmin_Camera - LiDARBB.xmin_LiDAR_px < width_C / 1.3) && (CameraBB.xmin_Camera - LiDARBB.xmin_LiDAR_px > -width_C / 1.3);
    bool miny = (CameraBB.ymin_Camera - LiDARBB.ymin_LiDAR_px < height_C / 1.3) && (CameraBB.ymin_Camera - LiDARBB.ymin_LiDAR_px > -height_C / 1.3);
    bool maxy = (CameraBB.ymax_Camera - LiDARBB.ymax_LiDAR_px < height_C / 1.3) && (CameraBB.ymax_Camera - LiDARBB.ymax_LiDAR_px > -height_C / 1.3);
    return maxx && minx && miny && maxy;
}

float check_IoU(Sensor_fusion::LiDAR_BB& LiDARBB, Sensor_fusion::Camera_BB& CameraBB) {
    //https://kuklife.tistory.com/125
    float CBB_area, LBB_area, BB_overlap_area, IoU;
    CBB_area = (CameraBB.xmax_Camera - CameraBB.xmin_Camera) * (CameraBB.ymax_Camera - CameraBB.ymin_Camera);
    LBB_area = (LiDARBB.xmax_LiDAR_px - LiDARBB.xmin_LiDAR_px) * (LiDARBB.ymax_LiDAR_px - LiDARBB.ymin_LiDAR_px);

    float intersect_xmin, intersect_xmax, intersect_ymin, intersect_ymax;
    intersect_xmin = max((float)CameraBB.xmin_Camera, (float)LiDARBB.xmin_LiDAR_px);
    intersect_xmax = min((float)CameraBB.xmax_Camera, (float)LiDARBB.xmax_LiDAR_px);
    intersect_ymin = max((float)CameraBB.ymin_Camera, (float)LiDARBB.ymin_LiDAR_px);
    intersect_ymax = min((float)CameraBB.ymax_Camera, (float)LiDARBB.ymax_LiDAR_px);
    BB_overlap_area = (intersect_xmax - intersect_xmin) * (intersect_ymax - intersect_ymin); // 겹치는 부분 넓이

    IoU = abs(BB_overlap_area / (CBB_area + LBB_area - BB_overlap_area));
    return IoU;
}

finalbox make_fusionBox(Sensor_fusion::LiDAR_BB& LiDAR_Bbox, Sensor_fusion::Camera_BB& Camera_Bbox){
    finalbox fusionbox;
    if(FBB_maked_CameraLiDARcenter){
        fusionbox.xmax = (Camera_Bbox.xmax_Camera + LiDAR_Bbox.xmax_LiDAR_px) / 2;
        fusionbox.ymax = (Camera_Bbox.ymax_Camera + LiDAR_Bbox.ymax_LiDAR_px) / 2;
        fusionbox.xmin = (Camera_Bbox.xmin_Camera + LiDAR_Bbox.xmin_LiDAR_px) / 2;
        fusionbox.ymin = (Camera_Bbox.ymin_Camera + LiDAR_Bbox.ymin_LiDAR_px) / 2;
    }
    else if(FBB_maked_Cameracenter){
        fusionbox.xmax = Camera_Bbox.xmax_Camera;
        fusionbox.ymax = Camera_Bbox.ymax_Camera;
        fusionbox.xmin = Camera_Bbox.xmin_Camera;
        fusionbox.ymin = Camera_Bbox.ymin_Camera;
    }
    fusionbox.xcenter = LiDAR_Bbox.xcenter;
    fusionbox.ycenter = LiDAR_Bbox.ycenter;
    fusionbox.zcenter = LiDAR_Bbox.zcenter;
    fusionbox.Lxmin = LiDAR_Bbox.xmin;
    fusionbox.Lxmax = LiDAR_Bbox.xmax;
    fusionbox.Lymin = LiDAR_Bbox.ymin;
    fusionbox.Lymax = LiDAR_Bbox.ymax;
    fusionbox.Lzmin = LiDAR_Bbox.zmin;
    fusionbox.Lzmax = LiDAR_Bbox.zmax;
    fusionbox.Class = Camera_Bbox.Class;
    //fusionbox.probability = Camera_Bbox[i].probability;
    return fusionbox;
}

vector<finalbox> fusion_iter_basedOn_LiDAR(cv::Mat& fin_Image, vector<Sensor_fusion::LiDAR_BB>& LiDAR_Bbox, vector<Sensor_fusion::Camera_BB>& Camera_Bbox){
    vector<finalbox> fusionBB; //라이다 기준으로 퓨전 진행. 라이다에 카메라 Bbox를 매칭시키자.
    int idx_tmp = 0;
    bool flag_LiDAR_only = 0;
    bool flag_camera_only = 0;
    for(int j = 0;j < LiDAR_Bbox.size(); j++){
        for(int i = 0; i < Camera_Bbox.size(); i++){
            //if(Camera_Bbox[i].probability < 0.4) continue;
            float width_C = Camera_Bbox[i].xmax_Camera - Camera_Bbox[i].xmin_Camera;
            float height_C =  Camera_Bbox[i].ymax_Camera - Camera_Bbox[i].ymin_Camera;     
            if(!check_overlap(LiDAR_Bbox[j],Camera_Bbox[i],width_C,height_C)) continue; //only check overlap_BB, cause no overlaped BB can be get invalid IOU
            if(check_IoU(LiDAR_Bbox[j],Camera_Bbox[i]) >= IOU_limit){
                finalbox fusionbox = make_fusionBox(LiDAR_Bbox[j],Camera_Bbox[i]);
                fusionBB.push_back(fusionbox);
                flag_LiDAR_only = flag_camera_only = true;
                idx_tmp = i;
                if(switch_Fusion_BB) print_Fusion_BB(fin_Image,fusionbox);
                break;//중복처리 과정이 따로 없이 바로브레이크 박음.. 이거 나중에 고쳐야함
            }
        }
        if(flag_LiDAR_only) used_LiDAR_DATA.push_back(j);
        if(flag_camera_only) used_camera_DATA.push_back(idx_tmp);
        flag_LiDAR_only = flag_camera_only = 0;
    }
    return fusionBB;
}

vector<finalbox> fusion_iter_basedOn_camera(cv::Mat& fin_Image, vector<Sensor_fusion::LiDAR_BB>& LiDAR_Bbox, vector<Sensor_fusion::Camera_BB>& Camera_Bbox){
    vector<finalbox> fusionBB; //카메라 기준으로 퓨전 진행. 카메라에 라이다 Bbox를 매칭시키자.
    bool flag_camera_only = 0;
    bool flag_LiDAR_only = 0;
    int idx_tmp = 0;
    for(int i = 0;i < Camera_Bbox.size(); i++){
        //if(Camera_Bbox[i].probability < 0.4) continue;
        float width_C = Camera_Bbox[i].xmax_Camera - Camera_Bbox[i].xmin_Camera;
        float height_C =  Camera_Bbox[i].ymax_Camera - Camera_Bbox[i].ymin_Camera;        
        for(int j = 0; j < LiDAR_Bbox.size(); j++){
            if(!check_overlap(LiDAR_Bbox[j],Camera_Bbox[i],width_C,height_C)) continue; //only check overlap_BB, cause no overlaped BB can be get invalid IOU
            if(check_IoU(LiDAR_Bbox[j],Camera_Bbox[i]) >= IOU_limit){
                finalbox fusionbox = make_fusionBox(LiDAR_Bbox[j],Camera_Bbox[i]);
                fusionBB.push_back(fusionbox);
                flag_LiDAR_only = flag_camera_only = true;
                idx_tmp = j;
                if(switch_Fusion_BB) print_Fusion_BB(fin_Image,fusionbox);
                break;//중복처리 과정이 따로 없이 바로브레이크 박음.. 이거 나중에 고쳐야함
            }
        }
        if(flag_LiDAR_only) used_LiDAR_DATA.push_back(idx_tmp);
        if(flag_camera_only) used_camera_DATA.push_back(i);
        flag_LiDAR_only = flag_camera_only = 0;
	}
    return fusionBB;
}

void final_fusion(vector<finalbox>& fusionBB, const sensor_msgs::Image& Img, Sensor_fusion::LiDAR_BB_arr& LiDARBB, Sensor_fusion::Camera_BB_arr& CameraBB){
    vector<Sensor_fusion::LiDAR_BB> LiDAR_Bbox(LiDARBB.LiDAR_BB_arr);
    vector<Sensor_fusion::Camera_BB> Camera_Bbox(CameraBB.Camera_BB_arr);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(Img, "8UC3");  //sensor_msgs::image_encodings::BGR8 8UC3
    cv::Mat fin_Image = cv_ptr -> image;

    if(switch_LiDAR_BB) print_LiDAR_BB(fin_Image,LiDAR_Bbox);
    if(switch_Camera_BB) print_Camera_BB(fin_Image,Camera_Bbox);

    if(LiDAR_base_fusion) fusionBB = fusion_iter_basedOn_LiDAR(fin_Image,LiDAR_Bbox,Camera_Bbox);
    if(camera_base_fusion) fusionBB = fusion_iter_basedOn_camera(fin_Image,LiDAR_Bbox,Camera_Bbox);
    
    if(onlyLiDARMSG_pub) only_LiDAR_obj_pub(fin_Image,LiDARBB,LiDAR_Bbox);
    if(onlyCameraMSG_pub) only_camera_obj_pub(fin_Image,CameraBB,Camera_Bbox);
    
    // sensor_msgs::ImagePtr pubImg = cv_bridge::CvImage(std_msgs::Header(), "8UC3", fin_Image).toImageMsg();
    // pub1.publish(pubImg);    
    cv::imshow("fusion", fin_Image);
    cv::waitKey(1);
}
