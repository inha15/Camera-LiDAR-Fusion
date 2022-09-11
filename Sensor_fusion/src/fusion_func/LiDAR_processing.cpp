#include <Sensor_fusion/fusion_declare.h>

int check_project_case(vector<float>& OBJ){
    //U = up, M = middle, D = down, L = left, R = right
    //UL = 0, UM = 1, UR = 2, ML = 3, MM = 4, ML = 5, DL = 6, DM = 7, DR = 8    
    // x is always positive, each case is devided y,z sign 
    if(OBJ[7] >= 0 && OBJ[8] >= 0){                     //zmin = +, zmax = +       =>  U?
        if(OBJ[5] >= 0 && OBJ[6] >= 0) return 0;        //ymin = +, ymax = +       =>  UL
        else if(OBJ[5] <= 0 && OBJ[6] >= 0) return 1;   //ymin = -, ymax = +       =>  UM
        else return 2;                                  //ymin = -, ymax = -       =>  UR
    }
    else if(OBJ[7] <= 0 && OBJ[8] >= 0){                //zmin = -, zmax = +       =>  M?
        if(OBJ[5] >= 0 && OBJ[6] >= 0) return 3;        //ymin = +, ymax = +       =>  ML
        else if(OBJ[5] <= 0 && OBJ[6] >= 0) return 4;   //ymin = -, ymax = +       =>  MM
        else return 5;                                  //ymin = -, ymax = -       =>  MR
    }
    else{                                               //zmin = -, zmax = -       =>  D?
        if(OBJ[5] >= 0 && OBJ[6] >= 0) return 6;        //ymin = +, ymax = +       =>  DL
        else if(OBJ[5] <= 0 && OBJ[6] >= 0) return 7;   //ymin = -, ymax = +       =>  DM
        else return 8;                                  //ymin = -, ymax = -       =>  DR
    }
}

void project_func(vector<float>& OBJ, Sensor_fusion::LiDAR_BB::Ptr tmp_Lidar, int case_num){
    cv::Mat LiDAR3DPoint(4,1,CV_32F, cv::Scalar(1,0)); //(XL, YL, ZL, 1)
    cv::Mat Img2DPoint(3,1,CV_32F); //(XC/ZC, YC/ZC, 1)
    
    switch(case_num){
    case 2:
    case 6: //this case   2,6
        //-------- get min fixel coord ---------
        LiDAR3DPoint.at<float>(0,0) = OBJ[3]; //xMin
        LiDAR3DPoint.at<float>(1,0) = OBJ[5]; //yMin
        LiDAR3DPoint.at<float>(2,0) = OBJ[7]; //zMin        
        //calibration
        Img2DPoint = final_project_mat * LiDAR3DPoint;
        //scale factor
        Img2DPoint.at<float>(0,0) /= Img2DPoint.at<float>(2,0);
        Img2DPoint.at<float>(1,0) /= Img2DPoint.at<float>(2,0);

        tmp_Lidar->xmin_LiDAR_px = Img2DPoint.at<float>(0,0);
        tmp_Lidar->ymin_LiDAR_px = Img2DPoint.at<float>(1,0);
        //-------- get max fixel coord ---------
        LiDAR3DPoint.at<float>(0,0) = OBJ[4]; //xMax
        LiDAR3DPoint.at<float>(1,0) = OBJ[6]; //yMax
        LiDAR3DPoint.at<float>(2,0) = OBJ[8]; //zMax        
        //calibration
        Img2DPoint = final_project_mat * LiDAR3DPoint;
        //scale factor
        Img2DPoint.at<float>(0,0) /= Img2DPoint.at<float>(2,0);
        Img2DPoint.at<float>(1,0) /= Img2DPoint.at<float>(2,0);
        
        tmp_Lidar->xmax_LiDAR_px = Img2DPoint.at<float>(0,0);
        tmp_Lidar->ymax_LiDAR_px = Img2DPoint.at<float>(1,0);
        break;
    default: //other case   0,1,3,4,5,7,8
        //-------- get min fixel coord ---------
        LiDAR3DPoint.at<float>(0,0) = OBJ[3]; //xMin
        LiDAR3DPoint.at<float>(1,0) = OBJ[6]; //yMax
        LiDAR3DPoint.at<float>(2,0) = OBJ[7]; //zMin        
        //calibration
        Img2DPoint = final_project_mat * LiDAR3DPoint;
        //scale factor
        Img2DPoint.at<float>(0,0) /= Img2DPoint.at<float>(2,0);
        Img2DPoint.at<float>(1,0) /= Img2DPoint.at<float>(2,0);

        tmp_Lidar->xmin_LiDAR_px = Img2DPoint.at<float>(0,0);
        tmp_Lidar->ymin_LiDAR_px = Img2DPoint.at<float>(1,0);
        //-------- get max fixel coord ---------
        LiDAR3DPoint.at<float>(0,0) = OBJ[4]; //xMax
        LiDAR3DPoint.at<float>(1,0) = OBJ[5]; //yMin
        LiDAR3DPoint.at<float>(2,0) = OBJ[8]; //zMax        
        //calibration
        Img2DPoint = final_project_mat * LiDAR3DPoint;
        //scale factor
        Img2DPoint.at<float>(0,0) /= Img2DPoint.at<float>(2,0);
        Img2DPoint.at<float>(1,0) /= Img2DPoint.at<float>(2,0);
        
        tmp_Lidar->xmax_LiDAR_px = Img2DPoint.at<float>(0,0);
        tmp_Lidar->ymax_LiDAR_px = Img2DPoint.at<float>(1,0);
        break;
    }
}

void project_LiDAR_to_Img(vector<vector<float>>& OBJ, Sensor_fusion::LiDAR_BB_arr::Ptr LiDAR_BoundingBox_arr){
    cv::Mat LiDAR3DPoint(4,1,CV_32F, cv::Scalar(1,0)); //(XL, YL, ZL, 1)
    cv::Mat Img2DPoint(3,1,CV_32F); //(XC/ZC, YC/ZC, 1)
    // cout<<"------------------- OBJ coord ----------------"<<endl;
    // for(int i=0;i<OBJ.size();i++){
    //     cout<< "x: " << OBJ[i][0]<<"    y: "<< OBJ[i][1]<<"    z: "<< OBJ[i][2]<<endl;
    // }
    for(int i = 0; i < OBJ.size(); i++){
        Sensor_fusion::LiDAR_BB::Ptr tmp_Lidar (new Sensor_fusion::LiDAR_BB);
        //x,y,z값은 라이다 좌표 그대로 보존해야 한다.
        tmp_Lidar->xcenter = OBJ[i][0]; //x
        tmp_Lidar->ycenter = OBJ[i][1]; //y
        tmp_Lidar->zcenter = OBJ[i][2]; //z
        tmp_Lidar->xmin = OBJ[i][3]; //xmin
        tmp_Lidar->xmax = OBJ[i][4]; //xmax
        tmp_Lidar->ymin = OBJ[i][5]; //ymin
        tmp_Lidar->ymax = OBJ[i][6]; //ymax
        tmp_Lidar->zmin = OBJ[i][7]; //zmin
        tmp_Lidar->zmax = OBJ[i][8]; //zmax

        project_func(OBJ[i],tmp_Lidar,check_project_case(OBJ[i]));

        // min max rearrange
        if(tmp_Lidar->xmin_LiDAR_px > tmp_Lidar->xmax_LiDAR_px){
            float tmp = tmp_Lidar->xmax_LiDAR_px;
            tmp_Lidar->xmax_LiDAR_px = tmp_Lidar->xmin_LiDAR_px;
            tmp_Lidar->xmin_LiDAR_px = tmp;
        }
            
        if(tmp_Lidar->ymin_LiDAR_px > tmp_Lidar->ymax_LiDAR_px){
            float tmp = tmp_Lidar->ymax_LiDAR_px;
            tmp_Lidar->ymax_LiDAR_px = tmp_Lidar->ymin_LiDAR_px;
            tmp_Lidar->ymin_LiDAR_px = tmp;
        }

        if(tmp_Lidar->ymin_LiDAR_px < PIXEL_YMIN) tmp_Lidar->ymin_LiDAR_px = 0;
        if(tmp_Lidar->xmin_LiDAR_px < PIXEL_XMIN) tmp_Lidar->xmin_LiDAR_px = 0;
        if(tmp_Lidar->ymax_LiDAR_px > PIXEL_YMAX) tmp_Lidar->ymax_LiDAR_px = PIXEL_YMAX;
        if(tmp_Lidar->xmax_LiDAR_px > PIXEL_XMAX) tmp_Lidar->xmax_LiDAR_px = PIXEL_XMAX;

        // cout<<"------------------- trans coord min ----------------"<<endl;
        // cout<< "x : " << tmp_Lidar->xmin_LiDAR_px << " y : "<< tmp_Lidar->ymin_LiDAR_px << endl;
        // cout<<"------------------- trans coord max ----------------"<<endl;
        // cout<< "x : " << tmp_Lidar->xmax_LiDAR_px << " y : " << tmp_Lidar->ymax_LiDAR_px << endl;

        LiDAR_BoundingBox_arr->LiDAR_BB_arr.push_back(*tmp_Lidar);  
    }
}

void trans_LiDAR_DATA(Sensor_fusion::LiDAR_BB_arr::Ptr LiDAR_BoundingBox_arr){
    Sensor_fusion::object_msg_arr tmp_LiDAR = LiDAR_DATA_comb; //data LOCK process... 중간의 데이터 변화를 막는 과정
    
    //OBJ msg trans
    vector<vector<float>> OBJ;
    vector<float> ONE_OBJ; //x,y,z,xmin,xmax,ymin,ymax,zmin,zmax
    ONE_OBJ.resize(9);
    for(int i=0;i<tmp_LiDAR.objc;i++){
        ONE_OBJ[0] = tmp_LiDAR.object_msg_arr[i].x;
        ONE_OBJ[1] = tmp_LiDAR.object_msg_arr[i].y;
        ONE_OBJ[2] = tmp_LiDAR.object_msg_arr[i].z;
        ONE_OBJ[3] = tmp_LiDAR.object_msg_arr[i].xMin;
        ONE_OBJ[4] = tmp_LiDAR.object_msg_arr[i].xMax;
        ONE_OBJ[5] = tmp_LiDAR.object_msg_arr[i].yMin;
        ONE_OBJ[6] = tmp_LiDAR.object_msg_arr[i].yMax;
        ONE_OBJ[7] = tmp_LiDAR.object_msg_arr[i].zMin;
        ONE_OBJ[8] = tmp_LiDAR.object_msg_arr[i].zMax;
        OBJ.push_back(ONE_OBJ);
    } 
    project_LiDAR_to_Img(OBJ,LiDAR_BoundingBox_arr);
}