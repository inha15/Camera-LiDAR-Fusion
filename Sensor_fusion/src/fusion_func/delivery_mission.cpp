#include <Sensor_fusion/fusion_declare.h>

//----------------------------camera-------------------------------

float get_area(Sensor_fusion::Camera_BB C_BB){
    return abs(C_BB.xmax_Camera-C_BB.xmin_Camera)*abs(C_BB.ymax_Camera-C_BB.ymin_Camera);
}

bool comp_camera_deliv(pair<float,Sensor_fusion::Camera_BB> a, pair<float,Sensor_fusion::Camera_BB> b){
    return a.first > b.first;
}

string get_sorted_camera_deliv(Sensor_fusion::Camera_BB_arr::Ptr Camera_BoundingBox_arr){
    string ret = "";
    vector<pair<float,Sensor_fusion::Camera_BB>> sign_board;
    for(int i=0;i<Camera_BoundingBox_arr->Camera_BB_arr.size();i++){
        if(check_class(Camera_BoundingBox_arr->Camera_BB_arr[i].Class) == 3){
            sign_board.push_back(make_pair(get_area(Camera_BoundingBox_arr->Camera_BB_arr[i]),Camera_BoundingBox_arr->Camera_BB_arr[i]));
        }
    }
    sort(sign_board.begin(),sign_board.end(),comp_camera_deliv);
    for(int i=0; i<sign_board.size();i++){
        ret += sign_board[i].second.Class;
    }
    return ret;
}

//----------------------------LiDAR-------------------------------

bool comp_LiDAR_deliv(vector<float> a, vector<float> b){
    return cal_dist(a[0],a[1]) < cal_dist(b[0],b[1]);
}

void sort_LiDAR_deliv(vector<vector<float>>& OBJ){
    sort(OBJ.begin(),OBJ.end(),comp_LiDAR_deliv);
}

vector<vector<float>> trans_LiDAR_DATA_deliv(Sensor_fusion::LiDAR_BB_arr::Ptr LiDAR_BoundingBox_arr){
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
    return OBJ;
}

//----------------------------total-------------------------------

string ret_class(std::string BB_class) {
    if (BB_class == "A1") return "3";
    else if (BB_class == "A2") return "4";
    else if (BB_class == "A3") return "5";
    else if (BB_class == "B1") return "6";
    else if (BB_class == "B2") return "7";
    else if (BB_class == "B3") return "8";
}

string tmp_msg_generator(vector<vector<float>> Lidar_data, string tmp_signboard){
    string ret = "000";
    string tmp;
    vector<string> sorted_signboard;
    for(int i=0;i<tmp_signboard.size();i+=2){
        sorted_signboard.push_back(tmp_signboard.substr(i,2));
    }
    ret += to_string(sorted_signboard.size());
    for(int i=0;i<Lidar_data.size();i++){
        string tmp_x = (Lidar_data[i][0] < 0) ? to_string(((int)(Lidar_data[i][0] * -100) / 2) * 2 + 1) : to_string(((int)(Lidar_data[i][0] * 100) / 2) * 2);
        string tmp_y = (Lidar_data[i][1] < 0) ? to_string(((int)(Lidar_data[i][1] * -100) / 2) * 2 + 1) : to_string(((int)(Lidar_data[i][1] * 100) / 2) * 2);
        
        ret = ret + tmp_x + tmp_y + "0000000000000000000" + ret_class(sorted_signboard[i]);
    }
    return ret;
}

void deliv_processing(){

    Sensor_fusion::LiDAR_BB_arr::Ptr LiDAR_BoundingBox_arr(new Sensor_fusion::LiDAR_BB_arr); 
    Sensor_fusion::Camera_BB_arr::Ptr Camera_BoundingBox_arr(new Sensor_fusion::Camera_BB_arr);
    Sensor_fusion::Camera_BB_arr::Ptr tmp_arr(new Sensor_fusion::Camera_BB_arr); //no use

    vector<vector<float>> Lidar_data = trans_LiDAR_DATA_deliv(LiDAR_BoundingBox_arr); //라이다 기존 데이터가 보존된 2차원 벡터 OBJ 만들어줌
    trans_Camera_DATA(Camera_BoundingBox_arr,tmp_arr);

    //-----------------delivery func--------------------
    sort_LiDAR_deliv(Lidar_data);
    // for(int i=0; i<Lidar_data.size();i++){
    //     cout<<"OBJ "<<i<<"dist : "<<cal_dist(Lidar_data[i][0],Lidar_data[i][1])<<endl;
    // }
    // cout<<endl;
    // if(!Lidar_data.empty()){
    //     cout<<"sorted data : "<<cal_dist(Lidar_data[0][0],Lidar_data[0][1])<<endl;
    // }
    
    //string tmp_signboard = get_sorted_camera_deliv(Camera_BoundingBox_arr);
    vector<string> sorted_signboard;
    for(int i=0;i<tmp_signboard.size();i+=2){
        sorted_signboard.push_back(tmp_signboard.substr(i,2));
    }

    if(real_finish == 1){
        if(finish_iter > 0){
            finish_iter--;
            cout<<"fin iter : "<<finish_iter<<endl;
        }
        else{
            cout << "------------delivery finish------------"<<endl;
            return;
        }
    }
    vector<float> tmp_Lidar_data;
    for(int z=0;z<1;z++){
        if(delete_flag == 1 && lap_cnt == 2){
            if(reset_iter > 0){
                reset_iter--;
                cout << "delay : " << reset_iter <<endl;
                break;
            }
            else{
                tmp_signboard = reset_tmp_signboard;
                finish_flag = 1;
                lap_cnt++;
                delete_flag = 0;
                break;
            }
        }
        if(Lidar_data.size()>0){
            if(Lidar_data[0][0] < 0.7 && Lidar_data[0][0] > -0.1){ //input delete zone
                tmp_Lidar_data = Lidar_data[0]; 
                vector<vector<float>> sb_tmp_Lidar_data;
                if(Lidar_data.size() == 1) Lidar_data.clear();
                else{
                    for(int i=1;i<Lidar_data.size();i++){
                        sb_tmp_Lidar_data.push_back(Lidar_data[i]);
                    }
                    Lidar_data.clear();
                    Lidar_data = sb_tmp_Lidar_data;
                }
                //Lidar_data.erase(Lidar_data.begin());
            }
            if(Lidar_data[0][0] < -0.1){
                //Lidar_data.erase(Lidar_data.begin());
                vector<vector<float>> sb_tmp_Lidar_data;
                if(Lidar_data.size() == 1) Lidar_data.clear();
                else{
                    for(int i=1;i<Lidar_data.size();i++){
                        sb_tmp_Lidar_data.push_back(Lidar_data[i]);
                    }
                    Lidar_data.clear();
                    Lidar_data = sb_tmp_Lidar_data;
                }
            }
            
            if(delete_flag == 1 && tmp_Lidar_data.size() == 0){
                //if judge OBJ in delete zone
                //no directly change delete flag, need delay for tracking
                if(tracking_iter > 0){
                    tracking_iter--;
                }
                else{
                    tracking_iter = tmp_tracking_iter;
                    delete_flag = 0;
                }
            }
        }

        for(int i=0;i<sorted_signboard.size();i++){
            if(Lidar_data.size() - 1 < i){
                cout<<sorted_signboard[i]<< "   Dist : 99999" <<endl;
            }
            else{
                if(Lidar_data.size()>i){//sync problem??... need mutex??
                    cout<<sorted_signboard[i]<< "   Dist : " << cal_dist(Lidar_data[i][0],Lidar_data[i][1]) << "   X : " << Lidar_data[i][0] <<endl;
                    if(cal_dist(Lidar_data[i][0],Lidar_data[i][1]) > 20 || cal_dist(Lidar_data[i][0],Lidar_data[i][1]) < 0.3){
                        return;
                    }
                }
            }
        }
        vector<vector<float>> tmpp;
        //tmpp.push_back(tmp_Lidar_data);
        tmpp = Lidar_data;
        for(int i = tmpp.size();i < sorted_signboard.size();i++){
            vector<float> sbtmp;
            sbtmp.resize(9);
            sbtmp[0] = 98;
            sbtmp[1] = 98;
            tmpp.push_back(sbtmp);
        }
        string msg_pub_st = tmp_msg_generator(tmpp,tmp_signboard);
        msg_process(msg_pub_st);
        if(!tmp_Lidar_data.empty() && delete_flag == 0){
            if(sorted_signboard.size() == 1){
                lap_cnt++;
                delete_flag = 1;
                if(finish_flag == 1) real_finish = 1;
            }
            else{
                
                tmp_signboard = tmp_signboard.substr(2);
                delete_flag = 1;
            }
        }
        if(Lidar_data.size() == 0){
            for(int i=0;i<sorted_signboard.size();i++){
                cout<<sorted_signboard[i]<< "   Dist : 99999" <<endl;
            }
        }
        cout<<"------------------------------------------"<<endl;
    }
}

//----------------------------MSG-------------------------------

void msg_process(string st){
    std_msgs::String fusion_st;
    fusion_st.data = st;
    deliv_pub.publish(fusion_st);
}