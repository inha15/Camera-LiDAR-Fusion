#include <Sensor_fusion/fusion_declare.h>

double s_to_f(string st){
    double tmp = 0;
    string tmp_st = st.substr(2,2);
    int tmp_int = stoi(tmp_st);
    tmp = (double)tmp_int / 100;
    return tmp;
}

pair<pair<int, int>, pair<int, int>> s_to_minmax(string st) {
    pair<pair<int, int>, pair<int, int>> tmp;
    int tmp_xmin = 0, tmp_xmax = 0, tmp_ymin = 0, tmp_ymax = 0;
    int srt = 0;
    int flag = 1; //xmin = 1, xmax = 2, ymin = 3, ymax = 4
    bool check_srt = 0;//숫자가 시작하는 시점인지 체크해야함

    for (int i = 0; i < st.size(); i++) {
        if (st[i] != '.') continue;
        if(flag==1){
            tmp_xmin = stoi(st.substr(srt, srt - i));
        }
        else if (flag == 2) {
            tmp_ymin = stoi(st.substr(srt, srt - i));
        }
        else if (flag == 3) {
            tmp_xmax = stoi(st.substr(srt, srt - i));
        }
        else {
            tmp_ymax = stoi(st.substr(srt, srt - i));
            break;
        }
        i += 3;
        flag++;
        srt = i;
    }

    tmp = make_pair(make_pair(tmp_xmin, tmp_xmax), make_pair(tmp_ymin, tmp_ymax));
    return tmp;
}

bool check_TFF_SIGN(string tmp){
    if(tmp == "R" || tmp == "G" || tmp == "O" || tmp == "Y" || tmp == "LR" || tmp == "LG") return 1;
}

void trans_Camera_DATA(Sensor_fusion::Camera_BB_arr::Ptr Camera_BoundingBox_arr, Sensor_fusion::Camera_BB_arr::Ptr TFF_BoundingBox_arr){
    std_msgs::String tmp_Camera = Camera_DATA_comb; //data LOCK process... 중간의 데이터 변화를 막는 과정
    string Camera_obj = tmp_Camera.data;
    vector<string> OBJ;
    int tmp = 0;

    for(int i = 0;i < Camera_obj.size(); i++){
        if(Camera_obj[i] != '/') continue;
        string tmp_st = Camera_obj.substr(tmp, i - tmp);
        OBJ.push_back(tmp_st + '-'); //코드 편의를 위해 끝에 - 추가
        tmp = i + 1;
    }

    for(int i = 0;i < OBJ.size(); i++){
        Sensor_fusion::Camera_BB::Ptr tmp_Camera_Bbox (new Sensor_fusion::Camera_BB);
        int flag = 1; // 1 = class, 2 = minmax, 3 = probability
        tmp = 0;

        for(int j = 0; j < OBJ[i].size(); j++){
            if(OBJ[i][j] != '-') continue;

            string tmp_st;
            if(flag == 1){//class
                tmp_st = OBJ[i].substr(tmp, j - tmp);
                tmp_Camera_Bbox->Class = tmp_st;
            }
            else if(flag == 2){//minmax
                tmp_st = OBJ[i].substr(tmp, j - tmp);
                pair<pair<int,int>,pair<int,int>> minmax = s_to_minmax(tmp_st);
                //xmin, xmax, ymin, ymax
                tmp_Camera_Bbox->xmin_Camera = minmax.first.first;
                tmp_Camera_Bbox->xmax_Camera = minmax.first.second;
                tmp_Camera_Bbox->ymin_Camera = minmax.second.first;
                tmp_Camera_Bbox->ymax_Camera = minmax.second.second;
            }
            else{//probability
                tmp_st = OBJ[i].substr(tmp, j - tmp);
                tmp_Camera_Bbox->probability = s_to_f(tmp_st);
            }
            tmp = j + 1;
            flag++;
        }
        if(check_TFF_SIGN(tmp_Camera_Bbox->Class)) TFF_BoundingBox_arr->Camera_BB_arr.push_back(*tmp_Camera_Bbox);
        else Camera_BoundingBox_arr->Camera_BB_arr.push_back(*tmp_Camera_Bbox);
    }
    //test print
    // cout << "<Camera OBJ>" << endl;
    // for(int i=0;i<Camera_BoundingBox_arr->Camera_BB_arr.size();i++){
    //     cout<<Camera_BoundingBox_arr->Camera_BB_arr[i].Class<< " "<<Camera_BoundingBox_arr->Camera_BB_arr[i].xmin_Camera<< " "<<Camera_BoundingBox_arr->Camera_BB_arr[i].xmax_Camera<< " "<<Camera_BoundingBox_arr->Camera_BB_arr[i].ymin_Camera<< " "<<Camera_BoundingBox_arr->Camera_BB_arr[i].ymax_Camera<< " "<<Camera_BoundingBox_arr->Camera_BB_arr[i].probability<<endl;
    // }
    // for(int i=0;i<Camera_BoundingBox_arr->Camera_BB_arr.size();i++){
    //     Camera_BoundingBox_arr->Camera_BB_arr[i].Class 
    // }
}

float get_TFF_area(Sensor_fusion::Camera_BB tmp){
    return abs((tmp.xmax_Camera - tmp.xmin_Camera) * (tmp.ymax_Camera - tmp.ymin_Camera));
}

bool TFF_comp(pair<string,float> a, pair<string,float> b){
    return a.second > b.second;
}

void extract_traffic_SIGN(Sensor_fusion::Camera_BB_arr::Ptr TFF_BoundingBox_arr, std_msgs::String& Traffic_SIGN){
    vector<pair<string,float>> TFF_arr;
    for(int i = 0; i < TFF_BoundingBox_arr->Camera_BB_arr.size(); i++){
        TFF_arr.push_back(make_pair(TFF_BoundingBox_arr->Camera_BB_arr[i].Class, get_TFF_area(TFF_BoundingBox_arr->Camera_BB_arr[i])));
    }
    sort(TFF_arr.begin(),TFF_arr.end(),TFF_comp);
    Traffic_SIGN.data = TFF_arr[0].first;
}