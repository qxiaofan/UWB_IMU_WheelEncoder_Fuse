

#include "readData.h"

bool getSubstr(std::string string_in, char char1, char char2, std::string &sub_get, std::string &sub_other)
{

    if(string_in.length() == 0)
        return false;
    size_t i = string_in.find(char1);
    size_t j = string_in.find(char2);

    if(string_in[i] != char1 || string_in[j] != char2)
        return false;

    sub_get = string_in.substr(i+1, j-i-1);

    if( j < string_in.length() - 2)
        sub_other = string_in.substr(j+1, string_in.length()-j-1);
    else
        sub_other = "";

    return true;
}

std::vector<double> split(std::string s, char token){
    std::stringstream iss(s);
    std::string word;
    std::vector<double> vs;
    while(getline(iss,word,token)){
        vs.push_back(std::atof(word.c_str()));
    }
    return vs;
}


void LoadWheelMeasure(std::string file_path, std::vector<wheel_observe> &all_observation)
{
    std::ifstream file;
    file.open(file_path.c_str());
    if (!file.is_open())
    {
        cout << "open file failed !" << std::endl;
        return;
    }

    bool read_start = false;

    std::string s;
    std::getline(file, s);

    while(!file.eof())
    {
        if(s.empty())
        {
            std::getline(file, s);
            continue;
        }

        if(!read_start)
        {
            read_start = true;
            continue;
        }

        wheel_observe wheel_oi;

        std::vector<double> split_data = split(s, ' ');

        if(split_data.size() < 15)
            cout << "error in load imu data" << std::endl;

        wheel_oi.timestamp = split_data[0]/double(1000);
        wheel_oi.wheel = cv::Point3d(split_data[2], split_data[3], split_data[4]);

        all_observation.push_back(wheel_oi);

        std::getline(file, s);
    }

    cout << "all_wheel.size() = " << all_observation.size() << std::endl
         << "data 0 = " << all_observation[0].timestamp
         << ", " << all_observation[0].wheel << std::endl
         << "data end = " << all_observation[all_observation.size() - 1].timestamp
         << ", " << all_observation[all_observation.size() -1].wheel
         << std::endl << std::endl;

    return;
}

void LoadVslamMeasure(std::string file_path, std::vector<vslam_observe> &all_observation)
{

    std::ifstream file;
    file.open(file_path.c_str());
    if (!file.is_open())
    {
        cout << "open file failed !" << std::endl;
        return;
    }

    bool read_start = false;

    std::string s;
    std::getline(file, s);

    while(!file.eof())
    {
        if(s.empty())
        {
            std::getline(file, s);
            continue;
        }

        if(!read_start)
        {
            read_start = true;
            continue;
        }

        vslam_observe vslam_oi;

        std::vector<double> split_data = split(s, ' ');

        if(split_data.size() < 15)
            cout << "error in load imu data" << std::endl;

        vslam_oi.timestamp = split_data[0]/double(1000);
        vslam_oi.vslam = cv::Point3d(split_data[13], split_data[14], split_data[15]);

        all_observation.push_back(vslam_oi);

        std::getline(file, s);
    }

    cout << "all_vslam.size() = " << all_observation.size() << std::endl
         << "data 0 = " << all_observation[0].timestamp
         << ", " << all_observation[0].vslam << std::endl
         << "data end = " << all_observation[all_observation.size() - 1].timestamp
         << ", " << all_observation[all_observation.size() -1].vslam
         << std::endl << std::endl;

    return;
}

void LoadUWBMeasure(std::string file_path, std::vector<uwb_observe> &all_observation)
{

    std::ifstream file;
    file.open(file_path.c_str());
    if (!file.is_open())
    {
        cout << "open file failed !" << std::endl;
        return;
    }

    bool read_start = false;

    std::string s;
    std::getline(file, s);

    while(!file.eof())
    {
        if(s.empty())
        {
            std::getline(file, s);
            continue;
        }

        if(!read_start)
        {
            if(s.length() >= 34 && s.substr(14,4) == "dwm>")
                read_start = true;
            else
            {
                std::getline(file, s);
                continue;
            }

        }

        uwb_observe uwb_oi;

        double timestamp = atof(s.substr(0,13).c_str());

//        cout << timestamp << std::endl;

        bool is_get = false;
        std::string sub_get, sub_other;

        is_get = getSubstr(s, '[', ']', sub_get, sub_other);

        while(is_get)
        {
            is_get = getSubstr(sub_other, '[', ']', sub_get, sub_other);
        }

        std::vector<double> split_data = split(sub_get, ',');

//        cout << split_data[0] << "; " << split_data[1] << "; " << split_data[2] << "; " << split_data[3] << std::endl;

        if(split_data.size() == 4)
        {
            uwb_oi.timestamp = timestamp/double(1000);

            uwb_oi.point3d.x = split_data[0];
            uwb_oi.point3d.y = split_data[1];
            uwb_oi.point3d.z = split_data[2];

            uwb_oi.quality = split_data[3];
            all_observation.push_back(uwb_oi);
        }
        std::getline(file, s);
    }

    cout << "all_uwb.size() = " << all_observation.size() << std::endl
              << "data 0 = " << all_observation[0].timestamp << ", "
              << all_observation[0].point3d << ", "
              << all_observation[0].quality << std::endl
              << "data end = " << all_observation[all_observation.size() - 1].timestamp
              << ", " << all_observation[all_observation.size() -1].point3d
              << ", " << all_observation[all_observation.size() -1].quality
              << std::endl << std::endl;
    return;
}

void LoadIMUMeasure(std::string file_path, std::vector<imu_observe> &all_observation)
{
    std::ifstream file;
    file.open(file_path.c_str());
    if (!file.is_open())
    {
        cout << "open file failed !" << std::endl;
        return;
    }
    bool read_start = false;
    std::string s;
    std::getline(file, s);
    while(!file.eof())
    {
        if(s.empty())
        {
            std::getline(file, s);
            continue;
        }

        if(!read_start)
        {
            read_start = true;
            std::getline(file, s);
            continue;
        }

        imu_observe imu_oi;

        std::vector<double> split_data = split(s, ' ');

        if(split_data.size() != 10)
            cout << "error in load imu data" << std::endl;

        imu_oi.timestamp = split_data[0]/double(1000);
        imu_oi.angle = cv::Point3d(split_data[1], split_data[2], split_data[3]);
        imu_oi.acceleration = cv::Point3d(split_data[4], split_data[5], split_data[6]);
        imu_oi.angulay_velocity = cv::Point3d(split_data[7], split_data[8], split_data[9]);

        all_observation.push_back(imu_oi);

        std::getline(file, s);
    }
    cout << "all_imu.size() = " << all_observation.size() << std::endl
              << "data 0 = " << all_observation[0].timestamp
              << ", " << all_observation[0].angle
              << ", " << all_observation[0].acceleration
              << ", " << all_observation[0].angulay_velocity << std::endl
              << "data end = " << all_observation[all_observation.size() - 1].timestamp
              << ", " << all_observation[all_observation.size() -1].angle
              << ", " << all_observation[all_observation.size() -1].acceleration
              << ", " << all_observation[all_observation.size() -1].angulay_velocity
              << std::endl << std::endl;

    return;
}

void loadWheelIMUMeasure(const std::string file_path, std::vector<wheel_observe> &wheel_observation,
                         std::vector<imu_observe> &imu_observation)
{
    std::ifstream file;
    file.open(file_path.c_str());
    if(!file.is_open())
    {
        std::cout<<"open file failed,please check!!!"<<std::endl;
        return;
    }
    bool read_start = false;
    std::string s;
    std::getline(file,s);

    while(!file.eof())
    {
        if(s.empty())
        {
            std::getline(file,s);
            continue;
        }

        if(!read_start)
        {
            read_start = true;
            continue;
        }

        wheel_observe wheel_oi;
        imu_observe imu_oi;

        std::vector<double> split_data = split(s,',');

        if(split_data.size() < 16)
            std::cout<<"error in load imu wheel data"<<std::endl;

        wheel_oi.timestamp = split_data[0]/double(1000000);
        //here,wheel theta is poor,please do not use
        wheel_oi.wheel = cv::Point3d(split_data[1],split_data[2],split_data[3]);

        imu_oi.timestamp = split_data[0]/double(1000000);
        imu_oi.angle = cv::Point3d(split_data[6],split_data[7],split_data[8]);
        imu_oi.angulay_velocity = cv::Point3d(split_data[9],split_data[10],split_data[11]);
        imu_oi.acceleration = cv::Point3d (split_data[12],split_data[13],split_data[14]);

        wheel_observation.push_back(wheel_oi);
        imu_observation.push_back(imu_oi);
        std::getline(file,s);
    }
    cout << "all_wheel.size() = " << wheel_observation.size() << std::endl
         << "data 0 = " << wheel_observation[0].timestamp
         << ", " << wheel_observation[0].wheel << std::endl
         << "data end = " << wheel_observation[wheel_observation.size() - 1].timestamp
         << ", " << wheel_observation[wheel_observation.size() -1].wheel
         << std::endl << std::endl;

    cout << "all_imu.size() = " << imu_observation.size() << std::endl
         << "data 0 = " << imu_observation[0].timestamp
         << ", " << imu_observation[0].angle
         << ", " << imu_observation[0].angulay_velocity
         << ", " << imu_observation[0].acceleration << std::endl
         << "data end = " << imu_observation[imu_observation.size() - 1].timestamp
         << ", " << imu_observation[imu_observation.size() -1].angle
         << ", " << imu_observation[imu_observation.size() -1].angulay_velocity
         << ", " << imu_observation[imu_observation.size() -1].acceleration
         << std::endl << std::endl;
    return;
}


