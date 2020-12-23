#include "SerialPort.h"
#include "utils.h"
#include "readData.h"
#include "opencv2/opencv.hpp"

#include "CoreAlgorithm.h"
#include "calibrate.h"

//利用UWB采集定位数据，保存到data.txt文件中
int main01() {

    //自动识别串口
    list<string> l = getComList();

    list<string>::iterator it = l.begin();

    std::string serial_port= "";
    std::string b = "ACM";
    string::size_type idx;

    int success_num = 0;

    while (it != l.end()) {
        cout << *it << endl;

        idx = it->find(b);
        if(idx != string::npos)
        {
            cout<<"found\n";
            success_num++;
            serial_port = *it;
        }
        it++;
    }

    if(success_num == 1)
    {
        std::cout<<"find serial port, OK!!!"<<std::endl;
        std::cout<<"serial_port == "<<serial_port<<std::endl;
    }

    if(success_num > 1)
    {
        std::cout<<"find more than one serial port,please choose check!!!"<<std::endl;
    }

    if(success_num < 1)
    {
        std::cout<<"not found right serial port,please check!!!"<<std::endl;
    }

    
    SerialPort UART;
    UART.Config.BaudRate = SerialPort::BR115200;
    UART.Config.DataBits = SerialPort::DataBits8;
    UART.Config.StopBits = SerialPort::StopBits1;
    UART.Config.Parity = SerialPort::ParityNone;
    //UART.Config.DevicePath = (char *) &"/dev/ttyACM0";
    UART.Config.DevicePath = const_cast<char*>(serial_port.c_str());

    if (UART.Open() == false)printf("OPEN error!\n");
    else printf("OPEN OK!\n");
    if (UART.LoadConfig() == false)printf("Set error!\n");
    else printf("Set OK!\n");

    //string RXtxt = "apg";

    //UWB需要两次输入enter，这里的13对应的是enter对应的char
    UART << 13;
    usleep(100);
    UART << 13;
    
    int cmdID = 0;
    while (1) {
        string scmd = "";
        cin >> scmd;
        if (scmd == "exit") {
            break;
        }
        std::cout<<"input == "<<scmd.c_str()<<std::endl;
        UART << scmd.c_str();

        UART << '\n';
        cmdID++;
        cout << "cmdID = " << cmdID << endl;
        sleep(1);
    }

    cout << "finish run!" << endl;
    return 0;
}

int main(int argc,char **argv)
{
    //ofstream cmdoutfile("yong_output.txt");
    //std::cout.rdbuf(cmdoutfile.rdbuf());

    string wheelEncoderFile = string(argv[1]) + "/wheel_encoder.txt";
    std::vector<wheel_observe> all_wheel_data;
    LoadWheelMeasure(wheelEncoderFile,all_wheel_data);

    std::vector<vslam_observe> all_vslam_data;
    LoadVslamMeasure(wheelEncoderFile,all_vslam_data);

    std::string file_uwb_data = string(argv[1]) + "/data_UWB.log";
    std::vector<uwb_observe> uwb_observe_origin;
    LoadUWBMeasure(file_uwb_data,uwb_observe_origin);

    std::string file_imu_data =string(argv[1]) + "/data_IMU.txt";
    std::vector<imu_observe> all_imu_data;
    LoadIMUMeasure(file_imu_data,all_imu_data);

    //imshow the vslam pose
    showVslamTrajectory(all_vslam_data,"vslam.png",true);

    //filter the uwb data by ukf
    std::vector<uwb_observe> uwb_observe_filtered;
    //runUKarmanFilter(uwb_observe_origin,uwb_observe_filtered);

    runParticleFilter(uwb_observe_origin,uwb_observe_filtered);

    //yong.qi added for test
    //showOriginTime(all_imu_data,all_wheel_data,"origin_time.png",true);
    //yong.qi ended

    double time_imu_vslam = -1.0;
    double time_imu_uwb = 0.0;
    RunTimeCalibration(all_imu_data, all_wheel_data, time_imu_vslam);

    Eigen::Matrix4d pose_imu_vslam = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pose_uwb_vslam = Eigen::Matrix4d::Identity();
    double time_uwb_vslam = time_imu_vslam;
    RunPoseCalibration(uwb_observe_filtered, all_vslam_data, time_uwb_vslam, pose_uwb_vslam);

    runAccumulateWheel(all_wheel_data);
    runAccumulateImu(all_imu_data);

    return 0;
}