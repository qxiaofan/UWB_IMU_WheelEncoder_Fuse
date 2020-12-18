#include "common_include.h"
#include "run_paticle_filter.h"


void drawCross(cv::Mat &img, cv::Point center, cv::Scalar color, int d) {
    cv::line(img, cv::Point(center.x - d, center.y - d),
             cv::Point(center.x + d, center.y + d), color, 2, 16, 0);
    cv::line(img, cv::Point(center.x + d, center.y - d),
             cv::Point(center.x - d, center.y + d), color, 2, 16, 0);
}


/*
 * paticle_filter
 * */
void RunPaticleFilter(std::vector<uwb_observe> &uwb_observe)
{

    std::vector<cv::Point2f> vcurrent_pos,vparticle_pos;

    int DP = 2;
    int nParticles = 200;
    float xRange = 800.0f;
    float flocking = 0.9f;
    float minRange[] = {0.0f,0.0f};
    float maxRange[] = {xRange,xRange};
    cv::Mat_<float> LB(1,DP,minRange);
    cv::Mat_<float> UB(1,DP,maxRange);
    cv::Mat_<float> measurement(1,DP);
    cv::Mat_<float> dyna(cv::Mat_<float>::eye(2,2));

    ConDensation condens(DP,nParticles);

    cv::Mat img((int)xRange,(int)xRange,CV_8UC3);
    cv::namedWindow("UWB particle filter");

    condens.initSampleSet(LB,UB,dyna);

    for(size_t i = 0; i < uwb_observe.size(); ++i)
    {
        cv::Point2f cur_uwb;
        cv::waitKey(30);
        cur_uwb.x = uwb_observe[i].point3d.x*100;
        cur_uwb.y = uwb_observe[i].point3d.y*100;

        measurement(0) = float(cur_uwb.x);
        measurement(1) = float(cur_uwb.y);

        cv::Point2f measPt(cur_uwb.x,cur_uwb.y);
        vcurrent_pos.push_back(measPt);

        //Clear screen
        img = cv::Scalar::all(60);

        //Update and get prediction:
        const cv::Mat_<float> &pred = condens.correct(measurement);

        cv::Point2f statePt(pred(0),pred(1));
        vparticle_pos.push_back(statePt);

        for(int s = 0; s < condens.sampleCount();s++)
        {
            cv::Point2f partPt(condens.sample(s,0), condens.sample(s,1));
            drawCross(img,partPt,cv::Scalar(255,90,(int)(s*255.0)/(float)condens.sampleCount()),2);
        }

        for(size_t i = 0; i < vcurrent_pos.size() - 1; i++)
        {
            cv::line(img,vcurrent_pos[i],vcurrent_pos[i + 1],cv::Scalar(255,255,0),1);
        }

        for(size_t i = 0; i < vparticle_pos.size() - 1; i++)
        {
            cv::line(img,vparticle_pos[i],vparticle_pos[i + 1],cv::Scalar(0,255,0),1);
        }

        drawCross(img,statePt,cv::Scalar(255,255,255),5);
        drawCross(img,measPt,cv::Scalar(0,0,255),5);
        cv::Mat imgshow = cv::Mat::zeros(640,640,CV_8UC3);
        cv::resize(img,imgshow,cv::Size(500,500));
        cv::imshow("UWB particle filter",imgshow);
    }
    return;
}

