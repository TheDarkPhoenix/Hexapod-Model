#include "robot.h"
#include <iostream>
#include <strstream>

using namespace cv;
using namespace std;

Robot::Robot(cv::Point3f pos, cv::Point3f ang, float width1, float length1, cv::Point3f leglengths)
{
    position = pos;
    angles = ang;
    width = width1;
    length = length1;
    position.y = -position.y;
    initPosition = position;
    initAngles = angles;
    /*Rx = (Mat_<float>(3,3) << 1, 0, 0, 0, cos(angles.x), -sin(angles.x), 0, sin(angles.x), cos(angles.x));
    Ry = (Mat_<float>(3,3) << cos(angles.y), 0, sin(angles.y), 0, 1, 0, -sin(angles.y), 0, cos(angles.y));
    Rz = (Mat_<float>(3,3) << cos(angles.z), -sin(angles.z), 0, sin(angles.z), cos(angles.z), 0, 0, 0, 1);
    R = Rz*Ry*Rx;
    Mat P1 = (Mat_<float>(3,1) << position.x - width/2, -position.y, position.z - length/2);
    Mat P2 = (Mat_<float>(3,1) << position.x + width/2, -position.y, position.z - length/2);
    Mat P3 = (Mat_<float>(3,1) << position.x - width/2, -position.y, position.z + length/2);
    Mat P4 = (Mat_<float>(3,1) << position.x + width/2, -position.y, position.z + length/2);
    Mat P11 = R*P1;
    Mat P22 = R*P2;
    Mat P33 = R*P3;
    Mat P44 = R*P4;
    frame.dl = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2));
    frame.dr = Point3f(P22.at<float>(0,0), P22.at<float>(0,1), P22.at<float>(0,2));
    frame.ul = Point3f(P33.at<float>(0,0), P33.at<float>(0,1), P33.at<float>(0,2));
    frame.ur = Point3f(P44.at<float>(0,0), P44.at<float>(0,1), P44.at<float>(0,2));*/

    update();
    Mat Ryy = (Mat_<float>(3,3) << -1, 0, 0, 0, 1, 0, 0, 0, -1);


    for(int i = 3; i < 6; ++i)
        legs[i].setR(Rz*Ryy*Ry*Rx);
    for(int i = 0; i < 3; ++i)
        legs[i].setR(R);

    Point3f ang1(0.0 ,-0.3 ,1.2);
    //Point3f ang1(0.0 ,0.0 ,CV_PI/2);
    legs[0].setJointA(frame.ur);
    legs[0].setAgnles(ang1);
    legs[0].setLengths(leglengths);
    legs[0].setSignals(Point3f(5900,5100,5200));
    legs[0].setServos(Point3i(5,4,3));

    legs[1].setJointA((frame.ur+frame.dr)/2);
    legs[1].setAgnles(ang1);
    legs[1].setLengths(leglengths);
    legs[1].setSignals(Point3f(6000,5200,4900));
    legs[1].setServos(Point3i(9,10,11));

    legs[2].setJointA(frame.dr);
    legs[2].setAgnles(ang1);
    legs[2].setLengths(leglengths);
    legs[2].setSignals(Point3f(6200,5300,5300));
    legs[2].setServos(Point3i(15,16,17));

    legs[3].setJointA(frame.ul);
    legs[3].setAgnles(ang1);
    legs[3].setLengths(leglengths);
    legs[3].setSignals(Point3f(6000,5300,4900));
    legs[3].setServos(Point3i(0,1,2));

    legs[4].setJointA((frame.ul+frame.dl)/2);
    legs[4].setAgnles(ang1);
    legs[4].setLengths(leglengths);
    legs[4].setSignals(Point3f(6000,5000,5000));
    legs[4].setServos(Point3i(6,7,8));

    legs[5].setJointA(frame.dl);
    legs[5].setAgnles(ang1);
    legs[5].setLengths(leglengths);
    legs[5].setSignals(Point3f(6000,5300,5200));
    legs[5].setServos(Point3i(12,13,14));

    for(int i = 0; i < 6; ++i)
    {
        legs[i].initJointPoints();
    }
    stringstream ss;
    for(int i = 0; i < 18; ++i)
    {
        ss << "usccmd --speed "<<(i)<<","<<(100);
        string str = ss.str();
        //system(str.c_str());
        ss.clear();
    }
    /*ss << "usccmd --speed "<<(0)<<","<<(10);
    string str = ss.str();
    system(str.c_str());
    ss.clear();

    ss << "usccmd --speed "<<(1)<<","<<(10);
     str = ss.str();
    system(str.c_str());
    ss.clear();

    ss << "usccmd --speed "<<(2)<<","<<(10);
     str = ss.str();
    system(str.c_str());
    ss.clear();*/
}

joints Robot::getLegJoints(int n)
{
    if(n>=0 && n<=5)
        return legs[n].getJoints();
}

void Robot::update()
{
    Rx = (Mat_<float>(3,3) << 1, 0, 0, 0, cos(angles.x), -sin(angles.x), 0, sin(angles.x), cos(angles.x));
    Ry = (Mat_<float>(3,3) << cos(angles.y), 0, sin(angles.y), 0, 1, 0, -sin(angles.y), 0, cos(angles.y));
    Rz = (Mat_<float>(3,3) << cos(angles.z), -sin(angles.z), 0, sin(angles.z), cos(angles.z), 0, 0, 0, 1);

    R = Rz*Ry*Rx;

    /*Mat P1 = (Mat_<float>(3,1) << position.x - width/2, -position.y, position.z - length/2);
    Mat P2 = (Mat_<float>(3,1) << position.x + width/2, -position.y, position.z - length/2);
    Mat P3 = (Mat_<float>(3,1) << position.x - width/2, -position.y, position.z + length/2);
    Mat P4 = (Mat_<float>(3,1) << position.x + width/2, -position.y, position.z + length/2);*/

    Mat P1 = (Mat_<float>(3,1) << - width/2, 0, - length/2);
    Mat P2 = (Mat_<float>(3,1) << width/2, 0, - length/2);
    Mat P3 = (Mat_<float>(3,1) << - width/2, 0, length/2);
    Mat P4 = (Mat_<float>(3,1) << width/2, 0, length/2);

    Mat P11 = R*P1;
    Mat P22 = R*P2;
    Mat P33 = R*P3;
    Mat P44 = R*P4;

    frame.dl = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2)) + position;
    frame.dr = Point3f(P22.at<float>(0,0), P22.at<float>(0,1), P22.at<float>(0,2)) + position;
    frame.ul = Point3f(P33.at<float>(0,0), P33.at<float>(0,1), P33.at<float>(0,2)) + position;
    frame.ur = Point3f(P44.at<float>(0,0), P44.at<float>(0,1), P44.at<float>(0,2)) + position;

    Mat Ryy = (Mat_<float>(3,3) << -1, 0, 0, 0, 1, 0, 0, 0, -1);

    for(int i = 3; i < 6; ++i)
        legs[i].setR(Rz*Ryy*Ry*Rx);

    for(int i = 0; i < 3; ++i)
        legs[i].setR(R);

    legs[0].setJointA(frame.ur);
    legs[1].setJointA((frame.ur+frame.dr)/2);
    legs[2].setJointA(frame.dr);
    legs[3].setJointA(frame.ul);
    legs[4].setJointA((frame.ul+frame.dl)/2);
    legs[5].setJointA(frame.dl);
}

void Robot::move(Point3f p)
{

    position += p;

    update();

    if(legs[0].calculateAngles(-angles) == -1)
            this->move(-p);
    if(legs[4].calculateAngles(-angles) == -1)
            this->move(-p);
    if(legs[2].calculateAngles(-angles) == -1)
            this->move(-p);
    if(legs[5].calculateAngles(-angles) == -1)
            this->move(-p);
    if(legs[1].calculateAngles(-angles) == -1)
            this->move(-p);
    if(legs[3].calculateAngles(-angles) == -1)
            this->move(-p);


    /*for(int i = 0; i < 3; ++i)
    {
        if(legs[i].calculateAngles(-angles) == -1)
            this->move(-p);
    }
    for(int i = 3; i < 6; ++i)
    {
        if(legs[i].calculateAngles(angles) == -1)
            this->move(-p);
    }*/
}

void Robot::rotate(Point3f ang)
{
    angles += ang;

    update();

    for(int i = 0; i < 3; ++i)
    {
        if(legs[i].calculateAngles(-angles) == -1)
            this->rotate(-ang);
    }
    for(int i = 3; i < 6; ++i)
    {
        if(legs[i].calculateAngles(angles) == -1)
            this->rotate(-ang);
    }
}
