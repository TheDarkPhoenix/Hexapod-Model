#include "robot.h"
#include <iostream>

using namespace cv;
using namespace std;

Robot::Robot(cv::Point3f pos, cv::Point3f ang, float width1, float length1, cv::Point3f leglengths)
{
    position = pos;
    angles = ang;
    width = width1;
    length = length1;
    position.y = -position.y;
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
    legs[0].setJointA(frame.ur);
    legs[0].setAgnles(ang1);
    legs[0].setLengths(leglengths);

    legs[1].setJointA((frame.ur+frame.dr)/2);
    legs[1].setAgnles(ang1);
    legs[1].setLengths(leglengths);

    legs[2].setJointA(frame.dr);
    legs[2].setAgnles(ang1);
    legs[2].setLengths(leglengths);

    legs[3].setJointA(frame.ul);
    legs[3].setAgnles(ang1);
    legs[3].setLengths(leglengths);

    legs[4].setJointA((frame.ul+frame.dl)/2);
    legs[4].setAgnles(ang1);
    legs[4].setLengths(leglengths);

    legs[5].setJointA(frame.dl);
    legs[5].setAgnles(ang1);
    legs[5].setLengths(leglengths);

    for(int i = 0; i < 6; ++i)
    {
        legs[i].initJointPoints();
    }

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

    for(int i = 0; i < 3; ++i)
        legs[i].calculateAngles(-angles);
    for(int i = 3; i < 6; ++i)
        legs[i].calculateAngles(angles);
}

void Robot::rotate(Point3f ang)
{
    angles += ang;

    update();

    for(int i = 0; i < 3; ++i)
        legs[i].calculateAngles(-angles);
    for(int i = 3; i < 6; ++i)
        legs[i].calculateAngles(angles);
}
