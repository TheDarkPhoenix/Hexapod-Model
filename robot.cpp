#include "robot.h"
#include <iostream>
#include <strstream>
#include <unistd.h>

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
    walkingStep = 0;
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

    //Point3f ang1(0. ,-0.3 ,1.2);
    //Point3f ang1(0. ,-0.1 ,1.5);
    Point3f ang1(0.0 ,0.0 ,CV_PI/2);
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
        legs[i].setDevice(&device);
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
    if(!(n>=0 && n<=5))
        return joint();



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

void Robot::walk(Point3f steps)
{
    if(walkingStep == 0)
    {
        steps.x /= 2;
        steps.y = -2;
        steps.z /= 2;
        legs[0].setLegEnd(legs[0].getJoints().D+steps);
        legs[0].calculateAngles(-angles);

        legs[4].setLegEnd(legs[4].getJoints().D+steps);
        legs[4].calculateAngles(-angles);

        legs[2].setLegEnd(legs[2].getJoints().D+steps);
        legs[2].calculateAngles(-angles);

        ++walkingStep;
    }
    else if(walkingStep == 1)
    {
        steps.x /= 2;
        steps.y = 2;
        steps.z /= 2;

        legs[0].setLegEnd(legs[0].getJoints().D+steps);
        legs[0].calculateAngles(-angles);

        legs[4].setLegEnd(legs[4].getJoints().D+steps);
        legs[4].calculateAngles(-angles);

        legs[2].setLegEnd(legs[2].getJoints().D+steps);
        legs[2].calculateAngles(-angles);

        ++walkingStep;
    }
    else if(walkingStep == 2)
    {
        move(steps);
        steps.x/=2;
        steps.y = -2;
        steps.z /=2;

        legs[3].setLegEnd(legs[3].getJoints().D+steps);
        legs[3].calculateAngles(-angles);

        legs[1].setLegEnd(legs[1].getJoints().D+steps);
        legs[1].calculateAngles(-angles);

        legs[5].setLegEnd(legs[5].getJoints().D+steps);
        legs[5].calculateAngles(-angles);

        ++walkingStep;
    }
    else if(walkingStep == 3)
    {
        steps.x /= 2;
        steps.y = 2;
        steps.z /= 2;

        legs[3].setLegEnd(legs[3].getJoints().D+steps);
        legs[3].calculateAngles(-angles);

        legs[1].setLegEnd(legs[1].getJoints().D+steps);
        legs[1].calculateAngles(-angles);

        legs[5].setLegEnd(legs[5].getJoints().D+steps);
        legs[5].calculateAngles(-angles);

        walkingStep = 0;
    }
}

void Robot::walkRot(float angle)
{
    //Mat Rx1 = (Mat_<float>(3,3) << 1, 0, 0, 0, cos(angle), -sin(angle), 0, sin(angle), cos(angle));
    Mat Rx1 = (Mat_<float>(3,3) << cos(angle), 0, sin(angle), 0, 1, 0, -sin(angle), 0, cos(angle));
    //Mat Rx1 = (Mat_<float>(3,3) << cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1);

    Point3f g11 = legs[0].getJoints().D-position;
    Mat P1 = (Mat_<float>(3,1) << g11.x, g11.y, g11.z);
    Mat P11 = Rx1*P1;
    Point3f g12 = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2));
    Point3f steps1 = g12 - g11;

    Point3f g21 = legs[1].getJoints().D-position;
    Mat P2 = (Mat_<float>(3,1) << g21.x, g21.y, g21.z);
    Mat P21 = Rx1*P2;
    Point3f g22 = Point3f(P21.at<float>(0,0), P21.at<float>(0,1), P21.at<float>(0,2));
    Point3f steps2 = g22 - g21;

    //cout << steps1 << endl << steps2 << endl;


    for(int i = 0; i < 6; ++i)
    {
        g11 = (legs[i].getJoints().D-position);
        P1 = (Mat_<float>(3,1) << g11.x, g11.y, g11.z);
        P11 = Rx1*P1;
        g12 = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2));
        cout << g11 << ' ' << g12 << endl;
        legs[i].setLegEnd(g12+position);
        legs[i].calculateAngles(-angles);
    }
    rotate(Point3f(0,angle,0));
    /*Mat Rx1 = (Mat_<float>(2,2) << cos(angle), -sin(angle), sin(angle), cos(angle));
    Point2f g11 = Point2f(legs[0].getJoints().D.x-position.x, legs[0].getJoints().D.y-position.y);
    Mat P1 = (Mat_<float>(2,1) << g11.x, g11.y);
    Mat P11 = Rx1*P1;
    Point2f g22 = Point2f(P11.at<float>(0,0), P11.at<float>(0,1));
    Point3f steps1 = Point3f(g22.x - g11.x, g22.y;

    g11 = Point2f(legs[1].getJoints().D.x-position.x, legs[1].getJoints().D.y-position.y);
    P1 = (Mat_<float>(2,1) << g11.x, g11.y);
    P11 = Rx1*P1;
    g22 = Point3f(P11.at<float>(0,0), P11.at<float>(0,1));
    Point3f steps2 = g22 - g11;

    cout << steps1 << endl << steps2 << endl;*/

    /*if(walkingStep == 0)
    {
        steps1.x /= 2;
        steps1.y = -2;
        steps1.z /= 2;

        steps2.x /= 2;
        steps2.y = -2;
        steps2.z /= 2;

        legs[0].setLegEnd(legs[0].getJoints().D+steps1);
        legs[0].calculateAngles(-angles);

        legs[4].setLegEnd(legs[4].getJoints().D+steps2);
        legs[4].calculateAngles(-angles);

        legs[2].setLegEnd(legs[2].getJoints().D+steps1);
        legs[2].calculateAngles(-angles);

        ++walkingStep;
    }
    else if(walkingStep == 1)
    {
        steps1.x /= 2;
        steps1.y = 2;
        steps1.z /= 2;

        steps2.x /= 2;
        steps2.y = 2;
        steps2.z /= 2;

        legs[0].setLegEnd(legs[0].getJoints().D+steps1);
        legs[0].calculateAngles(-angles);

        legs[4].setLegEnd(legs[4].getJoints().D+steps2);
        legs[4].calculateAngles(-angles);

        legs[2].setLegEnd(legs[2].getJoints().D+steps1);
        legs[2].calculateAngles(-angles);

        ++walkingStep;
    }
    else if(walkingStep == 2)
    {
        rotate(Point3f(0,angle,0));
        steps1.x /= 2;
        steps1.y = -2;
        steps1.z /= 2;

        steps2.x /= 2;
        steps2.y = -2;
        steps2.z /= 2;

        legs[3].setLegEnd(legs[3].getJoints().D+steps1);
        legs[3].calculateAngles(-angles);

        legs[1].setLegEnd(legs[1].getJoints().D+steps2);
        legs[1].calculateAngles(-angles);

        legs[5].setLegEnd(legs[5].getJoints().D+steps1);
        legs[5].calculateAngles(-angles);

        ++walkingStep;
    }
    else if(walkingStep == 3)
    {
        steps1.x /= 2;
        steps1.y = 2;
        steps1.z /= 2;

        steps2.x /= 2;
        steps2.y = 2;
        steps2.z /= 2;

        legs[3].setLegEnd(legs[3].getJoints().D+steps1);
        legs[3].calculateAngles(-angles);

        legs[1].setLegEnd(legs[1].getJoints().D+steps2);
        legs[1].calculateAngles(-angles);

        legs[5].setLegEnd(legs[5].getJoints().D+steps1);
        legs[5].calculateAngles(-angles);

        walkingStep = 0;
    }*/
}

void Robot::walkC(Point3f steps)
{
    Point3f steps1 = steps;
    steps.x /= 2;
    steps.z /= 2;

    ///1
    steps.y = -2;

    legs[0].setLegEnd(legs[0].getJoints().D+steps);
    legs[0].calculateAngles(-angles);

    legs[4].setLegEnd(legs[4].getJoints().D+steps);
    legs[4].calculateAngles(-angles);

    legs[2].setLegEnd(legs[2].getJoints().D+steps);
    legs[2].calculateAngles(-angles);

    usleep(500000);

    ///2
    steps.y = 2;

    legs[0].setLegEnd(legs[0].getJoints().D+steps);
    legs[0].calculateAngles(-angles);

    legs[4].setLegEnd(legs[4].getJoints().D+steps);
    legs[4].calculateAngles(-angles);

    legs[2].setLegEnd(legs[2].getJoints().D+steps);
    legs[2].calculateAngles(-angles);

    usleep(500000);

    ///3
    move(steps1);

    steps.y = -2;

    legs[3].setLegEnd(legs[3].getJoints().D+steps);
    legs[3].calculateAngles(-angles);

    legs[1].setLegEnd(legs[1].getJoints().D+steps);
    legs[1].calculateAngles(-angles);

    legs[5].setLegEnd(legs[5].getJoints().D+steps);
    legs[5].calculateAngles(-angles);

    usleep(500000);

    ///4
    steps.y = 2;

    legs[3].setLegEnd(legs[3].getJoints().D+steps);
    legs[3].calculateAngles(-angles);

    legs[1].setLegEnd(legs[1].getJoints().D+steps);
    legs[1].calculateAngles(-angles);

    legs[5].setLegEnd(legs[5].getJoints().D+steps);
    legs[5].calculateAngles(-angles);

    usleep(500000);
}
