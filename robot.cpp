#include "robot.h"
#include "view.h"
#include <opencv2/highgui/highgui.hpp>
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

    lFrame.dl = Point3f(-width/2,0,-length/2);
    lFrame.dr = Point3f(width/2,0,-length/2);
    lFrame.ul = Point3f(-width/2,0,length/2);
    lFrame.ur = Point3f(width/2,0,length/2);

    for(int i = 3; i < 6; ++i)
        legs[i].setR((Mat_<float>(3,3) << -1, 0, 0, 0, 1, 0, 0, 0, -1));
    for(int i = 0; i < 3; ++i)
        legs[i].setR((Mat_<float>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1));

    Point3f ang1(0.0 ,0.0 ,CV_PI/2);
    legs[0].setJointA(lFrame.ur);
    legs[0].setAgnles(ang1);
    legs[0].setLengths(leglengths);
    legs[0].setSignals(Point3f(5900,5100,5200));
    legs[0].setServos(Point3i(5,4,3));

    legs[1].setJointA((lFrame.ur+lFrame.dr)/2);
    legs[1].setAgnles(ang1);
    legs[1].setLengths(leglengths);
    legs[1].setSignals(Point3f(6000,5200,4900));
    legs[1].setServos(Point3i(9,10,11));

    legs[2].setJointA(lFrame.dr);
    legs[2].setAgnles(ang1);
    legs[2].setLengths(leglengths);
    legs[2].setSignals(Point3f(6200,5300,5300));
    legs[2].setServos(Point3i(15,16,17));

    legs[3].setJointA(lFrame.ul);
    legs[3].setAgnles(ang1);
    legs[3].setLengths(leglengths);
    legs[3].setSignals(Point3f(6000,5300,4900));
    legs[3].setServos(Point3i(0,1,2));

    legs[4].setJointA((lFrame.ul+lFrame.dl)/2);
    legs[4].setAgnles(ang1);
    legs[4].setLengths(leglengths);
    legs[4].setSignals(Point3f(6000,5000,5000));
    legs[4].setServos(Point3i(6,7,8));

    legs[5].setJointA(lFrame.dl);
    legs[5].setAgnles(ang1);
    legs[5].setLengths(leglengths);
    legs[5].setSignals(Point3f(6000,5300,5200));
    legs[5].setServos(Point3i(12,13,14));

    for(int i = 0; i < 6; ++i)
    {
        legs[i].initJointPoints();
        legs[i].setDevice(&device);
    }
}

joints Robot::getLegJoints(int n)
{
    if(!(n>=0 && n<=5))
        return joints();

    joints x = legs[n].getJoints();

    Rx = (Mat_<float>(3,3) << 1, 0, 0, 0, cos(angles.x), -sin(angles.x), 0, sin(angles.x), cos(angles.x));
    Ry = (Mat_<float>(3,3) << cos(angles.y), 0, sin(angles.y), 0, 1, 0, -sin(angles.y), 0, cos(angles.y));
    Rz = (Mat_<float>(3,3) << cos(angles.z), -sin(angles.z), 0, sin(angles.z), cos(angles.z), 0, 0, 0, 1);

    R = Rz*Ry*Rx;

    Mat P1 = (Mat_<float>(3,1) << x.A.x, x.A.y, x.A.z);
    Mat P2 = (Mat_<float>(3,1) << x.B.x, x.B.y, x.B.z);
    Mat P3 = (Mat_<float>(3,1) << x.C.x, x.C.y, x.C.z);
    Mat P4 = (Mat_<float>(3,1) << x.D.x, x.D.y, x.D.z);

    Mat P11 = R*P1;
    Mat P22 = R*P2;
    Mat P33 = R*P3;
    Mat P44 = R*P4;

    x.A = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2)) + position;
    x.B = Point3f(P22.at<float>(0,0), P22.at<float>(0,1), P22.at<float>(0,2)) + position;
    x.C = Point3f(P33.at<float>(0,0), P33.at<float>(0,1), P33.at<float>(0,2)) + position;
    x.D = Point3f(P44.at<float>(0,0), P44.at<float>(0,1), P44.at<float>(0,2)) + position;

    return x;
}

rect Robot::getFrame()
{
    Rx = (Mat_<float>(3,3) << 1, 0, 0, 0, cos(angles.x), -sin(angles.x), 0, sin(angles.x), cos(angles.x));
    Ry = (Mat_<float>(3,3) << cos(angles.y), 0, sin(angles.y), 0, 1, 0, -sin(angles.y), 0, cos(angles.y));
    Rz = (Mat_<float>(3,3) << cos(angles.z), -sin(angles.z), 0, sin(angles.z), cos(angles.z), 0, 0, 0, 1);

    R = Rz*Ry*Rx;

    Mat P1 = (Mat_<float>(3,1) << - width/2, 0, - length/2);
    Mat P2 = (Mat_<float>(3,1) << width/2, 0, - length/2);
    Mat P3 = (Mat_<float>(3,1) << - width/2, 0, length/2);
    Mat P4 = (Mat_<float>(3,1) << width/2, 0, length/2);

    Mat P11 = R*P1;
    Mat P22 = R*P2;
    Mat P33 = R*P3;
    Mat P44 = R*P4;

    gFrame.dl = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2)) + position;
    gFrame.dr = Point3f(P22.at<float>(0,0), P22.at<float>(0,1), P22.at<float>(0,2)) + position;
    gFrame.ul = Point3f(P33.at<float>(0,0), P33.at<float>(0,1), P33.at<float>(0,2)) + position;
    gFrame.ur = Point3f(P44.at<float>(0,0), P44.at<float>(0,1), P44.at<float>(0,2)) + position;

    return gFrame;
}

void Robot::moveCoordinates(Point3f p, Point3f ang)
{
    angles += ang;

    ang = -ang;


    Rx = (Mat_<float>(3,3) << 1, 0, 0, 0, cos(ang.x), -sin(ang.x), 0, sin(ang.x), cos(ang.x));
    Ry = (Mat_<float>(3,3) << cos(ang.y), 0, sin(ang.y), 0, 1, 0, -sin(ang.y), 0, cos(ang.y));
    Rz = (Mat_<float>(3,3) << cos(ang.z), -sin(ang.z), 0, sin(ang.z), cos(ang.z), 0, 0, 0, 1);

    R = Rz*Ry*Rx;

    joints x;

    Mat P1, P11;

    for(int i = 0; i < 6; ++i)
    {
        x = legs[i].getJoints();
        P1 = (Mat_<float>(3,1) << x.D.x, x.D.y, x.D.z);
        P11 = R*P1;
        x.D = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2)) - p;
        legs[i].setLegEnd(x.D);
        legs[i].calculateAngles();
    }
    Rx = (Mat_<float>(3,3) << 1, 0, 0, 0, cos(angles.x), -sin(angles.x), 0, sin(angles.x), cos(angles.x));
    Ry = (Mat_<float>(3,3) << cos(angles.y), 0, sin(angles.y), 0, 1, 0, -sin(angles.y), 0, cos(angles.y));
    Rz = (Mat_<float>(3,3) << cos(angles.z), -sin(angles.z), 0, sin(angles.z), cos(angles.z), 0, 0, 0, 1);

    R = Rz*Ry*Rx;
    P1 = (Mat_<float>(3,1) << p.x, p.y, p.z);
    P11 = R*P1;
    p = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2));
    position += p;
}

void Robot::move(Point3f p)
{
    moveCoordinates(p, Point3f(0,0,0));

    for(int i = 0; i < 6; ++i)
    {
        if(legs[i].calculateAngles() == -1)
            this->move(-p);
    }
}

void Robot::rotate(Point3f ang)
{
    moveCoordinates(Point3f(0,0,0), ang);

    for(int i = 0; i < 6; ++i)
    {
        if(legs[i].calculateAngles() == -1)
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
        legs[0].calculateAngles();

        legs[4].setLegEnd(legs[4].getJoints().D+steps);
        legs[4].calculateAngles();

        legs[2].setLegEnd(legs[2].getJoints().D+steps);
        legs[2].calculateAngles();

        ++walkingStep;
    }
    else if(walkingStep == 1)
    {
        steps.x /= 2;
        steps.y = 2;
        steps.z /= 2;

        legs[0].setLegEnd(legs[0].getJoints().D+steps);
        legs[0].calculateAngles();

        legs[4].setLegEnd(legs[4].getJoints().D+steps);
        legs[4].calculateAngles();

        legs[2].setLegEnd(legs[2].getJoints().D+steps);
        legs[2].calculateAngles();

        ++walkingStep;
    }
    else if(walkingStep == 2)
    {
        move(steps);
        ++walkingStep;
    }
    else if(walkingStep == 3)
    {
        steps.x/=2;
        steps.y = -2;
        steps.z /=2;

        legs[3].setLegEnd(legs[3].getJoints().D+steps);
        legs[3].calculateAngles();

        legs[1].setLegEnd(legs[1].getJoints().D+steps);
        legs[1].calculateAngles();

        legs[5].setLegEnd(legs[5].getJoints().D+steps);
        legs[5].calculateAngles();

        ++walkingStep;
    }
    else if(walkingStep == 4)
    {
        steps.x /= 2;
        steps.y = 2;
        steps.z /= 2;

        legs[3].setLegEnd(legs[3].getJoints().D+steps);
        legs[3].calculateAngles();

        legs[1].setLegEnd(legs[1].getJoints().D+steps);
        legs[1].calculateAngles();

        legs[5].setLegEnd(legs[5].getJoints().D+steps);
        legs[5].calculateAngles();

        walkingStep = 0;
    }
}

void Robot::walkRot(float angle)
{
    /*Mat Rx1 = (Mat_<float>(3,3) << cos(angle), 0, sin(angle), 0, 1, 0, -sin(angle), 0, cos(angle));
    for(int i = 0; i < 6; ++i)
    {
        Point3f g11 = (legs[i].getJoints().D);
        Mat P1 = (Mat_<float>(3,1) << g11.x, g11.y, g11.z);
        Mat P11 = Rx1*P1;
        Point3f g12 = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2));
        cout << g11 << ' ' << g12 << endl;
        legs[i].setLegEnd(g12);
        legs[i].calculateAngles();
    }
    rotate(Point3f(0,angle,0));*/
    Mat Rx1 = (Mat_<float>(3,3) << cos(angle), 0, sin(angle), 0, 1, 0, -sin(angle), 0, cos(angle));

    if(walkingStep == 0)
    {

        /*Point3f g11 = legs[0].getJoints().D;
        Mat P1 = (Mat_<float>(3,1) << g11.x, g11.y, g11.z);
        Mat P11 = Rx1*P1;
        Point3f g12 = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2));
        steps1 = g12 - g11;

        Point3f g21 = legs[1].getJoints().D;
        Mat P2 = (Mat_<float>(3,1) << g21.x, g21.y, g21.z);
        Mat P21 = Rx1*P2;
        Point3f g22 = Point3f(P21.at<float>(0,0), P21.at<float>(0,1), P21.at<float>(0,2));
        steps2 = g22 - g21;*/

        for(int i = 0; i < 6; ++i)
        {
            Point3f g11 = (legs[i].getJoints().D);
            Mat P1 = (Mat_<float>(3,1) << g11.x, g11.y, g11.z);
            Mat P11 = Rx1*P1;
            Point3f g12 = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2));
            stepsl[i] = g12 - g11;
            stepsl[i].x /= 2;
            stepsl[i].y = -2;
            stepsl[i].z /= 2;
        }

        legs[0].setLegEnd(legs[0].getJoints().D+stepsl[0]);
        legs[0].calculateAngles();

        legs[4].setLegEnd(legs[4].getJoints().D+stepsl[4]);
        legs[4].calculateAngles();

        legs[2].setLegEnd(legs[2].getJoints().D+stepsl[2]);
        legs[2].calculateAngles();

        ++walkingStep;
    }
    else if(walkingStep == 1)
    {
        stepsl[0].y = 2;
        stepsl[4].y = 2;
        stepsl[2].y = 2;

        legs[0].setLegEnd(legs[0].getJoints().D+stepsl[0]);
        legs[0].calculateAngles();

        legs[4].setLegEnd(legs[4].getJoints().D+stepsl[4]);
        legs[4].calculateAngles();

        legs[2].setLegEnd(legs[2].getJoints().D+stepsl[2]);
        legs[2].calculateAngles();

        ++walkingStep;
    }
    else if(walkingStep == 2)
    {
        rotate(Point3f(0,angle,0));
        for(int i = 0; i < 6; ++i)
        {
            Point3f g11 = (legs[i].getJoints().D);
            Mat P1 = (Mat_<float>(3,1) << g11.x, g11.y, g11.z);
            Mat P11 = Rx1*P1;
            Point3f g12 = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2));
            stepsl[i] = g12 - g11;
            stepsl[i].x /= 2;
            stepsl[i].y = -2;
            stepsl[i].z /= 2;
        }
        legs[3].setLegEnd(legs[3].getJoints().D+stepsl[3]);
        legs[3].calculateAngles();

        legs[1].setLegEnd(legs[1].getJoints().D+stepsl[1]);
        legs[1].calculateAngles();

        legs[5].setLegEnd(legs[5].getJoints().D+stepsl[5]);
        legs[5].calculateAngles();

        ++walkingStep;
    }
    else if(walkingStep == 3)
    {
        stepsl[3].y = 2;
        stepsl[1].y = 2;
        stepsl[5].y = 2;

        legs[3].setLegEnd(legs[3].getJoints().D+stepsl[3]);
        legs[3].calculateAngles();

        legs[1].setLegEnd(legs[1].getJoints().D+stepsl[1]);
        legs[1].calculateAngles();

        legs[5].setLegEnd(legs[5].getJoints().D+stepsl[5]);
        legs[5].calculateAngles();

        walkingStep = 0;

    }
}

void Robot::walkC(Point3f steps, View& view1)
{
    Point3f steps1 = steps;
    steps.x /= 2;
    steps.z /= 2;

    ///1
    steps.y = -2;

    legs[0].setLegEnd(legs[0].getJoints().D+steps);
    legs[0].calculateAngles();

    legs[4].setLegEnd(legs[4].getJoints().D+steps);
    legs[4].calculateAngles();

    legs[2].setLegEnd(legs[2].getJoints().D+steps);
    legs[2].calculateAngles();

    view1.update('b', *this);

    usleep(500000);

    ///2
    steps.y = 2;

    legs[0].setLegEnd(legs[0].getJoints().D+steps);
    legs[0].calculateAngles();

    legs[4].setLegEnd(legs[4].getJoints().D+steps);
    legs[4].calculateAngles();

    legs[2].setLegEnd(legs[2].getJoints().D+steps);
    legs[2].calculateAngles();

    view1.update('b', *this);

    usleep(500000);

    ///3
    move(steps1);

    steps.y = -2;

    legs[3].setLegEnd(legs[3].getJoints().D+steps);
    legs[3].calculateAngles();

    legs[1].setLegEnd(legs[1].getJoints().D+steps);
    legs[1].calculateAngles();

    legs[5].setLegEnd(legs[5].getJoints().D+steps);
    legs[5].calculateAngles();

    view1.update('b', *this);

    usleep(500000);

    ///4
    steps.y = 2;

    legs[3].setLegEnd(legs[3].getJoints().D+steps);
    legs[3].calculateAngles();

    legs[1].setLegEnd(legs[1].getJoints().D+steps);
    legs[1].calculateAngles();

    legs[5].setLegEnd(legs[5].getJoints().D+steps);
    legs[5].calculateAngles();

    usleep(500000);
}
