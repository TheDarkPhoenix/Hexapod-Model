#include "leg.h"
#include <iostream>
#include <cmath>
#include <string>
#include <strstream>

using namespace std;
using namespace cv;

Leg::Leg(Point3f joint1, Point3f angles1, Point3f lengths1, cv::Point3f signals1)
{
    legJoints.A = joint1;
    angles = angles1;
    lengths = lengths1;
    signals = signals1;
}

void Leg::initJointPoints()
{
    calculateJointPoints(Point3f(0,0,0));
    legEnd = legJoints.D;
    initAngles = angles;
}

void Leg::calculateJointPoints(Point3f angl)
{
    Mat P1 = (Mat_<float>(3,1) << lengths.x, 0, 0);
    Mat R1 = (Mat_<float>(3,3) << cos(angles.x), 0, sin(angles.x), 0, 1, 0, -sin(angles.x), 0, cos(angles.x));
    Mat P11 = R1*P1;
    P11 = R*P11;
    legJoints.B = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2)) + legJoints.A;

    Mat P2 = (Mat_<float>(3,1) << lengths.y, 0, 0);
    Mat R2 = (Mat_<float>(3,3) << cos(angles.y+angl.z), -sin(angles.y+angl.z), 0, sin(angles.y+angl.z), cos(angles.y+angl.z), 0, 0, 0, 1);
    Mat P22 = R1*(R2*P2);
    P22 = R*P22;
    legJoints.C = Point3f(P22.at<float>(0,0), P22.at<float>(0,1), P22.at<float>(0,2)) + legJoints.B;

    Mat P3 = (Mat_<float>(3,1) << lengths.z, 0, 0);
    Mat R3 = (Mat_<float>(3,3) << cos(angles.z+angl.z), -sin(angles.z+angl.z), 0, sin(angles.z+angl.z), cos(angles.z+angl.z), 0, 0, 0, 1);
    Mat P33 = R1*(R3*P3);
    P33 = R*P33;
    legJoints.D = Point3f(P33.at<float>(0,0), P33.at<float>(0,1), P33.at<float>(0,2)) + legJoints.C;
}

int Leg::calculateAngles(Point3f angl)
{
    Point3f newPos = legEnd-legJoints.A; // pozycja poczatku nogi po przekszta³ceniu
    //cout << angl.z << endl;
    float lx = lengths.x*cos(angl.z);
    //newPos.y += lengths.x*sin(angl.z);
    float L = sqrt(pow(newPos.x,2)+pow(newPos.z,2));
    float iksw = sqrt(pow((L-lx),2)+pow(newPos.y,2));

    float a1 = atan((L-lx)/newPos.y);
    float a2 = acos((pow(lengths.z,2)-pow(lengths.y,2)-pow(iksw,2))/((-2.)*iksw*lengths.y));
    float b = acos((pow(iksw,2)-pow(lengths.y,2)-pow(lengths.z,2))/((-2.)*lengths.y*lengths.z));

    if(newPos.x&&newPos.z)
        angles.x = -atan(newPos.z/newPos.x);
    else
        angles.x = 0;
    angles.y = -(a1+a2-0.5*CV_PI);
    angles.z = (CV_PI - b + angles.y);

    if(angles.x < -1.5 || angles.x > 1.5 || angles.y < -1.5 || angles.y > 1.5 || angles.z < -1.5 || angles.z > 1.5 || angles.x != angles.x || angles.y != angles.y || angles.z!=angles.z)//nan detect
    {
        return -1;
        /*angles = initAngles;
        calculateJointPoints(angl);
        legEnd = legJoints.D;*/
    }

    calculateJointPoints(angl);
    //cout << angles << endl;
    calculateServoSignals();
    return 1;
}

void Leg::calculateServoSignals()
{
    float wspolczynnik = 100/(CV_PI/2 - 1.18);//100/(CV_PI/2 - 1.18);//wspolczynnik zamiany katow na sygnaly
    int sygnalA, sygnalB, sygnalC;
    sygnalA = angles.x*wspolczynnik + signals.x;
    sygnalB = angles.y*wspolczynnik + signals.y;
    sygnalC = (CV_PI/2 - angles.z)*wspolczynnik + signals.z;
    cout << sygnalA << ' ' << sygnalB << ' ' << sygnalC << endl;

    stringstream ss;
    ss << "/home/pi/maestrolinux/maestro-linux/UscCmd --servo "<<(servos.x)<<","<<(sygnalA);
    string str = ss.str();
    system(str.c_str());
    ss.clear();

    ss<< "/home/pi/maestrolinux/maestro-linux/UscCmd --servo "<<(servos.y)<<","<<(sygnalB);
    str = ss.str();
    system(str.c_str());
    ss.clear();

    ss<<"/home/pi/maestrolinux/maestro-linux/UscCmd --servo "<<(servos.z)<<","<<(sygnalC);
    str = ss.str();
    system(str.c_str());
}

void Leg::setServos(cv::Point3i servos1)
{
    servos = servos1;
}

void Leg::setJointA(cv::Point3f joint1)
{
    legJoints.A = joint1;
}

void Leg::setAgnles(cv::Point3f angles1)
{
    angles = angles1;
}

void Leg::setR(cv::Mat R1)
{
    R = R1;
}

void Leg::setSignals(cv::Point3f sig)
{
    signals = sig;
}

void Leg::setLengths(cv::Point3f lengths1)
{
    lengths = lengths1;
}
