#ifndef ROBOT_H
#define ROBOT_H

#include <opencv2/core/core.hpp>
#include "util.h"
#include "leg.h"
#include "maestro.h"

class Robot
{
    private:
        rect lFrame, gFrame;// l - lokalne g - globalne
        cv::Point3f position;
        cv::Point3f initPosition;
        cv::Point3f initAngles;
        cv::Point3f angles;
        float width, length;
        cv::Mat Rx, Ry, Rz, R;
        Leg legs[6];
        void moveCoordinates(cv::Point3f p, cv::Point3f ang);
        int walkingStep;
        Maestro device;

    public:
        Robot(cv::Point3f pos, cv::Point3f ang, float width1, float length1, cv::Point3f leglengths);
        cv::Point3f getPosition(){return position;};
        cv::Point3f getAngles(){return angles;};
        rect getFrame();
        joints getLegJoints(int n);
        void move(cv::Point3f p);
        void rotate(cv::Point3f ang);
        void walk(cv::Point3f steps);
        void walkC(cv::Point3f steps);
        void walkRot(float angle);
};

#endif // ROBOT_H
