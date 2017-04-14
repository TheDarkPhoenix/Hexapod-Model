#ifndef ROBOT_H
#define ROBOT_H

#include <opencv2/core/core.hpp>
#include "util.h"
#include "leg.h"

class Robot
{
    private:
        rect frame;
        cv::Point3f position;
        cv::Point3f angles;
        float width, length;
        cv::Mat Rx, Ry, Rz, R;
        Leg legs[6];
        void update();
    public:
        Robot(cv::Point3f pos, cv::Point3f ang, float width1, float length1, cv::Point3f leglengths);
        cv::Point3f getPosition(){return position;};
        cv::Point3f getAngles(){return angles;};
        rect getFrame(){return frame;};
        joints getLegJoints(int n);
        void move(cv::Point3f p);
        void rotate(cv::Point3f ang);
};

#endif // ROBOT_H
