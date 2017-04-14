#ifndef VIEW_H
#define VIEW_H

///Obsluga wyswietlania obrazu
///Rzutowanie perspektywiczne na plaszczyzne ekranu
///Tu jest to opisane i sa wykorzystywane rownania https://en.wikipedia.org/wiki/3D_projection

#include "util.h"
#include "robot.h"
#include <opencv2/core/core.hpp>

class View
{
    private:
        double alfa, beta, gamma;//katy kamery (alfa - os x, beta - os y, gamma - os z)
        double f; //
        cv::Point3f T; //T - pozycja
        float translationStep, rotationStep; //o ile sie przesuwa kamera; o ile sie obraca
    public:
        View(double f1, cv::Point3f angles, cv::Point3f translation);
        void rotateView(cv::Point3f angles);
        void setAngles(cv::Point3f angles);
        void translateView(cv::Point3f translation);
        void drawPoint(cv::Mat& screen, cv::Point3f point, cv::Scalar color, int size1 = 2);
        void drawLine(cv::Mat& screen, cv::Point3f p1, cv::Point3f p2, cv::Scalar color, int thickness=1);
        void drawRectangle(cv::Mat& screen, rect rec, cv::Scalar color, int thickness=1);
        void drawRobot(cv::Mat& screen, Robot& rob);
        void drawFloor(cv::Mat& screen);
        void change(char key);
};

#endif // VIEW_H
