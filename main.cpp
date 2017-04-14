#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "view.h"
#include "robot.h"

#define SCALE 0.017453

using namespace std;
using namespace cv;


int main()
{
    View view1(1000, Point3f(0,0,0), Point3f(0, 0, -400));
    Mat screen(480, 640, CV_8UC3, Scalar(255,255,255));

    namedWindow("img");
    int alfa = 180, beta = 180, gamma = 180;
    createTrackbar("alfa", "img", &alfa, 360);
    createTrackbar("beta", "img", &beta, 360);
    createTrackbar("gamma", "img", &gamma, 360);
    //namedWindow("SterowanieRobotem");
    char key = 'm';

    Robot rob(Point3f(0, 13 ,100), Point3f(0,0,0), 11.8, 36.5, Point3f(3.7, 5.8, 16.3));

    float transStep = 1;
    float rotStep = 0.05;

    while(key != 'q')
    {
        switch(key)
        {
            case '1':
                rob.move(Point3f(transStep,0,0));
                break;
            case '2':
                rob.move(Point3f(-transStep,0,0));
                break;
            case '3':
                rob.move(Point3f(0,0,transStep));
                break;
            case '4':
                rob.move(Point3f(0,0,-transStep));
                break;
            case '5':
                rob.move(Point3f(0,transStep,0));
                break;
            case '6':
                rob.move(Point3f(0,-transStep,0));
                break;

            case '7':
                rob.rotate(Point3f(0,rotStep,0));
                break;
            case '8':
                rob.rotate(Point3f(0,-rotStep,0));
                break;
            case '9':
                rob.rotate(Point3f(rotStep,0,0));
                break;
            case '0':
                rob.rotate(Point3f(-rotStep,0,0));
                break;
            case '[':
                rob.rotate(Point3f(0,0,rotStep));
                break;
            case ']':
                rob.rotate(Point3f(0,0,-rotStep));
                break;
        }

        screen = Mat(480, 640, CV_8UC3, Scalar(255,255,255));
        view1.change(key);
        view1.setAngles(Point3f((alfa-180)*SCALE, (beta-180)*SCALE, (gamma-180)*SCALE));
        view1.drawFloor(screen);
        view1.drawRobot(screen, rob);
        imshow("img", screen);
        key = waitKey(10);
    }
    return 0;
}
