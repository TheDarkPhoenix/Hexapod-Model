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
    View view1(1000, Point3f(0,0,0), Point3f(0, -300, 0));

    char key = 'm';

    Robot rob(Point3f(0, 17 ,100), Point3f(0,1,0), 11.8, 36.5, Point3f(3.7, 5.8, 16.3));

    float transStep = 1;
    float rotStep = 0.05;

    bool walking = true;

    float walkStep = 3;

    while(key != 27)
    {
        switch(key)
        {
            case 'D':
                if(walking)
                    rob.walk(Point3f(walkStep,0,0));
                else
                    rob.move(Point3f(transStep,0,0));
                break;
            case 'A':
                if(walking)
                    rob.walk(Point3f(-walkStep,0,0));
                else
                    rob.move(Point3f(-transStep,0,0));
                break;
            case 'Q':
                if(walking)
                    rob.walkC(Point3f(0,0,walkStep), view1);
                else
                    rob.move(Point3f(0,0,transStep));
                break;
            case 'E':
                if(walking)
                    rob.walkC(Point3f(0,0,-walkStep), view1);
                else
                    rob.move(Point3f(0,0,-transStep));
                break;
            case 'S':
                //rob.move(Point3f(0,transStep,0));
                rob.walkRot(0.05);
                break;
            case 'W':
                rob.walkRot(-0.05);
                //rob.move(Point3f(0,-transStep,0));
                break;

            case '4':
                rob.rotate(Point3f(0,rotStep,0));
                break;
            case '6':
                rob.rotate(Point3f(0,-rotStep,0));
                break;
            case '8':
                rob.rotate(Point3f(rotStep,0,0));
                break;
            case '2':
                rob.rotate(Point3f(-rotStep,0,0));
                break;
            case '7':
                rob.rotate(Point3f(0,0,rotStep));
                break;
            case '9':
                rob.rotate(Point3f(0,0,-rotStep));
                break;
            /*case '8':
                rob.walk(Point3f(0,0,walkStep));
                break;
            case '2':
                rob.walk(Point3f(0,0,-walkStep));
                break;
            case '4':
                rob.walk(Point3f(-walkStep,0,0));
                break;
            case '6':
                rob.walk(Point3f(walkStep,0,0));
                break;

            case '7':
                rob.walk(Point3f(-walkStep,0,walkStep));
                break;
            case '3':
                rob.walk(Point3f(walkStep,0,-walkStep));
                break;
            case '9':
                rob.walk(Point3f(walkStep,0,walkStep));
                break;
            case '1':
                rob.walk(Point3f(-walkStep,0,-walkStep));
                break;*/
        }

        view1.update(key, rob);
        key = waitKey(10);
    }
    return 0;
}
