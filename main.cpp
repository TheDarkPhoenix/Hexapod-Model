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

    Robot rob(Point3f(0, 15 ,100), Point3f(0,0,0), 11.8, 36.5, Point3f(3.7, 5.8, 16.3));

    float transStep = 1;
    float rotStep = 0.05;

    bool walking = false;

    float walkStep = 5;

    ///Tryby:
    ///1 - stanie w miejscu i ruch translacyjny
    ///2 - stanie w miesjscu i obroty
    ///3 - poruszanie siê manualne
    ///4 - poruszanie siê automatyczne

    int mode = 1;

    while(key != 27)
    {
        if(mode == 1)
        {
            switch(key)
            {
                case 'D':
                    rob.move(Point3f(transStep,0,0));
                    break;
                case 'A':
                    rob.move(Point3f(-transStep,0,0));
                    break;
                case 'Q':
                    rob.move(Point3f(0,0,transStep));
                    break;
                case 'E':
                    rob.move(Point3f(0,0,-transStep));
                    break;
                case 'S':
                    rob.move(Point3f(0,transStep,0));
                    break;
                case 'W':
                    rob.move(Point3f(0,-transStep,0));
                    break;
            }
        }
        else if(mode == 2)
        {
            switch(key)
            {
                case 'W':
                    rob.rotate(Point3f(0,rotStep,0));
                    break;
                case 'S':
                    rob.rotate(Point3f(0,-rotStep,0));
                    break;
                case 'A':
                    rob.rotate(Point3f(rotStep,0,0));
                    break;
                case 'D':
                    rob.rotate(Point3f(-rotStep,0,0));
                    break;
                case 'Q':
                    rob.rotate(Point3f(0,0,rotStep));
                    break;
                case 'E':
                    rob.rotate(Point3f(0,0,-rotStep));
                    break;
                }
        }
        else if(mode == 3)
        {
            switch(key)
            {
                case 'D':
                    rob.walk(Point3f(walkStep,0,0));
                    break;
                case 'A':
                    rob.walk(Point3f(-walkStep,0,0));
                    break;
                case 'W':
                    rob.walk(Point3f(0,0,walkStep));
                    break;
                case 'S':
                    rob.walk(Point3f(0,0,-walkStep));
                    break;
                case 'Q':
                    rob.walkRot(0.05);
                    break;
                case 'E':
                    rob.walkRot(-0.05);
                    break;
            }
        }
        else if(mode == 4)
        {
            switch(key)
            {
                case 'D':
                    rob.walkC(Point3f(walkStep,0,0), view1);
                    break;
                case 'A':
                    rob.walkC(Point3f(-walkStep,0,0), view1);
                    break;
                case 'W':
                    rob.walkC(Point3f(0,0,walkStep), view1);
                    break;
                case 'S':
                    rob.walkC(Point3f(0,0,-walkStep), view1);
                    break;
                case 'E':
                    rob.walkRotC(0.05, view1);
                    break;
                case 'Q':
                    rob.walkRotC(-0.05, view1);
                    break;
            }
        }

        switch(key)
        {
            case '1':
                mode = 1;
                break;
            case '2':
                mode = 2;
                break;
            case '3':
                mode = 3;
                break;
            case '4':
                mode = 4;
                break;
        }
        view1.update(key, rob);
        key = waitKey(10);
    }
    return 0;
}
