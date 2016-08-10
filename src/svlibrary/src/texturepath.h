//Specify the path to textures
#include <string.h>               
#include <ros/ros.h>
#include <ros/package.h>
using namespace std;

const string path = ros::package::getPath("scenevalidator") +
                         "/src/svlibrary/src/textures";

const char *charPath = path.c_str();

#ifndef DRAWSTUFF_TEXTURE_PATH
#define DRAWSTUFF_TEXTURE_PATH charPath
#endif

