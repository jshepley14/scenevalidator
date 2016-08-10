/****************************************************
       Author:  Joe Shepley   jls2303@columbia.edu
  Description:  This program loads three objects demonstrates how they behvae in the simulation.
                Then the some of the orientations or positions are changed and again the simulator shows
                how the objects behave.
****************************************************/

#include "sceneValidator.h"
#include <map>
#include <cassert>              
#include <string.h>
#include <fstream>  
#include <cmath>   
#include <chrono>                 
#include <stdio.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <ros/package.h>
using namespace std;


//get file path for models
const string wine_glass = ros::package::getPath("scenevalidator") +
                         "/src/examples/src/models/wine_glass.obj";

const string paper_bowl = ros::package::getPath("scenevalidator") +
                         "/src/examples/src/models/paper_bowl.obj";

const string red_mug = ros::package::getPath("scenevalidator") +
                         "/src/examples/src/models/red_mug.obj";

int main (int argc, char **argv)
{
  

 // Here's all the input***********************************************************************************************************
  
                              //doesn't load this flipped version of the file (reversing coordinates makes negative dot product calculations)
  vector<string> filenames = {wine_glass, paper_bowl, red_mug};
  vector<string> modelnames = {"wine_glass", "paper_bowl", "red_mug"};
  
  //make affine3d's
  Eigen::Quaterniond q;  
  q = Eigen::Quaterniond(0.5, 0.5, 0, 0);  //identity matrix
  q.normalize();
  Eigen::Affine3d aq = Eigen::Affine3d(q);
  Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(-4,0,1.25)));  //wine_glass
  Eigen::Affine3d a = (t*aq); 
    
  q = Eigen::Quaterniond(0.5, 0.5, 0, 0);  //identity matrix
  q.normalize();
  aq = Eigen::Affine3d(q);
  t = (Eigen::Translation3d(Eigen::Vector3d(0,0,1.1)));  //paper bowl
  Eigen::Affine3d b = (t*aq); 
    
  q = Eigen::Quaterniond(0.5, 0.5, 0, 0);  //identity matrix
  q.normalize();
  aq = Eigen::Affine3d(q);
  t =  (Eigen::Translation3d(Eigen::Vector3d(4,0,0.66)));  //red mug
  Eigen::Affine3d c = (t*aq); 

  vector<Eigen::Affine3d> model_poses = {a,b,c};  //vector 1 of affines

  q = Eigen::Quaterniond(0.3, 0.7, 0, 0);  //NOT identity matrix
  q.normalize();
  aq = Eigen::Affine3d(q);
  t =  (Eigen::Translation3d(Eigen::Vector3d(-2,0, 1.59)));  //wine_glass
  Eigen::Affine3d a2 = (t*aq); 
  
  vector<Eigen::Affine3d> model_poses2 = {a2,b,c};  //vector of new affines
// Here's all the input^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


  
  // construct object and set parameters
  SceneValidator *scene = new SceneValidator;    //construct the object
  scene->setParams("DRAW", true);                //visualize what's actually happening 
  scene->setParams("PRINT_CHKR_RSLT", true);     //print out the result of each check
  scene->setParams("STEP1", 1000);               //make the first check take 1000 steps so user has plenty of time to see what's going on
  scene->setScale(1, 200);                       //set modelnames[1] to be scaled down by factor of 200   

  // API functions
  scene->setModels(modelnames, filenames);       //set all the models and get their data  
  scene->isValidScene(modelnames, model_poses);  //check scene 1
  //Press Ctrl-X when window pops up to exit above^ and start the simulation below 
  scene->isValidScene(modelnames, model_poses2); //check scene 2

  
  return 0;
}