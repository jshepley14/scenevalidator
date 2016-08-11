/****************************************************
       Author:  Joe Shepley  jls2303@columbia.edu
  Description:  This program loads a single object four different times from the imperial data set. You can use this program to see if
                it's possible to load a .obj file of your choice.  The program also demonstrates how the
                object behaves in the simulation. While a window is open you can press Ctrl-X to slip to
                the next simulation of the object which will close current window and open new one. 
                Remember to scale the item if neccessary in setScale.
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


const string filename = ros::package::getPath("scenevalidator") +
                         "/src/examples/src/models/oreo1_reduced.obj";

int main (int argc, char **argv)
{
  

 // Here's all the input***********************************************************************************************************
  
 /*   Doesn't load flipped version of file (reversing coordinate order in faces makes negative dot product calculations)
   Doing the flipping operation in some other software screws up center of mass calculations because 
   the cross product produced is negative.  Just unflip the flipped figures in software (meshlab), and in the future don't flip
   them at all.  Or at the very least, just make sure your rotation/translation of the model in some other
   software doesn't screw up the vertice order for a face. */

  vector<string> filenames = {filename};
  vector<string> modelnames = {"test_object"};
  
  //make affine3d's
  Eigen::Quaterniond q;  
  q = Eigen::Quaterniond(0.5, 0.5, 0, 0);   //identity matrix
  q.normalize();
  Eigen::Affine3d aq = Eigen::Affine3d(q);
  Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(0,0,2)));  
  Eigen::Affine3d a = (t*aq); 
    
  q = Eigen::Quaterniond(0.7, 0.3, 0, 0);  //NOT identity matrix
  q.normalize();
  aq = Eigen::Affine3d(q);
  t = (Eigen::Translation3d(Eigen::Vector3d(0,0,2)));   
  Eigen::Affine3d b = (t*aq); 
    
  q = Eigen::Quaterniond(0.5, 0.5, 0, 0);   //identity matrix
  q.normalize();
  aq = Eigen::Affine3d(q);
  t =  (Eigen::Translation3d(Eigen::Vector3d(0,0,1)));
  Eigen::Affine3d c = (t*aq);   

  q = Eigen::Quaterniond(0.3, 0.7, 0, 0);   //NOT identity matrix
  q.normalize();
  aq = Eigen::Affine3d(q);
  t =  (Eigen::Translation3d(Eigen::Vector3d(0,0,1.7)));
  Eigen::Affine3d d = (t*aq); 
  
  vector<Eigen::Affine3d> model_poses1 = {a};
  vector<Eigen::Affine3d> model_poses2 = {b};
  vector<Eigen::Affine3d> model_poses3 = {c};
  vector<Eigen::Affine3d> model_poses4 = {d};  
// Here's all the input^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


  // construct object and set parameters
  SceneValidator *scene = new SceneValidator;    //construct the object
  scene->setParams("DRAW", true);                //visualize what's actually happening 
  scene->setParams("STEP1", 1000);               //make the first check take 1000 steps (so viewer has plenty of time to see object's behavior)
  scene->setScale(0, 0.1);                       //set modelnames[0] to be scaled down by factor of 100   

  // API functions
  scene->setModels(modelnames, filenames);       //set model and get its data  
  scene->isValidScene(modelnames, model_poses1); //check scene 1
  scene->isValidScene(modelnames, model_poses2); //check scene 2
  scene->isValidScene(modelnames, model_poses3); //check scene 3
  scene->isValidScene(modelnames, model_poses4); //check scene 4


  return 0;
}