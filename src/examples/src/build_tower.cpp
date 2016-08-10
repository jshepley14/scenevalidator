/****************************************************
       Author:  Joe Shepley   jls2303@columbia.edu
  Description:  This code provides builds the highest stable tower of three objects.  The order of objects to build the tower is known.
                The three objects are 1. paper bowl, 2. red mug, and 3. a model dog.  First the paper bowl alone is placed in the simulator
                and it's z position is varried going from a low position incrementing grdually until it reaches a stable position. 
                Then the red mug is placed in the simulator with the paper bowl. The paper bowl is now permanently at its lowest stable position.
                The red mug's z position is varried going from a low position incrementing grdually until it reaches a stable position (on top of 
                the paper bowl). This process is repeated for the dog and it ends up stably on top of the mug which is on top of the paper bowl.
                All of this process takes place without drawing and happens in 0.3 sec. You can draw but it will take around 30 sec. because it has
                to open and immediately close around the window around 100 times.  No matter if you decide to draw or not during the search time, at
                the end of this program, a window appears and draws the stable heighest configuration.
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


//get file paths for models
const string paper_bowl = ros::package::getPath("scenevalidator") +
                         "/src/examples/src/models/paper_bowl.obj";

const string red_mug = ros::package::getPath("scenevalidator") +
                         "/src/examples/src/models/red_mug.obj";

const string dog = ros::package::getPath("scenevalidator") +
                         "/src/examples/src/models/dog.obj";

int main (int argc, char **argv)
{


 // Here's all the input***********************************************************************************************************
  //just the paper_bowl
  vector<string> filenames1 = {paper_bowl};
  vector<string> modelnames1 = {"paper_bowl"};



  //paper_bowl and red_mug
  vector<string> filenames2 = {paper_bowl, red_mug};
  vector<string> modelnames2 = {"paper_bowl", "red_mug"};



  //paper_bowl, red_mug, and dog
 vector<string> filenames3 = {paper_bowl, red_mug, dog};
  vector<string> modelnames3 = {"paper_bowl", "red_mug","dog"};

  //ground truth for a stable tower is paper_bowl (0,0,0.27)   red_mug (0,0,1.13)   dog (0,0,2.03)
  //make affine3d's
  Eigen::Quaterniond q;
  q = Eigen::Quaterniond(0.5, 0.5, 0, 0);  //orientation is just standard identity matrix
  q.normalize();
  Eigen::Affine3d aq = Eigen::Affine3d(q);
  Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(0,0,0))); //paper_bowl 
  Eigen::Affine3d a = (t*aq);

  q = Eigen::Quaterniond(0.5, 0.5, 0, 0);  //orientation is just standard identity matrix
  q.normalize();
  aq = Eigen::Affine3d(q);
  t = (Eigen::Translation3d(Eigen::Vector3d(0,0,0)));  //mug 
  Eigen::Affine3d b = (t*aq);

  q = Eigen::Quaterniond(0.5, 0.5, 0, 0);  //orientation is just standard identity matrix
  q.normalize();
  aq = Eigen::Affine3d(q);
  t =  (Eigen::Translation3d(Eigen::Vector3d(0,0,0)));  //dog 
  Eigen::Affine3d c = (t*aq);

  vector<Eigen::Affine3d> model_poses = {a,b,c};  //put affines into vector

///........................................not used
  q = Eigen::Quaterniond(0.3, 0.7, 0, 0);
  q.normalize();
  aq = Eigen::Affine3d(q);
  t =  (Eigen::Translation3d(Eigen::Vector3d(-2,0, 1.59)));
  Eigen::Affine3d a2 = (t*aq);

  vector<Eigen::Affine3d> model_poses2 = {a2,b,c};  //teacup falling from the sky
// Here's all the input^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^





   //timer variables require c++11 
   static chrono::steady_clock::time_point startTime, endTime;
   startTime = chrono::steady_clock::now(); //start the timer

  /* Search #1: find lowest stable position for paper_bowl*/

  // construct object and set parameters
  SceneValidator *scene = new SceneValidator;   // construct object and set parameters
  //scene->setParams("DRAW", true);             //you could draw the search if you want
  scene-> setModels(modelnames1, filenames1);   // set all the model's data to be ready for simulation
  scene-> setParams("THRESHOLD", 0.01);         // set the threshold to be 0.01
  double i = 0;       // this is where the bowl will start testing stability positions
  while(i <= 3){      // 3 is the highest z we will allow to be possibly in the search space
    model_poses[0].translation()[2]=i;    // sets the model's z position to i
    bool valid = scene-> isValidScene(modelnames1, model_poses);  //test if scene is stable (valid) or not
 	  if (valid){
       cout<<"TRUE paper bowl z pos at ";
       cout<<model_poses[0].translation()[2]<<endl;  //print the paper bowl's z position
       break;
     }  
     i=i+0.01;   //increment by 0.01 (that'll be our resolution)
  }
  delete scene;
  
 

  /* Search #2: find lowest stable position for red_mug (on top of paper_bowl) */

  SceneValidator *scene2 = new SceneValidator;
  //scene2->setParams("DRAW", true);            //you could draw the search if you want
  scene2->setModels(modelnames2, filenames2);
  scene2->setParams("THRESHOLD", 0.04); //decided threshold to be 0.04.  using 0.01 might find not solutions  
  //i = model_poses[0].translation()[2]*2;  //you could make i start at 2*(the stable z position that was just found)
  i=1.0;          //I just chose the z position search to start at this number 
  while(i <= 3){  // 3 is the highest z we will allow to be possibly in the search space
    model_poses[1].translation()[2]=i;
    bool valid = scene2->isValidScene(modelnames2, model_poses);  
 	  if (valid){
       cout<<"TRUE red mug z pos at ";
       cout<<model_poses[1].translation()[2]<<endl; //print the red mug's z position
       break;
     }  
     i=i+0.01;
  }
  delete scene2;

  /* Search #3: find lowest stable position for dog (on top of red_mug and paper_bowl) */

  SceneValidator *scene3  = new SceneValidator;
  //scene3 ->setParams("DRAW", true);   //you could draw the search if you want
  scene3->setScale(2,10);               //need to scale the 2rd item (vector[2] = dog) down by factor of 10 (default is 100)
  scene3 ->setModels(modelnames3, filenames3);
  scene3 ->setParams("THRESHOLD", 0.05); //decided threshold to be 0.04
  //i=model_poses[1].translation()[2]; //could start i based on position that was previusly found
  i=1.8;  //decided to choose to start search at this numeber
  while(i <= 3){  //don't want to search beyond z = 3
    model_poses[2].translation()[2]=i;
    bool valid = scene3 ->isValidScene(modelnames3, model_poses);  
 	  if (valid){
       cout<<"TRUE dog z pos at ";
       cout<<model_poses[2].translation()[2]<<endl;  //print the dog's z position 

       //end the timer
       endTime = chrono::steady_clock::now();
       auto diff = endTime - startTime;
       cout <<"Time: "<< chrono::duration <double, milli> (diff).count() << " ms" << endl;

       scene3->setParams("DRAW", true);  //draw the scene 
       scene3->isValidScene(modelnames3, model_poses); //running the scene here would return false for some reason ...However see below
       cout<<"Look above the simulation command menu to see timing results and positions.  Ctrl-X to close window"<<endl;
       break;
     }  
     i=i+0.01;
  }
  
  //Run the simulator on the scene whic we know is valid and has heighest height
  scene3->setParams("PRINT_CHKR_RSLT", true);     //print out the checking results
  scene3->setParams("STEP4", 1000);               //4th check is set to super long time so viewer can have time to see the configuration
  scene3->isValidScene(modelnames3, model_poses); //run the simulator on scene we know is valid and highest
  
  delete scene3 ;

  //program is finished
  return 0;
}