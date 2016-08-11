/****************************************************/
//     Author:  Joe Shepley
//    Contact:  jshepley14@gmail.com or jls2303@columbia.edu
//   Location:  Carnegie Mellon University Robotics Institute
//       Date:  July 2016
//Description:  This code provides functions which can be used to check if a set of objects are in static
//              equilibrium or not.  The method to do this relies on in Open Dynamics Engine, a physics engine.
//    Gitthub:  https://github.com/jshepley14/PhysicsEngine
//   Glossary:
//              "ODE": Open Dynamics Engine http://www.ode.org/
//              "draw":  render a scene using the drawstuff graphics library and actually see how the objects are behaving
//              "ODE trimesh demo": refers to demo_moving_trimesh.cpp a demo program in ODE's demo folder
/****************************************************/


/* headers */
#include "sceneValidator.h"       //contains the header file for this .cpp file
#include <map>                    //used to make hashmap
#include <cassert>                //used to make hashmap
#include <ode/ode.h>              //main physics engine library
#include <drawstuff/drawstuff.h>  //this is the graphics library
#include <string.h>               //allows strings to be used
#include <fstream>                //allows some extra printing functions
#include <cmath>                  //allows math functions like absolute value
#include <chrono>                 //used for timing code
#include <stdio.h>                //common and neccesary c++ library
#include <iostream>               //used for printing
#include <Eigen/Dense>            //used for dealing with Eigen data types
#include <Eigen/Geometry>         //used for dealing with Eigen data types
#include "objLoader.h"            //used for parsing .obj file
#include "texturepath.h"          //used for getting path to textures

/*definitions */

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions, really only dsDrawTriangleD is used in this program
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawLine dsDrawLineD
#define dsDrawTriangle dsDrawTriangleD
#endif

// some constants
#define NUM 200			    // max number of objects (FYI 14 objects make program 10x slower than 2 objects and Number of Objects vs Time is linear)
#define GPB 3		      	// maximum number of geometries per body
using namespace std;


/* variables IMPORTANT

 ---  Variables that will affect time to validate scene ---
  DRAW (rendering an image significantly slows down computation time)
  MAX_CONTACTS
  STEP1, STEP2, STEP3, and STEP4
  THRESHOLD 
  TIMESTEP

 ---  Variables that affect THRESHOLD  ---
	BOUNCE
  BOUNCE_vel
  DENSITY
  FRICTION_mu
  GRAVITYz
  SOFT_CFM
  
   ---  Variables that affect physics accuracy  ---
	BOUNCE
  BOUNCE_vel
  DENSITY
  FRICTION_mu
  GRAVITYz
  SOFT_CFM
  TIMESTEP
  DEFAULT_SCALE
  MAX_CONTACTS                                           */


//Variables that can be set in setParams() or in custom constructor or in setScale()
static double BOUNCE = 0.0;            //change the bounciness
static double BOUNCE_vel = 0.0;        //change the bounciness speed
static double DEFAULT_SCALE = 100;     //The default value each .obj files data is scaled down by. Set scale in setScale
static double DENSITY = 5.0;           //The default is from ODE trimesh demo
static bool   DRAW = false;            //used to switch on or off the drawing of the scene
static double FRICTION_mu =  1.0;      //if you set this to 0 objects will be very slippery
static double FRICTION_mu2 =  0.0;     //changing this doesn't seem to do much
static int    MAX_CONTACTS = 64;       //maximum number of contact points per body
static double GRAVITYx = 0;            //gravitational force coming from x direction
static double GRAVITYy = 0;            //gravitational force coming from y direction
static double GRAVITYz = -0.5;         //yes, this is not -9.8, but this was the default that ODE trimesh demo had. Using -9.8 in this program prevents accuracy unless you change other variables like TIMESTEP
static double PLANEa = 0;              //Equation of a plane: a*x+b*y+c*z = d The normal vector must have length 1
static double PLANEb = 0;
static double PLANEc = 1;
static double PLANEd = 0;
static bool   PRINT_AABB = false;      //print the object's Bounding Box
static bool   PRINT_CHKR_RSLT = false; //print the result of check1, check2 etc..
static bool   PRINT_COM = false;       //print the object's center of mass
static bool   PRINT_DELTA_POS = false; //print an object's delta x,y,z for its center
static bool   PRINT_END_POS   = false; //print an object's final x,y,z center
static bool   PRINT_START_POS = false; //print an object's intial x,y,z center
static double SOFT_CFM = 0.01;         //makes "system more numerically robust" according to ODE manual. Not 100% sure what it does... The current number is from a default demo.
static int    STEP1=6;                 //amount of simulation steps used in check #1
static int    STEP2=14;                //amount of simulation steps used in check #2
static int    STEP3=20;                //amount of simulation steps used in check #3
static int    STEP4=110;               //amount of simulation steps used in check #4
static double THRESHOLD  = 0.08;       //amount objects allowed to move while still being marked as in static equilibrium
static double TIMESTEP = 0.05;         //controls how far each physics simulation step is taken



//variables used when DRAW = true
float  xyz[3]={ -0.0559,  -8.2456, 6.0500};  //this sets the x,y,z of the camera position when you view a drawing
float  hpr[3]={ 89.0000, -25.0000, 0.0000};  //this sets the heading, pitch and roll numbers in degrees(camera angle) of the camera when you view a drawing
static int    counter=0;    //used within simulation to count until dsSTEP, indicates termination of drawing window
static int    dsSTEP=100;   //default simulation step number when drawing a scene. To change dsSTEP, just change STEP1,2,3 or 4.
static int    HEIGHT=500;   //window height
static int    WIDTH=1000;   //window width

//timer variables, requires c++11 
static chrono::steady_clock::time_point startTime, endTime;



/* dynamics and collision object (this is the model's data) */

struct MyObject {
  dBodyID body;			                     // the body of the object
  dGeomID geom[GPB];                     // geometries representing this body
  dReal matrix_dblbuff[ 16 * 2 ];        // double buffered matrices for 'last transform' setup (not sure what this does, it was from ODE trimesh demo)
  int last_matrix_index;                 // has to do with double buffered matrices (not sure what this does, it was from ODE trimesh demo) 
  string model_ID;                       //model's I.D.      
  dReal center[3];                       //the center x,y,z coordinates
  int indCount;                          //number of triangles (indices)
  int vertCount;                         //number of vertices
  vector< vector<int> > indexDrawVec;    //index list for Trimesh, but only used when drawing the Trimesh
  vector<int> indexGeomVec;              //index list for the Trimesh, used all the time for Trimesh
  vector<float> vertexDrawVec;           //vertex list for Trimesh, but only used when drawing the Trimesh
  vector<float> vertexGeomVec;           //vetex list for the Trimesh, used all the time for Trimesh
  vector<float> centerOfMass;            //center of mass x,y,z
};

// this class is for center of mass calculations
class data
{
public:
    float x1,y1,z1;
    float x2,y2,z2;
    float x3,y3,z3;
};

//more variables, not really parameters though
static int num=0;	                         //number of objects in simulation
static int nextobj=0;		                   //next object to recycle if num==NUM
static dWorldID world;                     //define the world in which simulation takes place
static dSpaceID space;                     //define the space in which simulation takes place
static MyObject obj[NUM];                  //array of MyObject's
static dJointGroupID contactgroup;         //define the contactgroup in which objects have their contacts
static int show_contacts = 0;	             //show contact points
static int random_pos = 1;	               //drop objects from random position?
static std::map<std::string, MyObject> m;  //hashmap of object names and their MyObject data
typedef dReal dVector3R[3];                //probably don't need this
static double scaling[NUM];                //array to be filled with scaling info for each object



/*functions are below*/

/* Handles objects' collisions (makes a termporary joint)
   This is called by dSpaceCollide when two objects in space are potentially colliding. 
   I did not alter this function from ODE trimesh demo except for the parameter values. */
static void nearCallback (void *, dGeomID o1, dGeomID o2)
{
  int i;
  // exit without doing anything if the two bodies are connected by a joint
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;

  //set parameters for each contact on the object
  dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
  for (i=0; i<MAX_CONTACTS; i++) {
    contact[i].surface.mode = dContactBounce | dContactSoftCFM;
    contact[i].surface.mu = FRICTION_mu;
    contact[i].surface.mu2 = FRICTION_mu2;
    contact[i].surface.bounce = BOUNCE;
    contact[i].surface.bounce_vel = BOUNCE_vel;
    contact[i].surface.soft_cfm = SOFT_CFM;
  }

  //execute collision force (temporary joint)
  if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,
			   sizeof(dContact))) {
    dMatrix3 RI;
    dRSetIdentity (RI);
    const dReal ss[3] = {0.02,0.02,0.02};
    for (i=0; i<numc; i++) {
      dJointID c = dJointCreateContact (world,contactgroup,contact+i);
      dJointAttach (c,b1,b2);
      if (show_contacts) dsDrawBox (contact[i].geom.pos,RI,ss);
    }
  }
}

/* user can set viewpoint (camera angle) */
bool SceneValidator::setCamera(float x, float y, float z, float h, float p, float r){
     xyz[0]=x;
     xyz[1]=y;
     xyz[2]=z;
     hpr[0]=h;
     hpr[1]=p;
     hpr[2]=r;
}


/* start simulation when drawing (sets the viewpoint) */
static void start(){
  dsSetViewpoint (xyz,hpr);
}


/* after a certaint number of simulation steps, checks if an object from a the scene is valid or not */
static bool inStaticEquilibrium(MyObject &object){
    double startX = object.center[0];                  //get the object's initial x pos.
    double startY = object.center[1];                  //get the object's initial y pos.
    double startZ = object.center[2];                  //get the object's initial z pos.
    double endX = dBodyGetPosition(object.body)[0];    //get the object's final x pos.
    double endY = dBodyGetPosition(object.body)[1];    //get the object's final y pos.
    double endZ = dBodyGetPosition(object.body)[2];    //get the object's final z pos.
    double deltaX = std::abs(startX - endX);           //calculate change in x
    double deltaY = std::abs(startY - endY);           //calculate change in y
    double deltaZ = std::abs(startZ - endZ);           //calculate change in z

    //some if statements for if you want to print out what's happening
    if (PRINT_START_POS || PRINT_END_POS || PRINT_DELTA_POS){
      cout<<object.model_ID<<endl;
    }
    if (PRINT_START_POS){
      cout<<"Start: "<<startX<<", "<<startY<<", "<<startZ<<endl;
    }
    if (PRINT_END_POS){
      cout<<"  End: "<<endX<<", "<<endY<<", "<<endZ<<endl;
    }
    if (PRINT_DELTA_POS){
      cout<<"Delta: "<<deltaX<<", "<<deltaY<<", "<<deltaZ<<endl;
    }

    // compare the change in the object's position to see how far it moved
    // if change in object's position is greater than some threshold, return false
    if ( deltaX > THRESHOLD  || deltaY > THRESHOLD  || deltaZ > THRESHOLD ){
        return false;
    } else{
        return true;
    }
}



/* after a certaint number of simulation steps, checks if a scene is valid or not
by iterating through the list of objects and checking if any of the object's moved too far */
static bool isValid(std::vector<string> modelnames){
    bool stable = true;  //bool that says scene is stable (valid) or not
    for (int i=0; i<num; i++){  //iterate through hashmap of modelnames that correspond to objects
       auto mappedObject= m.find(modelnames[i]);
       if (!inStaticEquilibrium(mappedObject->second) ){  //check if an object has moved too much (beyond threshold)
          stable = false;
          break;
       }
    }

    //print out the results and return the results as bools
    if (stable == true){
      if(PRINT_CHKR_RSLT){
        cout << "TRUE"<<endl;
      }
      return true;
    } else{
      if(PRINT_CHKR_RSLT){
        cout << "FALSE"<<endl;
      }
      return false;
    }
}



/* sets all the objects' data */
void setObject (MyObject &object, double number, char* filename){

  double SCALE = number; //set the scale, or else object will be too big or too small, can set the scale manually if you want in setScale()

  //Load the file
  objLoader *objData = new objLoader();     //this objLoader code relies on objLoader.h and it's dependencies
	objData->load(filename);                  //load the file to be referenced as an objData object
  object.indCount = objData->faceCount;     //get number of faces that make up the trimesh
  object.vertCount = objData->vertexCount;  //get the number of vertices that make up the trimesh

  //get the center of mass.  The procedure to do this was inspired by http://stackoverflow.com/questions/2083771/a-method-to-calculate-the-centre-of-mass-from-a-stl-stereo-lithography-file
  int numTriangles = objData->faceCount;
	data triangles[numTriangles];
	for (int i =0; i < numTriangles; i ++){  // fill the triangles array with the data in the obj file
		obj_face *o = objData->faceList[i];
		triangles[i].x1=objData->vertexList[ o->vertex_index[0] ]->e[0] ;
		triangles[i].y1=objData->vertexList[ o->vertex_index[0] ]->e[1] ;
		triangles[i].z1=objData->vertexList[ o->vertex_index[0] ]->e[2] ;
		triangles[i].x2=objData->vertexList[ o->vertex_index[1] ]->e[0] ;
		triangles[i].y2=objData->vertexList[ o->vertex_index[1] ]->e[1] ;
		triangles[i].z2=objData->vertexList[ o->vertex_index[1] ]->e[2] ;
		triangles[i].x3=objData->vertexList[ o->vertex_index[2] ]->e[0] ;
		triangles[i].y3=objData->vertexList[ o->vertex_index[2] ]->e[1] ;
		triangles[i].z3=objData->vertexList[ o->vertex_index[2] ]->e[2] ;
	}
  double totalVolume = 0, currentVolume;
  double xCenter = 0, yCenter = 0, zCenter = 0;
  for (int i = 0; i < numTriangles; i++)
  {
      totalVolume += currentVolume = (triangles[i].x1*triangles[i].y2*triangles[i].z3 - triangles[i].x1*triangles[i].y3*triangles[i].z2 - triangles[i].x2*triangles[i].y1*triangles[i].z3 + triangles[i].x2*triangles[i].y3*triangles[i].z1 + triangles[i].x3*triangles[i].y1*triangles[i].z2 - triangles[i].x3*triangles[i].y2*triangles[i].z1) / 6;
      xCenter += ((triangles[i].x1 + triangles[i].x2 + triangles[i].x3) / 4) * currentVolume;
      yCenter += ((triangles[i].y1 + triangles[i].y2 + triangles[i].y3) / 4) * currentVolume;
      zCenter += ((triangles[i].z1 + triangles[i].z2 + triangles[i].z3) / 4) * currentVolume;
  }
  double COMX = (xCenter/totalVolume)/SCALE;
  double COMY = (yCenter/totalVolume)/SCALE;
  double COMZ = (zCenter/totalVolume)/SCALE;
  if (PRINT_COM){
    cout<<object.model_ID<<endl;
    printf("COM:    %.4f, %.4f, %.4f\n", COMX, COMY, COMZ);
  }
  object.centerOfMass = {COMX,COMY,COMZ};  //finished getting the center of mass


  //Now get all the data from the .obj file (faces and vertices)
  //and put it in vectors so ODE can make a trimesh out of that data.
  //vectors "for drawing" have the same data as vectors "for geometry",
  //but it's more manageable to have seperate vectors when DRAW = true and when DRAW = false

  //Make 2D vector of indices for drawing
  int indexCount = object.indCount;
	for(int i=0; i<indexCount; i++)
	{	vector<int> temp_vec;
		temp_vec.push_back((objData->faceList[i])->vertex_index[0]);
		temp_vec.push_back((objData->faceList[i])->vertex_index[1]);
		temp_vec.push_back((objData->faceList[i])->vertex_index[2]);
		object.indexDrawVec.push_back(temp_vec);
	}

  //Make 1D vector of indices for geometry
  for(int i=0; i< indexCount; i++)
  {
    object.indexGeomVec.push_back((objData->faceList[i])->vertex_index[0]);
    object.indexGeomVec.push_back((objData->faceList[i])->vertex_index[1]);
    object.indexGeomVec.push_back((objData->faceList[i])->vertex_index[2]);
  }

  //The object's center of mass must be at 0,0,0 relative to the rest of the object, that's why we scale and then shift by the calculated centerOfMass
  //Make 1D vector of vertices for drawing
  int vertCount =  object.vertCount;
	for(int i=0; i< vertCount ; i++){
		object.vertexDrawVec.push_back( objData->vertexList[i]->e[0]/SCALE - object.centerOfMass[0]);
		object.vertexDrawVec.push_back( objData->vertexList[i]->e[1]/SCALE - object.centerOfMass[1]);
		object.vertexDrawVec.push_back( objData->vertexList[i]->e[2]/SCALE - object.centerOfMass[2]);
	}

  //Make 1D vector of vertices for geometry
  for(int i=0; i< vertCount ; i++){
		object.vertexGeomVec.push_back( objData->vertexList[i]->e[0]/SCALE - object.centerOfMass[0]);
		object.vertexGeomVec.push_back( objData->vertexList[i]->e[1]/SCALE - object.centerOfMass[1]);
		object.vertexGeomVec.push_back( objData->vertexList[i]->e[2]/SCALE - object.centerOfMass[2]);
	}

}


/* construct the object and put it into the world */
void makeObject (MyObject &object){
  int i,j,k;
  dMass m;  //this is ODE's special "mass" object. It contains inertia info, actual weight and center of mass. Look at mass.h and mass.cpp for more info in ODE library

  object.body = dBodyCreate (world);  //you must create a "body" AND a "geom" (geometry) to represent an model in ODE
  dBodySetData (object.body,(void*)(size_t)i);

  //build Trimesh geom
  dTriMeshDataID new_tmdata = dGeomTriMeshDataCreate();  //set a trimesh ODE data type 
  dGeomTriMeshDataBuildSingle(new_tmdata, object.vertexGeomVec.data(), 3 * sizeof(float),    //build the geometry of the trimesh
	     object.vertCount, (int*)object.indexGeomVec.data(), object.indCount*3, 3 * sizeof(int));
  object.geom[0] = dCreateTriMesh(space, new_tmdata, 0, 0, 0);  //create the trimesh using the ODE trimesh data that was just defined
  dGeomSetData(object.geom[0], new_tmdata);  //officially set the data into the object's geom (geometry)
  dMassSetTrimesh( &m, DENSITY, object.geom[0] );  //set the trimesh's mass
  
  //gets the absolute bounding box, you can print it. Nothing currently used the the AABB info, but could be helpful at some point
  dReal aabb[6];
  dGeomGetAABB (object.geom[0], aabb);
  if (PRINT_AABB){
    printf("AABB: minX %.3f, maxX %.3f, minY %.3f, maxY %.3f, minZ %.3f, maxZ %.3f\n",aabb[0],aabb[1],aabb[2],aabb[3],aabb[4],aabb[5] );
    printf("\n");
  }

  dGeomSetPosition(object.geom[0], m.c[0], m.c[1], m.c[2]);  //this is required because ODE's mass has to be at (0,0,0)
  dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);  //object's center of mass must be at 0,0,0 relative to the rest of the object

  //build Trimesh body and unite geom with body
  for (k=0; k < GPB; k++){  //set body loop
      if (object.geom[k]){
          dGeomSetBody(object.geom[k],object.body); //unite body and geometry 
      }
  }
  dBodySetMass(object.body,&m);  //set the body's mass

}



/* set the objects' 6DoF poses */
void translateObject(MyObject &object, const dReal* center, const dMatrix3 R ){
      object.center[0] = center[0];  //set the object's initial x position (used when comparing initial vs final positions)
      object.center[1] = center[1];  //set the object's initial y position
      object.center[2] = center[2];  //set the object's initial z position
      dBodySetPosition (object.body, center[0], center[1], center[2]);  //now we ACTUALLY set it's position in the simulation
      dBodySetRotation (object.body, R);        //set it's rotation
      dBodySetLinearVel(object.body, 0, 0, 0);  //set linear velocity to 0, else it would keep old lin. & ang. vel. from before we translated it
      dBodySetAngularVel(object.body, 0, 0, 0); //set angular velocity to 0
}


/* set previous transformation matrix for trimesh 
   not entirely sure what this function's purpose is as it was directly from ODE trimesh demo */
void setCurrentTransform(dGeomID geom){
   const dReal* Pos = dGeomGetPosition(geom);  //get the object's current position
   const dReal* Rot = dGeomGetRotation(geom);  //get the object's current rotation
   const dReal Transform[16] =
   {
     Rot[0], Rot[4], Rot[8],  0,
     Rot[1], Rot[5], Rot[9],  0,
     Rot[2], Rot[6], Rot[10], 0,
     Pos[0], Pos[1], Pos[2],  1
   };
   dGeomTriMeshSetLastTransform( geom, *(dMatrix4*)(&Transform) );
}




/* simulation loop */
static void simLoop (int pause)
{
  //if DRAW = true, this is used to terminate the simloop from dsSimulationLoop()
  if (counter == dsSTEP){
      dsStop();
  }
  counter ++;


  //define the space and collide function
  dSpaceCollide (space,0,&nearCallback);

//not quite sure what this code block or what setCurrentTransform() does, but it was from ODE trimesh demo
#if 1
  if (!pause)
  {
    for (int i=0; i<num; i++)
      for (int j=0; j < GPB; j++)
        if (obj[i].geom[j])
          if (dGeomGetClass(obj[i].geom[j]) == dTriMeshClass)
            setCurrentTransform(obj[i].geom[j]);
  }
#endif

  if (!pause) dWorldQuickStep (world,TIMESTEP); //<- this is a big factor in accuracy and how long simulation takes

  //not 100% what dSpaceGetNumGeoms() does... It was in ODE trimesh demo.
  for (int j = 0; j < dSpaceGetNumGeoms(space); j++){
	  dSpaceGetGeom(space, j);
  }
  // remove all contact joints
  dJointGroupEmpty (contactgroup);

  //set the color and the texture for the objects when drawing them
  if(DRAW){
    dsSetColor (1,1,0);
    dsSetTexture (DS_WOOD);
  }

  //updates the position and rotation with every step through the simulation
  for (int i=0; i<num; i++) {
    for (int j=0; j < GPB; j++) {
      if (obj[i].geom[j]) {
        if (dGeomGetClass(obj[i].geom[j]) == dTriMeshClass) {
          const dReal* Pos = dGeomGetPosition(obj[i].geom[j]);  //get and set the new position
          const dReal* Rot = dGeomGetRotation(obj[i].geom[j]);  //get and set the new rotation

        //this is where drawstuff library actually draws the trimesh
        if (DRAW) {
            for (int ii = 0; ii < obj[i].indCount; ii++) {
                const dReal v[9] = { // explicit conversion from float to dReal
                  obj[i].vertexDrawVec[obj[i].indexDrawVec[ii][0] * 3 + 0],
                  obj[i].vertexDrawVec[obj[i].indexDrawVec[ii][0] * 3 + 1],
                  obj[i].vertexDrawVec[obj[i].indexDrawVec[ii][0] * 3 + 2],
                  obj[i].vertexDrawVec[obj[i].indexDrawVec[ii][1] * 3 + 0],
                  obj[i].vertexDrawVec[obj[i].indexDrawVec[ii][1] * 3 + 1],
                  obj[i].vertexDrawVec[obj[i].indexDrawVec[ii][1] * 3 + 2],
                  obj[i].vertexDrawVec[obj[i].indexDrawVec[ii][2] * 3 + 0],
                  obj[i].vertexDrawVec[obj[i].indexDrawVec[ii][2] * 3 + 1],
                  obj[i].vertexDrawVec[obj[i].indexDrawVec[ii][2] * 3 + 2]
                };
                dsDrawTriangle(Pos, Rot, &v[0], &v[3], &v[6], 1);  //a trimesh is made up of triangles so triangles are drawn

              }
        }

      //not quite sure what the following code from here until the end of this function does but it was from ODE's trimesh demos
      //the following comments are from the demo as well

      // tell the tri-tri collider the current transform of the trimesh --
      // this is fairly important for good results.
		  // Fill in the (4x4) matrix.
		  dReal* p_matrix = obj[i].matrix_dblbuff + ( obj[i].last_matrix_index * 16 );

		  p_matrix[ 0 ] = Rot[ 0 ];	p_matrix[ 1 ] = Rot[ 1 ];	p_matrix[ 2 ] = Rot[ 2 ];	p_matrix[ 3 ] = 0;
		  p_matrix[ 4 ] = Rot[ 4 ];	p_matrix[ 5 ] = Rot[ 5 ];	p_matrix[ 6 ] = Rot[ 6 ];	p_matrix[ 7 ] = 0;
		  p_matrix[ 8 ] = Rot[ 8 ];	p_matrix[ 9 ] = Rot[ 9 ];	p_matrix[10 ] = Rot[10 ];	p_matrix[11 ] = 0;
		  p_matrix[12 ] = Pos[ 0 ];	p_matrix[13 ] = Pos[ 1 ];	p_matrix[14 ] = Pos[ 2 ];	p_matrix[15 ] = 1;

		  // Flip to other matrix.
		  obj[i].last_matrix_index = !obj[i].last_matrix_index;

		  dGeomTriMeshSetLastTransform( obj[i].geom[j],
			  *(dMatrix4*)( obj[i].matrix_dblbuff + obj[i].last_matrix_index * 16 ) );

        }
      }
    }
  }
}




/* special simulation loop needed when drawing a scene (ultimately still uses simloop() though) */
void drawstuffsimLoop(){
  int argc=NULL;           //just an argument that dsSimulationLoop must take. not used.
  char **argv=NULL;        //just an argument that dsSimulationLoop must take. not used.
  dsFunctions fn;          //defines callback functions used in dsSimulationLoop
  fn.version = DS_VERSION; //gets version number 
  fn.start = &start;       //start() is a function defined above which set the viewpoint
  fn.step = &simLoop;      //simloop() is the simulaiton routine used in dsSimulationLoop
  fn.command = NULL;       //if you want to have keyboard input. not used.
  fn.stop = NULL;          //if you want to customize the stop function. not used.
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;  //Remember to include text path in texturepath.h
  dsSimulationLoop (argc,argv,WIDTH,HEIGHT,&fn);
}



/* sets all the models' data */
void SceneValidator::setModels(std::vector<string> modelnames, std::vector<string> filenames){

   //set the data in an obj array
   if( modelnames.size() != filenames.size()){
          std::cout<<"***ERROR*** in setModels(std::vector<string> modelnames, std::vector<string> filenames). The problem is that modelnames is not the same size as filenames"<<endl;
   } else{
      num = filenames.size();  //number of models in scene
      for (int i =0; i < num; i++){
          obj[i].model_ID=modelnames[i];  //set model ID to the corresponding model name
          char *charfilenames = new char[filenames[i].length() + 1]; //convert to string
          std::strcpy(charfilenames, filenames[i].c_str());  //convert to string
          setObject(obj[i], scaling[i], charfilenames );   //set object's data
          makeObject(obj[i]);  //create an object that can be used in simulation
      }
   }
   //make hashmap between modelnames and their data
   for (int i =0; i < num; i++){    
      m[modelnames[i]]=obj[i];
   }
}



/*checks if scene is stable after certain number of steps */
static bool isStableStill(std::vector<string> modelnames, int step){
    if (DRAW){
      counter=0;
      dsSTEP=step;
      drawstuffsimLoop();
    } else {
      for(int i = 0; i <= step; i++) {
        simLoop(0);
       }
    }
    return isValid(modelnames);
}



/* allows user to set certain parameters */
bool  SceneValidator::setParams(std::string param_name, double param_value){
      if( param_name.compare("STEP1") == 0 ){
        STEP1 = param_value;
        return true;
      } else if( param_name.compare("STEP2") == 0 ){
        STEP2 = param_value;
        return true;
      } else if( param_name.compare("STEP3") == 0 ){
        STEP3 = param_value;
        return true;
      } else if( param_name.compare("STEP4") == 0 ){
        STEP4 = param_value;
        return true;
      } else if( param_name.compare("GRAVITYx") == 0 ){
        cout<<"Need to set GRAVITYx in the custom SceneValidator constructor.  See sceneValidator.h for how to do that";
        return true;
      } else if( param_name.compare("GRAVITYy") == 0 ){
        cout<<"Need to set GRAVITYy in the custom SceneValidator constructor.  See sceneValidator.h for how to do that";
        return true;
      } else if( param_name.compare("GRAVITYz") == 0 ){
        cout<<"Need to set GRAVITYz in the custom SceneValidator constructor.  See sceneValidator.h for how to do that";
        return true;
      } else if( param_name.compare("PLANEa") == 0 ){
        cout<<"Need to set PLANEa in the custom SceneValidator constructor.  See sceneValidator.h for how to do that";
        return true;
      } else if( param_name.compare("PLANEb") == 0 ){
        cout<<"Need to set PLANEb in the custom SceneValidator constructor.  See sceneValidator.h for how to do that";
        return true;
      } else if( param_name.compare("PLANEc") == 0 ){
        cout<<"Need to set PLANEc in the custom SceneValidator constructor.  See sceneValidator.h for how to do that";
        return true;
      } else if( param_name.compare("PLANEd") == 0 ){
        cout<<"Need to set PLANEd in the custom SceneValidator constructor.  See sceneValidator.h for how to do that";
        return true;
      } else if( param_name.compare("THRESHOLD") == 0 ){
        THRESHOLD  = param_value;
        return true;
      } else if( param_name.compare("TIMESTEP") == 0 ){
        TIMESTEP = param_value;
        return true;
      } else if( param_name.compare("FRICTION_mu") == 0 ){
        FRICTION_mu = param_value;
        return true;
      } else if( param_name.compare("FRICTION_mu2") == 0 ){
        FRICTION_mu2 = param_value;
        return true;
      } else if( param_name.compare("BOUNCE") == 0 ){
        BOUNCE = param_value;
        return true;
      } else if( param_name.compare("BOUNCE_vel") == 0 ){
        BOUNCE_vel = param_value;
        return true;
      } else if( param_name.compare("SOFT_CFM") == 0 ){
        SOFT_CFM = param_value;
        return true;
      } else if( param_name.compare("DRAW") == 0 ){
        DRAW = param_value;
        return true;
      } else if( param_name.compare("PRINT_START_POS") == 0 ){
        PRINT_START_POS = param_value;
        return true;
      } else if( param_name.compare("PRINT_END_POS") == 0 ){
        PRINT_END_POS = param_value;
        return true;
      } else if( param_name.compare("PRINT_DELTA_POS") == 0 ){
        PRINT_DELTA_POS = param_value;
        return true;
      } else if( param_name.compare("PRINT_CHKR_RSLT") == 0 ){
        PRINT_CHKR_RSLT = param_value;
        return true;
      } else if( param_name.compare("DENSITY") == 0 ){
        DENSITY = param_value;
        return true;
      } else if( param_name.compare("MAX_CONTACTS") == 0 ){
        MAX_CONTACTS = param_value;
        return true;
      } else if( param_name.compare("PRINT_AABB") == 0 ){
        PRINT_AABB = param_value;
        return true;
      } else if( param_name.compare("PRINT_COM") == 0 ){
        PRINT_COM = param_value;
        return true;
      } else {
        cout<<"Invalid parameter name: "<<param_name;
        return false;
      }
}

/* allows user to set the scale of a specific object */
bool  SceneValidator::setScale(int thisObject, double scaleFactor){
      scaling[thisObject] = scaleFactor;
}



/* checks if a given scene is in static equilibrium or not */
bool SceneValidator::isValidScene(std::vector<string> modelnames, std::vector<Eigen::Affine3d> model_poses){
    //set all the Objects's positions
    num = modelnames.size();
    for (int i =0; i < num; i++){
       auto mappedObject= m.find(modelnames[i]);   //get model from hashmap
       Eigen::Affine3d a = model_poses[i];
       const dMatrix3 R = {                        //convert affine info to a 3x3 rotation matrix and a x,y,z position array
         a(0,0), a(0,1), a(0,2),
         a(1,0), a(1,1), a(1,2),
         a(2,0), a(2,1), a(2,2)    };
       const dReal center[3] = {a.translation()[0],a.translation()[1], a.translation()[2]};
       translateObject(mappedObject->second, center, R);  //get the model name's MyObject info and feed it the position and rotation
    }

    //complete series of checks to see if scene is still stable or not
    if (!isStableStill(modelnames, STEP1)){   //check #1
         return false;
    } else
    if (!isStableStill(modelnames, STEP2)){   //check #2
         return false;
    } else
    if (!isStableStill(modelnames, STEP3)){   //check #3
         return false;
    } else
    if (!isStableStill(modelnames, STEP4)){   //check #4
         return false;
    }
    else{
         return true;
    }
}

/* custom constructor to construct a SceneValidator object */
SceneValidator::SceneValidator(double GRAVITYx, double GRAVITYy, double GRAVITYz, double PLANEa, double PLANEb, double PLANEc, double PLANEd, double DEFAULT_SCALE){
  //initialize ODE and the simulation enviornment
  dInitODE2(0);
  world = dWorldCreate();
  space = dSimpleSpaceCreate(0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,GRAVITYx,GRAVITYy,GRAVITYz);
  dWorldSetCFM (world,1e-5);
  dCreatePlane (space,PLANEa,PLANEb,PLANEc,PLANEd);
  //initialize ODE's threading functions
  dAllocateODEDataForThread(dAllocateMaskAll);
  threading = dThreadingAllocateMultiThreadedImplementation();
  pool = dThreadingAllocateThreadPool(4, 0, dAllocateFlagBasicData, NULL);
  dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);
  dWorldSetStepThreadingImplementation(world, dThreadingImplementationGetFunctions(threading), threading);
  //scale the ALL objects to DEFAULT_SCALE
  for (int i =0; i < NUM; i++){
        scaling[i] = DEFAULT_SCALE;
  }
}


/* default constructor to construct a SceneValidator object */
SceneValidator::SceneValidator(){
  //initialize ODE and the simulation enviornment
  dInitODE2(0);
  world = dWorldCreate();
  space = dSimpleSpaceCreate(0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,GRAVITYx,GRAVITYy,GRAVITYz);
  dWorldSetCFM (world,1e-5);
  dCreatePlane (space,PLANEa,PLANEb,PLANEc,PLANEd);
  //initialize ODE's threading functions
  dAllocateODEDataForThread(dAllocateMaskAll);
  threading = dThreadingAllocateMultiThreadedImplementation();
  pool = dThreadingAllocateThreadPool(4, 0, dAllocateFlagBasicData, NULL);
  dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);
  dWorldSetStepThreadingImplementation(world, dThreadingImplementationGetFunctions(threading), threading);
  //scale the ALL objects to DEFAULT_SCALE
  for (int i =0; i < NUM; i++){
        scaling[i] = DEFAULT_SCALE;
  }
}

/* default destructor to destruct a SceneValidator object */
SceneValidator::~SceneValidator(){
  //shut down threading
  dThreadingImplementationShutdownProcessing(threading);
  dThreadingFreeThreadPool(pool);
  dWorldSetStepThreadingImplementation(world, NULL, NULL);
  dThreadingFreeImplementation(threading);
  //shut down simulation enviornment
  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();
}