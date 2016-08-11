# scenevalidator
catkin package


I.)  INSTALL instructions:

1.) Install libraries which SceneValidator depends on ODE(comes with Drawstuff), Eigen, OpenGl. (this assumes you already have ROS)

To install ODE follow this guide: http://www.cs.cmu.edu/~cga/ode/ 

To install Eigen, follow this guide: http://eigen.tuxfamily.org/index.php?title=Main_Page

To install OpenGl look online or try this: http://kiwwito.com/installing-opengl-glut-libraries-in-ubuntu/

2.) 

In catkin workspace:  catkin_make

In ../devel/lib/scenevalidator:  ./singleModel

You can also try the demos build_tower, testParams, and load3twice


RESOURCES

ODE API: http://opende.sourceforge.net/docs/group__collide__sphere.html#ga25d47698e042a909be1fb9ffd81d0786

ODE Manual: http://ode-wiki.org/wiki/index.php?title=Manual:_All&printable=yes

ODE guide and demos: http://demura.net/english


II.) Understanding the code and loading models

sceneValidator.cpp and sceneValidator.h are where the sceneValidator library is defined.  There are a variety of parameters one can set in the physics simulator, so please consult line 73 in sceneValidator.cpp to find out info on those. You can choose to graphically visualize what is going on in the simulator and this is the default in the examples folder.  To turn off the graphical rendering, just set the DRAW parameter to false.  You can print out a lot of info about a scene's simulation by setting the PRINTxxx parameter to true.  Some known limitations are that models with > 100,000 vertices can behave abnormally at the current parameter settings (however some parameters can be adjusted to allow better collision interaction). Additionally, scaling the models is important.  One can customize the object's size in the setScale() function.  For models in Imperial College' s data set, to scale one object, you would do setScale(0, 0.01), but in sbpl_perception's data set you would do setScale(0,100).  The difference is a factor of 10,000 in terms of scale.  That's because each model's data in the .obj file can be represented with large or smaller numbers so that's why scaling is important. Useing meshLab software can be helpful for reducing the number of vertices of an object.  Follow the video here for instructions. https://www.youtube.com/watch?v=w_r-cT2jngk   Some 3Dmodel scans may have holes in theobject and it may be advantageous to close thos holes too.  If you load an object, but don't see anything it is most likely because you need to scale the object up (or down).  The other files within src/svlibrary/src are files dedicated to parsing an object file's data.  You'll also find a textures folder and that contains texture files wich the drawstuff library relies on when drawing a scene.  

 In testParams.cpp,a window opens showing a scene including a falling wine glass model. Then closes in around 0.5 sec. This is because the scene was considered not in static equilibrium.  However if you wish to see the full unfolding of certain events even in a scene which is not in static equilibrium, then set CHECK1 to 1000 and the window will continue showing itself.  
 
III.) Abstract for this project
