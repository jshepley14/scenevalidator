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


II.) Understanding the code

sceneValidator.cpp and sceneValidator.h are where the sceneValidator library is defined.  There are a variety of parameters one can set in the physics simulator, so please consult line 73 in sceneValidator.cpp to find out info on those. You can choose to graphically visualize what is going on in the simulator and this is the default in the examples folder.  To turn off the graphical rendering, just set the DRAW parameter to false.  You can print out a lot of info about a scene's simulation by setting the PRINT_xxx parameter to true.  Some known limitations are that models with > 100,000 vertices can behave abnormally at the current parameter settings (however some parametes can be adjusted to allow better collision interaction). Additionally, scaling the models is important.  One can customize the object's size in the setScale() function.  If you load an object, but don't see anything it is most likely because you need to scale the object up (or down).  The other files within src/svlibrary/src are files dedicated to parsing an object file's data.       
