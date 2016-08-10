# scenevalidator
catkin package


INSTALL instructions:

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
