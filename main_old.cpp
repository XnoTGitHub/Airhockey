#include <Perception/opencv.h> //always include this first! OpenCV headers define stupid macros
#include <Geo/depth2PointCloud.h>
#include <KOMO/komo.h>

#include <Kin/feature.h>
#include <Kin/frame.h>
#include <Kin/simulation.h>
#include <Kin/viewer.h>

using namespace std;
using namespace cv;

//===========================================================================

arr findBlueObject(byteA& _rgb, floatA& _depth){
  cv::Mat rgb = CV(_rgb);
  cv::Mat depth = CV(_depth);
  //convert to BGR -> RGB -> HSV
  cv::Mat hsv, mask1, mask2, mask;
  //cv::cvtColor(rgb, rgb, cv::COLOR_RGB2BGR);
  cv::cvtColor(rgb, hsv, cv::COLOR_RGB2HSV);
  //find red areas
  cv::inRange(hsv, cv::Scalar(  0, 120, 70), cv::Scalar( 10, 255, 255), mask1);
  cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
  cv::bitwise_or(mask1, mask2, mask);
  //find contour
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
  arr redObjPos; //in image space
  if (contours.size() >= 1){
    //draw largest (first) contour
    cv::drawContours(rgb, contours, 0, CV_RGB(0,255,0), 2);
    //find contour center
    cv::Moments M = cv::moments(contours[0]);
    cout <<"M.m00: " << M.m00 << endl;

    double cx = 0;
    double cy = 0;
    double cz = 0;

    if(M.m00 != 0){
      cx = M.m10/M.m00;
      cy = M.m01/M.m00;
    }
    //compute average depth
    cv::Mat contourMask = cv::Mat::zeros(depth.size(), CV_8UC1);
    cv::drawContours(contourMask, contours, 0, cv::Scalar(1), CV_FILLED);
    cv::Scalar redDepth = cv::mean(depth, contourMask);
    //compute global coordinate via relative transoform
    cz = (double)redDepth(0);
    redObjPos = {cx, cy, cz};
  }else{
    redObjPos = NoArr;
  }
  return redObjPos;
}

arr findRedObject(byteA& _rgb, floatA& _depth){
  cv::Mat rgb = CV(_rgb);
  cv::Mat depth = CV(_depth);
  //convert to BGR -> RGB -> HSV
  cv::Mat hsv, mask1, mask2, mask;
  cv::cvtColor(rgb, rgb, cv::COLOR_RGB2BGR);
  cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);
  //find red areas
  cv::inRange(hsv, cv::Scalar(  0, 120, 70), cv::Scalar( 10, 255, 255), mask1);
  cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
  cv::bitwise_or(mask1, mask2, mask);
  //find contour
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
  arr redObjPos; //in image space
  if (contours.size() >= 1){
    //draw largest (first) contour
    cv::drawContours(rgb, contours, 0, CV_RGB(0,255,0), 2);
    //find contour center
    cv::Moments M = cv::moments(contours[0]);
    cout <<"M.m00: " << M.m00 << endl;

    double cx = 0;
    double cy = 0;
    double cz = 0;

    if(M.m00 != 0){
      cx = M.m10/M.m00;
      cy = M.m01/M.m00;
    }
    //compute average depth
    cv::Mat contourMask = cv::Mat::zeros(depth.size(), CV_8UC1);
    cv::drawContours(contourMask, contours, 0, cv::Scalar(1), CV_FILLED);
    cv::Scalar redDepth = cv::mean(depth, contourMask);
    //compute global coordinate via relative transoform
    cz = (double)redDepth(0);
    redObjPos = {cx, cy, cz};
  }else{
    redObjPos = NoArr;
  }
  return redObjPos;
}
/*
void grasp_the_hopping_ball(){
   //-- setup your RealWorld
  rai::Configuration RealWorld;
  RealWorld.addFile("../scenarios/project_challenge.g");

  KOMO komo;
  komo.setModel(RealWorld, false);
  komo.setTiming(1, 20, 5., 2);
  //delete frames with certain names
  for(uint o=1; o<30; o++){
    rai::Frame *f = RealWorld[STRING("obj"<<o)];
    if(f) delete f;
  }
  rai::Frame *realObj = RealWorld["obj0"];
  realObj->setColor({0,0,1.}); //set the color of one objet to blue
  realObj->setShape(rai::ST_cylinder, {.01,0.05});
//  realObj->setShape(rai::ST_ssBox, {.05, .05, .2, .01});
  realObj->setPosition({.5, 0.85, 2.});
  realObj->setContact(1);
  rai::Frame *pusher = RealWorld["strick"];
  pusher->setColor({1.,0,0}); //set the color of one objet to red!
  //pusher->setPosition({.0, .4, 2.});

  rai::Simulation S(RealWorld, S._bullet, 1);
  S.cameraview().addSensor("camera");
  //add an imp!!
  S.addImp(S._objectImpulses, {"obj0"}, {});
  //-- setup your model world
  rai::Configuration C;
  C.addFile("../scenarios/stadium.g");
  rai::Frame *obj = C.addFrame("object", "camera");
  obj->setColor({1.,0,0}); //set the color of one objet to red!
  obj->setShape(rai::ST_sphere, {.02});

  rai::Frame *disk = C.addFrame("disk", "camera");
  disk->setColor({0,0,1}); //set the color of one objet to red!
  disk->setShape(rai::ST_cylinder, {.01,0.05});
  //obj->setShape(rai::ST_cylinder, {.1, 0.02});
  C.watch(false, "model world start state");
  //set the intrinsic camera parameters
  double f = 0.895;
  f *= 360.; //focal length is needed in pixels (height!), not meters!
  arr Fxypxy = {f, f, 320., 180.};
  //get a reference to the camera frame
  rai::Frame *cameraFrame = C["camera"];
  //initial joint
  arr qInit = C.getJointState();
  //-- the following is the simulation loop
  arr q;
  byteA _rgb;
  floatA _depth;
  double tau = .005; //time step
  bool gripping = false;
  bool grasped = false;
  bool opening = false;
  for(uint t=0;t<200000;t++){
    rai::wait(tau); //remove to go faster
    //grab sensor readings from the simulation
    q = S.get_q();
    C.setJointState(q); //set your robot model to match the real q
    if(!(t%10)){
      S.getImageAndDepth(_rgb, _depth); //we don't need images with 100Hz, rendering is slow
      //--<< perception pipeline
      arr redObjRelPos = findRedObject(_rgb, _depth); //redObject's position in image space
      arr blueObjRelPos = findBlueObject(_rgb, _depth); //redObject's position in image space

      if(!!redObjRelPos && t > 100 ){
        depthData2point(redObjRelPos, Fxypxy);
        cout << "relPos: " << redObjRelPos << endl;
        obj->setRelativePosition(redObjRelPos);
      }
      if(!!blueObjRelPos && t > 200 ){
        depthData2point(blueObjRelPos, Fxypxy);
        disk->setRelativePosition(blueObjRelPos);
        disk->setPosition(disk->getPosition() + ARR(0,0,0.1));
        cout <<"pos disk: " << disk->getPosition() << endl;
      }

    }
    C.watch();
    arr vel;
    if(opening){
      vel = 0.8*(qInit-q);
      if(S.getGripperIsOpen("R_gripper")){
        cout <<"OPENED!" <<endl;
        grasped = false;
        opening = false;
      }
    }else if(!grasped){
      cout <<"grasped == false" <<endl;
      C.watch();
      //some good old fashioned IK
      auto diff = C.feature(FS_positionDiff, {"R_gripperCenter", "object"})->eval(C);
      /*auto vecX = C.feature(FS_scalarProductXZ, {"R_gripperCenter", "world"})->eval(C);
      auto vecZ = C.feature(FS_scalarProductZZ, {"R_gripperCenter", "world"})->eval(C);
      cout << "vecZ: "<< vecZ << endl;
      //stack them
      arr y, J;
      y.append(1e0*diff.y); //multiply, to make faster
      J.append(1e0*diff.J);
      y.append(vecX.y-arr{0.}); //subtract target, here scalarProduct=0
      J.append(vecX.J);
      y.append(vecZ.y-arr{1.0}); //subtract target, here scalarProduct=0.1
      J.append(vecZ.J);
      vel = 10.* pseudoInverse(J, NoArr, 1e-2) * (-y);
      komo.add_qControlObjective({}, 2, 1.);
      komo.addObjective({}, FS_positionDiff, {"R_gripperCenter", "strick"}, OT_sos);
      komo.optimize();
      komo.checkGradients(); //this checks all gradients of the problem by finite difference
      komo.getReport(true); //true -> plot the cost curves
      cout << !opening << " , " << length(diff.y) << endl;

      if(!opening && !gripping && length(diff.y) < .025){
        cout << "START CLOSING" << endl;
        S.closeGripper("R_gripper", .02, 2.6); //what does the numbers mean?
        gripping = true;
      }
      if(gripping && S.getGripperIsGrasping("R_gripper")){
        cout <<"GRASPED!" <<endl;
        gripping = false;
        grasped = true;
      }else if(gripping && S.getGripperIsClose("R_gripper")){
        cout <<"GRASPING FAILED!" <<endl;
        S.openGripper("R_gripper");
        gripping = false;
        opening = true;
      }
    }else{
      /*auto ee = C.feature(FS_position, {"R_gripperCenter"})->eval(C);
      cout << "ee.y(2) " << ee.y(2) << endl;
      ee.y(2) = 1.1;
      vel = pseudoInverse(ee.J, NoArr, 1e-2) * ARR(0.3,-0.2,0.05);
      //C.watch();
      komo.addObjective({}, FS_positionDiff, {"R_gripperCenter", "L_gripperCenter"}, OT_eq);
      komo.optimize();
      komo.checkGradients(); //this checks all gradients of the problem by finite difference
      komo.getReport(true); //true -> plot the cost curves
//  for(uint i=0;i<2;i++) komo.displayTrajectory(.1, true); //play the trajectory
      //rai::ConfigurationViewer V;
      //V.setConfiguration(komo.pathConfig, "optimized motion", true);
      /*auto diff = C.feature(FS_positionDiff, {"R_gripperCenter", "disk"})->eval(C);
      //auto vecX = C.feature(FS_scalarProductXZ, {"R_gripperCenter", "world"})->eval(C);
      auto vecZ = C.feature(FS_scalarProductZZ, {"R_gripperCenter", "world"})->eval(C);

      cout << "diff " << diff.y << endl;
      //cout << "vecZ: "<< vecZ << endl;
      //stack them
      arr y, J;
      y.append(1e0*diff.y); //multiply, to make faster
      J.append(1e0*diff.J);
      //y.append(vecX.y-arr{0.}); //subtract target, here scalarProduct=0
      //J.append(vecX.J);
      y.append(vecZ.y-arr{1.}); //subtract target, here scalarProduct=0.1
      J.append(vecZ.J);
      vel = 10.* pseudoInverse(J, NoArr, 1e-2) * (-y);*/
      /*if(ee.y(2) > 1.2){
        S.openGripper("R_gripper");
        opening = true;
      }
    }
    //send control to the simulation
    S.step(vel, tau, S._velocity);
  }
  rai::wait();
}

//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  grasp_the_hopping_ball();

  return 0;
}
*/
#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <KOMO/komo.h>
#include <Kin/TM_default.h>
#include <Optim/opt-nlopt.h>
#include <Kin/viewer.h>

//===========================================================================

void tutorialBasics(){
  rai::Configuration RealWorld("../scenarios/project_challenge.g");

  KOMO komo;
  /* there are essentially three things that KOMO needs to be specified:
   * 1) the kinematic model
   * 2) the timing parameters (duration/phases, number os time slices per phase)
   * 3) the tasks */
//  komo.solver = rai::KS_sparse; //set in rai.cfg!

  for(uint o=1; o<30; o++){
    rai::Frame *f = RealWorld[STRING("obj"<<o)];
    if(f) delete f;
  }
  rai::Frame *realObj = RealWorld["obj0"];
  realObj->setColor({0,0,1.}); //set the color of one objet to blue
  realObj->setShape(rai::ST_cylinder, {.01,0.05});
//  realObj->setShape(rai::ST_ssBox, {.05, .05, .2, .01});
  realObj->setPosition({.5, 0.85, 1.});
  realObj->setContact(1);

  rai::Simulation S(RealWorld, S._bullet, 1);
  S.cameraview().addSensor("camera");
  //add an imp!!
  S.addImp(S._objectImpulses, {"obj0"}, {});

    //-- setting the model; false -> NOT calling collision detection (SWIFT) -> faster
  komo.setModel(RealWorld, false);

  //-- the timing parameters: 2 phases, 20 time slices, 5 seconds, k=2 (acceleration mode)
  komo.setTiming(1, 20, 5., 2);

  //-- default tasks for transition costs
  komo.add_qControlObjective({}, 2, 1.);

//  komo.addSquaredQuaternionNorms(-1., -1., 1e1); //when the kinematics includes quaternion joints, keep them roughly regularized

  //-- simple tasks, called low-level

  //in phase-time [1,\infty] position-difference between "endeff" and "target" shall be zero (eq objective)
  komo.addObjective({1.,-1.}, FS_positionDiff, {"R_gripperCenter", "obj0"}, OT_eq, {1e0});

  //in phase-time [1,\infty] quaternion-difference between "endeff" and "target" shall be zero (eq objective)
//  komo.addObjective({1., -1.}, FS_quaternionDiff, {"endeff", "target"}, OT_eq, {1e1});
  //I don't aleays recommend setting quaternion tasks! This is only for testing here. As an alternative, one can use alignment tasks as in test/KOMO/komo

  //slow down around phase-time 1. (not measured in seconds, but phase)
//  komo.setSlow(1., -1., 1e1);

  //-- call the optimizer
//  komo.animateOptimization = 1;
  komo.optimize();
    komo.checkGradients(); //this checks all gradients of the problem by finite difference
  komo.getReport(true); //true -> plot the cost curves
//  for(uint i=0;i<2;i++) komo.displayTrajectory(.1, true); //play the trajectory
  rai::ConfigurationViewer V;
  V.setConfiguration(komo.pathConfig, "optimized motion", true);

  /* next step:
   *
   * Have a look at all the other set*** methods, which all add tasks to the KOMO problem. Look into
   * their implementation: they mainly just call setTask(..) with various TaskMaps.
   *
   * The last three arguments of setTask are important:
   *
   * type allows to define whether this is a sumOfSqr, equality, or inequality task
   *
   * target defines the target value in the task space; {} is interpreted as the zero vector
   *
   * order=0 means that the task is about the position(absolute value) in task space
   * order=1 means that the task is about the velocity in task space
   * order=2 means that the task is about the acceleration in task space
   *
   * For instance, setSquaredQAccelerations sets a tasks about the acceleration in the identity map
   *
   * Next, perhaps learn about all the available taskMaps, or implement new differentiable MappingSuccess
   *
   */
}

//===========================================================================


int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  tutorialBasics();

  //tutorialInverseKinematics();

  return 0;
}
