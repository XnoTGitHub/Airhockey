#include <Perception/opencv.h> //always include this first! OpenCV headers define stupid macros
#include <Geo/depth2PointCloud.h>

#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/feature.h>
#include <Kin/simulation.h>
#include <Kin/viewer.h>
#include <KOMO/komo.h>

#define BLUE 1
#define RED 2
#define GREEN 3

using namespace std;
using namespace cv;

uint NUMB_OF_STEPS = 20;

//double f = 0.895;
//f *= 360.; //focal length is needed in pixels (height!), not meters!
arr Fxypxy = {322.2, 322.2, 320., 180.};

//===========================================================================

arr findObject(byteA& _rgb, floatA& _depth, int color_of_object){
  cv::Mat rgb = CV(_rgb);
  cv::Mat depth = CV(_depth);

  //convert to RGB -> BGR ->  HSV
  cv::Mat hsv, mask1, mask2, mask;
  cv::cvtColor(rgb, rgb, cv::COLOR_RGB2BGR);
  cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);

  if(color_of_object == BLUE){
    //cout << "findBlue!" << endl;  
    //find red areas
    cv::inRange(hsv, cv::Scalar(  90, 120, 70), cv::Scalar( 140, 255, 255), mask);
  }
  else if(color_of_object == RED){
    //cout << "findRed!" << endl;
    cv::inRange(hsv, cv::Scalar(  0, 120, 70), cv::Scalar( 10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
    cv::bitwise_or(mask1, mask2, mask);
  }
  else if(color_of_object == GREEN){
    //cout << "findGreen!" << endl;
    cv::inRange(hsv, cv::Scalar(  30, 120, 70), cv::Scalar( 90, 255, 255), mask);

  }
  else{
    cout << "THIS COLOR IS NOT YET DEFINED: " << color_of_object << endl;
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
  arr ObjPos; //in image space
  //cout << "Size: "<< contours.size() << endl;
  if (contours.size() >= 1){
    //draw largest (first) contour
    cv::drawContours(rgb, contours, 0, CV_RGB(255,255,255), 2);
    //find contour center
    cv::Moments M = cv::moments(contours[0]);
    //cout <<"M.m00: " << M.m00 << endl;

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
    cv::Scalar ObjDepth = cv::mean(depth, contourMask);
    //compute global coordinate via relative transoform
    cz = (double)ObjDepth(0);
    ObjPos = {cx, cy, cz};
    //cout << "ObjPos: " << ObjPos << endl;
  }else{
    ObjPos = NoArr;
  }
  return ObjPos;
}

void grab_Red(rai::Simulation& S,rai::Configuration& C, arr Pos_Set_Hight, double tau){

  KOMO komo;                     //create a solver
  komo.setModel(C, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  komo.setTiming(3., NUMB_OF_STEPS, 5., 2);  //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)

  //now add objectives!

  //the default control objective:
  komo.add_qControlObjective({}, 2, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

  //cout << "obj Pos: " << objred->getPosition() << endl;
  //arr Pos_Set_Hight = objred->getPosition();
  Pos_Set_Hight(2) = 1.;

  komo.addObjective({1.,2.}, FS_position, {"R_gripperCenter"}, OT_sos,{1e1}, Pos_Set_Hight);
  //Gerade Halten
  komo.addObjective({1.,3.}, FS_scalarProductXZ, {"R_gripperCenter","world"}, OT_eq);
  komo.addObjective({1.,3.}, FS_scalarProductYZ, {"R_gripperCenter","world"}, OT_eq);
  //Um den Pusher nicht gleich umzuhauen
  komo.addObjective({1.,2.}, FS_scalarProductXX, {"R_gripperCenter","world"}, OT_eq);

  //stand still at the end!
  komo.addObjective({2.,3.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);
  cout << "komo.verbose: " + komo.verbose << endl;

  komo.optimize();

  cout << komo.T << endl;

  for(uint t=0;t<komo.T;t++){
    arr q = komo.getConfiguration_qOrg(t);

    if(t==2*NUMB_OF_STEPS){
      S.closeGripper("R_gripper", .02, 2.6);
      cout << "closeGripper" <<endl;
    }

    cout << t << endl;
    rai::wait(tau); //remove to go faster
    S.step(q, tau, S._position);
  }
  //cout << "pos red: " << objred->getPosition() << endl;
}
void grab_Green(rai::Simulation& S,rai::Configuration& C, arr Pos_Set_Hight, double tau){

  KOMO komo;                     //create a solver
  komo.setModel(C, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  komo.setTiming(3., NUMB_OF_STEPS, 5., 2);  //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)

  //now add objectives!

  //the default control objective:
  komo.add_qControlObjective({}, 2, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

  //cout << "obj Pos: " << objred->getPosition() << endl;
  //arr Pos_Set_Hight = objred->getPosition();
  Pos_Set_Hight(2) = 1.;
  cout << "obj Pos: " << Pos_Set_Hight << endl;

  komo.addObjective({1.,3.}, FS_position, {"L_gripperCenter"}, OT_sos,{1e1}, Pos_Set_Hight);
  //Gerade Halten
  komo.addObjective({1.,3.}, FS_scalarProductXZ, {"L_gripperCenter","world"}, OT_eq);
  komo.addObjective({1.,3.}, FS_scalarProductYZ, {"L_gripperCenter","world"}, OT_eq);
  //Um den Pusher nicht gleich umzuhauen
  komo.addObjective({1.,2.}, FS_scalarProductXX, {"L_gripperCenter","world"}, OT_eq);

  //stand still at the end!
  komo.addObjective({3.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);
  cout << "komo.verbose: " + komo.verbose << endl;

  komo.optimize();

  cout << komo.T << endl;

  for(uint t=0;t<komo.T;t++){
    arr q = komo.getConfiguration_qOrg(t);

    if(t==2*NUMB_OF_STEPS){
      S.closeGripper("L_gripper", .02, 2.6);
      cout << "closeGripper" <<endl;

    }

    cout << t << endl;
    rai::wait(tau); //remove to go faster
    S.step(q, tau, S._position);
  }
  //cout << "pos red: " << objred->getPosition() << endl;
}
arr calc_veloctiy(rai::Simulation& S, rai::Frame* objblue, arr q){
  byteA _rgb;
  floatA _depth;

  S.getImageAndDepth(_rgb, _depth);
  arr blueObjRelPos = findObject(_rgb, _depth, BLUE); //redObject's position in image space
  if(!!blueObjRelPos){
    depthData2point(blueObjRelPos, Fxypxy);
    objblue->setRelativePosition(blueObjRelPos);
  }
  //rai::wait();
  arr Pos_Puck = objblue->getPosition();
  cout << "POS_Puck : " << Pos_Puck << endl;
  S.step(q, .05, S._position);
  S.step(q, .05, S._position);
  S.step(q, .05, S._position);
  S.step(q, .05, S._position);
  S.step(q, .05, S._position);
  S.step(q, .05, S._position);

  //rai::wait();
  objblue->setRelativePosition({0,0,0});
  S.getImageAndDepth(_rgb, _depth);
  arr blueObjRelPos_two = findObject(_rgb, _depth, BLUE); //redObject's position in image space
  cout << "relPos: " << blueObjRelPos_two << endl;
  if(!!blueObjRelPos_two){
    cout << "if" << endl;
    depthData2point(blueObjRelPos_two, Fxypxy);
    cout << "relPos: " << blueObjRelPos_two << endl;
    objblue->setRelativePosition(blueObjRelPos_two);
  }
  cout << "objblue->getPosition(): " << objblue->getPosition() << endl;
  arr Velo = (objblue->getPosition()-Pos_Puck)/0.15;
  
  //if(abs(Velo(1))>0.5)
    //Velo = Velo/1.4;
  return Velo;
}
float calc_time(arr Velo, arr Position){

  cout << "distance to collision: " << 0.25+Position(0)*Position(0) << endl;
  //one-step approximation
  float xpos = (0.25+Position(0)*Position(0))/Velo(1) * Velo(0) + Position(0);

  if(xpos < -0.48)
    xpos = -0.48;
  if(xpos > 0.48)
    xpos = 0.48;

  cout << "approx. of the x value: " << xpos << endl;
  cout << "distance to collision: " << 0.25+xpos*xpos << endl;

  /*if(abs(xpos)>0.4){
    return(0.25+(xpos*xpos))/Velo(1);
  }*/

  //////
  //New way to compute the time with the knowledge of x²+y²=r² and X_t = V*t+X_0

  float R = 0.5;

  arr V = -Velo/1.0;
  arr P = -Position;
  P(1) += -1.1;

  float a = V(0)*V(0) + V(1)*V(1);
  float b = 2*(P(0)*V(0)+P(1)*V(1));//1.1 is half of the length of the table
  float c = (P(0)*P(0))+(P(1)*P(1)) - R*R;

  //cout << "a: " << a << " b: " << b << " c: " << c << endl;
  //cout << "b*b: " << b*b << " 4*a*c: " << 4*a*c << endl;
  float t_new_1 = (-b + sqrt(b*b-4*a*c))/(2*a);
  float t_new_2 = (-b - sqrt(b*b-4*a*c))/(2*a);

  /*cout << "t_new_1: " << t_new_1 << endl;
  cout << "t_new_2: " << t_new_2 << endl;

  cout << "t_line: " << -P(1)/V(1) << endl;*/

  ///
  V = -Velo/1.0;
  P = -Position;
  P(1) += 1.1;

  a = V(0)*V(0) + V(1)*V(1);
  b = 2*(P(0)*V(0)+P(1)*V(1));//1.1 is half of the length of the table
  c = (P(0)*P(0))+(P(1)*P(1)) - R*R;

  cout << "a: " << a << " b: " << b << " c: " << c << endl;
  cout << "b*b: " << b*b << " 4*a*c: " << 4*a*c << endl;
  t_new_1 = (-b + sqrt(b*b-4*a*c))/(2*a);
  t_new_2 = (-b - sqrt(b*b-4*a*c))/(2*a);

  cout << "t_new_1: " << t_new_1 << endl;
  cout << "t_new_2: " << t_new_2 << endl;

  cout << "t_line: " << -P(1)/V(1) << endl;

  float time_of_colission = t_new_2;
  if(time_of_colission != time_of_colission)
    time_of_colission = -P(1)/V(1);

  cout << "time_of_colission: " << time_of_colission << endl;
    ///
  /*V = -Velo/1.0;
  P = Position;
  P(1) -= 1.1;

  a = V(0)*V(0) + V(1)*V(1);
  b = 2*(P(0)*V(0)+P(1)*V(1));//1.1 is half of the length of the table
  c = (P(0)*P(0))+(P(1)*P(1)) - R*R;

  cout << "a: " << a << " b: " << b << " c: " << c << endl;
  cout << "b*b: " << b*b << " 4*a*c: " << 4*a*c << endl;
  t_new_1 = (-b + sqrt(b*b-4*a*c))/(2*a);
  t_new_2 = (-b - sqrt(b*b-4*a*c))/(2*a);

  cout << "t_new_1: " << t_new_1 << endl;
  cout << "t_new_2: " << t_new_2 << endl;

  cout << "t_line: " << -P(1)/V(1) << endl;
    ///
  V = -Velo/1.0;
  P = Position;
  P(1) += 1.1;

  a = V(0)*V(0) + V(1)*V(1);
  b = 2*(P(0)*V(0)+P(1)*V(1));//1.1 is half of the length of the table
  c = (P(0)*P(0))+(P(1)*P(1)) - R*R;

  cout << "a: " << a << " b: " << b << " c: " << c << endl;
  cout << "b*b: " << b*b << " 4*a*c: " << 4*a*c << endl;
  t_new_1 = (-b + sqrt(b*b-4*a*c))/(2*a);
  t_new_2 = (-b - sqrt(b*b-4*a*c))/(2*a);

  cout << "t_new_1: " << t_new_1 << endl;
  cout << "t_new_2: " << t_new_2 << endl;

  cout << "t_line: " << -P(1)/V(1) << endl;*/

  //return (0.3+(xpos*xpos)/0.5)/Velo(1);

  return time_of_colission;
}
/*arr calc_veloctiy(rai::Simulation& S, rai::Frame* puck, arr q){

  arr Pos_Puck = puck->getPosition(); //------------------geschummelt!!
  cout << "POS_Puck : " << Pos_Puck << endl;
  S.step(q, 0.05, S._position);
  rai::wait(0.05);
  S.step(q, 0.05, S._position);
  rai::wait(0.05);

  arr Velo = (puck->getPosition()-Pos_Puck)/0.1;
  
  return Velo;
}*/
void subStep(rai::Simulation& S, const arr& q, double tau, double subSteps=100.) {
  arr dq = (q-S.get_q())/subSteps;
  for(uint s=0;s<subSteps;s++){
    S.step(S.get_q()+dq, tau/subSteps, S._position);
  }
}

//===========================================================================

void using_KOMO_for_PathPlanning(){
  byteA _rgb;
  floatA _depth;

  arr q;

  //-- MODEL WORLD configuration, this is the data structure on which you represent
  rai::Configuration C;
  //C.addFile("../scenarios/project_challenge.g");
  C.addFile("project_challenge.g");

 /* //delete frames with certain names
  for(uint o=1; o<30; o++){
    rai::Frame *f = C[STRING("obj"<<o)];
    if(f) delete f;
  }
  rai::Frame *puck = C["obj0"];*/
  rai::Frame *puck = C.addFrame("obj0");
  puck->setColor({0,0,1.}); //set the color of one objet to blue
  puck->setShape(rai::ST_cylinder, {.02,0.05});
  puck->setPosition({0, 0.5, 1.});
  puck->setMass(0.1);
  puck->setContact(1); // What is this line really doing?
  puck->addAttribute("friction", .01);
  puck->addAttribute("restitution", .01); 

  rai::Frame *pusher_red = C["stick_red"];
  pusher_red->setColor({1.,0,0}); //set the color of one objet to red!
  rai::Frame *pusher_green = C["stick_green"];
  pusher_green->setColor({0,1.,0}); //set the color of one objet to red!

  //rai::Frame *surf = C["surf"];
  //surf->setColor({1.,0,0}); //set the color of one objet to red!
  //surf->setContact(1);

  C.watch(true, "model world start state");

  rai::Simulation S(C, S._bullet, 1);
  S.cameraview().addSensor("camera_red");

  rai::Frame *objred = C.addFrame("object_red", "camera_red");
  objred->setColor({1.,0,0}); //set the color of one objet to red!
  objred->setShape(rai::ST_sphere, {.03});

  rai::Frame *objblue = C.addFrame("object_blue", "camera_red");
  objblue->setColor({0,0,1.}); //set the color of one objet to blue!
  objblue->setShape(rai::ST_sphere, {.03});

  rai::Frame *objgreen = C.addFrame("object_green", "camera_green");
  objgreen->setColor({0,1.,0}); //set the color of one objet to blue!
  objgreen->setShape(rai::ST_sphere, {.03});

  double tau = .05; //time step

////////////////////////////////////////////////////////////////////////////////////////////////
  
		//Grab the Pushers
  rai::wait();

  S.getImageAndDepth(_rgb, _depth); //we don't need images with 100Hz, rendering is slow

  arr redObjRelPos = findObject(_rgb, _depth, RED); //redObject's position in image space
  depthData2point(redObjRelPos, Fxypxy);
  cout << "relPos: " << redObjRelPos << endl;
  objred->setRelativePosition(redObjRelPos);

  grab_Red(S,C,objred->getPosition(), tau);


  S.cameraview().addSensor("camera_green");
  S.getImageAndDepth(_rgb, _depth);
  arr greenObjRelPos = findObject(_rgb, _depth, GREEN); //redObject's position in image space
  depthData2point(greenObjRelPos, Fxypxy);
  cout << "relPos: " << greenObjRelPos << endl;
  objgreen->setRelativePosition(greenObjRelPos);

  grab_Green(S,C,objgreen->getPosition(), tau);

  arr Start_Pos_RED = objred->getPosition();//-arr({1,3}, { 0,0.05,0}); is the overlengh 
  Start_Pos_RED(2) = 1.;
  arr Start_Pos_GREEN = objgreen->getPosition();
  Start_Pos_GREEN(2) = 1.;
  cout << "Start_Pos_RED: " << Start_Pos_RED << endl;
  //rai::wait();
//////////////////////////////////////////////////////////////////////////////////////
  cout << "------------------------------------------" << endl;
  cout << "-              * komo 2 *                -" << endl;
  cout << "------------------------------------------" << endl;
  KOMO komo2;                     //create a solver
  komo2.setModel(C, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  komo2.setTiming(3.8, NUMB_OF_STEPS, 5., 2);  //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
  komo2.add_qControlObjective({}, 2, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

  S.cameraview().addSensor("camera_red");
  cout << "getImageAndDepth: ";
  S.getImageAndDepth(_rgb, _depth); //we don't need images with 100Hz, rendering is slow
        //--<< perception pipeline
  arr blueObjRelPos = findObject(_rgb, _depth, BLUE); //redObject's position in image space
  if(!!blueObjRelPos){
    depthData2point(blueObjRelPos, Fxypxy);
    cout << "relPos: " << blueObjRelPos << endl;
    objblue->setRelativePosition(blueObjRelPos);
  }
  
  cout << "objblue Pos: " << objblue->getPosition()  << "puck Pos: " << puck->getPosition() << endl;
  arr Pos_Set_Hight = objblue->getPosition();
  arr Shift_Konst = Pos_Set_Hight-objred->getPosition();
  Pos_Set_Hight += Shift_Konst * 0.;
  Pos_Set_Hight(2) = 1.0;
  objblue->setPosition(Pos_Set_Hight);
  cout << "SET_POS_HIGHT: " << Pos_Set_Hight << endl;
  arr A = {0.,0.,1.};
  komo2.addObjective({0.,3.8}, FS_position, {"R_gripperCenter"}, OT_eq, A, {0.,0.,1.});
  //komo2.addObjective({1.}, FS_position, {"R_gripperCenter"}, OT_eq,arr(2,3 {1.,0,0, 0,1,0}), Pos_Set_Hight);
  komo2.addObjective({2}, FS_position, {"R_gripperCenter"}, OT_sos,{1e1}, Pos_Set_Hight);
  //Gerade Halten
  komo2.addObjective({1.,3.8}, FS_scalarProductXZ, {"R_gripperCenter","world"}, OT_eq);
  komo2.addObjective({1.,3.8}, FS_scalarProductYZ, {"R_gripperCenter","world"}, OT_eq);
  komo2.addObjective({2.-0.1, 2.+0.1}, FS_position, {"R_gripperCenter"}, OT_eq, 1e1*arr({2, 3}, {1, 0, 0, 0, 1, 0}) , {0, 0., 0}, 1);
  //komo2.addObjective({1.}, FS_position, {"R_gripperCenter"}, OT_eq, {1e1}, {1.,2.,0.}, 1);
  komo2.addObjective({3.8}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);
  //komo2.verbose = 1;y
  komo2.optimize();
  cout << "komo.verbose: " + komo2.verbose << endl;
  for(uint t=0;t<komo2.T;t++){
  	//C.stepSwift();
    q = komo2.getConfiguration_qOrg(t);
    C.setJointState(q);
    C.watch(false, "optimized configuration");
    cout << t << " " << puck->getPosition() << endl;
    rai::wait(tau); //remove to go faster
    if(t%5==0){
    	S.getImageAndDepth(_rgb, _depth);
    }
    //rai::wait(0.1);
    subStep(S,q,tau);
    //S.step(q, tau, S._position);
  }
///////////////////////////////////////////////////////////////////////////
  cout << "------------------------------------------" << endl;
  cout << "-              * komo 3 *                -" << endl;
  cout << "------------------------------------------" << endl;
  S.cameraview().addSensor("camera_green");
  cout << "getImageAndDepth: ";
  S.getImageAndDepth(_rgb, _depth);

  arr blueObjRelPos_newside = findObject(_rgb, _depth, BLUE); //Blue Object's position in image space
  //rai::wait();
  if(!!blueObjRelPos_newside){
  	cout << "found something " << endl;
    depthData2point(blueObjRelPos_newside, Fxypxy);
    cout << "relPos: " << blueObjRelPos_newside << endl;
    objblue->setRelativePosition(blueObjRelPos_newside);
  }
  cout << "objblue Pos: " << objblue->getPosition()  << "puck Pos: " << puck->getPosition() << endl;

  Pos_Set_Hight = -objblue->getPosition();
  Pos_Set_Hight(2) = 1.05;
  objblue->setPosition(Pos_Set_Hight);
  cout << "SET_POS_HIGHT: " << Pos_Set_Hight << endl;
  A = {0.,0.,1.};
  KOMO komo3;                     //create a solver
  komo3.setModel(C, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  komo3.setTiming(3., NUMB_OF_STEPS, 5., 2);  //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
  komo3.add_qControlObjective({}, 2, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

  komo3.addObjective({0.,2.}, FS_position, {"L_gripperCenter"}, OT_eq, A, {0.,0.,1.});
  //komo2.addObjective({1.}, FS_position, {"R_gripperCenter"}, OT_eq,arr(2,3 {1.,0,0, 0,1,0}), Pos_Set_Hight);
  komo3.addObjective({1.}, FS_position, {"L_gripperCenter"}, OT_sos,{1e1}, Pos_Set_Hight);
  //Gerade Halten
  komo3.addObjective({1.,2.}, FS_scalarProductXZ, {"L_gripperCenter","world"}, OT_eq);
  komo3.addObjective({1.,2.}, FS_scalarProductYZ, {"L_gripperCenter","world"}, OT_eq);
  //komo2.addObjective({1.}, FS_position, {"R_gripperCenter"}, OT_eq, {1e1}, {1.,2.,0.}, 1);
  //komo3.addObjective({2.}, FS_position, {"L_gripperCenter"}, OT_eq,{1e1}, Start_Pos_GREEN);
  komo3.addObjective({3.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);
  komo3.optimize();

  for(uint t=0;t<komo3.T;t++){
  	//C.stepSwift();
    q = komo3.getConfiguration_qOrg(t);
    C.setJointState(q);
    C.watch(false, "optimized configuration");
    cout << t << endl;
    rai::wait(tau); //remove to go faster
    if(t%5==0){
    	S.getImageAndDepth(_rgb, _depth);
      arr blueObjRelPos_next = findObject(_rgb, _depth, BLUE);
      cout << "blueObjRelPos_next: " << blueObjRelPos_next << endl;
      if (!blueObjRelPos_next){
        cout << "pos_y: not existing!" << endl;
        break;
      }
      cout << "pos_y: " << blueObjRelPos_next(1) << endl;
    }
    if(t==19){
      objblue->setPosition({0,0,0});
    }

    subStep(S, q, tau);
    //S.step(q, tau, S._position);
  }
///////////////////////////////////////////////////////////////////////////
  for (uint i=0;i<10;i++){
    cout << "------------------------------------------" << endl;
    cout << "-       * LOOP * * ROUND " << i << "               -" << endl;
    cout << "------------------------------------------" << endl;
  	//REDs Turn
  	S.cameraview().addSensor("camera_red");
  	cout << "getImageAndDepth: ";

  	int found = 0;
  	while(!found){
  		S.getImageAndDepth(_rgb, _depth); 
  		arr blueObjRelPos_next = findObject(_rgb, _depth, BLUE);
  		//cout << "blueObjRelPos_next: " << blueObjRelPos_next << endl;
  		if (!!blueObjRelPos_next && blueObjRelPos_next(1) > 40){
        depthData2point(blueObjRelPos_next, Fxypxy);
        objblue->setRelativePosition(blueObjRelPos_next);
  			found = 1;
        //rai::wait();
  		} else{
        cout << "wait ";
        rai::wait(0.05);
        S.step(q, 0.05, S._position);
      }
  	}
    Pos_Set_Hight = objblue->getPosition();
  	cout << "POS_SET_HIGHT : " << Pos_Set_Hight << endl;
    /*if(i==4){
        rai::wait();
    }*/
    arr Velo = calc_veloctiy(S,objblue,q);
  	cout << "Velocity: " << Velo << endl;;

    float time = calc_time(Velo,objblue->getPosition());
    cout << "time: " << time << endl;

    arr Shift = {Velo(0) * time,Velo(1) * time,Velo(2) * time};
  	arr Prediction = puck->getPosition() + Shift;
  	cout << "Prediction: " << Prediction << endl;
    if(Prediction(0) > 0.5){
      cout << "BANDE Left" << endl;
      Prediction = Start_Pos_RED + arr({1,3}, { 0.47 , 0. , 0. }); // full on the left boarder
      //time += 0.3; // timeshift cause of friction of the boarder
      cout << "Prediction changed to: " << Prediction << endl;
    }
    else if(Prediction(0) < -0.5){
      cout << "BANDE Right" << endl;
      Prediction = Start_Pos_RED - arr({1,3}, { 0.48 , 0. , 0. }); // full on the left boarder
      //time += 0.3; // timeshift cause of friction of the boarder
      cout << "Prediction changed to: " << Prediction << endl;
    }

  	Pos_Set_Hight = Prediction;
  	Pos_Set_Hight(2) = 1.0;
  	objblue->setPosition(Pos_Set_Hight);
  	cout << "SET_POS_HIGHT: " << Pos_Set_Hight << endl;
  	//rai::wait();
    if(i==4){
      Pos_Set_Hight(1) += -0.05;
      objblue->setPosition(Pos_Set_Hight);
      cout << "SET_POS_HIGHT: " << Pos_Set_Hight << endl;
    }

    arr gripper_red_velo = {0.08,0.08,0.06,0.08,0.08,0.06,0.035,0.04,0.065, 0.075};

  	KOMO komoLOOPRed;                     //create a solver
  	komoLOOPRed.setModel(C, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  	komoLOOPRed.setTiming(time+1, NUMB_OF_STEPS, 5., 2);  //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
  	komoLOOPRed.add_qControlObjective({}, 2, 1.); 

  	A = {0.,0.,1.};
  	komoLOOPRed.addObjective({0.,time+1}, FS_position, {"R_gripperCenter"}, OT_eq, A, {0.,0.,1.});
    komoLOOPRed.addObjective({time-0.3}, FS_position, {"R_gripperCenter"}, OT_sos,{1e1}, Pos_Set_Hight+arr({1,3}, { 0,.1,0}));
    //komoLOOPRed.addObjective({time-0.3}, FS_position, {"R_gripperCenter"}, OT_eq, 1e1*arr({2, 3}, {1, 0, 0, 0, 1, 0}), Pos_Set_Hight+arr({1,3}, { 0,.1,0})); 
    //komoLOOPRed.addObjective({time}, FS_position, {"R_gripperCenter"}, OT_eq, 1e1*arr({2, 3}, {1, 0, 0, 0, 1, 0}), Pos_Set_Hight-arr({1,3}, { 0,.1,0})); 
   	komoLOOPRed.addObjective({time}, FS_position, {"R_gripperCenter"}, OT_sos,{1e1}, Pos_Set_Hight-arr({1,3}, { 0,0.,0}));
    komoLOOPRed.addObjective({.5,time+1}, FS_scalarProductXZ, {"R_gripperCenter","world"}, OT_eq);
  	komoLOOPRed.addObjective({.5,time+1}, FS_scalarProductYZ, {"R_gripperCenter","world"}, OT_eq);
    //komoLOOPRed.addObjective({0.,time+1}, FS_position, {"R_gripperCenter"}, OT_ineq, {1e2}, {.55,.55,0.}, 1);
    komoLOOPRed.addObjective({time-0.1, time+0.1}, FS_position, {"R_gripperCenter"}, OT_eq, 1e1*arr({2, 3}, {1, 0, 0, 0, 1, 0}) , {0, -gripper_red_velo(i), 0}, 1);

    if(i >= 6){
      komoLOOPRed.addObjective({time,time+1}, FS_position, {"L_gripperCenter"}, OT_sos,{1e1}, Start_Pos_GREEN);
      komoLOOPRed.addObjective({}, FS_scalarProductXZ, {"L_gripperCenter","world"}, OT_eq);
      komoLOOPRed.addObjective({}, FS_scalarProductYZ, {"L_gripperCenter","world"}, OT_eq);
  	}
    komoLOOPRed.optimize();

  	for(uint t=0;t<komoLOOPRed.T;t++){
   		q = komoLOOPRed.getConfiguration_qOrg(t);
      C.setJointState(q);
      C.watch(false, "optimized configuration");

    	//cout << t << ": " << puck->getPosition() << " i: " << i << endl;
    	if(t==(uint)(time*NUMB_OF_STEPS-5) && i==7){
  			rai::wait();
  		}
      /*if(t%5==0){
        rai::wait();
        S.getImageAndDepth(_rgb, _depth);
        arr blueObjRelPos_next = findObject(_rgb, _depth, BLUE);
        cout << "blueObjRelPos_next: " << blueObjRelPos_next << endl;
        if (!blueObjRelPos_next || ((t > (uint)(time*NUMB_OF_STEPS)) && blueObjRelPos_next(1) > 230)){
          cout << "pos_y: not existing!" << endl;
          break;
        }
        cout << "pos_y: " << blueObjRelPos_next(1) << endl;
      }*/
    	rai::wait(tau); //remove to go faster
      subStep(S, q, tau);
    	//S.step(q, tau, S._position);
  	}
  	//rai::wait(); //TODO: otherwise the opengl gets hung up?

  	////////////////////////////////////////////////
    double tau_small = 0.01;
  	//GREENs Turn
  	S.cameraview().addSensor("camera_green");
    
    puck->setPosition({0,0,0});
  	found = 0;
  	while(!found){
  		S.getImageAndDepth(_rgb, _depth); 
  		arr blueObjRelPos_next = findObject(_rgb, _depth, BLUE);
  		//cout << "blueObjRelPos_next: " << blueObjRelPos_next << endl;
  		if (!!blueObjRelPos_next && blueObjRelPos_next(1) > 25){
  			found = 1;
        depthData2point(blueObjRelPos_next, Fxypxy);
        objblue->setRelativePosition(blueObjRelPos_next);
        cout << "objblue Pos: " << objblue->getPosition()  << "puck Pos: " << puck->getPosition() << endl;

        //rai::wait();
  		}else{
        cout << "wait ";
        rai::wait(tau_small);
        S.step(q, tau_small, S._position);
      }
  	}
    //Velo = calc_veloctiy(S,puck,q);
    Velo = calc_veloctiy(S,objblue,q);
  	cout << "Velocity: " << Velo << endl;
  	time = calc_time(Velo,objblue->getPosition());
    cout << "time: " << time << endl;
    Shift = {Velo(0) * time,Velo(1) * time,Velo(2) * time};
    Prediction = -objblue->getPosition() - Shift;
  	cout << "Prediction: " << Prediction << endl;
    if(Prediction(0) > 0.5){
      cout << "BANDE Left" << endl;
      Prediction = Start_Pos_GREEN + arr({1,3}, { 0.48 , 0. , 0. }); // full on the left boarder
      //time += 0.3; // timeshift cause of friction of the boarder
      cout << "Prediction changed to: " << Prediction << endl;
    }
    else if(Prediction(0) < -0.5){
      cout << "BANDE Right" << endl;
      Prediction = Start_Pos_GREEN - arr({1,3}, { 0.48 , 0. , 0. }); // full on the left boarder
      time += 0.3; // timeshift cause of friction of the boarder
      cout << "Prediction changed to: " << Prediction << endl;
    }
    if (i == 5)
      Prediction += -.02;
    Pos_Set_Hight = Prediction;
  	Pos_Set_Hight(2) = 1.0;
  	objblue->setPosition(Pos_Set_Hight); 
  	cout << "SET_POS_HIGHT: " << Pos_Set_Hight << endl;
    float komo_time = 2.;
    if(time > 1.)
      komo_time = time + 1.;

    arr gripper_velo = {0.03,0.035,0.04,0.07,0.03,0.04,0.08,0.055,0.05,0.03};
  	KOMO komoLOOPGreen;                    
  	komoLOOPGreen.setModel(C, true);        
  	komoLOOPGreen.setTiming(komo_time, NUMB_OF_STEPS, 5., 2); 
  	komoLOOPGreen.add_qControlObjective({}, 2, 1.); 
  	A = {0.,0.,1.};
  	komoLOOPGreen.addObjective({0.,komo_time}, FS_position, {"L_gripperCenter"}, OT_eq, A, {0.,0.,1.});
  	komoLOOPGreen.addObjective({time-0.1}, FS_position, {"L_gripperCenter"}, OT_sos,{1e1}, Pos_Set_Hight-arr({1,3}, { 0,0.2,0}));
  	komoLOOPGreen.addObjective({time}, FS_position, {"L_gripperCenter"}, OT_sos,{1e1}, Pos_Set_Hight+arr({1,3}, { 0,0.,0}));
    komoLOOPGreen.addObjective({0.5,komo_time}, FS_scalarProductXZ, {"L_gripperCenter","world"}, OT_eq);
  	komoLOOPGreen.addObjective({0.5,komo_time}, FS_scalarProductYZ, {"L_gripperCenter","world"}, OT_eq);
    komoLOOPGreen.addObjective({time-0.1, time+0.1}, FS_position, {"L_gripperCenter"}, OT_eq, 1e1*arr({2, 3}, {1, 0, 0, 0, 1, 0}) , {0, gripper_velo(i), 0}, 1);
  	
    komoLOOPGreen.addObjective({time,komo_time}, FS_position, {"R_gripperCenter"}, OT_sos,{1e1}, Start_Pos_RED);
    komoLOOPGreen.addObjective({}, FS_scalarProductXZ, {"R_gripperCenter","world"}, OT_eq);
    komoLOOPGreen.addObjective({}, FS_scalarProductYZ, {"R_gripperCenter","world"}, OT_eq);

    komoLOOPGreen.optimize();
    cout << "Start_Pos_RED: " << Start_Pos_RED << endl;
    //rai::wait();
  	for(uint t=0;t<komoLOOPGreen.T;t++){
  		q = komoLOOPGreen.getConfiguration_qOrg(t);
      C.setJointState(q);
      C.watch(false, "optimized configuration");
  		
    	//cout << t << ": " << puck->getPosition() << " i: " << i << endl;
    	if(t==(uint)((komo_time-1)*NUMB_OF_STEPS) && i==7){
  			rai::wait();
  		}
    	rai::wait(tau_small); //remove to go faster
      subStep(S, q, tau);
  	}
  	//rai::wait(); //TODO: otherwise the opengl gets hung up?

  }
///////////////////////////////////////////////////////////////////////////
  KOMO komoEND;                     //create a solver
  komoEND.setModel(C, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  komoEND.setTiming(3., NUMB_OF_STEPS, 5., 2);  //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
  
  komoEND.addObjective({0.,3.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);
  
  komoEND.optimize();

  for(uint t=0;t<komoEND.T;t++){
  	q = komoEND.getConfiguration_qOrg(t);
    cout << t << endl;
    rai::wait(tau); //remove to go faster
    S.step(q, tau, S._position);
  }
  rai::wait(); //TODO: otherwise the opengl gets hung up?
}
int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  using_KOMO_for_PathPlanning();

  return 0;
}
