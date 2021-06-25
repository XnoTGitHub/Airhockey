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

//===========================================================================

arr findObject(byteA& _rgb, floatA& _depth, int color_of_object){
  cv::Mat rgb = CV(_rgb);
  cv::Mat depth = CV(_depth);

  //convert to RGB -> BGR ->  HSV
  cv::Mat hsv, mask1, mask2, mask;
  cv::cvtColor(rgb, rgb, cv::COLOR_RGB2BGR);
  cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);

  if(color_of_object == BLUE){
    cout << "findBlue!" << endl;  
    //find red areas
    cv::inRange(hsv, cv::Scalar(  90, 120, 70), cv::Scalar( 140, 255, 255), mask);
  }
  else if(color_of_object == RED){
    cout << "findRed!" << endl;
    cv::inRange(hsv, cv::Scalar(  0, 120, 70), cv::Scalar( 10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
    cv::bitwise_or(mask1, mask2, mask);
  }
  else if(color_of_object == GREEN){
    cout << "findGreen!" << endl;
    cv::inRange(hsv, cv::Scalar(  30, 120, 70), cv::Scalar( 90, 255, 255), mask);

  }
  else{
    cout << "THIS COLOR IS NOT YET DEFINED: " << color_of_object << endl;
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
  arr ObjPos; //in image space
  cout << "Size: "<< contours.size() << endl;
  if (contours.size() >= 1){
    //draw largest (first) contour
    cv::drawContours(rgb, contours, 0, CV_RGB(255,255,255), 2);
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


//===========================================================================

void using_KOMO_for_PathPlanning(){
  byteA _rgb;
  floatA _depth;

  arr q;

  double f = 0.895;
  f *= 360.; //focal length is needed in pixels (height!), not meters!
  arr Fxypxy = {f, f, 320., 180.};

  //-- MODEL WORLD configuration, this is the data structure on which you represent
  rai::Configuration C;
  C.addFile("../scenarios/project_challenge.g");

  //delete frames with certain names
  for(uint o=1; o<30; o++){
    rai::Frame *f = C[STRING("obj"<<o)];
    if(f) delete f;
  }
  rai::Frame *puck = C["obj0"];
  puck->setColor({0,0,1.}); //set the color of one objet to blue
  puck->setShape(rai::ST_cylinder, {.02,0.05});
  puck->setPosition({0, 0.5, 1.});
  puck->setMass(0.1);
  puck->setContact(1); // What is this line really doing?

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

//////////////////////////////////////////////////////////////////////////////////////
  cout << "------------------komo2-------------------" << endl;
  KOMO komo2;                     //create a solver
  komo2.setModel(C, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  komo2.setTiming(4., NUMB_OF_STEPS, 5., 2);  //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
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
  cout << "objblue Pos: " << objblue->getPosition() << endl;
  arr Pos_Set_Hight = objblue->getPosition();
  arr Shift_Konst = Pos_Set_Hight-objred->getPosition();
  Pos_Set_Hight += Shift_Konst * .5;
  Pos_Set_Hight(2) = 1.05;
  objblue->setPosition(Pos_Set_Hight);
  cout << "SET_POS_HIGHT: " << Pos_Set_Hight << endl;
  arr A = {0.,0.,1.};
  komo2.addObjective({0.,4.}, FS_position, {"R_gripperCenter"}, OT_eq, A, {0.,0.,1.});
  //komo2.addObjective({1.}, FS_position, {"R_gripperCenter"}, OT_eq,arr(2,3 {1.,0,0, 0,1,0}), Pos_Set_Hight);
  komo2.addObjective({0.99}, FS_position, {"R_gripperCenter"}, OT_sos,{1e1}, Pos_Set_Hight);
  //Gerade Halten
  komo2.addObjective({1.,4.0}, FS_scalarProductXZ, {"R_gripperCenter","world"}, OT_eq);
  komo2.addObjective({1.,4.0}, FS_scalarProductYZ, {"R_gripperCenter","world"}, OT_eq);
  //komo2.addObjective({1.}, FS_position, {"R_gripperCenter"}, OT_eq, {1e1}, {1.,2.,0.}, 1);
  komo2.addObjective({4.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);
  //komo2.verbose = 1;
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

    S.step(q, tau, S._position);
  }
///////////////////////////////////////////////////////////////////////////
  cout << "------------------komo3-------------------" << endl;
  S.cameraview().addSensor("camera_green");
  cout << "getImageAndDepth: ";
  S.getImageAndDepth(_rgb, _depth); //we don't need images with 100Hz, rendering is slow
        //--<< perception pipeline
  arr blueObjRelPos_newside = findObject(_rgb, _depth, BLUE); //Blue Object's position in image space
  //rai::wait();
  if(!!blueObjRelPos_newside){
  	cout << "found something " << endl;
    depthData2point(blueObjRelPos_newside, Fxypxy);
    cout << "relPos: " << blueObjRelPos_newside << endl;
    objblue->setRelativePosition(blueObjRelPos_newside);
  }
  cout << "objblue Pos: " << objblue->getPosition() << endl;

  Pos_Set_Hight = objblue->getPosition();
  Pos_Set_Hight = puck->getPosition(); //------------------geschummelt!!
  //Shift_Konst = -Pos_Set_Hight+objred->getPosition();
  //Pos_Set_Hight += Shift_Konst * .5;
  Pos_Set_Hight(2) = 1.05;
  objblue->setPosition(Pos_Set_Hight);
  cout << "SET_POS_HIGHT: " << Pos_Set_Hight << endl;
  A = {0.,0.,1.};
  KOMO komo3;                     //create a solver
  komo3.setModel(C, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  komo3.setTiming(2., NUMB_OF_STEPS, 5., 2);  //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
  komo3.add_qControlObjective({}, 2, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

  komo3.addObjective({0.,2.}, FS_position, {"L_gripperCenter"}, OT_eq, A, {0.,0.,1.});
  //komo2.addObjective({1.}, FS_position, {"R_gripperCenter"}, OT_eq,arr(2,3 {1.,0,0, 0,1,0}), Pos_Set_Hight);
  komo3.addObjective({1.}, FS_position, {"L_gripperCenter"}, OT_sos,{1e1}, Pos_Set_Hight);
  //Gerade Halten
  komo3.addObjective({1.,2.}, FS_scalarProductXZ, {"L_gripperCenter","world"}, OT_eq);
  komo3.addObjective({1.,2.}, FS_scalarProductYZ, {"L_gripperCenter","world"}, OT_eq);
  //komo2.addObjective({1.}, FS_position, {"R_gripperCenter"}, OT_eq, {1e1}, {1.,2.,0.}, 1);
  komo3.addObjective({2.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);
  komo3.optimize();
  //komo3.view(true, "optimized motion");
  //rai::wait();
  for(uint t=0;t<komo3.T;t++){
  	//C.stepSwift();
    q = komo3.getConfiguration_qOrg(t);
    C.setJointState(q);
    C.watch(false, "optimized configuration");
    cout << t << endl;
    rai::wait(tau); //remove to go faster
    if(t%5==0){
    	S.getImageAndDepth(_rgb, _depth);
    }

    S.step(q, tau, S._position);
  }
///////////////////////////////////////////////////////////////////////////
  cout << "------------------LOOP-------------------" << endl;
  for (uint i=0;i<2;i++){
  	//REDs Turn
  	S.cameraview().addSensor("camera_red");
  	cout << "getImageAndDepth: ";

  	int found = 0;
  	while(!found){
  		cout << "wait ";
  		rai::wait(tau);
      S.step(q, tau, S._position);
		  //S.step();
  		S.getImageAndDepth(_rgb, _depth); 
  		arr blueObjRelPos_next = findObject(_rgb, _depth, BLUE);
  		cout << "blueObjRelPos_next: " << blueObjRelPos_next << endl;
  		if (!!blueObjRelPos_next && blueObjRelPos_next(1) > 120){
  			/*for(int i = 0; i<20;i++){
		  		cout << "still waiting ";
		  		rai::wait(0.01);
				S.step();
		  		S.getImageAndDepth(_rgb, _depth); 
		  		arr blueObjRelPos_next = findObject(_rgb, _depth, BLUE);
		  		cout << "blueObjRelPos_next: " << blueObjRelPos_next << endl;
  			}*/
  			found = 1;
  		}
  		//blueObjRelPos = blueObjRelPoss;
  		//cout << blueObjRelPoss << " "<< blueObjRelPos<<endl;
  	}
  	/*Pos_Set_Hight(0) = 0.45;
  	Pos_Set_Hight(1) = 1.0;
  	Pos_Set_Hight(2) = 1.05;
  	objblue->setPosition(Pos_Set_Hight);*/
  	cout << "finishLoop" << endl;
	  Pos_Set_Hight = puck->getPosition(); //------------------geschummelt!!
  	cout << "POS_SET_HIGHT : " << Pos_Set_Hight << endl;
  	S.step(q, tau, S._position);
  	arr Velo = (puck->getPosition()-Pos_Set_Hight)/0.07;
  	cout << "Velocity: " << Velo << endl;
  	arr Prediction = puck->getPosition() + Velo;
  	cout << "Prediction: " << Prediction << endl;
  	Pos_Set_Hight = Prediction;
  	Pos_Set_Hight(2) = 1.05;
  	objblue->setPosition(Pos_Set_Hight);
  	cout << "SET_POS_HIGHT: " << Pos_Set_Hight << endl;
  	//rai::wait();


  	KOMO komoLOOPRed;                     //create a solver
  	komoLOOPRed.setModel(C, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  	komoLOOPRed.setTiming(2., NUMB_OF_STEPS, 5., 2);  //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
  	komoLOOPRed.add_qControlObjective({}, 2, 1.); 

  	A = {0.,0.,1.};
  	komoLOOPRed.addObjective({0.,2.}, FS_position, {"R_gripperCenter"}, OT_eq, A, {0.,0.,1.});
    komoLOOPRed.addObjective({0.4}, FS_position, {"R_gripperCenter"}, OT_sos,{1e1}, Pos_Set_Hight+arr({1,3}, { 0,.15,0}));
   	//komo2.addObjective({1.}, FS_position, {"R_gripperCenter"}, OT_eq,arr(2,3 {1.,0,0, 0,1,0}), Pos_Set_Hight);
  	komoLOOPRed.addObjective({1.}, FS_position, {"R_gripperCenter"}, OT_sos,{1e1}, Pos_Set_Hight);
    komoLOOPRed.addObjective({.5,2.0}, FS_scalarProductXZ, {"R_gripperCenter","world"}, OT_eq);
  	komoLOOPRed.addObjective({.5,2.0}, FS_scalarProductYZ, {"R_gripperCenter","world"}, OT_eq);

  	komoLOOPRed.optimize();
  	arr old_Position = puck->getPosition();

  	for(uint t=0;t<komoLOOPRed.T;t++){
   		q = komoLOOPRed.getConfiguration_qOrg(t);
      C.setJointState(q);
      C.watch(false, "optimized configuration");
  		arr new_Position = puck->getPosition();
  		arr Velocity = (new_Position-old_Position)/tau;
    	cout << t << ": " << old_Position << " : " << new_Position << " Velocity: " << Velocity << endl;
    	if(t==19){
  			//rai::wait();
  		}
    	rai::wait(tau); //remove to go faster
    	old_Position = new_Position;
    	S.step(q, tau, S._position);
  	}
  	//rai::wait(); //TODO: otherwise the opengl gets hung up?

  	////////////////////////////////////////////////

  	//GREENs Turn
  	S.cameraview().addSensor("camera_green");
  	cout << "getImageAndDepth: ";
  	S.getImageAndDepth(_rgb, _depth); 
  	blueObjRelPos = findObject(_rgb, _depth, BLUE);
  	cout << "blueObjRelPos: " << blueObjRelPos << endl;
  	found = 0;
  	while(!found){
  		cout << "wait ";
  		rai::wait(tau);
		  S.step(q, tau, S._position);
  		S.getImageAndDepth(_rgb, _depth); 
  		arr blueObjRelPos_next = findObject(_rgb, _depth, BLUE);
  		cout << "blueObjRelPos_next: " << blueObjRelPos_next << endl;
  		if (!!blueObjRelPos_next && blueObjRelPos_next(1) > 120){
  			/*for(int i = 0; i<20;i++){
		  		cout << "still waiting ";
		  		rai::wait(0.01);
				S.step();
		  		S.getImageAndDepth(_rgb, _depth); 
		  		arr blueObjRelPos_next = findObject(_rgb, _depth, BLUE);
		  		cout << "blueObjRelPos_next: " << blueObjRelPos_next << endl;
  			}*/
  			found = 1;
  		}
  		//blueObjRelPos = blueObjRelPoss;
  		//cout << blueObjRelPoss << " "<< blueObjRelPos<<endl;
  	}
	cout << "finishLoop" << endl;
	Pos_Set_Hight = puck->getPosition(); //------------------geschummelt!!
  	cout << "POS_SET_HIGHT : " << Pos_Set_Hight << endl;
  	S.step(q, tau, S._position);
  	Velo = (puck->getPosition()-Pos_Set_Hight)/0.07;
  	cout << "Velocity: " << Velo << endl;
  	Prediction = puck->getPosition() + Velo;
  	cout << "Prediction: " << Prediction << endl;
  	Pos_Set_Hight = Prediction;
  	Pos_Set_Hight(2) = 1.05;
  	objblue->setPosition(Pos_Set_Hight);
  	rai::wait();
  	cout << "SET_POS_HIGHT: " << Pos_Set_Hight << endl;


  	KOMO komoLOOPGreen;                    
  	komoLOOPGreen.setModel(C, true);        
  	komoLOOPGreen.setTiming(2., NUMB_OF_STEPS, 5., 2);  //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
  	komoLOOPGreen.add_qControlObjective({}, 2, 1.); 
  	A = {0.,0.,1.};
  	komoLOOPGreen.addObjective({0.,2.}, FS_position, {"L_gripperCenter"}, OT_eq, A, {0.,0.,1.});
  	//komo2.addObjective({1.}, FS_position, {"R_gripperCenter"}, OT_eq,arr(2,3 {1.,0,0, 0,1,0}), Pos_Set_Hight);
  	komoLOOPGreen.addObjective({0.5}, FS_position, {"L_gripperCenter"}, OT_sos,{1e1}, Pos_Set_Hight-arr({1,3}, { 0,.15,0}));
  	komoLOOPGreen.addObjective({1.}, FS_position, {"L_gripperCenter"}, OT_sos,{1e1}, Pos_Set_Hight);
    komoLOOPGreen.addObjective({1.,2.0}, FS_scalarProductXZ, {"L_gripperCenter","world"}, OT_eq);
  	komoLOOPGreen.addObjective({1.,2.0}, FS_scalarProductYZ, {"L_gripperCenter","world"}, OT_eq);
  	komoLOOPGreen.optimize();
  	old_Position = puck->getPosition();

  	for(uint t=0;t<komoLOOPGreen.T;t++){
  		q = komoLOOPGreen.getConfiguration_qOrg(t);
      C.setJointState(q);
      C.watch(false, "optimized configuration");
  		arr new_Position = puck->getPosition();
  		arr Velocity = (new_Position-old_Position)/tau;
    	cout << t << ": " << old_Position << " : " << new_Position << " Velocity: " << Velocity <<  endl;
    	if(t==19){
  			rai::wait();
  		}
    	rai::wait(tau); //remove to go faster
    	old_Position = new_Position;
    	S.step(q, tau, S._position);
  	}
  	rai::wait(); //TODO: otherwise the opengl gets hung up?

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
