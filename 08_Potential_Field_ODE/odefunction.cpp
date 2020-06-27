#include "odefunction.h"
#include "potentialfield.h"

// select correct drawing functions
#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawCylinder dsDrawCylinderD
#endif

#define NUM 4

dWorldID world;
dSpaceID space;
dGeomID  ground;
dJointGroupID contactgroup;
dsFunctions fn;
int         controlFlag = false;

// 포텐셜 필드 맵
forceVec** odePotentialMap;

// 로봇 제어
#define MIN_VEL 0.1
#define MAX_VEL 1.5
dReal Kp = 0.015;   // 조향제어 게인     // 0.006
dReal Psi = 0.003;  // 전후진제어 게인    //0.002
dReal W;    // 조향제어 아웃풋
dReal V;    // 전후진제어 아웃풋
dReal Qd;   // 목표 회전 각도
dReal Q;    // 현재 회전 각도

// 로봇 위치
dReal Xpos, Ypos, Zpos;

// 로봇 회전
dReal Rotation;

KSensorLidar2D	_oLidar2D; //BeamSensor를 저장하는 Hash Map

typedef struct {
  dBodyID body;
  dGeomID geom;
  double  l,r,m;
} MyObject;

dReal   sim_step = 0.05;

MyObject wheel[NUM], base, neck, camera, ball;
dJointID joint[NUM], neck_joint, camera_joint;
const dReal  BALL_R    = 0.15;
const dReal  BALL_P[3] = {1.0, 1.0, 0.15};
dReal w_r = 0.0, w_l = 0.0 ;                   // 좌우 바퀴 속도
dReal BASE_M    = 9.4;                        // 베이스의 질량
dReal BASE_S[3] = {0.4,0.3,0.2};              // 베이스 크기

const dReal  CAMERA_M  = 0.2;
const dReal  CAMERA_S[3]  = {0.2,0.2,0.05};
const dReal  NECK_M    = 0.5;
const dReal  NECK_L    = 0.2;
const dReal  NECK_R    = 0.025;

dReal WL;   //바퀴 축 길이
dReal WH_R0 = 0.05, WH_R1 = 0.10;            // 바퀴 반경
dReal WH_W  = 0.02, WH_M0 = 0.1, WH_M1 = 0.2; // 바퀴 폭, 질량
dReal START_X = 0, START_Y = 0, START_Z=0.20; // 초기위치

dReal s_x = 0.0, s_y = 0.0, s_t = 0;     // deadreckoning

// 장애물 변수
#define GRIDSIZE 0.05
#define MAPSIZE  GRIDSIZE * 200
#define XINIT GRIDSIZE*100
#define YINIT GRIDSIZE*100

dJointID ObsJointID[11]; // 장애물의 JOINT ID

MyObject    obBarrier[4], obBox[5], obCylinder[2];          // obstacle map
dReal       Obstacle_M    = 1000000;                        // 베이스의 질량

// obstacle size
dReal       ObCy0_R(GRIDSIZE * 15), ObCy0_L(GRIDSIZE * 15), ObCy1_R(GRIDSIZE * 15), ObCy1_L(GRIDSIZE * 15);

dReal       ObBoxSize0[3]       = {GRIDSIZE * 20, GRIDSIZE * 30, GRIDSIZE * 10};
dReal       ObBoxSize1[3]       = {GRIDSIZE * 50, GRIDSIZE * 30, GRIDSIZE * 10};
dReal       ObBoxSize2[3]       = {GRIDSIZE * 20, GRIDSIZE * 80, GRIDSIZE * 10};
dReal       ObBoxSize3[3]       = {GRIDSIZE * 20, GRIDSIZE * 20, GRIDSIZE * 10};
dReal       ObBoxSize4[3]       = {GRIDSIZE * 20, GRIDSIZE * 30, GRIDSIZE * 10};

dReal       ObBarrierSize0[3]   = {GRIDSIZE * 10, GRIDSIZE * 190, GRIDSIZE * 10};
dReal       ObBarrierSize1[3]   = {GRIDSIZE * 190, GRIDSIZE * 10, GRIDSIZE * 10};
dReal       ObBarrierSize2[3]   = {GRIDSIZE * 10, GRIDSIZE * 190, GRIDSIZE * 10};
dReal       ObBarrierSize3[3]   = {GRIDSIZE * 190, GRIDSIZE * 10, GRIDSIZE * 10};

// obstacle pos
dReal       ObCyPos0[3]         = {GRIDSIZE * 30 + ObCy0_R, GRIDSIZE * 40 + ObCy0_R, ObCy0_L/2.};
dReal       ObCyPos1[3]         = {GRIDSIZE * 120 + ObCy1_R, GRIDSIZE * 30 + ObCy1_R, ObCy1_L/2.};

dReal       ObBoxPos0[3]        = {GRIDSIZE * 30 + ObBoxSize0[0]/2., GRIDSIZE * 140 + ObBoxSize0[1]/2., ObBoxSize0[2]/2.};
dReal       ObBoxPos1[3]        = {GRIDSIZE * 120 + ObBoxSize1[0]/2., GRIDSIZE * 140 + ObBoxSize1[1]/2., ObBoxSize1[2]/2.};
dReal       ObBoxPos2[3]        = {GRIDSIZE * 70 + ObBoxSize2[0]/2., GRIDSIZE * 70 + ObBoxSize2[1]/2., ObBoxSize2[2]/2.};
dReal       ObBoxPos3[3]        = {GRIDSIZE * 140 + ObBoxSize3[0]/2., GRIDSIZE * 100 + ObBoxSize3[1]/2., ObBoxSize3[2]/2.};
dReal       ObBoxPos4[3]        = {GRIDSIZE * 150 + ObBoxSize4[0]/2., GRIDSIZE * 70 + ObBoxSize4[1]/2., ObBoxSize4[2]/2.};

dReal       ObBarrierPos0[3]    = {GRIDSIZE * 30 + ObBarrierPos0[0]/2., GRIDSIZE * 140 + ObBarrierPos0[1]/2., ObBarrierSize0[2]/2.};
dReal       ObBarrierPos1[3]    = {GRIDSIZE * 30 + ObBarrierPos1[0]/2., GRIDSIZE * 140 + ObBarrierPos1[1]/2., ObBarrierSize1[2]/2.};
dReal       ObBarrierPos2[3]    = {GRIDSIZE * 30 + ObBarrierPos2[0]/2., GRIDSIZE * 140 + ObBarrierPos2[1]/2., ObBarrierSize2[2]/2.};
dReal       ObBarrierPos3[3]    = {GRIDSIZE * 30 + ObBarrierPos3[0]/2., GRIDSIZE * 140 + ObBarrierPos3[1]/2., ObBarrierSize3[2]/2.};

void makeObstacle(){

    dMass mass;

    //make barrier
    obBarrier[0].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,Obstacle_M, ObBarrierSize0[0], ObBarrierSize0[1], ObBarrierSize0[2]);
    dBodySetMass(obBarrier[0].body,&mass);
    obBarrier[0].geom = dCreateBox(space,ObBarrierSize0[0], ObBarrierSize0[1], ObBarrierSize0[2]);
    dGeomSetBody(obBarrier[0].geom,obBarrier[0].body);
    dBodySetPosition(obBarrier[0].body,ObBarrierSize0[0]/2., ObBarrierSize0[1]/2., ObBarrierSize0[2]/2.);

    obBarrier[1].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,Obstacle_M, ObBarrierSize1[0], ObBarrierSize1[1], ObBarrierSize1[2]);
    dBodySetMass(obBarrier[1].body,&mass);
    obBarrier[1].geom = dCreateBox(space,ObBarrierSize1[0], ObBarrierSize1[1], ObBarrierSize1[2]);
    dGeomSetBody(obBarrier[1].geom,obBarrier[1].body);
    dBodySetPosition(obBarrier[1].body,ObBarrierSize1[0]/2., MAPSIZE - ObBarrierSize1[1]/2., ObBarrierSize1[2]/2.);

    obBarrier[2].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,Obstacle_M, ObBarrierSize2[0], ObBarrierSize2[1], ObBarrierSize2[2]);
    dBodySetMass(obBarrier[2].body,&mass);
    obBarrier[2].geom = dCreateBox(space,ObBarrierSize2[0], ObBarrierSize2[1], ObBarrierSize2[2]);
    dGeomSetBody(obBarrier[2].geom,obBarrier[2].body);
    dBodySetPosition(obBarrier[2].body,MAPSIZE - ObBarrierSize2[0]/2.,ObBarrierSize1[1] + ObBarrierSize2[1]/2., ObBarrierSize2[2]/2.);

    obBarrier[3].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,Obstacle_M, ObBarrierSize3[0], ObBarrierSize3[1], ObBarrierSize3[2]);
    dBodySetMass(obBarrier[3].body,&mass);
    obBarrier[3].geom = dCreateBox(space,ObBarrierSize3[0], ObBarrierSize3[1], ObBarrierSize3[2]);
    dGeomSetBody(obBarrier[3].geom,obBarrier[3].body);
    dBodySetPosition(obBarrier[3].body,ObBarrierSize0[0] + ObBarrierSize3[0]/2., ObBarrierSize3[1]/2., ObBarrierSize3[2]/2.);

    //make box
    obBox[0].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,Obstacle_M, ObBoxSize0[0], ObBoxSize0[1], ObBoxSize0[2]);
    dBodySetMass(obBox[0].body,&mass);
    obBox[0].geom = dCreateBox(space,ObBoxSize0[0], ObBoxSize0[1], ObBoxSize0[2]);
    dGeomSetBody(obBox[0].geom,obBox[0].body);
    dBodySetPosition(obBox[0].body,ObBoxPos0[0], ObBoxPos0[1], ObBoxPos0[2]);

    obBox[1].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,Obstacle_M, ObBoxSize1[0], ObBoxSize1[1], ObBoxSize1[2]);
    dBodySetMass(obBox[1].body,&mass);
    obBox[1].geom = dCreateBox(space,ObBoxSize1[0], ObBoxSize1[1], ObBoxSize1[2]);
    dGeomSetBody(obBox[1].geom,obBox[1].body);
    dBodySetPosition(obBox[1].body,ObBoxPos1[0], ObBoxPos1[1], ObBoxPos1[2]);

    obBox[2].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,Obstacle_M, ObBoxSize2[0], ObBoxSize2[1], ObBoxSize2[2]);
    dBodySetMass(obBox[2].body,&mass);
    obBox[2].geom = dCreateBox(space,ObBoxSize2[0], ObBoxSize2[1], ObBoxSize2[2]);
    dGeomSetBody(obBox[2].geom,obBox[2].body);
    dBodySetPosition(obBox[2].body,ObBoxPos2[0], ObBoxPos2[1], ObBoxPos2[2]);

    obBox[3].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,Obstacle_M, ObBoxSize3[0], ObBoxSize3[1], ObBoxSize3[2]);
    dBodySetMass(obBox[3].body,&mass);
    obBox[3].geom = dCreateBox(space,ObBoxSize3[0], ObBoxSize3[1], ObBoxSize3[2]);
    dGeomSetBody(obBox[3].geom,obBox[3].body);
    dBodySetPosition(obBox[3].body,ObBoxPos3[0], ObBoxPos3[1], ObBoxPos3[2]);

    obBox[4].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,Obstacle_M, ObBoxSize4[0], ObBoxSize4[1], ObBoxSize4[2]);
    dBodySetMass(obBox[4].body,&mass);
    obBox[4].geom = dCreateBox(space,ObBoxSize4[0], ObBoxSize4[1], ObBoxSize4[2]);
    dGeomSetBody(obBox[4].geom,obBox[4].body);
    dBodySetPosition(obBox[4].body,ObBoxPos4[0], ObBoxPos4[1], ObBoxPos4[2]);

    // make cylinder
    obCylinder[0].body = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetCylinderTotal(&mass, Obstacle_M, 3,ObCy0_R, ObCy0_L);
    dBodySetMass(obCylinder[0].body,&mass);
    obCylinder[0].geom = dCreateCylinder(space,ObCy0_R, ObCy0_L);
    dGeomSetBody(obCylinder[0].geom,obCylinder[0].body);
    dBodySetPosition(obCylinder[0].body, ObCyPos0[0], ObCyPos0[1], ObCyPos0[2]);

    obCylinder[1].body = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetCylinderTotal(&mass, Obstacle_M, 3,ObCy1_R, ObCy1_L);
    dBodySetMass(obCylinder[1].body,&mass);
    obCylinder[1].geom = dCreateCylinder(space, ObCy1_R, ObCy1_R);
    dGeomSetBody(obCylinder[1].geom,obCylinder[1].body);
    dBodySetPosition(obCylinder[1].body, ObCyPos1[0], ObCyPos1[1], ObCyPos1[2]);

}

void makeBall()
{
  dMass m1;

  dReal  ball_mass   = 0.45;
  ball.body    = dBodyCreate(world);

  dMassSetZero(&m1);
  dMassSetSphereTotal(&m1,ball_mass,BALL_R);
  dMassAdjust(&m1,ball_mass);
  dBodySetMass(ball.body,&m1);

  ball.geom = dCreateSphere(space,BALL_R);
  dGeomSetBody(ball.geom,ball.body);
  dBodySetPosition(ball.body,BALL_P[0],BALL_P[1],BALL_P[2]);
}

void makeRobot()
{
  dMatrix3 R;
  dMass mass, mass2, mass3;

  //베이스
  base.body  = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass,BASE_M,BASE_S[0],BASE_S[1],BASE_S[2]);
  dBodySetMass(base.body,&mass);

  base.geom = dCreateBox(space,BASE_S[0],BASE_S[1],BASE_S[2]);
  dGeomSetBody(base.geom,base.body);
  dBodySetPosition(base.body,START_X+XINIT,START_Y+YINIT,START_Z);

  //목 - 실린더
  neck.body  = dBodyCreate(world);
  dMassSetZero(&mass2);
  dMassSetCapsuleTotal(&mass2,NECK_M,3,NECK_R,NECK_L); //인수 3은 capsule 장축이 z축
  dMassAdjust(&mass2,NECK_M);
  dBodySetMass(neck.body,&mass2);

  neck.geom = dCreateCCylinder(space,NECK_R,NECK_L);
  dGeomSetBody(neck.geom,neck.body);
  dBodySetPosition(neck.body,START_X +XINIT + 0.5*BASE_S[0] - NECK_R,
            START_Y +YINIT, START_Z + 0.5*BASE_S[2] + 0.5*NECK_L);
  dGeomDisable(neck.geom);

  //카메라 - 박스
  camera.body  = dBodyCreate(world);
  dMassSetZero(&mass3);
  dMassSetBoxTotal(&mass3,CAMERA_M,CAMERA_S[0],CAMERA_S[1],CAMERA_S[2]);
  dMassAdjust(&mass3,CAMERA_M);
  dBodySetMass(camera.body,&mass3);

  camera.geom = dCreateBox(space,CAMERA_S[0],CAMERA_S[1],CAMERA_S[2]);
  dGeomSetBody(camera.geom,camera.body);
  dBodySetPosition(camera.body,START_X+XINIT + 0.5*BASE_S[0] - NECK_R, START_Y+YINIT,
           START_Z + 0.5*BASE_S[2] + NECK_L);
  dGeomDisable(camera.geom);


  //바퀴
  dRFromAxisAndAngle(R,1,0,0, M_PI/2.0);
  for (int i = 0; i < NUM; i++) {
    wheel[i].body = dBodyCreate(world);

    dMassSetZero(&mass);
    if ((i % 2) == 0) {
      dMassSetCylinderTotal(&mass,WH_M0, 2, WH_R0, WH_W);
      dBodySetMass(wheel[i].body,&mass);
      wheel[i].geom = dCreateCylinder(space, WH_R0, WH_W);
    }
    else {
      dMassSetCylinderTotal(&mass,WH_M1, 2, WH_R1, WH_W);
      dBodySetMass(wheel[i].body,&mass);
      wheel[i].geom = dCreateCylinder(space, WH_R1, WH_W);
    }
    dGeomSetBody(wheel[i].geom, wheel[i].body);
    dBodySetRotation(wheel[i].body,R);
  }

  WL        = BASE_S[1]+WH_W;

  dReal w1y = 0.5 * (BASE_S[1] + WH_W);
  dReal w1z = START_Z - 0.5 * BASE_S[2];
  dReal w0x = 0.5 * BASE_S[0] - WH_R0;
  dReal w0z = START_Z - 0.5 * BASE_S[2] - WH_R0;

  dBodySetPosition(wheel[0].body,    w0x+XINIT,  0+YINIT, w0z);
  dBodySetPosition(wheel[1].body,    0+XINIT,  w1y+YINIT, w1z);
  dBodySetPosition(wheel[2].body,   -w0x+XINIT,  0+YINIT, w0z);
  dBodySetPosition(wheel[3].body,    0+XINIT, -w1y+YINIT, w1z);

  //힌지조인트
  for (int i = 0; i < NUM; i++) {
    joint[i] = dJointCreateHinge(world,0);
    dJointAttach(joint[i], base.body, wheel[i].body);
  }
  dJointSetHingeAxis(joint[0],0, -1, 0);
  dJointSetHingeAxis(joint[1],0, -1, 0);
  dJointSetHingeAxis(joint[2],0, -1, 0);
  dJointSetHingeAxis(joint[3],0, -1, 0);
  dJointSetHingeAnchor(joint[0],    w0x+XINIT,  0+YINIT, w0z);
  dJointSetHingeAnchor(joint[1],    0+XINIT,  w1y+YINIT, w1z);
  dJointSetHingeAnchor(joint[2],   -w0x+XINIT,  0+YINIT, w0z);
  dJointSetHingeAnchor(joint[3],    0+XINIT, -w1y+YINIT, w1z);

  //목 조인트
  neck_joint = dJointCreateHinge(world,0);
  dJointAttach(neck_joint,neck.body,base.body);
  dJointSetHingeAxis(neck_joint,0, 0, 1);
  dJointSetHingeAnchor(neck_joint, START_X+XINIT + 0.5*BASE_S[0]-NECK_R,
             START_Y+YINIT, START_Z + 0.5*BASE_S[0]);
  dJointSetHingeParam(neck_joint,dParamLoStop, 0);
  dJointSetHingeParam(neck_joint,dParamHiStop, 0);


  //카메라 조인트
  camera_joint = dJointCreateHinge(world,0);
  dJointAttach(camera_joint,neck.body,camera.body);
  dJointSetHingeAxis(camera_joint,1,0,0);
  dJointSetHingeAnchor(camera_joint,START_X+XINIT + 0.5*BASE_S[0]-NECK_R,
              START_Y+YINIT, START_Z + 0.5*BASE_S[0] + NECK_L);
  dJointSetHingeParam(camera_joint,dParamLoStop, 0.0);
  dJointSetHingeParam(camera_joint,dParamHiStop, 0.0);

  //Lidar 2D 센서
//  KSENSORLIDAR2D_INFO	oInfo;
//  OBJECT_ODE			oBase = { base.body, base.geom };	//Beam Sensor를 부착할 Object

//  strcpy(oInfo.szID, "SensorLidar2D");
//  // 모두 한점에서 라이더가 출발하도록 설정해줬다.
//  oInfo.fR			= oInfo.fG = oInfo.fB = 1.0;
//  oInfo.dX			= START_X+XINIT + 0.5*BASE_S[0];  //로봇 앞쪽에 부착
//  oInfo.dY          = START_Y+YINIT;
//  oInfo.dZ          = START_Z;
//  oInfo.dMaxLength	= 3.0;      //최대 거리는 3m
//  // 범위가 180도임
//  oInfo.nLoRange	= -90;
//  oInfo.nHiRange	= 90;
//  oInfo.oBase       = oBase;

//  _oLidar2D.Create(oInfo, world, space);
}

void command(int cmd)
{
  switch (cmd) {
  case 'j': w_r += 0.1; break;
  case 'f': w_l += 0.1; break;
  case 'k': w_r -= 0.1; break;
  case 'd': w_l -= 0.1; break;
  case 's': w_r = w_l = 0.0;break;
  default: printf("\nInput j,f,k,d,l,s \n");break;
  }
}

void GetPos(dBodyID obj_body)
{
  const dReal *pos;
  pos = dBodyGetPosition(obj_body);
  Xpos = pos[0]; Ypos = pos[1]; Zpos = pos[2];
}


void GetRotation(dBodyID obj_body)
{
  const dReal* rot;
  rot = dBodyGetRotation(obj_body);

  // sin, cos순으로 대입
  Rotation = atan2(rot[1], rot[0]);// *  57.29578;
}

void SimultaneousPC()
{
    GetPos(base.body);
    GetRotation(base.body);

    dReal Real_Xpos = Xpos * 20.;
    dReal Real_Ypos = (MAPSIZE-Ypos) * 20.;

    int curX = (int)(Real_Xpos);
    int curY = (int)(Real_Ypos);
//    printf("x=%d, y=%d ",curX, curY);

    // aU / aQy   &   aU / aQx
    dReal PotenSumX = odePotentialMap[curY][curX].first;
    dReal PotenSumY = odePotentialMap[curY][curX].second;

//    printf("protenSumX=%f, potenSumY=%f", PotenSumX, PotenSumY);

    // Desired Orientation
    Qd = atan2(PotenSumY, PotenSumX);

    // Using a Proportional control
    W = Kp * (Qd - Rotation);
    V = Psi * (PotenSumX * cos(Rotation) + PotenSumY * sin(Rotation));

    // 로봇 움직임
    dReal omega_r = (V+W) / WH_R0;
    dReal omega_l = (V-W) / WH_R0;

    // Local Minima에 빠졌을 경우 일단 전진 시킨다.
    if(_ABS(omega_r) < MIN_VEL && _ABS(omega_l) < MIN_VEL) {
        omega_l = -0.4;
        omega_r = -0.4;
    }
    else if(_ABS(omega_r) > MAX_VEL && _ABS(omega_l) > MAX_VEL){
        omega_l = -MAX_VEL;
        omega_r = -MAX_VEL;
    }

    controlWheel(omega_r ,omega_l);

//    cout << "  omega_r  = " << omega_r;
//    cout << "  omega_l  = " << omega_l;
//    cout << " SUM X = " << PotenSumX;
//    cout << " SUM Y = " << PotenSumY << endl;
}

void control() {
  double fMax = 100;         // 최대 토크[Nm]

  dJointSetHingeParam(joint[1],  dParamVel, w_l );
  dJointSetHingeParam(joint[1], dParamFMax, fMax);
  dJointSetHingeParam(joint[3],  dParamVel, w_r );
  dJointSetHingeParam(joint[3], dParamFMax, fMax);
}

void start()
{
    static float xyz[3] = {MAPSIZE/2.,MAPSIZE/2.,9.0};
    static float hpr[3] = {90, -90, 0};

    dsSetViewpoint(xyz,hpr);
    dsSetSphereQuality(2);
}

void nearCallback (void *data, dGeomID o1, dGeomID o2)
{

  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);

  if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;

  const int nMAX_CONTACT = 10;
  dContact contact[nMAX_CONTACT];

  //충돌점 계산
  int nNumContact = dCollide(o1, o2, nMAX_CONTACT, &contact[0].geom, sizeof(dContact));

  //센싱
  if(nNumContact == 0 || _oLidar2D.Measure(contact[0].geom) == true)
      return;

  //총돌 설정
  for(int i=0; i<nNumContact; i++)
  {
      contact[i].surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
      contact[i].surface.mu = dInfinity;
      contact[i].surface.slip1 = 0.1;
      contact[i].surface.slip2 = 0.1;
      contact[i].surface.soft_erp = 0.8;
      contact[i].surface.soft_cfm = 1e-5;

      dJointAttach(dJointCreateContact(world, contactgroup, &contact[i]), b1, b2);
  }
}

void drawObstacle(){

    dsSetColor(0., 0., 0.);

    // barrier obstacle
    dsDrawBox(dGeomGetPosition(obBarrier[0].geom),
            dGeomGetRotation(obBarrier[0].geom), ObBarrierSize0);
    dsDrawBox(dGeomGetPosition(obBarrier[1].geom),
            dGeomGetRotation(obBarrier[1].geom), ObBarrierSize1);
    dsDrawBox(dGeomGetPosition(obBarrier[2].geom),
            dGeomGetRotation(obBarrier[2].geom), ObBarrierSize2);
    dsDrawBox(dGeomGetPosition(obBarrier[3].geom),
            dGeomGetRotation(obBarrier[3].geom), ObBarrierSize3);

    // box obstacle
    dsDrawBox(dGeomGetPosition(obBox[0].geom),
            dGeomGetRotation(obBox[0].geom), ObBoxSize0);
    dsDrawBox(dGeomGetPosition(obBox[1].geom),
            dGeomGetRotation(obBox[1].geom), ObBoxSize1);
    dsDrawBox(dGeomGetPosition(obBox[2].geom),
            dGeomGetRotation(obBox[2].geom), ObBoxSize2);
    dsDrawBox(dGeomGetPosition(obBox[3].geom),
            dGeomGetRotation(obBox[3].geom), ObBoxSize3);
    dsDrawBox(dGeomGetPosition(obBox[4].geom),
            dGeomGetRotation(obBox[4].geom), ObBoxSize4);

    // cylinder obstacle
    dsDrawCylinder(dBodyGetPosition(obCylinder[0].body),
                  dBodyGetRotation(obCylinder[0].body), ObCy0_L, ObCy0_R);
    dsDrawCylinder(dBodyGetPosition(obCylinder[1].body),
                  dBodyGetRotation(obCylinder[1].body),ObCy1_L, ObCy1_R);

}

void drawBall()
{
  dsSetColor(1.0,0.0,0.0);
  dsDrawSphere(dGeomGetPosition(ball.geom),
           dGeomGetRotation(ball.geom),BALL_R);
}

void drawRobot()
{
  //베이스
  dsSetColor(1.3,1.3,0.0);
  dsDrawBox(dGeomGetPosition(base.geom),
        dGeomGetRotation(base.geom),BASE_S);

  //목
  dsSetColor(1.3,0.0,1.3);
  dsDrawCylinder(dBodyGetPosition(neck.body),
             dBodyGetRotation(neck.body),NECK_L,NECK_R);

  //카메라
  dsSetColor(0,1.0,0);
  dsDrawBox(dGeomGetPosition(camera.geom),
        dGeomGetRotation(camera.geom),CAMERA_S);

  //바퀴
  dsSetColor(1.1,1.1,1.1);
  for (int i=0; i< NUM; i++) {
        if ((i % 2) == 0) {
        dsDrawCylinder(dBodyGetPosition(wheel[i].body),
             dBodyGetRotation(wheel[i].body),WH_W,WH_R0);
    }
    else {
        dsDrawCylinder(dBodyGetPosition(wheel[i].body),
             dBodyGetRotation(wheel[i].body),WH_W,WH_R1);
        }
  }

  // 거리센서
  for(auto& beam : _oLidar2D)
      beam.second->Draw();
}

//월드좌표계를 로봇좌표계로 변환
void worldToRobot(const dReal p[2], const dReal c[2], dReal theta,
  dReal pr[2], dReal *r, dReal *phi)
{
  pr[0] =   (p[0]-c[0]) * cos(theta) + (p[1]-c[1]) * sin(theta);
  pr[1] = - (p[0]-c[0]) * sin(theta) + (p[1]-c[1]) * cos(theta);

  *r   = sqrt(pr[0] * pr[0] + pr[1] * pr[1]);  //물체까지의 거리
  *phi = atan2(pr[1], pr[0]);                  //물체의 각도
}

void odeInitialize(){
    dInitODE();
    setDrawStuff();

    world        = dWorldCreate();
    space        = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);
    ground       = dCreatePlane(space,0,0,1,0);

    dWorldSetGravity(world, 0, 0, -9.8);

    makeRobot();
//    makeBall();
    makeObstacle();
    jointFix();

}

//로봇방향
dReal heading()
{
  const dReal *c,*cam;

  c   = dBodyGetPosition(base.body);     //베이스 월드좌표
  cam = dBodyGetPosition(camera.body);   //카메라 월드좌표

  return atan2(cam[1]-c[1], cam[0]-c[0]); // 로봇방향
}

//로봇 비전 함수
int vision(dReal *r, dReal *phi, dBodyID obj_body)
{
  const dReal *p,*c,*cam;
  dReal pr[2], view_angle = M_PI/4.0;  //로봇좌표계에서의 물체위치,시야갹

  p   = dBodyGetPosition(obj_body);    //물체의 월드좌표
  c   = dBodyGetPosition(base.body);   //베이스의 월드좌표
  cam = dBodyGetPosition(camera.body); //카메라의 월드좌표z

  dReal theta  = atan2(cam[1] - c[1], cam[0] - c[0]); //로봇 방향

  worldToRobot(p, c, theta, pr, r, phi);

  if ((-view_angle <= *phi) && (*phi <= view_angle)) return 1;
  else                                               return 0;
}

//바퀴 제어
void controlWheel(dReal left, dReal right)
{
  double fMax = 100;

  dJointSetHingeParam(joint[1],  dParamVel, left);
  dJointSetHingeParam(joint[1], dParamFMax, fMax);
  dJointSetHingeParam(joint[3],  dParamVel, right);
  dJointSetHingeParam(joint[3], dParamFMax, fMax);
}

//회전
void turn(dReal speed)
{
  controlWheel(speed, -speed);
}

//추적
void track(dReal phi, dReal speed)
{
  dReal k = 0.5;
  if (phi > 0) controlWheel(k * speed,      speed);
  else         controlWheel(    speed,  k * speed);
}

void deadreckoning(dReal& s_x, dReal& s_y, dReal& s_t)
{
    const dReal  *c;
    dReal        theta;  // 로봇 방향
    dReal        omega_r, omega_l;
    dReal        fx, fy, ft;
    dReal omega_l_old = 0, omega_r_old = 0, theta_old = 0;


    //로봇 월드좌표와 방향
    c     = dBodyGetPosition(base.body);
    theta = heading();

    //각속도
    omega_r = dJointGetHingeAngleRate(joint[3]);
    omega_l = dJointGetHingeAngleRate(joint[1]);

    fx      = 0.5 * WH_R1 * ((omega_r + omega_l) * cos(theta) + (omega_r_old + omega_l_old) * cos(theta_old) );
    fy      = 0.5 * WH_R1 * ((omega_r + omega_l) * sin(theta) + (omega_r_old + omega_l_old) * sin(theta_old) );
    ft      = 0.5 * WH_R1/WL* ( (omega_r - omega_l) + (omega_r_old - omega_l_old) );

    s_x    += fx * sim_step * 0.5;
    s_y    += fy * sim_step * 0.5;
    s_t    += ft * sim_step * 0.5;

    omega_r_old = omega_r;
    omega_l_old = omega_l;
    theta_old   = theta;

    printf("Real x = %f  Dead Reckoning s_x = %f Error = %f\n", c[0], s_x, c[0] -s_x);
    printf("Real y = %f  Dead Reckoning s_y = %f Error = %f\n", c[1], s_y, c[1] -s_y);
    printf("Real t = %f  Dead Reckoning s_t = %f Error = %f\n", theta,s_t, theta-s_t);
}

void simLoop(int pause)
{
  dReal        r, phi; // r:물체까지의 거리, phi: 물체의 방위각

  if(!pause)
  {

      for (auto& beam : _oLidar2D)      //_oLidar2D : 라이다가 beam 변수에 다 들어온다. 거리를 다 3m로 초기화 해준다(계속 다음 루프에서 사용하기 위해서)
          beam.second->_dRange = 3;

     dSpaceCollide (space,0,&nearCallback);

     if(controlFlag){
        SimultaneousPC();
     }
     else{
         dJointSetHingeParam(joint[1],  dParamVel, 0 );
         dJointSetHingeParam(joint[1], dParamFMax, 0);
         dJointSetHingeParam(joint[3],  dParamVel, 0 );
         dJointSetHingeParam(joint[3], dParamFMax, 0);
     }

     dWorldStep(world, sim_step);
     dJointGroupEmpty (contactgroup);

//     deadreckoning(s_x, s_y, s_t);
  }

//  drawBall();
  drawRobot();
  drawObstacle();
}

void setDrawStuff() {
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = &command;
  fn.path_to_textures = "c:/ode-0.13/drawstuff/textures";
}


void jointFix(){

    // 고정 Joint 생성
    for(int l = 0; l<12; ++l){
        ObsJointID[l] = dJointCreateFixed(world, 0);
    }

    // body1 과 body2 를 결합 -> 둘중 하나가 0이면 world와 결합한다.
    //obBarrier[4], obBox[5], obCylinder[2];
    for(int i = 0; i < 4; ++i){
        dJointAttach(ObsJointID[i] , obBarrier[i].body, 0);
    }
    for(int j = 0; j < 5; ++j){
        dJointAttach(ObsJointID[4 + j] , obBox[j].body, 0);
    }
    for(int j = 0; j < 2; ++j){
        dJointAttach(ObsJointID[9 + j] , obCylinder[j].body, 0);
    }

    // 지정된 Joint 를 고정
    for(int k=0; k < 12; ++k){
        dJointSetFixed(ObsJointID[k]);
    }
}
