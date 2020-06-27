#include <stdio.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <windows.h>
#include <src/resource.h>
#include "math.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions
#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawCylinder dsDrawCylinderD
#endif

dWorldID    world;                      // a dynamics world
dSpaceID    space;
dGeomID     ground;
dJointGroupID contactgroup;
dsFunctions fn;                         // drawstuff structure

struct MyObject{
  dBodyID body;        //  body ID(동력학계산용)
  dGeomID geom;        //  geometry ID(충돌계산용）
  double  l,r,m;       //  길이, 반경, 질량
};

dReal       sim_step = 0.0005;

// 장애물 변수
#define GRIDSIZE 0.05
#define MAPSIZE  GRIDSIZE * 200

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

// 콜백 함수
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  static const int N = 10; // 최대 점촉점의 수
  dContact contact[N];     // 점촉점

  // 접촉하고 있는 물체가 지면인지 여부
  int isGround = ((ground == o1) || (ground == o2));

  // 충돌정보의 생성(n은 충돌점 개수)
  int n =  dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));

  if (isGround)  {
     for (int i = 0; i < n; i++) {
       contact[i].surface.mode = dContactBounce; // 접촉면의 반발성
       contact[i].surface.bounce = 1.0;          // 반발계수(0.0~1.0)
       contact[i].surface.bounce_vel = 0.0;      // 반발에 필요한 최저속도

       // 접촉조인트 생성
       dJointID c = dJointCreateContact(world,contactgroup, &contact[i]);

       // 두 강체를 접촉조인트로 연결
       dJointAttach(c,dGeomGetBody(contact[i].geom.g1),
                      dGeomGetBody(contact[i].geom.g2));
    }
  }
}



static void makeObstacle(){

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

static void drawObstacle(){

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

// 시뮬레이션
static void simLoop(int pause)
{
  dSpaceCollide(space,0,&nearCallback);// 충돌검출

  dWorldStep(world,0.01);
  dJointGroupEmpty(contactgroup);

  drawObstacle();
}

void start()
{
    static float xyz[3] = {MAPSIZE/2.,MAPSIZE/2.,15.0};
    static float hpr[3] = {90, -90, 0};
    dsSetViewpoint(xyz,hpr);
}

void setDrawStuff()           //그리기 설정
{
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.path_to_textures = "c:\\ode-0.13\\drawstuff\\textures";
}

int main(int argc, char **argv)
{
  setDrawStuff();

  dInitODE();                           // ODE 초기화
  world = dWorldCreate();               // world 생성
  dWorldSetGravity(world,0,0,-0.5);

  space        = dHashSpaceCreate(0);   // space 생성
  contactgroup = dJointGroupCreate(0);  // 조인트 그룹 생성
  ground = dCreatePlane(space,0,0,1,0); // 평면 geometry 생성

  makeObstacle();

  dsSimulationLoop(argc,argv,640, 480,&fn);
  dSpaceDestroy(space);
  dWorldDestroy(world);
  dCloseODE();
  return 0;
}


//#include <stdio.h>
//#include <ode/ode.h>
//#include <drawstuff/drawstuff.h>
//#include <windows.h>
//#include <src/resource.h>

//// select correct drawing functions
//#ifdef dDOUBLE
//#define dsDrawBox      dsDrawBoxD
//#define dsDrawSphere   dsDrawSphereD
//#define dsDrawCapsule  dsDrawCapsuleD
//#define dsDrawCylinder dsDrawCylinderD
//#endif


//dWorldID    world;                      // a dynamics world
//dSpaceID    space;
//dGeomID     ground;
//dJointGroupID contactgroup;
//dsFunctions fn;                         // drawstuff structure

//struct MyObject{
//  dBodyID body;        //  body ID(동력학계산용)
//  dGeomID geom;        //  geometry ID(충돌계산용）
//  double  l,r,m;       //  길이, 반경, 질량
//};

//dReal       sim_step = 0.0005;

//// 장애물 변수

//MyObject    obBox[4], obCylinder[2];     // obstacle map
//dReal       Obstacle_M    = 10000000000000;                        // 베이스의 질량
//dReal       ObCy0_R(1), ObCy0_L(10);
//dReal       ObObox0[3] ={10, 10, 10};

//void start()                                  /*** (start funciton) ***/
//{
//    float xyz[3] = {  -30.0f,   0.0f, 50.f};
//    float hpr[3] = { 0.0f, -40.f, 0.0f};

//    dsSetViewpoint(xyz,hpr);                     // (set the view point and direction)
//}

//static void command(int cmd)
//{
//  switch (cmd) {
//  default: printf("\nInput j,f,k,d,l,s \n");break;
//  }
//}

//static void nearCallback (void *data, dGeomID o1, dGeomID o2)
//{
//    static const int N = 10; // 최대 점촉점의 수
//    dContact contact[N];     // 점촉점

//    // 접촉하고 있는 물체가 지면인지 여부
//    int isGround = ((ground == o1) || (ground == o2));

//    // 충돌정보의 생성(n은 충돌점 개수)
//    int n =  dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));

//    if (isGround)  {
//       for (int i = 0; i < n; i++) {
//         contact[i].surface.mode = dContactBounce; // 접촉면의 반발성
//         contact[i].surface.bounce = 1.0;          // 반발계수(0.0~1.0)
//         contact[i].surface.bounce_vel = 0.0;      // 반발에 필요한 최저속도

//         // 접촉조인트 생성
//         dJointID c = dJointCreateContact(world,contactgroup, &contact[i]);

//         // 두 강체를 접촉조인트로 연결
//         dJointAttach(c,dGeomGetBody(contact[i].geom.g1),
//                        dGeomGetBody(contact[i].geom.g2));
//      }
//    }
//}

//static void makeObstacle(){

//    dMass mass;

//    obCylinder[0].body  = dBodyCreate(world);
//    dMassSetZero(&mass);
//    dMassSetCapsuleTotal(&mass,Obstacle_M,3, ObCy0_R, ObCy0_L); //인수 3은 capsule 장축이 z축
//    dMassAdjust(&mass,Obstacle_M);
//    dBodySetMass(obCylinder[0].body,&mass);

//    obCylinder[0].geom = dCreateCCylinder(space, ObCy0_R, ObCy0_L);
//    dGeomSetBody(obCylinder[0].geom,obCylinder[0].body);
//    dBodySetPosition(obCylinder[0].body,0, 0, 0);

//    obBox[0].body  = dBodyCreate(world);
//    dMassSetZero(&mass);
//    dMassSetBoxTotal(&mass,Obstacle_M, ObObox0[0], ObObox0[1], ObObox0[2]);
//    dBodySetMass(obBox[0].body,&mass);

//    obBox[0].geom = dCreateBox(space,ObObox0[0], ObObox0[1], ObObox0[2]);
//    dGeomSetBody(obBox[0].geom,obBox[0].body);
//    dBodySetPosition(obBox[0].body,ObObox0[0], ObObox0[1], ObObox0[2]);
//}

//static void drawObstacle(){
////    dsSetColor(1.3,0.0,1.3);
////    dsDrawCylinder(dBodyGetPosition(obCylinder[0].body),
////               dBodyGetRotation(obCylinder[0].body),ObCy0_R, ObCy0_L);

//    dsSetColor(0,1.0,0);
//    dsDrawBox(dGeomGetPosition(obCylinder[0].geom),
//            dGeomGetRotation(obCylinder[0].geom), ObObox0);

//}


//void simLoop(int pause)           // simulation loop
//{
//    dSpaceCollide(space,0,&nearCallback);// 충돌검출

//    dWorldStep(world, sim_step);        // step a simulation
//    dJointGroupEmpty(contactgroup);

//    drawObstacle();
//}

//void setDrawStuff()           /*** (set a drawstuff) ***/{
//    fn.version = DS_VERSION;
//    fn.start   = &start;
//    fn.step    = &simLoop;
//    fn.command = &command;
//    fn.path_to_textures = "c:/ode-0.13/drawstuff/textures";
//}

//int main(int argc, char *argv[])
//{
//    dInitODE();                              // (initialize ODE)
//    setDrawStuff();                          // (set drawstuff)

//    world        = dWorldCreate();
//    space        = dHashSpaceCreate(0);   // space 생성
//    contactgroup = dJointGroupCreate(0);  // 조인트 그룹 생성
//    ground       = dCreatePlane(space,0,0,1,0); // 평면 geometry 생성

//    dWorldSetGravity(world,0,0,-9.8);        // (set gravity)

//    makeObstacle();

//    dsSimulationLoop(argc,argv,640, 480,&fn); // (simulation loop)

//    dSpaceDestroy(space);
//    dWorldDestroy(world);                     // (destroy the world)
//    dCloseODE();                              // (close ODE)


//    return 0;
//}
