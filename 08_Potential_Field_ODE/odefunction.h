#ifndef ODEFUNCTION_H
#define ODEFUNCTION_H

#include "sensorBeam.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

void makeObstacle();

void drawObstacle();

void makeBall();

void makeRobot();

void command(int cmd);

void control();

void start();

void nearCallback (void *data, dGeomID o1, dGeomID o2);

void drawBall();

void drawRobot();

void odeInitialize();

//월드좌표계를 로봇좌표계로 변환
void worldToRobot(const dReal p[2], const dReal c[2], dReal theta,
  dReal pr[2], dReal *r, dReal *phi);

//로봇방향
dReal heading();


//로봇 비전 함수
int vision(dReal *r, dReal *phi, dBodyID obj_body);

//바퀴 제어
void controlWheel(dReal left, dReal right);

//회전
void turn(dReal speed);

//추적
void track(dReal phi, dReal speed);

void deadreckoning(dReal& s_x, dReal& s_y, dReal& s_t);

void simLoop(int pause);

void setDrawStuff();

void jointFix();

#endif
