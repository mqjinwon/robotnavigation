#include "sensorBeam.h"
#include <algorithm>


void KSensorBeam::Create(const KSENSORBEAM_INFO& oInfo, dWorldID hWorld, dSpaceID hSpace)
{
	//초기화 정보 저장
    _oInfo = oInfo;

	//object 생성
	_hBody = dBodyCreate(hWorld);
    _hGeom = dCreateRay(hSpace,_oInfo.dMaxLength);
		

	//센싱값 초기화
    _dRange = _oInfo.dMaxLength;
	
	//body와 geomtry 결합
	dGeomSetBody(_hGeom, _hBody);

	//body 위치 설정
    dBodySetPosition(_hBody, _oInfo.dX, _oInfo.dY, _oInfo.dZ);
	
	//body 자세 설정
    dMatrix3  mR;
    if (_oInfo.eTYPE == _AXIS_ANGLE)
        dRFromAxisAndAngle(mR, _oInfo.dNx, _oInfo.dNy, _oInfo.dNz, _oInfo.dAng);
	else //_EULER_ANGLE
        dRFromEulerAngles(mR, _oInfo.dPhi, _oInfo.dTheta, _oInfo.dPsi);
    dBodySetRotation(_hBody, mR);

		
	//두 개의 object를 하나의 body로 결합함		
    dMass oMassBase;
    dBodyGetMass(_oInfo.oBase.hBody, &oMassBase);
	
    const dReal *dpPosBase	= dBodyGetPosition(_oInfo.oBase.hBody);
	const dReal *dpPos		= dGeomGetPosition(_hGeom);
    const dReal *dpRotBase	= dBodyGetRotation(_oInfo.oBase.hBody);
	const dReal *dpRot		= dBodyGetRotation(_hBody);

    dMassRotate(&oMassBase, dpRotBase);
    dMassTranslate(&oMassBase, dpPosBase[0], dpPosBase[1], dpPosBase[2]);
		
	
    dGeomID hGeomTmp = dBodyGetFirstGeom(_oInfo.oBase.hBody);
	while (hGeomTmp)
	{
		const dReal *dpPosTmp = dGeomGetPosition(hGeomTmp);
		const dReal *dpRotTmp = dGeomGetRotation(hGeomTmp);
        dGeomSetOffsetPosition(hGeomTmp, dpPosTmp[0] - oMassBase.c[0], dpPosTmp[1] - oMassBase.c[1], dpPosTmp[2] - oMassBase.c[2]);
		dGeomSetOffsetRotation(hGeomTmp, dpRotTmp);
		hGeomTmp = dBodyGetNextGeom(hGeomTmp);
	}

    dGeomSetBody(_hGeom, _oInfo.oBase.hBody);

    dGeomSetOffsetPosition(_hGeom, dpPos[0] - oMassBase.c[0], dpPos[1] - oMassBase.c[1], dpPos[2] - oMassBase.c[2]);
	dGeomSetOffsetRotation(_hGeom, dpRot);

    dBodySetPosition(_oInfo.oBase.hBody, oMassBase.c[0], oMassBase.c[1], oMassBase.c[2]);
    dRSetIdentity(mR);
    dBodySetRotation(_oInfo.oBase.hBody, mR);

    dMassTranslate(&oMassBase, -oMassBase.c[0], -oMassBase.c[1], -oMassBase.c[2]);
    dBodySetMass(_oInfo.oBase.hBody, &oMassBase);
	dBodyDestroy(_hBody);
    _hBody = _oInfo.oBase.hBody;
	
    //Ray의 크기 결정
    dGeomRaySetLength(_hGeom, _oInfo.dMaxLength);
}

void KSensorBeam::Draw()
{
	double		dLength;
	double		dpStart[3], dpEnd[3];
    dVector3	vOri, vDir;

    dGeomRayGet(_hGeom, vOri, vDir);
    dLength = std::min(_oInfo.dMaxLength, _dRange);

    dpStart[0]	= vOri[0];
    dpStart[1]	= vOri[1];
    dpStart[2]	= vOri[2];
    dpEnd[0]	= vOri[0] + vDir[0] * dLength;
    dpEnd[1]	= vOri[1] + vDir[1] * dLength;
    dpEnd[2]	= vOri[2] + vDir[2] * dLength;
		
    //로봇의 위치를 그릴 때 사용하는 함수
    dsSetColor(_oInfo.fR, _oInfo.fG, _oInfo.fB);
	dsDrawLineD(dpStart, dpEnd); 

}

void KSensorBeam::End(OPoint &_particle, double &_nX, double &_nY)
{
    double		dLength;
    double		dpEnd[2];
    dVector3	vOri, vDir;

    dGeomRayGet(_hGeom, vOri, vDir);
    dLength = std::min(_oInfo.dMaxLength, _dRange);

//    dpEnd[0]	= vOri[0] + vDir[0] * dLength;
//    dpEnd[1]	= vOri[1] + vDir[1] * dLength;


    dpEnd[0]	= _particle.x + vDir[0] * dLength;
    dpEnd[1]	= _particle.y + vDir[1] * dLength;

    _nX = dpEnd[0];
    _nY = dpEnd[1];
}

bool KSensorBeam::Fail()
{
    if(_dRange == _oInfo.dMaxLength)
        return true;
    else
        return false;

}

void KSensorLidar2D::Create(KSENSORLIDAR2D_INFO& oInfo,dWorldID world, dSpaceID space)
{
    KSENSORBEAM_INFO	oBeamInfo;
    KSensorBeam*		opBeam;
		
    oBeamInfo.fR			= oInfo.fR;
    oBeamInfo.fG			= oInfo.fG;
    oBeamInfo.fB			= oInfo.fB;
    oBeamInfo.dX			= oInfo.dX;
    oBeamInfo.dY			= oInfo.dY;
    oBeamInfo.dZ			= oInfo.dZ;
    oBeamInfo.dMaxLength	= oInfo.dMaxLength;
    oBeamInfo.oBase         = oInfo.oBase;
    oBeamInfo.eTYPE		    = _AXIS_ANGLE;

    //beam은 Z축을 기준으로 생성되어 있다. 이를 y축으로 회전 시켜서 사용한다.
    for(int deg = oInfo.nLoRange; deg <= oInfo.nHiRange; deg += 5)
	{
        sprintf(oBeamInfo.szID, "Beam_(%d)", deg);

        oBeamInfo.dNx	= cos(_RADIAN(deg + 90));
        oBeamInfo.dNy	= sin(_RADIAN(deg + 90));
        oBeamInfo.dNz	= 0.0;
        oBeamInfo.dAng	= _RADIAN(90);

        opBeam = new KSensorBeam(oBeamInfo, world, space);
        std::map<dGeomID, KSensorBeam*>::insert(std::map<dGeomID, KSensorBeam*>::value_type(opBeam->_hGeom, opBeam));
        
	}
}
