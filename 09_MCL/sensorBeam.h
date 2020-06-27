#ifndef SENSORBEAM_H
#define SENSORBEAM_H

#include <map>
#include <algorithm>

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "odometrymotion.h"

#define _RADIAN(a)		((a)/57.2957795130823)

enum eROTATION { _AXIS_ANGLE, _EULER_ANGLE };

struct OBJECT_ODE
{
	dBodyID		hBody;
	dGeomID		hGeom;
};


struct KSENSORBEAM_INFO
{
	char				szID[100];	
	float				fR, fG, fB;
	dReal				dX, dY, dZ;
	eROTATION			eTYPE;
	dReal				dNx, dNy, dNz, dAng;
	dReal				dPhi, dTheta, dPsi;	
	dReal				dMaxLength;

    OBJECT_ODE			oBase;
};

class KSensorBeam
{
public:
    KSENSORBEAM_INFO	_oInfo;
	dBodyID             _hBody;
	dGeomID				_hGeom;
	dReal				_dRange;

    KSensorBeam(){ }
    KSensorBeam(const KSENSORBEAM_INFO& oInfo, dWorldID hWorld, dSpaceID hSpace){ Create(oInfo, hWorld, hSpace); }
    ~KSensorBeam(){ }

    void		Create(const KSENSORBEAM_INFO& oInfo, dWorldID hWorld, dSpaceID hSpace);
	void		Draw();

    void        End(OPoint &_particle, double &_nX, double &_nY);
    bool        Fail();
};

struct KSENSORLIDAR2D_INFO
{
	char				szID[100];
	float				fR, fG, fB;
	dReal				dX, dY, dZ;
	dReal				dMaxLength;
	int					nLoRange;
	int					nHiRange;
    OBJECT_ODE			oBase;
};

class KSensorLidar2D : public std::map < dGeomID, KSensorBeam* >
{
    std::map<dGeomID, KSensorBeam*>::iterator _itr;

public:
    KSensorLidar2D(){ }
    virtual ~KSensorLidar2D()
	{
		for (_itr = this->begin(); _itr != this->end(); _itr++)
			delete _itr->second;
	}

    void	Create(KSENSORLIDAR2D_INFO& oInfo, dWorldID hWorld, dSpaceID hSpace);
	bool	Measure(const dContactGeom& o_ConGeom)
			{
                //두개 모두 Ray인 경우
				if (dGeomGetClass(o_ConGeom.g1) == dRayClass && dGeomGetClass(o_ConGeom.g2) == dRayClass)		return false;
                //g1 Ray인 경우
				if ((_itr = this->find(o_ConGeom.g1)) != this->end()){ _itr->second->_dRange = o_ConGeom.depth;	return true; }
                //g2 Ray인 경우
				if ((_itr = this->find(o_ConGeom.g2)) != this->end()){ _itr->second->_dRange = o_ConGeom.depth;	return true; }

				return false;
			}

};

#endif
