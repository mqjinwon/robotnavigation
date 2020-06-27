#include "odesim.h"

odeSim::odeSim()
{

}

void odeSim::run()
{
    dsSimulationLoop(0, 0,640,480,&fn);
}
