#include "TestCuboid.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Map.h"
#include "Converter.h"

#include <mutex>

namespace ORB_SLAM2
{
using namespace std;
long unsigned int TestCuboid::nNextId=0;
mutex TestCuboid::mGlobalMutex;

TestCuboid::TestCuboid(Map *pMap,bool update_index): mbBad(false), mpMap(pMap)
{
    if(update_index)
        mnId=nNextId++;
    else
        mnId=-1; 
    nObs=0;
    mpRefKF=nullptr;
    mpLatestKF=nullptr;
    
    mnAssoId=-1;    
}
}