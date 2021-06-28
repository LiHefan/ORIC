#include "MapCuboid.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Map.h"
#include "Converter.h"

#include <mutex>

namespace ORB_SLAM2
{
using namespace std;
long unsigned int MapCuboid::nNextId=0;
mutex MapCuboid::mGlobalMutex;

MapCuboid::MapCuboid(Map *pMap,bool update_index): mbBad(false), mpMap(pMap)
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

void MapCuboid::SetWorldPos(const g2o::cuboid& Pos)
{ 
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    mCuboidTwc=Pos;
    mCuboidTcw=mCuboidTwc;
    mCuboidTcw.pose=mCuboidTwc.pose.inverse();
}

g2o::cuboid MapCuboid::GetWorldPos(){
    unique_lock<mutex> lock(mMutexPos);
    return mCuboidTwc;
}

g2o::cuboid MapCuboid::GetWorldPosInv()
{   
    unique_lock<mutex> lock(mMutexPos);
    return mCuboidTcw; 

}

void MapCuboid::SetReferenceKeyFrame(KeyFrame* refKF)
{
    mpRefKF=refKF;
}

KeyFrame* MapCuboid::GetReferenceKeyFrame()
{
    return mpRefKF;
}

KeyFrame* MapCuboid::GetLatestKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpLatestKF;

}
bool MapCuboid::isInKeyFrame(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

int MapCuboid::GetIndexInKeyFrame(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else 
        return -1;
}

unordered_map<KeyFrame*, size_t> MapCuboid::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapCuboid::Observations(){
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}
    
void MapCuboid::addObservation(KeyFrame *pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mpLatestKF==nullptr || pKF->mnId >= mpLatestKF->mnId)
        mpLatestKF=pKF;
    

    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;

}

void MapCuboid::EraseObservation(KeyFrame *pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            mObservations.erase(pKF);
            if(mpRefKF==pKF)
            {
                long unsigned int minKFId = mpLatestKF->mnId+1;
                KeyFrame* EldestKF=nullptr;
                for(auto it=mObservations.begin();it!=mObservations.end();++it)
                {
                    if(it->first->mnId<minKFId)
                    {
                        minKFId=it->first->mnId;
                        EldestKF=it->first;
                    }
                }
                mpRefKF=EldestKF;
            }

            nObs--;
            if(nObs<=0)
                bBad=true;
            if(bBad)
                SetBadFlag();

        }
    }
}

void MapCuboid::SetBadFlag()
{
    unordered_map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex>  lock1(mMutexFeatures);
        unique_lock<mutex>  lock2(mMutexPos);
        mbBad=true;
        obs=mObservations;
        mObservations.clear();    
    }
    for(auto it=obs.begin();it!=obs.end();++it)
    {
        KeyFrame* pKF=it->first;
        pKF->EraseMapCuboidMatch(it->second);    
    }
    mpMap->EraseMapCuboid(this);

}
bool MapCuboid::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

    
}