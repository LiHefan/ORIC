#ifndef MAPCUBOID_H
#define MAPCUBOID_H

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "g2o_cuboid.h"

#include <opencv2/core/core.hpp>
#include <mutex>
#include <unordered_map>
#include <string>

namespace ORB_SLAM2
{
class KeyFrame;
class Map;
class Frame;

class MapCuboid
{
public:
    MapCuboid(Map *pMap, bool update_index=false);
    void SetWorldPos(const g2o::cuboid &Pos);   //set cuboid pose in world coordinate
    g2o::cuboid GetWorldPos();              //get cuboid pose in world coordinate
    g2o::cuboid GetWorldPosInv();

    void SetReferenceKeyFrame(KeyFrame* refKF);
    KeyFrame* GetReferenceKeyFrame();
    KeyFrame* GetLatestKeyFrame();
    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool isInKeyFrame(KeyFrame* pKF);

    std::unordered_map<KeyFrame*, size_t> GetObservations();
    int Observations();    
    void addObservation(KeyFrame *pKF, size_t idx); //cuboid observed by frames
    void EraseObservation(KeyFrame *pKF);

    void SetBadFlag();
    bool isBad();

public:
    long unsigned int mnId; //unique id for this cuboid
    static long unsigned int nNextId; //static member, automatically add 1 when new
    int nObs;               //num of frame observations

    //variables used by local mapping
    long unsigned int mnBALocalForKF;
    bool mbBeenOpti=false;
    int mnPointCuboidBACounter=-1;

    long unsigned int mnAssoId;
    static std::mutex mGlobalMutex;

    //---------------- for local MapCuboid ------------------//
    int cuboid_id;
    int cuboid_graph_id;
    std::string cuboid_classname;
    // g2o::cuboid cuboid_global_data;
    // g2o::cuboid cuboid_local_meas;
    // double meas_quality;
    // Eigen::Vector4d bbox_vec;
    // cv::Rect bbox_2d;    
    
protected:
    g2o::cuboid mCuboidTwc;
    g2o::cuboid mCuboidTcw;

    //Keyframes observing the object and assosiacted localcuboid index in keyframe
    std::unordered_map<KeyFrame*, size_t> mObservations;
    KeyFrame* mpRefKF;      //reference KeyFrame,first frame see this
    KeyFrame* mpLatestKF;   //latest frame see this
    Map* mpMap;
    //Bad flag
    bool mbBad;

    

    std::mutex mMutexPos;
    std::mutex mMutexFeatures;
    std::mutex mMutexParam;
};


}

#endif