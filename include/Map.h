/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "MapCuboid.h"
#include "KeyFrame.h"
#include <set>
#include <vector>
#include <string>
#include <mutex>
#include <unordered_map>



namespace ORB_SLAM2
{

class MapCuboid;
class MapPoint;
class KeyFrame;

class KFIdCompare
{
public:
    bool operator()(const KeyFrame* kfleft, const KeyFrame* kfright) const;
};

class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void AddMapCuboid(MapCuboid* pMC);
    void EraseMapPoint(MapPoint* pMP);
    void EraseMapCuboid(MapCuboid* pMC);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapCuboid*> GetAllMapCuboids();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned int KeyFramesInMap();
    long unsigned int MapCuboidsInMap();

    long unsigned int GetMaxKFid();

    void clear();

    std::vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<MapCuboid*> mspMapCuboids;
    std::set<KeyFrame*,KFIdCompare> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;
    KeyFrame* mLatestKF;
    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;
//  ------------------------------------------------------------------------
//  add for IMU
public:
    //  Update after an absolute scale is available
    void UpdateScale(const double &scale);
};

} //namespace ORB_SLAM

#endif // MAP_H
