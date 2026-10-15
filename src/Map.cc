/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include "Map.h"
#include "ua_tag/MapTagData.h"

#include <mutex>

namespace ORB_SLAM3
{

long unsigned int Map::nNextId=0;

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0), mbImuInitialized(false), mnMapChange(0), mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
mbFail(false), mIsInUse(false), mHasTumbnail(false), mbBad(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
{
    mnId=nNextId++;
    mThumbnail = static_cast<GLubyte*>(NULL);
}

Map::Map(int initKFid):mnInitKFid(initKFid), mnMaxKFid(initKFid),/*mnLastLoopKFid(initKFid),*/ mnBigChangeIdx(0), mIsInUse(false),
                       mHasTumbnail(false), mbBad(false), mbImuInitialized(false), mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
                       mnMapChange(0), mbFail(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
{
    mnId=nNextId++;
    mThumbnail = static_cast<GLubyte*>(NULL);
}

Map::~Map()
{
    // Break KF <-> MapTag links first, then release the shared_ptr (avoid a dangling raw pointer in the KF)
    ClearMapTags();

    //TODO: erase all points from memory
    mspMapPoints.clear();

    //TODO: erase all keyframes from memory
    mspKeyFrames.clear();

    if(mThumbnail)
        delete mThumbnail;
    mThumbnail = static_cast<GLubyte*>(NULL);

    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    if(mspKeyFrames.empty()){
        cout << "First KF:" << pKF->mnId << "; Map init KF:" << mnInitKFid << endl;
        mnInitKFid = pKF->mnId;
        mpKFinitial = pKF;
        mpKFlowerID = pKF;
    }
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
    {
        mnMaxKFid=pKF->mnId;
    }
    if(pKF->mnId<mpKFlowerID->mnId)
    {
        mpKFlowerID = pKF;
    }
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

bool Map::AddMapTag(const MapTagPtr &pTag)
{
    if(!pTag)
        return false;

    const int tag_id = pTag->GetId();
    if(tag_id < 0)
        return false;

    unique_lock<mutex> lock(mMutexMap);
    auto it = mMapTags.find(tag_id);
    if(it != mMapTags.end())
        return it->second == pTag;  // the same object succeeds idempotently; a different object is refused

    if(pTag->GetMap() && pTag->GetMap() != this)
        return false;

    pTag->SetMapInternal(this);
    mMapTags.emplace(tag_id, pTag);
    return true;
}

bool Map::EraseMapTag(int tagId)
{
    MapTagPtr pTag;
    {
        unique_lock<mutex> lock(mMutexMap);
        auto it = mMapTags.find(tagId);
        if(it == mMapTags.end())
            return false;
        pTag = it->second;
        mMapTags.erase(it);
    }

    if(!pTag)
        return true;

    const auto observations = pTag->GetObservations();
    for(const auto &kv : observations)
    {
        if(kv.first)
            kv.first->EraseMapTag(tagId);
    }
    pTag->SetMapInternal(nullptr);
    return true;
}

void Map::ClearMapTags()
{
    std::vector<MapTagPtr> tags;
    {
        unique_lock<mutex> lock(mMutexMap);
        tags.reserve(mMapTags.size());
        for(const auto &kv : mMapTags)
            tags.push_back(kv.second);
        mMapTags.clear();
        tag_initialized_ = false;
    }

    for(const MapTagPtr &pTag : tags)
    {
        if(!pTag)
            continue;
        const int tag_id = pTag->GetId();
        const auto observations = pTag->GetObservations();
        for(const auto &kv : observations)
        {
            if(kv.first)
                kv.first->EraseMapTag(tag_id);
        }
        pTag->SetMapInternal(nullptr);
    }
}

Map::MapTagPtr Map::GetMapTag(int tagId) const
{
    unique_lock<mutex> lock(mMutexMap);
    const auto it = mMapTags.find(tagId);
    return (it == mMapTags.end()) ? nullptr : it->second;
}

std::vector<Map::MapTagPtr> Map::GetAllMapTags() const
{
    unique_lock<mutex> lock(mMutexMap);
    std::vector<MapTagPtr> out;
    out.reserve(mMapTags.size());
    for(const auto &kv : mMapTags)
        out.push_back(kv.second);
    return out;
}

bool Map::HasMapTag(int tagId) const
{
    unique_lock<mutex> lock(mMutexMap);
    return mMapTags.find(tagId) != mMapTags.end();
}

std::size_t Map::MapTagsInMap() const
{
    unique_lock<mutex> lock(mMutexMap);
    return mMapTags.size();
}

bool Map::IsTagInitialized() const
{
    unique_lock<mutex> lock(mMutexMap);
    return tag_initialized_;
}

void Map::SetTagInitialized(bool initialized)
{
    unique_lock<mutex> lock(mMutexMap);
    tag_initialized_ = initialized;
}

bool Map::CheckMapTagAssociations(std::string *error) const
{
    auto fail = [&](const std::string &msg) -> bool {
        if(error)
            *error = msg;
        return false;
    };

    unique_lock<mutex> lock(mMutexMap);

    // Index from MapTag to KF, then check that the KF's left/right MapTag observation indices match the indices stored on the MapTag
    for(const auto &tag_kv : mMapTags)
    {
        const int tag_id = tag_kv.first;
        const MapTagPtr &pTag = tag_kv.second;
        if(!pTag)
            return fail("null MapTag in container");
        if(pTag->GetId() != tag_id)
            return fail("MapTag container key != GetId()");
        if(pTag->GetMap() != this)
            return fail("MapTag::GetMap() mismatch");
        if(pTag->IsBad())
            continue;

        const auto kf_obs = pTag->GetObservations();
        for(const auto &obs_kv : kf_obs)
        {
            KeyFrame *pKF = obs_kv.first;
            const auto &idx = obs_kv.second;
            if(!pKF)
                return fail("null KeyFrame in MapTag observations");
            if(pKF->isBad())
                return fail("MapTag observes bad KeyFrame");
            if(pKF->GetMap() != this)
                return fail("observed KF belongs to another Map");
            if(mspKeyFrames.find(pKF) == mspKeyFrames.end())
                return fail("observed KF not in Map keyframe set");

            if(idx.left_index < 0 && idx.right_index < 0)
                return fail("MapTag observation has no valid index");

            KeyFrame::MapTagAssociation assoc;
            if(!pKF->GetMapTagAssociation(tag_id, assoc))
                return fail("KF missing reverse MapTag association");
            if(assoc.pMapTag != pTag.get())
                return fail("KF association points to different MapTag");
            if(assoc.leftObservationIndex != idx.left_index ||
               assoc.rightObservationIndex != idx.right_index)
                return fail("KF/MapTag observation indices mismatch");

            if(idx.left_index < -1 ||
               idx.left_index >= static_cast<int>(pKF->tag_frame_data_.left_.size()))
                return fail("left observation index out of range");
            if(idx.right_index < -1 ||
               idx.right_index >= static_cast<int>(pKF->tag_frame_data_.right_.size()))
                return fail("right observation index out of range");
            if(idx.left_index >= 0 &&
               pKF->tag_frame_data_.left_[idx.left_index].tag_id != tag_id)
                return fail("left TagObservation.tag_id mismatch");
            if(idx.right_index >= 0 &&
               pKF->tag_frame_data_.right_[idx.right_index].tag_id != tag_id)
                return fail("right TagObservation.tag_id mismatch");
        }
    }

    // Index from KF to MapTag, then check that the MapTag's left/right KF observation indices match the indices stored on the KF
    for(KeyFrame *pKF : mspKeyFrames)
    {
        if(!pKF)
            return fail("null KeyFrame in map");
        if(pKF->isBad())
            continue;

        const auto associations = pKF->GetMapTagAssociations();
        for(const auto &assoc_kv : associations)
        {
            const int tag_id = assoc_kv.first;
            const KeyFrame::MapTagAssociation &assoc = assoc_kv.second;
            if(!assoc.pMapTag)
                return fail("KF association has null MapTag");
            if(assoc.pMapTag->GetId() != tag_id)
                return fail("KF association key != MapTag::GetId()");
            if(assoc.leftObservationIndex < 0 && assoc.rightObservationIndex < 0)
                return fail("KF association has no valid index");

            const auto tag_it = mMapTags.find(tag_id);
            if(tag_it == mMapTags.end() || tag_it->second.get() != assoc.pMapTag)
                return fail("KF association MapTag not in Map container");
            if(assoc.pMapTag->GetMap() != this)
                return fail("KF association MapTag::GetMap() mismatch");
            if(!assoc.pMapTag->IsInKeyFrame(pKF))
                return fail("MapTag missing reverse KF observation");

            const auto kf_obs = assoc.pMapTag->GetObservations();
            const auto obs_it = kf_obs.find(pKF);
            if(obs_it == kf_obs.end())
                return fail("MapTag observations missing KF");
            if(obs_it->second.left_index != assoc.leftObservationIndex ||
               obs_it->second.right_index != assoc.rightObservationIndex)
                return fail("reverse observation indices mismatch");
        }
    }

    if(error)
        error->clear();
    return true;
}

void Map::SetImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    mbImuInitialized = true;
}

bool Map::isImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbImuInitialized;
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);
    if(mspKeyFrames.size()>0)
    {
        if(pKF->mnId == mpKFlowerID->mnId)
        {
            vector<KeyFrame*> vpKFs = vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
            sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
            mpKFlowerID = vpKFs[0];
        }
    }
    else
    {
        mpKFlowerID = 0;
    }

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetId()
{
    return mnId;
}
long unsigned int Map::GetInitKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnInitKFid;
}

void Map::SetInitKFid(long unsigned int initKFif)
{
    unique_lock<mutex> lock(mMutexMap);
    mnInitKFid = initKFif;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

KeyFrame* Map::GetOriginKF()
{
    return mpKFinitial;
}

void Map::SetCurrentMap()
{
    mIsInUse = true;
}

void Map::SetStoredMap()
{
    mIsInUse = false;
}

void Map::clear()
{
//    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
//        delete *sit;

    // Clear the MapTag two-way associations first, then disconnect the KF
    ClearMapTags();

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
    {
        KeyFrame* pKF = *sit;
        pKF->UpdateMap(static_cast<Map*>(NULL));
//        delete *sit;
    }

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = mnInitKFid;
    mbImuInitialized = false;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
    mbIMU_BA1 = false;
    mbIMU_BA2 = false;
}

bool Map::IsInUse()
{
    return mIsInUse;
}

void Map::SetBad()
{
    mbBad = true;
}

bool Map::IsBad()
{
    return mbBad;
}


void Map::ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel)
{
    unique_lock<mutex> lock(mMutexMap);

    // Body position (IMU) of first keyframe is fixed to (0,0,0)
    Sophus::SE3f Tyw = T;
    Eigen::Matrix3f Ryw = Tyw.rotationMatrix();
    Eigen::Vector3f tyw = Tyw.translation();

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(); sit!=mspKeyFrames.end(); sit++)
    {
        KeyFrame* pKF = *sit;
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        Twc.translation() *= s;
        Sophus::SE3f Tyc = Tyw*Twc;
        Sophus::SE3f Tcy = Tyc.inverse();
        pKF->SetPose(Tcy);
        Eigen::Vector3f Vw = pKF->GetVelocity();
        if(!bScaledVel)
            pKF->SetVelocity(Ryw*Vw);
        else
            pKF->SetVelocity(Ryw*Vw*s);

    }
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(); sit!=mspMapPoints.end(); sit++)
    {
        MapPoint* pMP = *sit;
        pMP->SetWorldPos(s * Ryw * pMP->GetWorldPos() + tyw);
        pMP->UpdateNormalAndDepth();
    }
    // Same handling as a KF: scale the translation first, then left-multiply by Tyw (gravity alignment / IMU scale)
    for(const auto &kv : mMapTags)
    {
        const MapTagPtr &pTag = kv.second;
        if(!pTag)
            continue;
        if(pTag->HasPose())
        {
            Sophus::SE3f Twt = pTag->GetPose();
            Twt.translation() *= s;
            pTag->SetPose(Tyw * Twt);
        }
        if(pTag->HasMirrorPose())
        {
            Sophus::SE3f TwtMirror = pTag->GetMirrorPose();
            TwtMirror.translation() *= s;
            pTag->SetMirrorPose(Tyw * TwtMirror);
        }
    }
    if(!mMapTags.empty())
        cout << "[TagMap] ApplyScaledRotation tags=" << mMapTags.size()
             << " scale=" << s << endl;
    mnMapChange++;
}

void Map::SetInertialSensor()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIsInertial = true;
}

bool Map::IsInertial()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIsInertial;
}

void Map::SetIniertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA1 = true;
}

void Map::SetIniertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA2 = true;
}

bool Map::GetIniertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA1;
}

bool Map::GetIniertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA2;
}

void Map::ChangeId(long unsigned int nId)
{
    mnId = nId;
}

unsigned int Map::GetLowerKFID()
{
    unique_lock<mutex> lock(mMutexMap);
    if (mpKFlowerID) {
        return mpKFlowerID->mnId;
    }
    return 0;
}

int Map::GetMapChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChange;
}

void Map::IncreaseChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChange++;
}

int Map::GetLastMapChange()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChangeNotified;
}

void Map::SetLastMapChange(int currentChangeId)
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChangeNotified = currentChangeId;
}

void Map::PreSave(std::set<GeometricCamera*> &spCams)
{
    int nMPWithoutObs = 0;
    for(MapPoint* pMPi : mspMapPoints)
    {
        if(!pMPi || pMPi->isBad())
            continue;

        if(pMPi->GetObservations().size() == 0)
        {
            nMPWithoutObs++;
        }
        map<KeyFrame*, std::tuple<int,int>> mpObs = pMPi->GetObservations();
        for(map<KeyFrame*, std::tuple<int,int>>::iterator it= mpObs.begin(), end=mpObs.end(); it!=end; ++it)
        {
            if(it->first->GetMap() != this || it->first->isBad())
            {
                pMPi->EraseObservation(it->first);
            }

        }
    }

    // Saves the id of KF origins
    mvBackupKeyFrameOriginsId.clear();
    mvBackupKeyFrameOriginsId.reserve(mvpKeyFrameOrigins.size());
    for(int i = 0, numEl = mvpKeyFrameOrigins.size(); i < numEl; ++i)
    {
        mvBackupKeyFrameOriginsId.push_back(mvpKeyFrameOrigins[i]->mnId);
    }


    // Backup of MapPoints
    mvpBackupMapPoints.clear();
    for(MapPoint* pMPi : mspMapPoints)
    {
        if(!pMPi || pMPi->isBad())
            continue;

        mvpBackupMapPoints.push_back(pMPi);
        pMPi->PreSave(mspKeyFrames,mspMapPoints);
    }

    // Backup of KeyFrames
    mvpBackupKeyFrames.clear();
    for(KeyFrame* pKFi : mspKeyFrames)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        mvpBackupKeyFrames.push_back(pKFi);
        pKFi->PreSave(mspKeyFrames,mspMapPoints, spCams);
    }

    mnBackupKFinitialID = -1;
    if(mpKFinitial)
    {
        mnBackupKFinitialID = mpKFinitial->mnId;
    }

    mnBackupKFlowerID = -1;
    if(mpKFlowerID)
    {
        mnBackupKFlowerID = mpKFlowerID->mnId;
    }

}

void Map::PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc/*, map<long unsigned int, KeyFrame*>& mpKeyFrameId*/, map<unsigned int, GeometricCamera*> &mpCams)
{
    std::copy(mvpBackupMapPoints.begin(), mvpBackupMapPoints.end(), std::inserter(mspMapPoints, mspMapPoints.begin()));
    std::copy(mvpBackupKeyFrames.begin(), mvpBackupKeyFrames.end(), std::inserter(mspKeyFrames, mspKeyFrames.begin()));

    map<long unsigned int,MapPoint*> mpMapPointId;
    for(MapPoint* pMPi : mspMapPoints)
    {
        if(!pMPi || pMPi->isBad())
            continue;

        pMPi->UpdateMap(this);
        mpMapPointId[pMPi->mnId] = pMPi;
    }

    map<long unsigned int, KeyFrame*> mpKeyFrameId;
    for(KeyFrame* pKFi : mspKeyFrames)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateMap(this);
        pKFi->SetORBVocabulary(pORBVoc);
        pKFi->SetKeyFrameDatabase(pKFDB);
        mpKeyFrameId[pKFi->mnId] = pKFi;
    }

    // References reconstruction between different instances
    for(MapPoint* pMPi : mspMapPoints)
    {
        if(!pMPi || pMPi->isBad())
            continue;

        pMPi->PostLoad(mpKeyFrameId, mpMapPointId);
    }

    for(KeyFrame* pKFi : mspKeyFrames)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->PostLoad(mpKeyFrameId, mpMapPointId, mpCams);
        pKFDB->add(pKFi);
    }


    if(mnBackupKFinitialID != -1)
    {
        mpKFinitial = mpKeyFrameId[mnBackupKFinitialID];
    }

    if(mnBackupKFlowerID != -1)
    {
        mpKFlowerID = mpKeyFrameId[mnBackupKFlowerID];
    }

    mvpKeyFrameOrigins.clear();
    mvpKeyFrameOrigins.reserve(mvBackupKeyFrameOriginsId.size());
    for(int i = 0; i < mvBackupKeyFrameOriginsId.size(); ++i)
    {
        mvpKeyFrameOrigins.push_back(mpKeyFrameId[mvBackupKeyFrameOriginsId[i]]);
    }

    mvpBackupMapPoints.clear();
}


} //namespace ORB_SLAM3
