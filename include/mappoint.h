

#pragma once


#ifndef GL_SLAM_MAPPOINT_H
#define GL_SLAM_MAPPOINT_H


#include "../include/common_include.h"

namespace GL_SLAM
{
  void mappoint_test();

    struct Feature;
    struct Frame;

    struct MapPoint
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;


      // create smart pointer to MapPoint struct
      // smart pointers provides automatic memory management, including reference counting, and helps prevent memory leaks.

      typedef std::shared_ptr<MapPoint> MapPoint_Ptr;

      unsigned long Map_Point_Id = 0;  // ID OF MapPoint
      bool Is_Outlier = false; // state of MapPoint -- whether it is an outlier (false MapPoint) or it is not an outlier
      EIGEN_DOUBLE_VEC3 Point_Pose = EIGEN_DOUBLE_VEC3::Zero(); // 3D coordinates of this MapPoint
      std::mutex Map_Point_Mutex; // mutex used for datalock -- to prevent change while in multi-threading
      int Observed_Times = 0;  // number of times a map point is being observed by orb


          /*
        --create as smart pointer to MapPoint struct

        --weak pointers doesnt contribute to the reference count of the managed object

        --this is often used in scenarios where you want to observe or access the MapPoint object
          without preventing it from being deleted when all strong references are gone.
        */

      std::list<std::weak_ptr<Feature>> Observation_Ptr;



      //default constructor for the MapPoint struct
      //It initializes an instance of the struct without any parameters
      MapPoint(){}

      //initializer constructor for the MapPoint struct
      MapPoint(long id , EIGEN_DOUBLE_VEC3 position);

      // get 3D pose of a MapPoint using thread save process
      EIGEN_DOUBLE_VEC3 Get_Pose ()
      {
        std::unique_lock<std::mutex> lck(Map_Point_Mutex);
        return Point_Pose;
      }

      // set 3D pose of a MapPoint using thread save process
      void Set_Pose(const EIGEN_DOUBLE_VEC3 &pose)
      {
        std::unique_lock<std::mutex> lck(Map_Point_Mutex);
        Point_Pose = pose;
      }


      // for a specific point seen more than once 
      //it is added to the observation pointer
      // the observation counter is incremented
      void Add_Observation(std::shared_ptr<Feature> add_feature_ptr)
      {
        std::unique_lock<std::mutex> lck (Map_Point_Mutex);
        Observation_Ptr.push_back(add_feature_ptr);
        Observed_Times ++;
      }

      // for a specific point seen more than once 
      //it is added to the observation pointer
      // the observation counter is incremented
      
      void Remove_Observation(std::shared_ptr<Feature> remove_feature_ptr);

      std::list<std::weak_ptr<Feature>> Get_Observations()
      {
        std::unique_lock<std::mutex> lck(Map_Point_Mutex);
        return Observation_Ptr;
      }


      static MapPoint::MapPoint_Ptr Create_New_MapPoint();



    };

}


#endif
