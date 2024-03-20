
#include "../include/map.h"

namespace GL_SLAM
{

    void map_test()
        {
        std::cout<<"map is working "<<std::endl;
        }


    // function responsible for inserting a new keyframe 
    void Map::Insert_Key_Frame(Frame::Frame_Ptr frame_ptr)
    {
        Current_Frame_Ptr = frame_ptr;

        // if the keyframe id is not present in the keyframes map
        if(Map_Keyframes.find(frame_ptr->Keyframe_Id) == Map_Keyframes.end())
        {
            //then the keyframe pointer is added with its id as a map key in the keyframe map
            Map_Keyframes.insert(make_pair(frame_ptr->Keyframe_Id, frame_ptr));
            
            //the keyframe pointer is added with its id as a map key in the active keyframe map
            Map_Active_keyframes.insert(make_pair(frame_ptr->Keyframe_Id, frame_ptr));
        }

        else
        {
            //if the keyframe is already present in the map then we just update the entry
            Map_Keyframes[frame_ptr->Keyframe_Id] = frame_ptr;
            Map_Active_keyframes[frame_ptr->Keyframe_Id] = frame_ptr;
        }

        if(Map_Active_keyframes.size() > Num_Active_Keyframes)
        {
            Remove_Old_Key_Frames();
        }
    }


    // function responsible for inserting a map point
    void Map::Insert_Map_Point(MapPoint::MapPoint_Ptr mappoint_ptr)
    {

        // if the landmark id is not present in the landmarks map
        if(Map_Landmarks.find(mappoint_ptr->Map_Point_Id) == Map_Landmarks.end())
        {
            //then the mappoint pointer is added with its id as a map key in the mappoint map
            Map_Landmarks.insert(make_pair(mappoint_ptr->Map_Point_Id, mappoint_ptr));
            //the mappoint pointer is added with its id as a map key in the active mappoint map
            Map_Active_landmarks.insert(make_pair(mappoint_ptr->Map_Point_Id, mappoint_ptr));
        }

        else
        {
            //if the mappoint is already present in the map then we just update the entry
            Map_Landmarks[mappoint_ptr->Map_Point_Id] = mappoint_ptr;
            Map_Active_landmarks[mappoint_ptr->Map_Point_Id] = mappoint_ptr;
        }
    }


    // removes old keyframes based on distance between two keyframs ,
    void Map::Remove_Old_Key_Frames()
    {
        if(Current_Frame_Ptr == nullptr) // if the currnet frame is null then don't complete the loop
            return;

        double max_distance = 0;
        double min_distance = 9999;
        double max_Keyframe_Id = 0;
        double min_Keyframe_Id = 0;
        const double min_distance_threshold = 0.2;
        auto Current_Twc = Current_Frame_Ptr->Get_Pose().inverse(); // get current camera pose in the world

        for(auto& keyframe: Map_Active_keyframes) // loop over all active key frames sotred in keyframe map
        {
            if(keyframe.second == Current_Frame_Ptr) //  skip the current key frame in the loop
                continue;

            //For each keyframe, it calculates the distance between its pose and the inverse pose of the current_frame.
            //The distance is computed using logarithmic mapping and then normalized.
            auto distance  = (keyframe.second->Get_Pose() * Current_Twc).log().norm(); 

            if(distance > max_distance) // assigning the farthest keyframe
            {
                max_distance = distance;
                max_Keyframe_Id = keyframe.first;
            }

            if(distance < min_distance) // assigning the nearest keyframe
            {
                min_distance = distance;
                min_Keyframe_Id = keyframe.first;
            }

            Frame::Frame_Ptr frame_to_remove = nullptr; // ptr referencing the frame to be removed

            // assigning the nearest key frame from the current keyframe to be removed
            if(min_distance < min_distance_threshold)
            {
                frame_to_remove = Map_Keyframes.at(min_Keyframe_Id);
            }

            // assigning the farthest key frame from the current keyframe to be removed
            else
            {
                frame_to_remove = Map_Keyframes.at(max_Keyframe_Id);
            }

            // removing the keyframes from the active keyframe map
            Map_Active_keyframes.erase(frame_to_remove->Keyframe_Id);

            // removing the features and corresponging map points for the left camera
            for (auto feature_left_to_remove : frame_to_remove->Feature_Ptr_Left)
            {
                if(feature_left_to_remove == nullptr)
                    continue;
                auto map_point_left_to_remove = feature_left_to_remove->Map_Point_Ptr.lock();
                if(map_point_left_to_remove) // check if not expired and not nullptr
                {
                    map_point_left_to_remove->Remove_Observation(feature_left_to_remove);
                }
            }

            // removing the features and corresponging map points for the right camera
            for (auto feature_rigth_to_remove : frame_to_remove->Feature_Ptr_Right)
            {
                if(feature_rigth_to_remove == nullptr)
                    continue;
                auto map_point_rigth_to_remove = feature_rigth_to_remove->Map_Point_Ptr.lock();
                if(map_point_rigth_to_remove) // check if not expired and not nullptr
                {
                    map_point_rigth_to_remove->Remove_Observation(feature_rigth_to_remove);
                }
            }

        }

        Clean_Map();
    }
    
    //remove any landmarks from Map_Active_landmarks that are no longer observed by any keyframe.
    void Map::Clean_Map()
    {
        int count_landmark_to_remove = 0;
        for(auto landmark = Map_Active_landmarks.begin(); landmark != Map_Active_landmarks.end();) // loop over active landmarks map
        {
            //For each landmark, it checks if the observed_times_ member variable is zero. 
            //If so, it means that the landmark has not been observed by any keyframes and is considered inactive
            //In this case, the landmark is removed from the map 
            if(landmark->second->Observed_Times == 0) 
            {
                landmark = Map_Active_landmarks.erase(landmark); // erasing a landmark points to the next one in the map
                count_landmark_to_remove++;

            }
            else
            {
                ++landmark; // jumping to the next landmark in the map
            }
        }
    }
}

