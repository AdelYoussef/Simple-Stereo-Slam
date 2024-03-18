
#include "../include/mappoint.h"
#include "../include/feature.h"



namespace GL_SLAM
{
    void mappoint_test()
        {
        std::cout<<"map point is working "<<std::endl;
        }

    MapPoint::MapPoint(long id, EIGEN_DOUBLE_VEC3 position): Map_Point_Id(id), Point_Pose(position){}


    MapPoint::MapPoint_Ptr MapPoint::Create_New_MapPoint()
    {
        static long Map_Point_F_Id = 0; // init for the first time of the static variable -- factory id 
        MapPoint::MapPoint_Ptr new_map_point(new MapPoint);
        new_map_point->Map_Point_Id = Map_Point_F_Id++; // Map_Point_F_Id increments every time because it's static
        return new_map_point; 
    }


    void MapPoint::Remove_Observation(std::shared_ptr<Feature> remove_feature_ptr)
    {
        std::unique_lock<std::mutex> lck (Map_Point_Mutex);
        //iterating over the Observations pointer
        for (auto iteration = Observation_Ptr.begin() ; iteration != Observation_Ptr.end() ; iteration++)
        {
            // lock converts iteration from a weak pointer to a shared pointer to enable comparison
            if(iteration->lock() == remove_feature_ptr)
            {
                //if a match is found then the corresponding obseration is Removed
                Observation_Ptr.erase(iteration);
                // Resets the map_point member of the provided remove_feature_ptr (pointer to feature class) by setting it to nullptr.
                // This operation may reset the shared ownership of the associated MapPoint.
                remove_feature_ptr->Map_Point_Ptr.reset(); // clears the weak ptr 
                Observed_Times--;
                break;
            }

        }
    }


}
