
#include "../include/frame.h"




 namespace GL_SLAM
 {
    void frame_test()
        {
        std::cout<<"frame is working "<<std::endl;
        }


    // constructor of the frame struct --  initializing the member variables with the provided arguments.
    Frame::Frame(long id, double time_stamp, const SOPHUS_DOUBLE_SE3 &pose, const CV_MATRIX &left_img, const CV_MATRIX &right_img)
    :Frame_Id(id), Time_Stamp(time_stamp), Camera_Pose(pose), Left_Image(left_img), Right_Image(right_img) {}


    //This is a static member function that creates a new instance of the Frame class. 
    //It initializes the Frame_Id of the new frame with a static counter (F_id) to ensure each frame has a unique ID.
    Frame::Frame_Ptr Frame::Create_Frame()
     {
        static long F_id = 0; // init for the first time of the static variable
        Frame::Frame_Ptr New_Frame(new Frame);
        New_Frame->Frame_Id = F_id++; // F_id increments every time because it's static
        return New_Frame;
     }

    //this member function sets the frame as a keyframe by setting the Is_Keyframe flag to true 
    // assigning a keyframe ID (Keyframe_Id) using another static counter (keyframe_F_id).
    void Frame::Set_Keyframe()
     {
        static long keyframe_F_id = 0; // init for the first time of the static variable
        Is_Keyframe = true;
        Keyframe_Id = keyframe_F_id++ ; // keyframe_F_id increments every time because it's static

     }



 }
