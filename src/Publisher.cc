// #include "Publisher.h"
// #include <cv_bridge/cv_bridge.h>
// #include "geometry_msgs/PoseStamped.h"
// #include "visualization_msgs/Marker.h"
//
//
// using namespace std;
//
// Publishser::Pulisher(System* pSystem, MapDrawer *pMapDrawer): mpSystem(pSystem), mpMapDrawer(pMapDrawer)
// {
//   pub_ = n_.advertise<geometry_msgs::PoseStamped>("Publisher/current_pose",1);
//   mark_ = n_.advertise<visualization_msgs::Marker>("Publisher/pose_marker", 1);
//   sub_ = n_.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,this);
//
// }
