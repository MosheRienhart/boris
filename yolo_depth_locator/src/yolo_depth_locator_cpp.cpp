#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// Includes if visualization with OpenCV becomes a priority
// #include <image_transport/image_transport.h>
// #include <opencv2/highgui/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>
ros::Publisher pub;
  

geometry_msgs::Point px2mm(geometry_msgs::Point& centerPixel)
{

  // Intrinsic values recorded from /camera/color/topic on RS435i
  double fx, fy, cx, cy;
  geometry_msgs::Point outPt;
  fx = 615.3138427734375;
  cx = 315.85211181640625;
  fy = 615.692138671875;
  cy = 248.609130859375;

  
  outPt.x = (centerPixel.x*-cx)/fx*centerPixel.z;
  outPt.y = (centerPixel.y*-cy)/fy*centerPixel.z;
  outPt.z = centerPixel.z;

}


void callback(const sensor_msgs::ImageConstPtr& depthImg, const darknet_ros_msgs::BoundingBoxesConstPtr& boxImges)
  {
    geometry_msgs::Point objectCenter;
      for(int i=0; i < boxImges->bounding_boxes.size(); i++)
      {
        darknet_ros_msgs::BoundingBox box = boxImges->bounding_boxes[i];
        double dlikelihood = box.probability;
        std::string dclass = box.Class;
        // if(dlikelihood >= 0.9 && dclass.compare("Trash") == 0)
        if(dlikelihood >= 0.7)
        {

          if ((i == 0))
          {
            objectCenter.x = (box.xmax - box.xmin)/2;
            objectCenter.y = (box.ymax - box.ymin)/2;
            objectCenter.z = depthImg->data[int(objectCenter.x),int(objectCenter.y)];
          }
          else
          {
            geometry_msgs::Point tmpPt;
            tmpPt.x = (box.xmax - box.xmin)/2;
            tmpPt.y = (box.ymax - box.ymin)/2;
            if(objectCenter.z > depthImg->data[int(tmpPt.x),int(tmpPt.y)] )
            {
              objectCenter.x = (box.xmax - box.xmin)/2;
              objectCenter.y = (box.ymax - box.ymin)/2;
              objectCenter.z = depthImg->data[int(objectCenter.x),int(objectCenter.y)];

            }
          }

        }

      }

      if(!(objectCenter.x==0 && objectCenter.y == 0 && objectCenter.z ==0))
        
        {pub.publish(px2mm(objectCenter));}

  }
 
using namespace sensor_msgs;
using namespace message_filters;

int main(int argc, char **argv)
{
  // The name of the node
  ros::init(argc, argv, "yolo_depth_detect");
   
  // Default handler for nodes in ROS
  ros::NodeHandle nh;

  pub = nh.advertise<geometry_msgs::Point>("goalpt",100);

  message_filters::Subscriber<sensor_msgs::Image> sub(nh,"/camera/aligned_depth_to_color/image_raw",1);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> darknetImg(nh,"/darknet_ros/bounding_boxes",1);

  typedef sync_policies::ApproximateTime<Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), sub, darknetImg);
  sync.registerCallback(boost::bind(&callback, _1, _2));


  ros::spin();
   

}