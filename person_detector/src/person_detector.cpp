

// PAL headers
#include <pal_detection_msgs/Detections2d.h>
#include "tf_transforms.hpp"

// ROS headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/Detection2DArray.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include <actionlib/server/simple_action_server.h>
#include <person_detector/person_detectorAction.h>

#include <string>
#include <algorithm>

// OpenCV headers
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Boost headers
#include <boost/scoped_ptr.hpp>
#include <boost/foreach.hpp>

// Std C++ headers
#include <vector>
#include <stdlib.h> 

/**
 * @brief The PersonDetector class encapsulating an image subscriber and the OpenCV's CPU HOG person detector
 *
 * @example rosrun person_detector_opencv person_detector image:=/camera/image _rate:=5 _scale:=0.5
 *
 */
class PersonDetector
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber img_sub_;
    image_transport::Publisher img_pub_;
    ros::Subscriber bbox_sub_;  //To be added
    ros::Publisher _detectionPub;
    ros::Publisher _PosePub;
    ros::Publisher _BoolPub;
    bool detected_people_;
    cv::Mat _cameraMatrix;
    int goal_;

public:
    PersonDetector(ros::NodeHandle& nh, std::string name);
    void bbox_callback(const vision_msgs::Detection3DArray &obstacles_msg);
    cv::Mat _mat_image;
    virtual ~PersonDetector();
    void goalCB()
    {
      // accept the new goal
      goal_ = as_.acceptNewGoal()->goal;
    };
    void preemptCB()
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_.setPreempted();
    };

protected:
  mutable cv_bridge::CvImage _cvImgDebug;
  ros::Time _imgTimeStamp;
  void detectPersons(const cv::Mat& img,
                     std::vector<cv::Rect>& detections);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  // void people_callback(const vision_msgs::Detection2DArray &people_msg);
  boost::scoped_ptr<cv::HOGDescriptor> _hogCPU;
  void publishDebugImage(cv::Mat& img,
                         const std::vector<cv::Rect>& detections) const;
  actionlib::SimpleActionServer<person_detector::person_detectorAction> as_;
  person_detector::person_detectorFeedback feedback_;
  person_detector::person_detectorResult result_;
  std::string action_name_;
};


PersonDetector::PersonDetector(ros::NodeHandle& nh,std::string name):
it_(nh),
nh_(nh),
as_(nh, name, false),
action_name_(name)
{
  ROS_INFO("Waiting for camera_info topic ...");
  sensor_msgs::CameraInfoConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/xtion/rgb/camera_info");
  ROS_INFO("Ok");
  as_.registerGoalCallback(boost::bind(&PersonDetector::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&PersonDetector::preemptCB, this));

  _cameraMatrix = cv::Mat::zeros(3,3, CV_32FC1);

  //get focal lengths and image centre from P matrix
  _cameraMatrix.at<float>(0,0) =    msg->P[0]; _cameraMatrix.at<float>(0,1) =         0; _cameraMatrix.at<float>(0,2) = msg->P[2];
  _cameraMatrix.at<float>(1,0) =            0; _cameraMatrix.at<float>(1,1) = msg->P[5]; _cameraMatrix.at<float>(1,2) = msg->P[6];
  _cameraMatrix.at<float>(2,0) =            0; _cameraMatrix.at<float>(2,1) =         0; _cameraMatrix.at<float>(2,2) =         1;
  // //rectified image has no distortion
  // _distCoeff = cv::Mat(1, 4, CV_32F, cv::Scalar(0.00001));
  _hogCPU.reset( new cv::HOGDescriptor );
  // (*_hogCPU).winSize=cv::Size(48, 96);
  // _hogCPU->setSVMDetector( cv::HOGDescriptor::getDaimlerPeopleDetector());
  (*_hogCPU).winSize=cv::Size(64, 128);
  _hogCPU->setSVMDetector( cv::HOGDescriptor::getDefaultPeopleDetector());
  // std::cout<<"Cam_matrix=\n"<<_cameraMatrix<<std::endl<<std::endl;
  const std::string &topic = "/xtion/rgb/image_raw";
  const std::string &transport="raw";
  image_transport::TransportHints transportHint(transport);
  img_sub_   = it_.subscribe(topic, 1, &PersonDetector::imageCallback, this, transport);
  bbox_sub_ = nh_.subscribe("/pcl_obstacle_detector_node/detections", 1, &PersonDetector::bbox_callback, this);
  _detectionPub = nh_.advertise<vision_msgs::Detection3DArray>("/person_detector/bbox", 10);
  _PosePub = nh_.advertise<geometry_msgs::PoseArray>("/person_detector/pose", 10);
  _BoolPub = nh_.advertise<std_msgs::Bool>("/hri_input", 10);
  img_pub_ = it_.advertise("/person_detector/visual", 3);//publisher for visualization
  detected_people_= false;
  cv::namedWindow("person detections");
  as_.start();
}
PersonDetector::~PersonDetector()
{
  cv::destroyWindow("person detections");
  as_.shutdown();
}


void PersonDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cvImgPtr;
  cvImgPtr = cv_bridge::toCvCopy(msg);
  _mat_image= cvImgPtr->image;
  _imgTimeStamp = msg->header.stamp;
  
}
void PersonDetector::bbox_callback(const vision_msgs::Detection3DArray &obstacles_msg)
{
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
    if (as_.isPreemptRequested())
      return;
    if (goal_!=1)
      return;
    // vision_msgs::Detection3DArray detections_filtered;
    std::vector<cv::Rect> movingObjects;
    cv::Mat top_left= cv::Mat::ones(3,1,CV_32FC1);
    cv::Mat down_right= cv::Mat::ones(3,1,CV_32FC1);
    cv::Mat top_left_2d,down_right_2d;
    cv::Rect image_rect= cv::Rect(0, 0, _mat_image.size().width, _mat_image.size().height);
    cv::Rect bbox_rect;
    int x,y,height,width;
    vision_msgs::Detection3DArray people_msg;
    std::vector<cv::Rect> debug_windows;
    pal::TfTransformGetter trans_getter;
    tf::Transform transform;
    std::string newFrameId = "map";
    std::string oldFrameId = "xtion_rgb_optical_frame";
    std_msgs::Bool bool_detected;
    vision_msgs::Detection3D box;
    geometry_msgs::PoseArray pose_msgs;
    geometry_msgs::Pose pose;
    // std::cout<<_mat_image<<std::endl;
    // sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", _mat_image).toImageMsg();
    // pub_msg->header=obstacles_msg.header;
    // img_pub_.publish(pub_msg);
    detected_people_= false;
    
    if (obstacles_msg.detections.empty())
    {
        ROS_INFO_STREAM("No object detected");
    }
    else
    {
      ROS_INFO_STREAM("Object detected");
      std::vector<cv::Rect> foundLocations;
      _hogCPU->detectMultiScale(_mat_image,
                            foundLocations,
                            0.8,                //hit threshold: decrease in order to increase number of detections but also false alarms
                            cv::Size(8,8),    //win stride
                            cv::Size(0,0),    //padding 24,16
                            1.02,             //scaling
                            1.0,                //final threshold
                            false);            //use mean-shift to fuse detections
      if (!foundLocations.empty()){
          trans_getter.getTransform(oldFrameId,newFrameId,transform);
          // std::cout<<(float)transform(0,0) <<std::endl;
          for (auto i = obstacles_msg.detections.begin(); i != obstacles_msg.detections.end(); i++)
          {
            if  ((*i).bbox.center.position.z > 0)
                top_left.at<float>(0,0)=(*i).bbox.center.position.x-(*i).bbox.size.x/2;
                top_left.at<float>(1,0)=(*i).bbox.center.position.y-(*i).bbox.size.y/2;
                top_left.at<float>(2,0)=(*i).bbox.center.position.z;
                down_right.at<float>(0,0)=(*i).bbox.center.position.x+(*i).bbox.size.x/2;
                down_right.at<float>(1,0)=(*i).bbox.center.position.y+(*i).bbox.size.y/2;
                down_right.at<float>(2,0)=(*i).bbox.center.position.z;
                top_left_2d=_cameraMatrix*top_left;
                down_right_2d=_cameraMatrix*down_right;
                x=(int) (top_left_2d.at<float>(0,0)/top_left_2d.at<float>(2,0));
                y=(int) (top_left_2d.at<float>(1,0)/top_left_2d.at<float>(2,0));
                width=(int) (down_right_2d.at<float>(0,0)/down_right_2d.at<float>(2,0))-x;
                height=(int) (down_right_2d.at<float>(1,0)/down_right_2d.at<float>(2,0))-y;
                // std::cout<<"x:"<<x<<std::endl;
                // std::cout<<"y:"<<y<<std::endl;
                // std::cout<<"width:"<<width<<std::endl;
                // std::cout<<"height:"<<height<<std::endl;
                bbox_rect = cv::Rect(x, y, width, height);
                // std::cout<<"bbox_rect:"<<bbox_rect<<std::endl;
                // std::cout<<"image_rect:"<<image_rect<<std::endl;
                // cv::Rect test_rect= image_rect & bbox_rect;
                // std::cout<<"test_mat:"<<test_rect<<std::endl;
                if (((float)(image_rect & bbox_rect).area())/((float) bbox_rect.area())>0.6){
                    // std::cout<<((float)(image_rect & bbox_rect).area())/((float) bbox_rect.area())<<std::endl;
                    bbox_rect=image_rect & bbox_rect;
                    float bbox_area=(float) bbox_rect.area(); 
                    for (auto j = foundLocations.begin(); j != foundLocations.end(); j++){
                        // std::cout<<*j<<std::endl;
                        debug_windows.push_back(*j);
                        // people_msg.detections.push_back(*i);
                        float detection_area=(float) j->area();
                        cv::Rect intersect= *j & bbox_rect;
                        float Intersection_area=(float)intersect.area();
                        if ((Intersection_area/bbox_area >0.4) && (Intersection_area/detection_area >0.2)){
                            // debug_windows.push_back(bbox_rect);
                            detected_people_= true;
                            people_msg.detections.push_back(*i);
                            std::cout<<"Person Detected!"<<std::endl;
                            tf::Vector3 center_cam_frame = tf::Vector3((*i).bbox.center.position.x, (*i).bbox.center.position.y, (*i).bbox.center.position.z);
                            // tf::Vector3 top_left_vec=tf::Vector3(top_left.at<float>(0,0), 
                            //                                      top_left.at<float>(1,0), 
                            //                                      top_left.at<float>(2,0));
                            // tf::Vector3 down_right_vec=tf::Vector3(down_right.at<float>(0,0), 
                            //                                        down_right.at<float>(1,0), 
                            //                                        down_right.at<float>(2,0));
                            // tf::Vector3 size_vec = trans_point(down_right_vec, transform)-trans_point(top_left_vec, transform);                                    
                            tf::Vector3 center = trans_point(center_cam_frame, transform);
                            pose.position.x=(float) center.getX();
                            pose.position.y=(float) center.getY();
                            pose.position.z=(float) center.getZ();
                            pose_msgs.poses.push_back(pose);
                            // box=*i;
                            // box.bbox.size.x = (float) abs(size_vec.getX());
                            // box.bbox.size.y = (float) abs(size_vec.getY());
                            // box.bbox.size.z = (float) abs(size_vec.getZ());
                            // box.bbox.center.position.x=(float) center.getX();
                            // box.bbox.center.position.y=(float) center.getY();
                            // box.bbox.center.position.z=(float) center.getZ();
                            // box.header.frame_id = newFrameId;
                            // people_msg.detections.push_back(box);
                            break;
                        }
                        
                    }
                    
                }
          }
      }
      bool_detected.data=detected_people_;
      pose_msgs.header.frame_id=newFrameId;
      people_msg.header.frame_id=oldFrameId;
      _detectionPub.publish(people_msg);
      _PosePub.publish(pose_msgs);
      _BoolPub.publish(bool_detected);
      publishDebugImage(_mat_image, debug_windows);
      if (detected_people_){
        feedback_.status=1;
        result_.status=1;
        as_.setSucceeded(result_);
      }
      else{
        feedback_.status=0;
        result_.status=0;
      }
      as_.publishFeedback(feedback_);
      // sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(header, "bgr8", _mat_image).toImageMsg();
      // img_pub_.publish(pub_msg);
    }
    
};


void PersonDetector::publishDebugImage(cv::Mat& img,
                                       const std::vector<cv::Rect>& detections) const
{
  //draw detections
  BOOST_FOREACH(const cv::Rect& roi, detections)
  {
    cv::rectangle(img, roi, CV_RGB(0,255,0), 2);
  }

  if ( img.channels() == 3 && img.depth() == CV_8U )
    _cvImgDebug.encoding = sensor_msgs::image_encodings::BGR8;

  else if ( img.channels() == 1 && img.depth() == CV_8U )
    _cvImgDebug.encoding = sensor_msgs::image_encodings::MONO8;
  else
    throw std::runtime_error("Error in Detector2dNode::publishDebug: only 24-bit BGR or 8-bit MONO images are currently supported");

  _cvImgDebug.image = img;
  sensor_msgs::Image imgMsg;
  imgMsg.header.stamp = _imgTimeStamp;
  _cvImgDebug.toImageMsg(imgMsg); //copy image data to ROS message

  img_pub_.publish(imgMsg);
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"person_detector"); // Create and name the Node
  ros::NodeHandle nh;

  ros::CallbackQueue cbQueue;
  nh.setCallbackQueue(&cbQueue);



  double freq = 10;


  ROS_INFO_STREAM("Creating person detector ...");

  PersonDetector detector(nh,ros::this_node::getName());

  ROS_INFO_STREAM("Spinning to serve callbacks ...");

  ros::Rate rate(freq);
  while ( ros::ok() )
  {
    cbQueue.callAvailable();
    rate.sleep();
  }

  return 0;
}

namespace pal {

void convert(const Eigen::Matrix4d& transform,
             geometry_msgs::Pose& pose)
{
  tf::Transform tfTransform;
  tfTransform.setOrigin( tf::Vector3(transform(0,3), transform(1,3), transform(2,3)) );
  tf::Matrix3x3 basis;

  basis.setValue(transform(0,0), transform(0,1), transform(0,2),
                 transform(1,0), transform(1,1), transform(1,2),
                 transform(2,0), transform(2,1), transform(2,2));
  tfTransform.setBasis(basis);

  geometry_msgs::PoseStamped poseMsg;
  tf::poseTFToMsg(tfTransform, pose);
}

TfTransformGetter::TfTransformGetter()
{

}

TfTransformGetter::~TfTransformGetter()
{

}

void TfTransformGetter::getTransform(const std::string& childFrame,
                                     const std::string& parentFrame,
                                     tf::Transform& transform)
{
  std::string errMsg;

  if ( !_tfListener.waitForTransform(parentFrame,
                                     childFrame,
                                     ros::Time(0),
                                     ros::Duration(1.5),
                                     ros::Duration(0.01),
                                     &errMsg)
       )
  {
    throw std::runtime_error("TfTransformGetter::getTransform: unable to get pose from TF: "
                             + errMsg);
  }
  else
  {
    try
    {
      tf::StampedTransform stampedTransform;
      _tfListener.lookupTransform( parentFrame, childFrame,
                                   ros::Time(0),                  //get latest available
                                   stampedTransform);
      transform = stampedTransform;
    }
    catch ( const tf::TransformException& e)
    {
      std::runtime_error("TfTransformGetter::getTransform: error in lookupTransform of " +
                         childFrame + " in " + parentFrame);
    }
  }
}

} //pal

tf::Vector3 trans_point(const tf::Vector3& point, tf::Transform& transform){
  tf::Vector3 trans_point=transform(point);
  return trans_point;
}