#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <sstream>
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include <sensor_msgs/PointCloud2.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseStamped.h"
#include "tf_conversions/tf_eigen.h"

char tstamp [80];	//Timestamp
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;
static const char WINDOW[] = "Image window";
ros::Time initial;
int numimage=0;
int lowercolor,uppercolor,hue1,hue2,lt1,lt2;

void callback(const ImageConstPtr& image1, const sensor_msgs::PointCloud2ConstPtr& input, const geometry_msgs::PoseWithCovarianceStampedConstPtr& data_in , ros::Publisher &pub , ros::Publisher &marker_pub , ros::Publisher &final_pose)

{

	pcl::PCLPointCloud2 pcl_pc;
	pcl_conversions::toPCL(*input,pcl_pc);
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromPCLPointCloud2(pcl_pc, cloud);


	cv_bridge::CvImagePtr cv_ptr;
	namedWindow(WINDOW);
	try
	{
		cv_ptr = cv_bridge::toCvCopy(image1, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}


	Mat HSV;
	Mat src_gray;
	cvtColor(cv_ptr->image,HSV,CV_BGR2HSV);
	inRange(HSV,Scalar(100,90,70),Scalar(145,255,255),src_gray);

	imshow(WINDOW,src_gray);
	int thresh = 180;
	int max_thresh = 255;
	RNG rng(12345);

	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	/// Detect edges using Threshold
	threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );
	/// Find contours
	findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	/// Find the rotated rectangles and ellipses for each contour
	vector<RotatedRect> minRect( contours.size() );
	vector<RotatedRect> minEllipse( contours.size() );

	for( int i = 0; i < contours.size(); i++ )
	{
		minRect[i] = minAreaRect( Mat(contours[i]) );
	}

	/// Draw contours + rotated rects 
	Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
	double largest_area=0;
	int largest_cont_ind=0;
	for( int i = 0; i< contours.size(); i++ )
	{
		double a=contourArea( contours[i],false);
		if(a>largest_area)
		{
			largest_area=a;
			largest_cont_ind=i;
		}
	}
	Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
	Point2f rect_points[4]; minRect[largest_cont_ind].points( rect_points );

	for( int j = 0; j < 4; j++ )
	{
		line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
	}

	//Following are the centre of largest rectangles found [center of t-shirt of Person Detected] in pixels of rgb image
	int centre_x,centre_y;
	centre_x=(rect_points[0].x+rect_points[2].x)/2;
	centre_y=(rect_points[0].y+rect_points[2].y)/2;
	Point pt;
	pt.x=centre_x;
	pt.y=centre_y;
	line( drawing, pt, pt, color, 1, 8 );
	imshow(WINDOW,src_gray);
	float xcor,ycor,zcor; //These are the x,y,z cordinates of that pixel in 3-D
	int ll;
	ofstream outfile;

	int pp = centre_x + ((centre_y)*(cloud.width)); 
	zcor = (float)cloud.points[pp].z;
	ycor = (float)cloud.points[pp].y;
	xcor = (float)cloud.points[pp].x;
	ros::Time begin = ros::Time::now();
	ros::Duration current = begin-initial;
	float timenow=current.toSec();
	cout<<xcor<<"\t"<<ycor<<"\t"<<zcor<<"\t"<<"\t"<<current<<endl;
	outfile.open(tstamp,std::ios_base::app);
	if( (pcl_isfinite(xcor)) || (pcl_isfinite(xcor)) )
		outfile<<xcor<<","<<zcor<<","<<current<<endl;


	//Pose msg publish// Publishing pose of person in Kinect's Refernece Frame under the topic person_pose
	float inx,iny,intheta;
	geometry_msgs::Pose2D msg;
	msg.x=xcor;
	msg.y=zcor;
	msg.theta=timenow;
	pub.publish(msg);

	//Final Pose
	//Get robot's pose data from amcl_pose 

//	geometry_msgs::Point robot_pose;
//	robot_pose.x = data_in->pose.pose.position.x;
//	geometry_msgs::Quaternion robot_orient;
//	robot_orient.x = data_in->pose.pose.orientation.x;
	
	//Perform Transformations : xcor,zcor are the person's cordinate w.r.t kinect in x-z plane. [Height of person is along y-axis and is not useful while finding the trajectory, x-z plane is the plane of ground in which person moves].
	tf::Transform robo_world;

	robo_world.setOrigin( tf::Vector3(data_in->pose.pose.position.x, data_in->pose.pose.position.y,data_in->pose.pose.position.z ));

	robo_world.setRotation( tf::Quaternion(data_in->pose.pose.orientation.x,data_in->pose.pose.orientation.y,data_in->pose.pose.orientation.z,data_in->pose.pose.orientation.w ));

	tf::Transform camera_robo;
	camera_robo.setOrigin( tf::Vector3(0.0,0.25,0.0) );
	camera_robo.setRotation( tf::Quaternion(0,0,0,1) );

	tf::Transform person_camera;
	person_camera.setOrigin( tf::Vector3(xcor,ycor,zcor) );
	person_camera.setRotation( tf::Quaternion(0,0,0,1) );

	tf::Transform final_transform;
	final_transform = robo_world * camera_robo;
	final_transform = final_transform * person_camera ;

	tf::Vector3 person_posev;
	person_posev = final_transform.getOrigin();

	geometry_msgs::Point person_pose;
	person_pose.x = person_posev[0];
	person_pose.y = person_posev[1];
	person_pose.z = person_posev[2];
	/////////////////////////
		
	geometry_msgs::Pose2D msg2;
	msg2.x=person_pose.x;	//Put the modified x-cordinate here
	msg2.y=person_pose.z;	//Put modifed z-cordinate here
	msg2.theta=timenow;	//Time
	final_pose.publish(msg2);



	//Visualization Marker// Marking the position in Kinect's Frame of reference on Rviz

	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp. 
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();

	marker.ns = "basic_shapes";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  
	marker.pose.position.x = person_pose.x;	//change to the modifed corinate
	marker.pose.position.y = person_pose.y; //need to be set as same y-axis of sick laser
	marker.pose.position.z = person_pose.z;  //change to the modified cordinate
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker. 1x1x1 here means 1m on a side
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;

	// Set the color 
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();

	// Publish the marker
	marker_pub.publish(marker);
	/////////////////////////////

}

int main(int argc, char** argv)
{
	time_t rawtime;
	struct tm * timeinfo;
	time (&rawtime);
	timeinfo = localtime (&rawtime);
	strftime (tstamp,80,"%c",timeinfo);

	if (argc==3){
		lowercolor=atoi(argv[1]);uppercolor=atoi(argv[2]);hue1=90;hue2=255;lt1=80;lt2=255;	
	}
	else if (argc==5){
		lowercolor=atoi(argv[1]);uppercolor=atoi(argv[2]);hue1=atoi(argv[3]);hue2=atoi(argv[4]);lt1=80;lt2=255;
	}
	else if (argc==7){
		lowercolor=atoi(argv[1]);uppercolor=atoi(argv[2]);hue1=atoi(argv[3]);hue2=atoi(argv[4]);lt1=atoi(argv[5]);lt2=atoi(argv[6]);
	}
	else {
		lowercolor=30;uppercolor=90;hue1=100;hue2=255;lt1=70;lt2=255; //Default Values for Green Color
	}

	puts (tstamp);
	sleep(3);

	ros::init(argc, argv, "vision_node");

	ros::NodeHandle nh;
	ros::Publisher pub; //For person pose relative to kinect
	ros::Publisher marker_pub; //For marker
	ros::Publisher final_pose; //for pose of person relative to world coordinate frame

	initial = ros::Time::now();

	//Subscribed Topics
	message_filters::Subscriber<Image> image1_sub(nh, "/camera/rgb/image_color", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh,"/camera/depth/points", 1);
	message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> pose_sub(nh,"amcl_pose",10);

	typedef sync_policies::ApproximateTime<Image,sensor_msgs::PointCloud2,geometry_msgs::PoseWithCovarianceStamped> MySyncPolicy;

	//Published Topics
	pub=nh.advertise<geometry_msgs::Pose2D>("person_pose", 1000);
	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	final_pose=nh.advertise<geometry_msgs::Pose2D>("final_pose",1000);

	Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), image1_sub,cloud_sub,pose_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3,pub,marker_pub,final_pose));

	ros::spin();
	return 0;
}
