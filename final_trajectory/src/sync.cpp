#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sstream>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include<iostream>
#include<sstream>
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h> //I believe you were using pcl/ros/conversion.h
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <pcl/io/pcd_io.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <visualization_msgs/Marker.h>
char tstamp [80];

//ros::Publisher publ_;

	//ros::init(argc, argv, "vision_node");
//ros::NodeHandle nh;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;
 static const char WINDOW[] = "Image window";
ros::Time initial;
int numimage=0;
int lowercolor,uppercolor,hue1,hue2,lt1,lt2;

void callback(const ImageConstPtr& image1, const sensor_msgs::PointCloud2ConstPtr& input,ros::Publisher &pub,ros::Publisher &marker_pub)
{
//	cout<<"Test msg0"<<endl;
//	ROS_INFO("WRITTEN_DATA_FUNC!!");
//	pcl::PointCloud2<pcl::PointXYZ> cloud;
	pcl::PCLPointCloud2 pcl_pc;
	pcl_conversions::toPCL(*input,pcl_pc);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(pcl_pc, cloud);

//dep	pcl::fromROSMsg(*input, cloud);
	
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

//	inRange(HSV,Scalar(80,80,50),Scalar(140,255,255),src_gray);
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

	/// Draw contours + rotated rects + ellipses
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
	int centre_x,centre_y;
	centre_x=(rect_points[0].x+rect_points[2].x)/2;
	centre_y=(rect_points[0].y+rect_points[2].y)/2;
	Point pt;
	pt.x=centre_x;
	pt.y=centre_y;
	line( drawing, pt, pt, color, 1, 8 );
//	cout<<pt<<endl;
imshow(WINDOW,src_gray);
	float xcor,ycor,zcor;
//	cout<<cloud.width<<","<<cloud.height;
	int ll;
	ofstream outfile;

	int pp = centre_x + ((centre_y)*(cloud.width)); 
	zcor = (float)cloud.points[pp].z;
	ycor = (float)cloud.points[pp].y;
	xcor = (float)cloud.points[pp].x;
	ros::Time begin = ros::Time::now();
	ros::Duration current = begin-initial;
	float timenow=current.toSec();
//	float timenow=0;
	cout<<xcor<<"\t"<<ycor<<"\t"<<zcor<<"\t"<<"\t"<<current<<endl;
	outfile.open(tstamp,std::ios_base::app);
	if( (pcl_isfinite(xcor)) || (pcl_isfinite(xcor)) )
		outfile<<xcor<<","<<zcor<<","<<current<<endl;

//	publ_=nh.advertise<std_msgs::String>("chatter", 1000);

//Pose msg publish////////////////////////
	float inx,iny,intheta;
	geometry_msgs::Pose2D msg;
	int count=0;
	stringstream ss;
	ss << "hello world " << count;
//	msg.data = ss.str();	
	msg.x=xcor;
	msg.y=zcor;
	msg.theta=timenow;
	pub.publish(msg);
	count++;
///////////////////////////////////////	
//Visualization marker///////

//uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/camera_depth_optical_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = xcor;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = zcor;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    marker_pub.publish(marker);

    // Cycle between different shapes
    
  

/////////////////////////////

}

int main(int argc, char** argv)
{
 time_t rawtime;
 struct tm * timeinfo;
// char tstamp [80];
 time (&rawtime);
 timeinfo = localtime (&rawtime);
 strftime (tstamp,80,"%c",timeinfo);
	if (argc==1)
/*blue*/	{lowercolor=30;uppercolor=90;hue1=100;hue2=255;lt1=70;lt2=255;	}
	else if (argc==3)
		{lowercolor=atoi(argv[1]);uppercolor=atoi(argv[2]);hue1=90;hue2=255;lt1=80;lt2=255;	}
	else if (argc==5)
		{lowercolor=atoi(argv[1]);uppercolor=atoi(argv[2]);hue1=atoi(argv[3]);hue2=atoi(argv[4]);lt1=80;lt2=255;	}
	else if (argc==7)
		{lowercolor=atoi(argv[1]);uppercolor=atoi(argv[2]);hue1=atoi(argv[3]);hue2=atoi(argv[4]);lt1=atoi(argv[5]);lt2=atoi(argv[6]);}
	
 puts (tstamp);
	sleep(3);
	ros::init(argc, argv, "vision_node");

	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Publisher marker_pub;

	initial = ros::Time::now();

	message_filters::Subscriber<Image> image1_sub(nh, "/camera/rgb/image_color", 1);
	//	message_filters::Subscriber<Image> image2_sub(nh, "/camera/depth_registered/image", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh,"/camera/depth/points", 1);
	ROS_INFO("WRITTEN_DATAAAAA1");
	typedef sync_policies::ApproximateTime<Image,sensor_msgs::PointCloud2> MySyncPolicy;
	ROS_INFO("WRITTEN_DATA!!!2");
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	//	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub,cloud_sub);

//	pub=nh.advertise<std_msgs::String>("chatter", 1000);
	pub=nh.advertise<geometry_msgs::Pose2D>("person_pose", 1000);
	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), image1_sub,cloud_sub);
	ROS_INFO("WRITTEN_DATA###3");
	sync.registerCallback(boost::bind(&callback, _1, _2,pub,marker_pub));
	ROS_INFO("WRITTEN_DATA$$$4");

	ros::spin();
	ROS_INFO("WRITTEN_DATA5");

	return 0;
}
