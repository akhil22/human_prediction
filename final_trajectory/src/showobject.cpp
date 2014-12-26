#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<iostream>
#include<fstream>
#include "std_msgs/String.h"
#include <sstream>

namespace enc = sensor_msgs::image_encodings;
int lowercolor,uppercolor,hue1,hue2,lt1,lt2;
static const char WINDOW[] = "Image window";
using namespace cv;
using namespace std;
class ImageConverter
{


	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	public:
	ImageConverter()
		: it_(nh_)
	{
		image_pub_ = it_.advertise("out", 1);
			image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
//			image_sub_ = it_.subscribe("/rgb/image_raw", 1, &ImageConverter::imageCb, this);
		namedWindow(WINDOW);
	}

	~ImageConverter()
	{
		destroyWindow(WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		ofstream outfile;
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		//	if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
		//		circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

		////////////////////////////////////////////////////////////////////////////////
		Mat HSV;
		Mat src_gray;
		cvtColor(cv_ptr->image,HSV,CV_BGR2HSV);
		inRange(HSV,Scalar(lowercolor,hue1,lt1),Scalar(uppercolor,hue2,lt2),src_gray);
		imshow(WINDOW,src_gray);  
int thresh = 180;
int max_thresh = 255;
RNG rng(12345);

	//cout<<argv[1]<<endl<<argv[2];

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
		//	if( contours[i].size() > 5 )
		//	{ 
		//		minEllipse[i] = fitEllipse( Mat(contours[i]) );
		//	}
		}

		/// Draw contours + rotated rects + ellipses
		Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
		double largest_area=0;
		int largest_cont_ind=0;
		for( int i = 0; i< contours.size(); i++ )
		{
		
			// contour
		//	drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
			// ellipse
//			ellipse( drawing, minEllipse[i], color, 2, 8 );
			// rotated rectangle
			 double a=contourArea( contours[i],false);
			if(a>largest_area)
			{
				largest_area=a;
				largest_cont_ind=i;
			}
		
//			cout<<endl;
		}
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			Point2f rect_points[4]; minRect[largest_cont_ind].points( rect_points );
	
			for( int j = 0; j < 4; j++ )
			{
				line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
			//	cout<<rect_points[j]<<"\t";
			}
			float centre_x,centre_y;
			centre_x=(rect_points[0].x+rect_points[2].x)/2;
			centre_y=(rect_points[0].y+rect_points[2].y)/2;
			Point pt;
			pt.x=centre_x;
			pt.y=centre_y;
			ros::Time begin = ros::Time::now();
		//	double secs =ros::Time::now().toSec();
			line( drawing, pt, pt, color, 1, 8 );
			cout<<pt<<"\t\t"<<begin<<endl;
		/// Show in a window
//		namedWindow( "Contours", CV_WINDOW_AUTOSIZE );

//		imshow( WINDOW, drawing );
imshow(WINDOW,src_gray);
		/////////////////////////////////////////////////////////////////////

		//		cv::imshow(WINDOW, cv_ptr->image);
		waitKey(3);
	
/////////////////////publisher////////////////////
/*		ros::NodeHandle n;
		ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    std_msgs::String msg;

    stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();
*///////////////////////////////////////////////////////////



		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char** argv)
{
	cout<<argv[1]<<endl<<argv[2];
	if (argc==1)
	{
	lowercolor=100;//blue
	uppercolor=145;
	hue1=90;
	hue2=255;
	lt1=80;
	lt2=255;
	}
	else if (argc==3)
	{
	lowercolor=atoi(argv[1]);
	uppercolor=atoi(argv[2]);
	hue1=90;
	hue2=255;
	lt1=80;
	lt2=255;
	}
	else if (argc==5)
	{
	lowercolor=atoi(argv[1]);
	uppercolor=atoi(argv[2]);
	hue1=atoi(argv[3]);
	hue2=atoi(argv[4]);
	lt1=80;
	lt2=255;
	}
	else if (argc==7)
	{
	lowercolor=atoi(argv[1]);
	uppercolor=atoi(argv[2]);
	hue1=atoi(argv[3]);
	hue2=atoi(argv[4]);
	lt1=atoi(argv[5]);
	lt2=atoi(argv[6]);
	}

	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}
