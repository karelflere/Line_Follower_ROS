#include <stdint.h>
#include <list>
#include <vector>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/CompressedImage.h"
#include "rosgraph_msgs/Log.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/RegionOfInterest.h"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/version.hpp"
#include <ros/transport_hints.h>
#include <geometry_msgs/Twist.h>
#if CV_MAJOR_VERSION == 2
// do opencv 2 code
#elif CV_MAJOR_VERSION == 3
// do opencv 3 code
#endif*/


ros::Publisher chatter_pub;
int closest;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
 	//ROS_INFO("I heard: [%s]", msg->format.c_str());

 	try
 	{
		cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat
		float image_scale = 0.5 ;
		cv::namedWindow("image", cv::WINDOW_NORMAL);
		cv::resizeWindow("image", image.cols*image_scale, image.rows*image_scale);
		cv::namedWindow("roi_camera");
		cv::namedWindow("roi_detect");

		//cv::resizeWindow("image", 600,600);
		cv::Mat roi = image(cv::Rect(4*image.cols/5, 10, image.cols/12, image.rows-20));
		cv::rotate(roi, roi, cv::ROTATE_90_CLOCKWISE);

		
		cv::rectangle //draw ROI on window
		(
    				image,
    				cv::Point(round((4*image.cols/5)), round(10)),
    				cv::Point(round((4*image.cols/5 + image.cols/12)), round(image.rows-10)),
    				cv::Scalar(0, 0, 255),10,8,0
		);
		cv::Mat mono,dst, blur, thresh, erode, erodeImg, dilate, dilateImg, notused, canny_output;
		cv::cvtColor(roi, mono, cv::COLOR_BGR2GRAY);
		cv::GaussianBlur(mono, blur, cv::Size(9, 9), 2, 2);
		cv::threshold(blur, thresh, 0, 255, cv::THRESH_BINARY_INV|cv::THRESH_OTSU);
		cv::erode(thresh, erodeImg, erode);
		cv::dilate(erodeImg, dilateImg, dilate);
		
		cv::copyMakeBorder( dilateImg, dst, 1, 1, 1, 1, cv::BORDER_ISOLATED, cv::Scalar(0, 0, 0) );

		std::vector<std::vector<cv::Point> > contours;

		cv::Canny(dst, canny_output, 50, 150, 3 );
		cv::findContours(canny_output, contours, notused, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
		cv::Mat contourImage = cv::Mat::zeros( canny_output.size(), CV_8UC3 );

		int contours_size = contours.size();

		for (size_t idx = 0; idx < contours_size; idx++) 
		{
		cv::drawContours(contourImage, contours, idx, cv::Scalar(255, 255, 255));
		}

		std::vector<cv::Moments> mu(contours_size);
		for( int i = 0; i < contours_size; i++ )
 		{ 
			mu[i] = moments( contours[i], false ); 
		}

		std::vector<cv::Point2f> mc(contours_size);
		for( int i = 0; i < contours_size; i++ )
		{
			if (mu[i].m00 != 0)
			{
    				mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
			}
			else
			{
				mc[i] = cv::Point2f( mu[i].m10 , mu[i].m01 );
			}
		}


		float totalX=0.0, totalY=0.0;    
		for(int i=0; i<contours_size; i++) 
		{
    			totalX+=mc[i].x;
    			totalY+=mc[i].y;
		}

		cv::Point2f massCenter(totalX/contours_size, totalY/contours_size); // condition: size != 0

		//Print info
		ROS_INFO("amount of columns: [%i]", image.cols); 
		ROS_INFO("amount of rows: [%i]", image.rows);	
		ROS_INFO("amount of objects: [%i]", contours_size);
		ROS_INFO("center x: [%f]", round(massCenter.x));


		closest = 530;

		for(int i=0; i<contours_size; i++) 
		{
			int normalized = 1060;
			int newValue  =abs(round(mc[i].x-530));
			if(newValue<normalized)
			{
		   		normalized = newValue;
		   		closest = mc[i].x;
			}

			//ROS_INFO("center x of object %i: [%f]", i, round(mc[i].x));
		}


		//ROS_INFO("center x of object %f: [%f]", round(massCenter.y));	

		cv::imshow("image", image); //show full image
		cv::imshow("roi_camera", roi); //show roi
		cv::imshow("roi_detect", canny_output); //show detected lines
		//cv::imshow("roi_detect", contourImage); //show detected lines
		cv::waitKey(1);
\
		float contour_area[contours_size] = {0} ;
		for( int i = 0; i < contours_size; i++ )
		{
			contour_area[i] = cv::contourArea(contours[i]) ;
			ROS_INFO("contourarea %i: [%f]", i, contour_area[i]);
			if (contour_area[i]>55000)
			{
				closest=530;
			}	
				
		}

  		if (ros::ok())
  		{
	
			geometry_msgs::Twist msg;

			if(abs(closest-530)<30)
			{
	    			msg.linear.x = 0.5 ;
	    			msg.angular.z = 0;
			}
			else if(closest<530)
			{
	    			msg.linear.x = 0 ;
	    			msg.angular.z = 0.5;
			}
			else
			{
	    			msg.linear.x = 0 ;
	    			msg.angular.z = -0.5;
			}
			chatter_pub.publish(msg);
		}

    	}

  	catch (cv_bridge::Exception& e)
  	{
    		ROS_ERROR("CVbridge Exception!");
	}

}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "camera_listener");
  //ros::init(argc, argv, "camera_talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
	
   ros::NodeHandle n;

  /**   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

 chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ros::Subscriber sub = n.subscribe("/camera/image/compressed", 1000, chatterCallback, ros::TransportHints().tcpNoDelay());


 

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */


  ros::spin();

  cv::destroyWindow("view");

  return 0;
}
