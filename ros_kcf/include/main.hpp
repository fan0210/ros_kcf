#ifndef MAIN_HPP_
#define MAIN_HPP_

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <msgs/Target.h>
#include <msgs/BoundingBox.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <algorithm>
#include <kcftracker.hpp>

class Main
{
	public:
                Main():state(TRACKER_INIT),tracker(KCFTracker(HOG, FIXEDWINDOW, MULTISCALE, LAB))
		{
                    pub1 = n.advertise<msgs::BoundingBox>("kcf_track_object", 1, true);
                    sub1 = n.subscribe("image", 1, &Main::imageReceivedCB, this);
                    sub2 = n.subscribe("target", 1, &Main::targetReceivedCB, this);
                    srv = n.advertiseService("cmd", &Main::cmdReceivedCB, this);

                    semaphore.lock();
		}

		void process();

	private:
                enum
                {
                        TRACKER_INIT,
                        TRACKING,
                        STOPPED
                } state;

                bool HOG = true;
                bool FIXEDWINDOW = false;
                bool MULTISCALE = true;
                bool SILENT = false;
                bool LAB = false;

                KCFTracker tracker;

                cv::Rect target_bb;
                cv::Mat target_image;

                bool correctBB = false;

		cv::Mat img;
		cv_bridge::CvImagePtr img_buffer_ptr;
                std_msgs::Header img_header;
		boost::interprocess::interprocess_mutex mutex;
		boost::interprocess::interprocess_mutex semaphore;

		ros::NodeHandle n;
		ros::Publisher pub1;
		ros::Subscriber sub1;
		ros::Subscriber sub2;
                ros::ServiceServer srv;

		bool newImageReceived();
		void getLastImageFromBuffer();
                void imageReceivedCB(const sensor_msgs::ImageConstPtr &);
                void targetReceivedCB(const msgs::Target &);
                bool cmdReceivedCB(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
                void sendTrackedObject(int, int, int, int);

		void stopTracking();
		void reset();
};

#endif
