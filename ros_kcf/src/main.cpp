#include <main.hpp>

namespace enc = sensor_msgs::image_encodings;

void Main::process()
{
    while (ros::ok())
    {
        switch (state)
        {
        case TRACKER_INIT:
            if(correctBB)
            {
                sendTrackedObject(target_bb.x,target_bb.y,target_bb.width,target_bb.height);

                ROS_INFO("Starting at %d %d %d %d\n", target_bb.x, target_bb.y, target_bb.width, target_bb.height);

                tracker.init(target_bb, target_image);
                state = TRACKING;
            }
            else
            {
                ros::Duration(1.0).sleep();
                ROS_INFO("Waiting for a BB");
            }
            break;
        case TRACKING:
            if(newImageReceived())
            {
              getLastImageFromBuffer();

              const cv::Rect &result = tracker.update(img);

              sendTrackedObject(result.x, result.y, result.width, result.height);
            }
            break;
        case STOPPED:
                ros::Duration(1.0).sleep();
                ROS_INFO("Tracking stopped");
            break;
        default:
            break;
        }
    }
    semaphore.unlock();
}

    void Main::imageReceivedCB(const sensor_msgs::ImageConstPtr &msg)
    {
      bool empty = false;
      mutex.lock();

      if(img_buffer_ptr.get() == 0)
      {
        empty = true;
      }

      try
      {
        if (enc::isColor(msg->encoding))
          img_buffer_ptr = cv_bridge::toCvCopy(msg, enc::RGB8);
        else
        {
          img_buffer_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);
          cv::cvtColor(img_buffer_ptr->image, img_buffer_ptr->image, CV_GRAY2BGR);
        }
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      if(empty)
      {
        semaphore.unlock();
      }
      mutex.unlock();
    }

    void Main::targetReceivedCB(const msgs::Target &msg)
    {
      ROS_ASSERT(msg.bb.x >= 0);
      ROS_ASSERT(msg.bb.y >= 0);
      ROS_ASSERT(msg.bb.width > 0);
      ROS_ASSERT(msg.bb.height > 0);
      ROS_INFO("Bounding Box received");

      target_bb.x = msg.bb.x;
      target_bb.y = msg.bb.y;
      target_bb.width = msg.bb.width;
      target_bb.height = msg.bb.height;

      try
      {
        target_image = cv_bridge::toCvCopy(msg.img, enc::BGR8)->image;
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
      correctBB = true;
    }

    bool Main::cmdReceivedCB(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
    {
      reset();
      return true;
    }

    void Main::sendTrackedObject(int x, int y, int width, int height)
    {
      msgs::BoundingBox msg;
      msg.header = img_header;
      msg.x = x;
      msg.y = y;
      msg.width = width;
      msg.height = height;          
      pub1.publish(msg);
    }

    bool Main::newImageReceived()
    {
      semaphore.lock();
      return true;
    }

    void Main::getLastImageFromBuffer()
    {
      mutex.lock();
      img_header = img_buffer_ptr->header;
      img = img_buffer_ptr->image;

      img_buffer_ptr.reset();
      mutex.unlock();
    }

    void Main::stopTracking()
    {
      if(state == STOPPED)
        state = TRACKER_INIT;
      else
        state = STOPPED;
      correctBB = false;
    }

    void Main::reset()
    {
        correctBB = false;
        state = TRACKER_INIT;
    }

