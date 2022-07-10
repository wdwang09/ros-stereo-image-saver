// STD
#include <iostream>
#include <thread>

// ROS
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// OpenCV
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

// boost
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

class StereoImageSaver {
 public:
  StereoImageSaver(const std::string& directory, const std::string& left_topic, const std::string& right_topic) {
    if (!fs::is_directory(directory)) {
      ROS_FATAL("\"%s\" isn't a directory!", directory.c_str());
      std::exit(1);
    }
    directory_ = fs::absolute(directory);
    fs::path f1, f2;
    do {
      filename_id_++;
      f1 = directory_ / ("l" + std::to_string(filename_id_) + ".png");
      f2 = directory_ / ("r" + std::to_string(filename_id_) + ".png");
    } while (fs::exists(f1) || fs::exists(f2));
    ROS_INFO("Images will be saved in \"%s\" and \"%s\".", f1.c_str(), f2.c_str());

    left_topic_ = left_topic;
    right_topic_ = right_topic;
    left_sub_ = std::make_shared<decltype(left_sub_)::element_type>(n_, left_topic_, 1);
    right_sub_ = std::make_shared<decltype(right_sub_)::element_type>(n_, right_topic_, 1);
    left_right_sync_ = std::make_shared<decltype(left_right_sync_)::element_type>(*left_sub_, *right_sub_, 8);
    left_right_sync_->registerCallback(
        boost::bind(&StereoImageSaver::stereoCallback, this, _1, _2));  // NOLINT(modernize-avoid-bind)
    save_images_thread_ = std::thread([this] { saveImage(); });
  }

  ~StereoImageSaver() {
    is_thread_end_ = true;
    save_images_thread_.join();
  }

 private:
  void stereoCallback(const sensor_msgs::ImageConstPtr& ros_img1, const sensor_msgs::ImageConstPtr& ros_img2) {
    {
      cv_bridge::CvImageConstPtr img1_ptr, img2_ptr;
      try {
        img1_ptr = cv_bridge::toCvShare(ros_img1);
        img2_ptr = cv_bridge::toCvShare(ros_img2);
      } catch (cv_bridge::Exception& e) {
        ROS_FATAL("cv_bridge exception: %s", e.what());
        return;
      }
      std::unique_lock<std::mutex> lck(mtx_);
      img1_ = img1_ptr->image;
      img2_ = img2_ptr->image;
    }
    cv::Mat two_images;
    std::unique_lock<std::mutex> lck(mtx_);
    cv::hconcat(img1_, img2_, two_images);
    lck.unlock();
    cv::resize(two_images, two_images, cv::Size(two_images.cols / 3, two_images.rows / 3));
    cv::imshow("Two Images", two_images);
    cv::waitKey(1000 / 30);
  }

  void saveImage() {
    ROS_INFO("Press Ctrl-C and Enter to exit the program.");
    fs::path f1, f2;
    std::string buf;
    while (true) {
      std::getline(std::cin, buf);
      if (is_thread_end_) return;
      f1 = directory_ / ("l" + std::to_string(filename_id_) + ".png");
      f2 = directory_ / ("r" + std::to_string(filename_id_) + ".png");
      std::unique_lock<std::mutex> lck(mtx_);
      if (img1_.empty() || img2_.empty()) continue;
      cv::imwrite(f1.c_str(), img1_);
      cv::imwrite(f2.c_str(), img2_);
      lck.unlock();
      ROS_INFO("Saved! ID is %d.", filename_id_);
      filename_id_++;
    }
  }

 private:
  fs::path directory_;
  unsigned filename_id_ = 0;
  std::string left_topic_, right_topic_;
  ros::NodeHandle n_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> left_sub_, right_sub_;
  std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>> left_right_sync_;
  cv::Mat img1_, img2_;

  std::thread save_images_thread_;
  std::mutex mtx_;
  bool is_thread_end_ = false;
};

int main(int argc, char** argv) {
  std::cout << std::boolalpha;
  ros::init(argc, argv, "stereo_image_saver", ros::InitOption::AnonymousName);

  if (argc < 4) {
    ROS_FATAL(
        "Usage: rosrun stereo_image_saver stereo_image_saver_node ~/Documents /t265/fisheye1/image_raw "
        "/t265/fisheye2/image_raw");
    return 1;
  }

  StereoImageSaver m(argv[1], argv[2], argv[3]);

  ros::spin();
  return 0;
}
