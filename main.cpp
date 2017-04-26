#include "opencv2/features2d/features2d.hpp"
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <pattern_follower/includes.h>
#include <queue>
#include <thread>
#include <time.h>

class Application {
public:
  Application(const cv::FileStorage &fs)
      : camera(new Camera(fs["Camera"])),
        rc(new RobotControl(fs["RobotControl"],
                            (int)fs["Application"]["simulation"])),
        t1(&Application::cam_proc, this), t2(&Application::rc_proc, this) {}

  ~Application() {
    t1.join();
    t2.join();
  }

private:
  std::mutex g_i_mutex;
  double dist{80.0}, angle{0.0};
  bool suceed{false};
  std::unique_ptr<Camera> camera;
  std::unique_ptr<RobotControl> rc;
  std::thread t1;
  std::thread t2;

  void cam_proc() {
    double d{0.0}, a{0.0};
    bool s{false};
    while (true) {
      s = camera->proceed(a, d);

      g_i_mutex.lock();
      dist = d;
      angle = a;
      suceed = s;
      g_i_mutex.unlock();

      if (waitKey(50) >= 0)
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  void rc_proc() {
    double d{0.0}, a{0.0};
    bool s{false};
    while (true) {
      g_i_mutex.lock();
      d = dist;
      a = angle;
      s = suceed;
      g_i_mutex.unlock();
      rc->calculateRobotSpeeds(std::vector<cv::Point2d>(), cv::Point2d(d, a),
                               s);

      if (waitKey(50) >= 0)
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
};
int main(int argc, char **argv) {

  std::string path = "/home/michail/pattern_follower/config.yaml";
  if (argc > 1)
    path = argv[1];
  FileStorage fs(path, FileStorage::READ);
  Mat grayImage, binaryImage, frame;

  /*Camera c1(fs["Camera"]);
  RobotControl rc(fs["RobotControl"], (int)fs["Application"]["simulation"]);*/
  Application app(fs);
  double dist{80.0}, angle{0.0};
  /*while (true) {
    bool suceed = c1.proceed(angle, dist);
    rc.calculateRobotSpeeds(std::vector<cv::Point2d>(),
                            cv::Point2d(dist, angle), suceed);
    if (waitKey(50) >= 0)
      break;
  }*/
  /*Mat cameraMatrix;
  Mat distCoeffs;
  std::vector<int> labels;
  std::vector<Point> locations, approx;
  std::vector<std::vector<Point>> contours;
  std::vector<Vec4i> hierarchy;
  std::vector<Point2f> vertices;

  CCameraCalibration calibrator;

  calibrator.readFromFile(camera_file);

  cameraMatrix = calibrator.cameraMatrix;

  distCoeffs = calibrator.distortionCoefficients;

  double fovx = 2 * atan((FRAME_SIZE / 2 * cameraMatrix.at<double>(0)));

  double angle = 0.0;
  double dist = 80;*/

  /*while (true) {
    if (camera.proceed(angle, dist)) {
    } else {
      dist = kf.getX();
      angle = kf.getY();
    }

    if (!simulation) {
      rc.calculateRobotSpeeds(std::vector<cv::Point2d>(),
                              cv::Point2d(dist, angle));
    }
    if (waitKey(50) >= 0)
      break;
  }*/
}
