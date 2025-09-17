// File: include/stabilizer.h

#pragma once

#include "config.h"
#include <deque>
#include <memory>

// Forward‐declare
namespace cv {
class Mat;
}

namespace tracking {

class Stabilizer {
public:
  explicit Stabilizer(std::shared_ptr<Config> cfg);
  cv::Mat stabilize(const cv::Mat &f, const cv::Mat &gray);
  void reset();

private:
  std::shared_ptr<Config> cfg_;
  cv::Mat prevGray_;
  std::deque<cv::Mat> ts_;
  cv::Mat sumH_;
  cv::Mat combineAverage();
};

} // namespace tracking
