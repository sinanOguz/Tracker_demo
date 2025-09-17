// File: src/stabilizer.cpp

#include "stabilizer.h"
#include "homography_estimator.h"
#include <opencv2/imgproc.hpp>

using namespace tracking;

Stabilizer::Stabilizer(std::shared_ptr<Config> cfg)
    : cfg_(std::move(cfg)), sumH_(cv::Mat::zeros(3, 3, CV_32F)) {}

cv::Mat Stabilizer::stabilize(const cv::Mat &f, const cv::Mat &gray) {
  // 1) Estimate homography against previous frame
  if (!prevGray_.empty()) {
    cv::Mat H = HomographyEstimator::estimate(prevGray_, gray);
    if (!H.empty()) {
      cv::Mat H32;
      H.convertTo(H32, CV_32F);

      ts_.push_back(H32);
      sumH_ += H32;

      // drop oldest if over history length
      if (ts_.size() > cfg_->stabilizer.maxHistory) {
        sumH_ -= ts_.front();
        ts_.pop_front();
      }
    }
  }

  // save this gray for next call
  prevGray_ = gray;

  // 2) If no history, passthrough
  if (ts_.empty()) {
    return f;
  }

  // 3) Compute average and warp
  cv::Mat Havg = combineAverage();
  cv::Mat invH = Havg.inv();
  cv::Mat warped;
  cv::warpPerspective(f, warped, invH, f.size(), cv::INTER_LINEAR);
  return warped;
}

void Stabilizer::reset() {
  prevGray_.release();
  ts_.clear();
  sumH_ = cv::Mat::zeros(3, 3, CV_32F);
}

cv::Mat Stabilizer::combineAverage() {
  // unweighted mean of all Ts
  return sumH_ / static_cast<float>(ts_.size());
}
