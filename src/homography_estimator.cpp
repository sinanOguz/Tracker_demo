// File: src/homography_estimator.cpp

#include "homography_estimator.h"
#include <opencv2/calib3d.hpp>

using namespace tracking;

cv::Mat HomographyEstimator::estimate(const cv::Mat &prev,
                                      const cv::Mat &curr) {
  // 1) Detect FAST keypoints
  static cv::Ptr<cv::FastFeatureDetector> fast =
      cv::FastFeatureDetector::create(500, true);
  std::vector<cv::KeyPoint> k1, k2;
  fast->detect(prev, k1);
  fast->detect(curr, k2);
  if (k1.size() < 10 || k2.size() < 10)
    return {};

  // 2) Compute AKAZE descriptors
  static cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();
  cv::Mat d1, d2;
  akaze->compute(prev, k1, d1);
  akaze->compute(curr, k2, d2);
  if (d1.empty() || d2.empty())
    return {};

  // 3) Match with ratio test
  static cv::BFMatcher bf(cv::NORM_HAMMING);
  std::vector<std::vector<cv::DMatch>> matches;
  bf.knnMatch(d1, d2, matches, 2);
  std::vector<cv::DMatch> good;
  for (auto &m : matches) {
    if (m.size() == 2 && m[0].distance < 0.75f * m[1].distance)
      good.push_back(m[0]);
  }
  if (good.size() < 8)
    return {};

  // 4) Collect point correspondences
  std::vector<cv::Point2f> p1, p2;
  for (auto &m : good) {
    p1.push_back(k1[m.queryIdx].pt);
    p2.push_back(k2[m.trainIdx].pt);
  }

  return cv::findHomography(p1, p2, 3.0);
}
