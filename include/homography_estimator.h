// File: include/homography_estimator.h

#pragma once
#include <opencv2/core.hpp>

namespace tracking {

/// @brief Estimate a 3×3 homography between prev→curr frames.
class HomographyEstimator {
public:
  static cv::Mat estimate(const cv::Mat &prev, const cv::Mat &curr);
};

} // namespace tracking
