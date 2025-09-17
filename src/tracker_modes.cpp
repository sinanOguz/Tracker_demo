// File: src/tracker.cpp

#include "tracker.h"
#include "utils.h"
#include "visualizer.h"
#include <opencv2/core/version.hpp> 

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>

using namespace tracking;
using namespace utils;
using Clock = std::chrono::steady_clock;

//=============================================================================
// Constructor / destructor
//=============================================================================
TrackerApp::TrackerApp(std::shared_ptr<Config> cfg, cv::VideoCapture cap)
    : cfg_(std::move(cfg)), cap_(std::move(cap)), stab_(cfg_), clahe_(cv::createCLAHE(cfg_->claheClipLimit, cfg_->claheTileGrid)) {

  lastGoodBox_ = {};
  if (!cap_.isOpened())
    throw std::runtime_error("Cannot open input source.");

  maxBufferSize_ =
      int(cfg_->bufferSeconds * std::max(30.0, cap_.get(cv::CAP_PROP_FPS)));

  double camFPS = cap_.get(cv::CAP_PROP_FPS);
  interval_ =
      (cfg_->videoPath.empty() || camFPS <= 1.0) ? 0.0 : 1000.0 / camFPS;

  cv::namedWindow(cfg_->windowName, cv::WINDOW_NORMAL);
  cv::resizeWindow(cfg_->windowName, int(cap_.get(cv::CAP_PROP_FRAME_WIDTH)),
                   int(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)));

  initInput(cfg_->windowName, [&](int x, int y) {
    mouseX_ = x;
    mouseY_ = y;
  });

  initThread_ = std::thread([this] {
    InitRequest req;
    while (initQueue_.pop(req)) {
      try {
        auto t = cv::TrackerCSRT::create();
        t->init(*req.frame, req.bbox);
        // quick probe
        cv::Rect probe = req.bbox;
        if (!t->update(*req.frame, probe)) {
          std::cerr << "CSRT init patch not trackable\n";
          continue;
        }
        std::lock_guard<std::mutex> lk(trackerMtx_);
        newTracker_ = t;
        reinitPending_ = true;
      } catch (...) {
      }
    }
  });
}

TrackerApp::~TrackerApp() {
  initQueue_.stop();
  if (initThread_.joinable())
    initThread_.join();
}

//=============================================================================
// grabFrameAndStabilize
//=============================================================================
bool TrackerApp::grabFrameAndStabilize() {
  // 1) Grab raw frame
  cv::Mat raw;
  cap_ >> raw;
  if (raw.empty())
    return false;

  // 2) Convert to full-res gray for feature tracking
  cv::Mat gray_full;
  cv::cvtColor(raw, gray_full, cv::COLOR_BGR2GRAY);

  // 3) Down-sample for CLAHE + blur (stabilization)
  static const double STAB_SCALE = 0.5;
  cv::Mat small_gray;
  cv::resize(gray_full, small_gray, cv::Size(), STAB_SCALE, STAB_SCALE,
             cv::INTER_LINEAR);

  // 4) Apply CLAHE + median blur on the small image
  cv::Mat small_eq;
  clahe_->apply(small_gray, small_eq);
  cv::medianBlur(small_eq, small_eq, 3);

  // 5) Upsample back to full resolution
  cv::Mat proc_gray;
  cv::resize(small_eq, proc_gray, gray_full.size(), 0, 0, cv::INTER_LINEAR);

  // 6) Stabilize using the processed gray
  last_ = stab_.stabilize(raw, proc_gray);

  return true;
}

//=============================================================================
// swapPendingTracker
//=============================================================================
void TrackerApp::swapPendingTracker() {
  if (!reinitPending_)
    return;
  std::lock_guard<std::mutex> lk(trackerMtx_);
  if (newTracker_) {
    tracker_ = newTracker_;
    newTracker_.release();
    state_ = State::Tracking;
    stab_.reset();
    framesSinceInit_ = 0;
    lastGoodBox_ = bbox_; // ← correct place
  }

  reinitPending_ = false;
}

//=============================================================================
// updateTracker
//=============================================================================

void TrackerApp::updateTracker(const cv::Mat &frame) {
    // Advance frame-since-init counter
    ++framesSinceInit_;

    // 1) CSRT tracker update
    bool ok = tracker_->update(frame, bbox_);

    // 2) Drift veto (only after warm-up frames)
    if (ok && framesSinceInit_ > cfg_->drift.warmupFrames) {
        // a) Compute IoU with last good box
        cv::Rect inter = lastGoodBox_ & bbox_;
        double iou = double(inter.area()) /
                     double(lastGoodBox_.area() + bbox_.area() - inter.area());

        // b) Optionally get CSRT confidence (available in OpenCV ≥4.7)
        double conf = 1.0;
#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
        conf = tracker_.dynamicCast<cv::TrackerCSRT>()->getTrackingScore();
#endif

        // Veto if both IoU and confidence are too low
        if (iou < 0.10 && conf < cfg_->drift.csrtConfThresh)
            ok = false;
    }

    // 3) Failure handling
    if (!ok) {
        if (++csrtFailCnt_ >= cfg_->confirmLost) {
            state_ = State::Lost;
            if (++lostCnt_ >= cfg_->maxLost) {
                state_ = State::Idle;
                tracker_.release();
            }
        }
        return;  // Skip the “success” path
    }

    // 4) Success path: reset failure counter and record this as the last good box
    csrtFailCnt_  = 0;
    lastGoodBox_ = bbox_;
}


//=============================================================================
// process
//=============================================================================
void TrackerApp::process(const cv::Mat &frame) {
  // 1) Base frame & helper box
  last_ = frame;
  vis_ = frame.clone();
  Visualizer::drawDashed(vis_, mouseX_, mouseY_, cfg_->selWidth,
                         cfg_->selHeight, cfg_->selColor, cfg_->selThick);

  // 2) Initialization banner or tracker update
  if (reinitPending_) {
    cv::putText(vis_, "INITIALIZING TRACKER...", {10, 90},
                cv::FONT_HERSHEY_PLAIN, 2, {0, 255, 255}, 2);
  } else if (state_ == State::Tracking) {
    updateTracker(frame);
  }

  // 3) Draw tracking overlays
  Visualizer::renderTracking(vis_, state_, bbox_, *cfg_, last_);
  Visualizer::drawLegend(vis_);
}

//=============================================================================
// run – main loop
//=============================================================================
void TrackerApp::run() {
  while (!shouldQuit_) {
    swapPendingTracker();
    if (!grabFrameAndStabilize())
      break;
    if (paused_)
      handlePausedMode();
    else if (!frameBuffer_.empty())
      handleCatchUpMode();
    else
      handleLiveMode();
  }
}
