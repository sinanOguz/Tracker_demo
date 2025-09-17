#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include <opencv2/core/types.hpp> 
#include <opencv2/imgproc.hpp>    
#include <opencv2/tracking.hpp>   
#include <opencv2/videoio.hpp>    

#include "config.h"
#include "stabilizer.h"
#include "utils.h"
#include "visualizer.h"

namespace tracking {

/// Tracker state machine
enum class State { Idle, Tracking, Lost };

/// Initialization request for the CSRT thread
struct InitRequest {
  std::shared_ptr<cv::Mat> frame;
  cv::Rect bbox;
};

/// Simple threadsafe queue for InitRequest
template <typename T> class ThreadSafeQueue {
public:
  ThreadSafeQueue() = default;

  /// Push a new item; wake one waiting pop()
  void push(const T &item) {
    std::lock_guard<std::mutex> lk(mtx_);
    q_.push(item);
    cv_.notify_one();
  }

  /// Pop next item; blocks until an item is available or stop() is called.
  /// Returns false if queue is stopped and empty.
  bool pop(T &item) {
    std::unique_lock<std::mutex> lk(mtx_);
    while (!stopped_ && q_.empty())
      cv_.wait(lk);
    if (q_.empty() && stopped_)
      return false;
    item = std::move(q_.front());
    q_.pop();
    return true;
  }

  /// Stop the queue; unblocks all waiting pops.
  void stop() {
    std::lock_guard<std::mutex> lk(mtx_);
    stopped_ = true;
    cv_.notify_all();
  }

private:
  std::queue<T> q_;
  std::mutex mtx_;
  std::condition_variable cv_;
  bool stopped_{false};
};

/// Main tracking application
class TrackerApp {
public:
  TrackerApp(std::shared_ptr<Config> cfg, cv::VideoCapture cap);
  ~TrackerApp();
  void run();

private:
  // core stages
  bool grabFrameAndStabilize();
  void swapPendingTracker();
  void handlePausedMode();
  void handleCatchUpMode();
  void handleLiveMode();
  void process(const cv::Mat &frame);
  void updateTracker(const cv::Mat &frame);
  void handleTrackerKey(char k);


  // new helper: shared between paused/live for mouse‐click → CSRT init
  void selectAndInit();

  // quit flag (instead of direct exit())
  bool shouldQuit_{false};

  /*  add a member in TrackerApp (tracker.h) */
  cv::Rect lastGoodBox_;
  /* initialise inside swapPendingTracker()   */
  int framesSinceInit_{0};
  // configuration & capture
  std::shared_ptr<Config> cfg_;
  cv::VideoCapture cap_;
  Stabilizer stab_;
  State state_{State::Idle};

  // timing & buffering
  size_t maxBufferSize_{0};
  double interval_{0};

  // CSRT + init thread
  cv::Ptr<cv::TrackerCSRT> tracker_, newTracker_;
  std::mutex trackerMtx_;
  ThreadSafeQueue<InitRequest> initQueue_;
  std::thread initThread_;
  std::atomic<bool> reinitPending_{false};

  // ROI & bookkeeping
  cv::Rect bbox_;
  std::deque<std::shared_ptr<cv::Mat>> frameBuffer_;

  // UI state
  bool paused_{false};
  int mouseX_{0}, mouseY_{0};
  double fpsDisp_{0};

  // failure & re-detect
  int csrtFailCnt_{0}, lostCnt_{0};

  // particle filter for bbox‐center and CLAHE
  cv::Ptr<cv::CLAHE> clahe_;

  // frame buffers & images
  cv::Mat last_;     ///< most recent stabilized frame
  cv::Mat vis_;      ///< visualization copy
  cv::Mat pauseVis_; ///< snapshot when paused
  cv::Mat prevGray_; ///< for stabilization’s next iterate

};

} // namespace tracking
