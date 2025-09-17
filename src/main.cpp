// File: src/main.cpp

#include "config.h"
#include "tinyfiledialogs.h"
#include "tracker.h"
#include "utils.h"
#include <cstdlib>
#include <iostream>
#include <memory>

int main() {
  const char *patterns[] = {"*.mp4", "*.avi", "*.mov"};

  // Determine CLI vs GUI dialogs
  bool useGui = true;
#if defined(__linux__)
  if (std::getenv("DISPLAY") == nullptr) {
    useGui = false;
  }
#endif

  int useCam = 0;
  if (useGui) {
    useCam =
        tinyfd_messageBox("Input Source", "Use camera instead of video file?",
                          "yesno", "question", 0);
  } else {
    std::cout << "Use camera instead of video file? [y/N]: ";
    char c;
    std::cin >> c;
    useCam = (c == 'y' || c == 'Y') ? 1 : 0;
  }

  cv::VideoCapture cap;
  auto cfg = std::make_shared<tracking::Config>();

  if (useCam == 1) {
    // camera mode: leave cfg->videoPath empty
    int camIdx = utils::selectCamera(10);
    if (camIdx < 0) {
      if (useGui)
        tinyfd_messageBox("Error", "No camera selected!", "ok", "error", 0);
      else
        std::cerr << "No camera selected! Exiting.\n";
      return EXIT_FAILURE;
    }
    cap.open(camIdx);
  } else {
    // file mode: record path in cfg so TrackerApp will throttle below
    std::string initialDir = utils::getParentOfExecutable();
    const char *path = tinyfd_openFileDialog(
        "Open Video File", initialDir.c_str(), 3, patterns, nullptr, 0);
    if (!path) {
      std::cerr << "No file selected. Exiting.\n";
      return EXIT_FAILURE;
    }
    cfg->videoPath = path;
    cap.open(path);
  }

  if (!cap.isOpened()) {
    if (useGui)
      tinyfd_messageBox("Error", "Cannot open input source.", "ok", "error", 0);
    else
      std::cerr << "Cannot open input source. Exiting.\n";
    return EXIT_FAILURE;
  }

  try {
    tracking::TrackerApp app(cfg, std::move(cap));
    app.run();
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << "\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
