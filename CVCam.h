#pragma once

#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

#include <libcamera/libcamera.h>
#include <opencv2/opencv.hpp>

#include "mapped_framebuffer.h"

#include "event_loop.h"

#define TIMEOUT_SEC 3

using namespace libcamera;

class CVCam : public cv::VideoCapture {
public:
  static void processRequest(Request* request);
  static void requestComplete(Request* request);
  std::string cameraName(Camera* camera);
  cv::String
  getBackendName() const {
    return "libcamera";
  }

  int start();
  int go();
  int finish();

  std::shared_ptr<Camera> camera;
  EventLoop loop;
  std::unique_ptr<std::thread> aThread;
  Stream* stream;
  FrameBufferAllocator* allocator;
  std::unique_ptr<CameraManager> cm;
  std::vector<std::unique_ptr<Request>> requests;
};
