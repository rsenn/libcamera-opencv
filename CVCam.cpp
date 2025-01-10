

#include "CVCam.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>

#include <libcamera/libcamera.h>
#include <opencv2/opencv.hpp>

#include "mapped_framebuffer.h"

#include "event_loop.h"

#define TIMEOUT_SEC 3

using namespace libcamera;

void
CVCam::requestComplete(Request* request) {

  std::cout << "Completed " << (void*)request << std::endl;

  const Request::BufferMap& buffers = request->buffers();

  for(auto bufferPair : buffers) {
    const Stream* stream = bufferPair.first;
    FrameBuffer* buffer = bufferPair.second;
    const FrameMetadata& metadata = buffer->metadata();

    std::cout << " seq: " << std::setw(6) << std::setfill('0') << metadata.sequence << " bytesused: ";

    unsigned int nplane = 0;
    for(const FrameMetadata::Plane& plane : metadata.planes()) {
      std::cout << plane.bytesused;
      if(++nplane < metadata.planes().size())
        std::cout << "/";
    }

    auto cfg = stream->configuration();
    unsigned int width = cfg.size.width;
    unsigned int height = cfg.size.height;
    unsigned int stride = cfg.stride;

    std::cout << " size " << width << "x" << height << " stride " << stride << " format " << cfg.pixelFormat.toString() << " sec "
              << (double)clock() / CLOCKS_PER_SEC << std::endl;

    libcamera::MappedFrameBuffer mappedBuffer(buffer, MappedFrameBuffer::MapFlag::Read);
    const std::vector<libcamera::Span<uint8_t>> mem = mappedBuffer.planes();
    cv::Mat image(height, width, CV_8UC1, (uint8_t*)(mem[0].data()));

    cv::imwrite("images/img" + std::to_string((double)clock() / CLOCKS_PER_SEC) + ".png", image);
  }

  request->reuse(Request::ReuseBuffers);
}

std::string
CVCam::cameraName(Camera* camera) {
  const ControlList& props = camera->properties();
  std::string name;

  switch(props.get(properties::Location).value()) {
    case properties::CameraLocationFront: name = "Internal front camera"; break;
    case properties::CameraLocationBack: name = "Internal back camera"; break;
    case properties::CameraLocationExternal:
      name = "External camera";

      if(props.contains(properties::Model.id()))
        name += " '" + props.get(properties::Model).value() + "'";
      break;
  }

  name += " (" + camera->id() + ")";

  return name;
}
void
h(Request* r) {
  std::cout << "C\n";
};

int
CVCam::start() {

  cm = std::make_unique<CameraManager>();
  cm->start();

  for(auto const& camera : cm->cameras()) { std::cout << " - " << cameraName(camera.get()) << std::endl; }

  if(cm->cameras().empty()) {
    std::cout << "No cameras were identified on the system." << std::endl;
    cm->stop();
    return EXIT_FAILURE;
  }

  std::string cameraId = cm->cameras()[0]->id();
  camera = cm->get(cameraId);
  camera->acquire();

  std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration({StreamRole::Viewfinder});

  StreamConfiguration& streamConfig = config->at(0);
  std::cout << "Default viewfinder configuration is: " << streamConfig.toString() << std::endl;

  streamConfig.size.width = 2592;
  streamConfig.size.height = 1944;

  int retconfig = camera->configure(config.get());
  if(retconfig) {
    std::cout << "CONFIGURATION FAILED!" << std::endl;
    return EXIT_FAILURE;
  }

  config->validate();
  std::cout << "Validated viewfinder configuration is: " << streamConfig.toString() << std::endl;

  camera->configure(config.get());

  auto cp = camera->properties();
  auto cc = camera->controls();
  std::cout << "controls:\n";
  for(auto& c : cc) { std::cout << c.first->name() << ": " << c.second.toString() << " = " << c.second.def().toString() << std::endl; }
  std::cout << "properies:\n";
  for(auto& c : cp) { std::cout << c.first << ": " << c.second.toString() << std::endl; }

  allocator = new FrameBufferAllocator(camera);

  for(StreamConfiguration& cfg : *config) {
    int ret = allocator->allocate(cfg.stream());
    if(ret < 0) {
      std::cerr << "Can't allocate buffers" << std::endl;
      return EXIT_FAILURE;
    }

    size_t allocated = allocator->buffers(cfg.stream()).size();
    std::cout << "Allocated " << allocated << " buffers for stream" << std::endl;
  }

  stream = streamConfig.stream();
  const std::vector<std::unique_ptr<FrameBuffer>>& buffers = allocator->buffers(stream);
  for(unsigned int i = 0; i < buffers.size(); ++i) {
    std::unique_ptr<Request> request = camera->createRequest();
    if(!request) {
      std::cerr << "Can't create request" << std::endl;
      return EXIT_FAILURE;
    }

    const std::unique_ptr<FrameBuffer>& buffer = buffers[i];
    for(auto& plane : buffer->planes()) { std::cout << "buffer " << i << " length " << plane.length << " at " << plane.offset << std::endl; }

    int ret = request->addBuffer(stream, buffer.get());
    if(ret < 0) {
      std::cerr << "Can't set buffer for request" << std::endl;
      return EXIT_FAILURE;
    }

    request->controls().set(controls::AnalogueGain, 100000);
    request->controls().set(controls::ExposureTime, 100000);
    request->controls().set(controls::ExposureValue, 100000);

    requests.push_back(std::move(request));
  }

  camera->requestCompleted.connect(requestComplete);

  camera->start();
  aThread = std::make_unique<std::thread>([&]() { loop.exec(); });
  return EXIT_SUCCESS;
}

int
CVCam::go() {

  for(std::unique_ptr<Request>& request : requests) {
    std::cout << "Queued " << (void*)request.get() << std::endl;
    camera->queueRequest(request.get());
  }

  return EXIT_SUCCESS;
}

int
CVCam::finish() {
  aThread->join();

  camera->stop();
  allocator->free(stream);
  delete allocator;
  camera->release();
  camera.reset();
  cm->stop();

  return EXIT_SUCCESS;
}
