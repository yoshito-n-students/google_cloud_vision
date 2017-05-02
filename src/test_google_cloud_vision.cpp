#include <fstream>
#include <iostream>

#include <google_cloud_vision/client.hpp>
#include <google_cloud_vision_msgs/encode.hpp>
#include <google_cloud_vision_srvs/Annotate.h>

namespace ba = boost::asio;
namespace bp = boost::property_tree;
namespace msgs = google_cloud_vision_msgs;
namespace srvs = google_cloud_vision_srvs;

int main(int argc, char *argv[]) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <image_file> <api_key>" << std::endl;
    return 1;
  }

  // open the given image file
  std::ifstream image_file(argv[1]);
  if (!image_file) {
    std::cerr << "Failed to open \"" << argv[1] << "\"" << std::endl;
    return 1;
  }

  // create the request
  srvs::Annotate::Request request;
  request.requests.resize(1);
  msgs::encode(image_file, request.requests[0].image);
  request.requests[0].features.resize(1);
  request.requests[0].features[0].type = msgs::Feature::TEXT_DETECTION;
  request.requests[0].features[0].has_max_results = true;
  request.requests[0].features[0].max_results = 10;

  // call google cloud vision api
  google_cloud_vision::Client client(argv[2]);
  srvs::Annotate::Response response;
  client.call(request, response);

  // show the response
  std::cout << response << std::endl;

  return 0;
}
