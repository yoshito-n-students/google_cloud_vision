#ifndef GOOGLE_CLOUD_VISION_CLIENT
#define GOOGLE_CLOUD_VISION_CLIENT

#include <sstream>
#include <string>

#include <boost/network/include/http/client.hpp>

#include <google_cloud_vision/json_parser.hpp>
#include <google_cloud_vision/namespace.hpp>
#include <google_cloud_vision_srvs/Annotate.h>

namespace google_cloud_vision {

class Client {
public:
  Client(const std::string &key)
      : http_destination_("https://vision.googleapis.com/v1/images:annotate?key=" + key) {}

  virtual ~Client() {}

  void call(srvs::Annotate &ros_service) { call(ros_service.request, ros_service.response); }

  void call(const srvs::Annotate::Request &ros_request, srvs::Annotate::Response &ros_response) {
    // layers (top to bottom):
    //   ROS:  request and response are written in ROS message types
    //   JSON: written in JSON texts
    //   HTTP: written in HTTP messages

    try {
      // ROS -> JSON
      std::ostringstream json_request;
      writeJson(json_request, ros_request, false);

      // JSON -> HTTP
      bnh::client::request http_request;
      bnh::destination(http_request, http_destination_);
      bnh::body(http_request, json_request.str());

      // call the google cloud vision apis on the HTTP layer
      bnh::client::response http_response;
      {
        bnh::client::options http_options;
        http_options.always_verify_peer(true);
        if (ros_request.http_timeout > 0) {
          http_options.timeout(ros_request.http_timeout);
        }
        bnh::client http_client(http_options);
        http_response = http_client.post(http_request);
      }

      // HTTP -> JSON
      std::istringstream json_response(bnh::body(http_response));

      // JSON -> ROS
      readJson(json_response, ros_response);
    } catch (const std::exception &error) {
      // set a local runtime error when happened
      msgs::AnnotateImageResponse error_response;
      error_response.has_error = true;
      error_response.error.message = error.what();
      ros_response = srvs::Annotate::Response();
      ros_response.responses.resize(ros_request.requests.size(), error_response);
    }
  }

private:
  const std::string http_destination_;
};
}

#endif