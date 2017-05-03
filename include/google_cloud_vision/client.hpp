#ifndef GOOGLE_CLOUD_VISION_CLIENT
#define GOOGLE_CLOUD_VISION_CLIENT

#include <sstream>
#include <string>

#ifndef BOOST_NETWORK_ENABLE_HTTPS
#warning "Define BOOST_NETWORK_ENABLE_HTTPS if google_cloud_vision::Client does not work"
#endif
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
    try {
      // ROS -> JSON
      std::ostringstream json_request;
      writeJson(json_request, ros_request, false);

      // call the google cloud vision apis
      std::istringstream json_response;
      {
        bnh::client::options http_options;
        if (ros_request.http_timeout > 0) {
          http_options.timeout(ros_request.http_timeout);
        }
        bnh::client http_client(http_options);
        json_response.str(bnh::body(http_client.post(http_destination_, json_request.str())));
      }

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
  const bnh::client::request http_destination_;
};
}

#endif