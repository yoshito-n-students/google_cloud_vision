#include <fstream>
#include <iostream>

#include <boost/asio/connect.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ssl/context.hpp>
#include <boost/asio/ssl/rfc2818_verification.hpp>
#include <boost/asio/ssl/stream.hpp>
#include <boost/asio/streambuf.hpp>

#include <google_cloud_vision/json_parser.hpp>
#include <google_cloud_vision_msgs/encode.hpp>
#include <google_cloud_vision_srvs/Annotate.h>

namespace ba = boost::asio;
namespace bp = boost::property_tree;
namespace msgs = google_cloud_vision_msgs;
namespace srvs = google_cloud_vision_srvs;

int main(int argc, char *argv[]) {
  if (argc < 2) {
    return 1;
  }

  // open the given image file
  std::ifstream image_file(argv[1]);
  if (!image_file) {
    return 1;
  }

  // create the request
  srvs::Annotate::Request request;
  request.requests.resize(1);
  // request.requests[0].image.content = "base64_encoded_image";
  msgs::encode(image_file, request.requests[0].image);
  request.requests[0].features.resize(1);
  request.requests[0].features[0].type = msgs::Feature::TEXT_DETECTION;
  request.requests[0].features[0].has_max_results = true;
  request.requests[0].features[0].max_results = 10;

  // write the request to the buffer
  ba::streambuf buffer;
  {
    std::ostream os(&buffer);
    google_cloud_vision::writeJson(os, request, false);
  }

  // TODO: Call google cloud vision api
  /*
    // Create a context that uses the default paths for
    // finding CA certificates.
    ba::ssl::context ctx(ba::ssl::context::sslv23);
    ctx.set_default_verify_paths();

    // Open a socket and connect it to the remote host.
    ba::io_service io_service;
    ba::ssl::stream< ba::ip::tcp::socket > sock(io_service, ctx);
    ba::ip::tcp::resolver resolver(io_service);
    ba::ip::tcp::resolver::query query("host.name", "https");
    ba::connect(sock.lowest_layer(), resolver.resolve(query));
    sock.lowest_layer().set_option(ba::ip::tcp::no_delay(true));

    // Perform SSL handshake and verify the remote host's
    // certificate.
    sock.set_verify_mode(ba::ssl::verify_peer);
    sock.set_verify_callback(ba::ssl::rfc2818_verification("host.name"));
    sock.handshake(ba::ssl::stream< ba::ip::tcp::socket >::client);
  */

  // read the response from the buffer
  srvs::Annotate::Response response;
  {
    std::istream is(&buffer);
    google_cloud_vision::readJson(is, response);
  }
  std::cout << response << std::endl;

  return 0;
}
