#include <boost/scoped_ptr.hpp>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/service_server.h>

#include <google_cloud_vision/client.hpp>
#include <google_cloud_vision_srvs/Annotate.h>

namespace gcv = google_cloud_vision;
namespace rp = ros::param;
namespace srvs = google_cloud_vision_srvs;

boost::scoped_ptr< gcv::Client > client;

bool call(srvs::Annotate::Request &request, srvs::Annotate::Response &response) {
  if (!client) {
    return false;
  }
  client->call(request, response);
  return true;
}

int main(int argc, char *argv[]) {
  // init
  ros::init(argc, argv, "google_cloud_vision");
  ros::NodeHandle nh;

  // load parameters
  std::string key;
  if (!rp::get("~key", key)) {
    ROS_ERROR("Required parameter ~key is missing");
    return 1;
  }
  const int thread_count(rp::param("~thread_count", 1));

  // create a service backend
  client.reset(new gcv::Client(key));

  // create the service
  ros::ServiceServer server(nh.advertiseService("annotate", call));

  // run the service
  ros::MultiThreadedSpinner spinner(thread_count);
  spinner.spin();

  return 0;
}
