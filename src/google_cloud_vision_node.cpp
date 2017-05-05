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
  ros::init(argc, argv, "google_cloud_vision");
  ros::NodeHandle nh;

  std::string key;
  if (!rp::get("~key", key)) {
    ROS_ERROR("Required parameter ~key is missing");
    return 1;
  }

  client.reset(new gcv::Client(key));

  ros::ServiceServer server(nh.advertiseService("annotete", call));

  ros::spin();

  return 0;
}