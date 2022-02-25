#include "ros/ros.h"                          // ROS Default Header File
#include "demo_service/AddTwoInts.h"// AddTwoInts Service File Header (Automatically created after build)
#include <cstdlib>                            // Library for using the "atoll" function

int main(int argc, char **argv)               // Node Main Function
{
  ros::init(argc, argv, "service_client");    // Initializes Node Name

  if (argc != 3)  // Input value error handling
  {
    ROS_INFO("cmd : rosrun demo_service service_client arg0 arg1");
    ROS_INFO("arg0: double number, arg1: double number");
    return 1;
  }

  ros::NodeHandle nh;       // Node handle declaration for communication with ROS system

  // Declares service client 'demo_service_client'
  // using the 'AddTwoInts' service file in the 'demo_service' package.
  // The service name is 'demo_srv'
  ros::ServiceClient demo_service_client = nh.serviceClient<demo_service::AddTwoInts>("demo_srv");

  // Declares the 'srv' service that uses the 'AddTwoInts' service file
  demo_service::AddTwoInts srv;
  // Parameters entered when the node is executed as a service request value are stored at 'a' and 'b'
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);

  // Request the service. If the request is accepted, display the response value
  if (demo_service_client.call(srv))
  {
    ROS_INFO("send srv, srv.Request.a and b: %ld, %ld", (long int)srv.request.a, (long int)srv.request.b);
    ROS_INFO("receive srv, srv.Response.result: %ld", (long int)srv.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service demo_service");
    return 1;
  }
  return 0;
}
