/*
 * Test for Topic RPC library
 *
 *
 * Author: Austin Hendrix
 */

#include <ros/ros.h>

#include <topic_rpc/rpc.h>
#include <topic_rpc/RpcTest.h>

int main(int argc, char ** argv) {
  ros::init(argc, argv, "topic_rpc_client");
  ros::NodeHandle nh;

  topic_rpc::Client<topic_rpc::RpcTest> client(nh, "rpc_test");

  while(ros::ok()) {
    topic_rpc::RpcTest::Request req;
    req.a = 1;
    req.b = 2;
    topic_rpc::RpcTest::Response::ConstPtr resp = client.call(req);
    if(resp) {
      ROS_INFO_STREAM("1 + 2 = " << resp->sum);
    } else {
      ROS_ERROR("RPC call failed");
    }
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }
}
