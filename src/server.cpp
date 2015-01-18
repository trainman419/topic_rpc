/*
 * Test for Topic RPC library
 *
 *
 * Author: Austin Hendrix
 */

#include <ros/ros.h>

#include <topic_rpc/rpc.h>
#include <topic_rpc/RpcTest.h>

void callback(topic_rpc::RpcTestRequest & req,
              topic_rpc::RpcTestResponse & resp) {
  resp.sum = req.a + req.b;
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "topic_rpc_server");

  ros::NodeHandle nh;

  topic_rpc::Server<topic_rpc::RpcTest> server(nh, "rpc_test");

  while(ros::ok()) {
    ros::spinOnce();
  }
}
