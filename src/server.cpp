/*
 * Test for Topic RPC library
 *
 *
 * Author: Austin Hendrix
 */

#include <ros/ros.h>

#include <topic_rpc/rpc.h>
#include <topic_rpc/RpcTest.h>

bool callback(topic_rpc::RpcTestRequest  & req,
              topic_rpc::RpcTestResponse & resp) {
  resp.sum = req.a + req.b;
  return true;
}

class TestServ {
  public:
    TestServ(int c) : c_(c) {}

    bool callback(topic_rpc::RpcTestRequest  & req,
                  topic_rpc::RpcTestResponse & resp) {
      resp.sum = req.a + req.b + c_;
      return true;
    }

  private:
    int c_;
};

int main(int argc, char ** argv) {
  ros::init(argc, argv, "topic_rpc_server");

  ros::NodeHandle nh;

  topic_rpc::Server<topic_rpc::RpcTest> server(nh, "rpc_test", callback);

  TestServ test_serv(42);

  topic_rpc::Server<topic_rpc::RpcTest> server2(nh, "rpc_test2",
      &TestServ::callback, &test_serv);

  while(ros::ok()) {
    ros::spinOnce();
  }
}
