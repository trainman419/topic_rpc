/*
 * Topic RPC: an ROS RPC library implemented using topics
 *
 * Author: Austin Hendrix
 */

#ifndef TOPIC_RPC__SERVER_H
#define TOPIC_RPC__SERVER_H

#include <string>

#include <ros/ros.h>

namespace topic_rpc {
  template <class RPC_TYPE>
    class Server {
      private:
        typedef typename RPC_TYPE::Request  Request;
        typedef typename RPC_TYPE::Response Response;

        std::string service_name_;
        ros::Publisher  response_pub_;
        ros::Subscriber request_sub_;

      public:
        Server(ros::NodeHandle & nh, std::string service_name) :
          service_name_(service_name) {

          // TODO: specify queue size
          response_pub_ = nh.advertise<Request>(service_name + "/response", 10);
          request_sub_ = nh.subscribe(service_name + "/request", 10,
              &topic_rpc::Server<RPC_TYPE>::callback, this);
          ROS_INFO_STREAM("Created topic_rpc server for " << service_name_);
        }

        void callback(const typename Request::ConstPtr & req) {
        }
    };
} // namespace topic_rpc
#endif
