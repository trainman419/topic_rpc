/*
 * Topic RPC: an ROS RPC library implemented using topics
 *
 * Author: Austin Hendrix
 */

#ifndef TOPIC_RPC__CLIENT_H
#define TOPIC_RPC__CLIENT_H

#include <string>
#include <ros/ros.h>

namespace topic_rpc {
  template <class RPC_TYPE>
  class Client {
    private:
      typedef typename RPC_TYPE::Request  Request;
      typedef typename RPC_TYPE::Response Response;

      std::string service_name_;
      ros::Publisher  request_pub_;
      ros::Subscriber response_sub_;

    public:
      Client(ros::NodeHandle & nh, const std::string service_name) :
        service_name_(service_name) {

        // TODO: queue size
        request_pub_ = nh.advertise<Request>(service_name_ + "/request", 10);

        response_sub_ = nh.subscribe(service_name_ + "/response", 10,
            &Client<RPC_TYPE>::callback, this);
        
        ROS_INFO_STREAM("Create topic_rpc client for " << service_name_);

      }

      // TODO: wait for subscribers? (ie request servers)

      void call(const typename RPC_TYPE::Request & req) {
        request_pub_.publish(req);
        // TODO: wait for response...
        // TODO: automatically fill in ID?
      }

    private:
      void callback(const typename RPC_TYPE::Response::ConstPtr & resp) {
      }
  };
} // namespace topic_rpc

#endif
