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

        typedef boost::function<bool(Request&, Response&)> CallbackType;
        CallbackType callback_;

      public:
        // TODO(1.0): specify queue size
        Server(ros::NodeHandle & nh, std::string service_name,
            CallbackType cb) :
          service_name_(service_name),
          callback_(cb) {
            setup(nh);
        }

        template <class T>
        Server(ros::NodeHandle & nh, std::string service_name,
            bool(T::*cb)(Request &, Response &), T * obj) :
          service_name_(service_name),
          callback_(boost::bind(cb, obj, _1, _2)) {
            setup(nh);
        }

        // TODO(1.0): version that takes a shared_ptr to a class
        // Note: no version that takes a callback on ServiceEvent

      private:
        void requestCallback(Request req) {
          ROS_INFO_STREAM("Got request for service " << service_name_ << ": "
              << req);
          Response resp;               // create response
          callback_(req, resp);        // call the callback
          resp.id = req.id;            // fill in response ID
          response_pub_.publish(resp); // publish response
        }

        void setup(ros::NodeHandle &nh) {
          service_name_ = nh.resolveName(service_name_);
          response_pub_ = nh.advertise<Response>(service_name_ + "/response",
              10);
          request_sub_ = nh.subscribe(service_name_ + "/request", 10,
              &topic_rpc::Server<RPC_TYPE>::requestCallback, this);

          // TODO: detect duplicate servers and fail to start
          ROS_INFO_STREAM("Created topic_rpc server for " << service_name_);
        }
    };
} // namespace topic_rpc
#endif
