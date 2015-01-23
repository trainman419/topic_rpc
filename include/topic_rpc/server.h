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
        std::string response_topic_;
        std::string request_topic_;

        bool started_;
        ros::Publisher  response_pub_;
        ros::Subscriber request_sub_;

        typedef boost::function<bool(Request&, Response&)> CallbackType;
        CallbackType callback_;

      public:
        // TODO(1.0): specify queue size
        Server(ros::NodeHandle & nh, std::string service_name,
            CallbackType cb) :
          service_name_(service_name),
          callback_(cb),
          started_(false) {
            setup(nh);
        }

        template <class T>
        Server(ros::NodeHandle & nh, std::string service_name,
            bool(T::*cb)(Request &, Response &), T * obj) :
          service_name_(service_name),
          callback_(boost::bind(cb, obj, _1, _2)),
          started_(false) {
            setup(nh);
        }

        // TODO(1.0): version that takes a shared_ptr to a class
        // Note: no version that takes a callback on ServiceEvent

        operator bool() { return started_; }

        bool isOk() { return started_; }

      private:
        void requestCallback(Request req) {
          ROS_DEBUG_STREAM_NAMED("topic_rpc", "Got request for service " <<
              service_name_ << ": " << req);
          Response resp;               // create response
          callback_(req, resp);        // call the callback
          resp.id = req.id;            // fill in response ID
          response_pub_.publish(resp); // publish response
        }

        // check if another server is already running for this service
        // returns true if another service exists, false otherwise
        bool checkForOtherServers() {
          std::string node_name = ros::this_node::getName();
          XmlRpc::XmlRpcValue request(node_name);
          XmlRpc::XmlRpcValue response;
          XmlRpc::XmlRpcValue payload;
          bool success = ros::master::execute("getSystemState", request,
              response, payload, false);

          ROS_ASSERT(payload.getType() == XmlRpc::XmlRpcValue::TypeArray);
          ROS_ASSERT(payload.size() == 3);

          bool request_sub  = false;
          bool response_pub = false;


          XmlRpc::XmlRpcValue & publishers = payload[0];
          ROS_ASSERT(publishers.getType() == XmlRpc::XmlRpcValue::TypeArray);
          for(int i=0; i<publishers.size(); i++) {
            XmlRpc::XmlRpcValue & topic = publishers[i];
            ROS_ASSERT(topic.getType() == XmlRpc::XmlRpcValue::TypeArray);
            ROS_ASSERT(topic[0].getType() == XmlRpc::XmlRpcValue::TypeString);
            std::string topic_name = topic[0];
            if(topic_name == response_topic_) {
              response_pub = true;
              ROS_INFO("Response topic is published");
            }
          }

          XmlRpc::XmlRpcValue & subscribers = payload[1];
          ROS_ASSERT(subscribers.getType() == XmlRpc::XmlRpcValue::TypeArray);
          for(int i=0; i<subscribers.size(); i++) {
            XmlRpc::XmlRpcValue & topic = subscribers[i];
            ROS_ASSERT(topic.getType() == XmlRpc::XmlRpcValue::TypeArray);
            ROS_ASSERT(topic[0].getType() == XmlRpc::XmlRpcValue::TypeString);
            std::string topic_name = topic[0];
            if(topic_name == request_topic_) {
              request_sub = true;
              ROS_INFO("Request topic is subscribed to");
            }
          }

          return response_pub && request_sub;
        }

        void setup(ros::NodeHandle &nh) {
          service_name_ = nh.resolveName(service_name_);
          response_topic_ = service_name_ + "/response";
          request_topic_ = service_name_ + "/request";

          // check for duplicate servers and fail to start
          if(checkForOtherServers()) {
            ROS_FATAL_STREAM_NAMED("topic_rpc", "Another topic_rpc server is "
                "already running for " << service_name_ << ", this server will"
                " not be started");
            return;
          }

          response_pub_ = nh.advertise<Response>(response_topic_, 10);
          request_sub_ = nh.subscribe(request_topic_, 10,
              &topic_rpc::Server<RPC_TYPE>::requestCallback, this);

          ROS_DEBUG_STREAM_NAMED("topic_rpc", "Created topic_rpc server for "
              << service_name_);
          started_ = true;
        }
    };
} // namespace topic_rpc
#endif
