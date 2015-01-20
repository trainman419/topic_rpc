/*
 * Topic RPC: an ROS RPC library implemented using topics
 *
 * Author: Austin Hendrix
 */

#ifndef TOPIC_RPC__CLIENT_H
#define TOPIC_RPC__CLIENT_H

#include <string>
#include <vector>

#include <uuid/uuid.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

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

      boost::mutex  queue_mutex_;
      boost::condition_variable queue_ready_;
      std::vector<std::string> requests_;

      typedef std::vector<typename Response::ConstPtr> response_vector;
      response_vector responses_;

    public:
      Client(ros::NodeHandle & nh, const std::string service_name) :
        service_name_(service_name) {

        service_name_ = nh.resolveName(service_name_);

        // TODO: queue size
        request_pub_ = nh.advertise<Request>(service_name_ + "/request", 10);

        response_sub_ = nh.subscribe(service_name_ + "/response", 10,
            &Client<RPC_TYPE>::callback, this);
        
        ROS_DEBUG_STREAM_NAMED("topic_rpc", "Created topic_rpc client for " <<
            service_name_);

      }

      // TODO: wait for subscribers? (ie servers)

      typename Response::ConstPtr call(const Request & req) {
        // automatically fill in ID
        uuid_t req_uuid;
        std::string node_name = ros::this_node::getName();
        uuid_generate(req_uuid);
        char uuid_str[40];
        uuid_unparse(req_uuid, uuid_str);
        Request request = req;
        request.id = node_name + "_" + uuid_str;

        // insert request ID into outstanding request list
        queue_mutex_.lock();
        requests_.push_back(request.id);
        queue_mutex_.unlock();

        while(request_pub_.getNumSubscribers() < 1 ||
              response_sub_.getNumPublishers() < 1) {
          ROS_WARN_STREAM_THROTTLE_NAMED(2.0, "topic_rpc", "Waiting for "
              "topic_rpc server on " << service_name_);
          ros::Duration(0.1).sleep();
          if(!ros::ok())
            return typename Response::ConstPtr();
        }

        ROS_DEBUG_STREAM_NAMED("topic_rpc", "Sending RPC request: " << request);

        // publish request
        request_pub_.publish(request);

        ROS_DEBUG_NAMED("topic_rpc", "Waiting for RPC response");
        
        // wait for response
        boost::unique_lock<boost::mutex> lock(queue_mutex_);
        while(std::count(requests_.begin(), requests_.end(), request.id) > 0) {

          lock.unlock();
          ros::spinOnce();
          lock.lock();

          queue_ready_.timed_wait(lock, boost::posix_time::milliseconds(100));
          if(!ros::ok())
            return typename Response::ConstPtr();
        }

        ROS_DEBUG_NAMED("topic_rpc", "RPC response received");

        // extract response from list
        typename response_vector::iterator itr;
        for(itr = responses_.begin(); itr != responses_.end(); itr++ ) {
          if((*itr)->id == request.id) break;
        }

        if( itr != responses_.end() ) {
          typename Response::ConstPtr response = *itr;
          responses_.erase(itr);
          ROS_DEBUG_STREAM_NAMED("topic_rpc", "Got RPC response: " <<
              *response);
          return response;
        } else {
          ROS_ERROR("topic_rpc internal error: response received but not "
                    "found");
          return typename Response::ConstPtr();
        }
      }

      // TODO(1.0): asynchronous API

    private:
      void callback(const typename Response::ConstPtr & resp) {
        ROS_DEBUG_STREAM_NAMED("topic_rpc", "Got RPC response: " << *resp);
        queue_mutex_.lock();
        std::vector<std::string>::iterator req;
        req = std::find(requests_.begin(), requests_.end(), resp->id);
        if(req != requests_.end()) {
          ROS_DEBUG_STREAM_NAMED("topic_rpc", "Processing response: " <<
              resp->id);
          requests_.erase(req);

          responses_.push_back(resp);
          // notify
          queue_ready_.notify_all();
        } else {
          ROS_DEBUG_STREAM_NAMED("topic_rpc", "Matching request for " <<
              resp->id << " not found");
        }
        queue_mutex_.unlock();
        ROS_DEBUG_NAMED("topic_rpc", "RPC response processed");
      }
  };
} // namespace topic_rpc

#endif
