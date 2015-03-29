#!/usr/bin/env python
#
# topic_rpc python client implementation
#
# Author: Austin Hendrix

import rospy
import threading
import uuid

class Client(object):
    def __init__(self, name, service_type):
        self.service_type = service_type
        self.responses = {}
        self.requests = []
        self.lock = threading.Lock()
        self.service_name = rospy.resolve_name(name)

        self.pub = rospy.Publisher(self.service_name + '/request',
                service_type._request_class, queue_size=10)
        self.sub = rospy.Subscriber(self.service_name + '/response',
                service_type._response_class, self._response_callback)

    def waitForServer(self, timeout=rospy.Duration(0)):
        do_timeout = (timeout != rospy.Duration(0))
        end = rospy.Time.now() + timeout
        while self.pub.get_num_connections() < 1 or \
                self.sub.get_num_connections() < 1:
            rospy.sleep(0.1)

            # return false on shutdown
            if rospy.is_shutdown():
                return False

            # return false on timeout
            if do_timeout and rospy.Time.now() >= end:
                return False

        return True

    def _response_callback(self, response):
        with self.lock:
            if response.id in self.requests:
                self.responses[response.id] = response

    def __call__(self, *args):
        # construct request ID
        request_id = rospy.get_name() + '_' + str(uuid.uuid4())
        with self.lock:
            self.requests.append(request_id)
        req = self.service_type._request_class(request_id, *args)

        # TODO: wait for server to exist. fail if no server.
        if not self.waitForServer(rospy.Duration(0.01)):
            rospy.logwarn("Waiting for server on %s", self.service_name)
        if not self.waitForServer():
            raise rospy.ServiceException("topic_rpc server not available for" +
                    " service: %s"%self.service_name)

        # send request
        self.pub.publish(req)

        # wait for response
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            with self.lock:
                if request_id in self.responses:
                    response = self.responses[request_id]
                    del(self.responses[request_id])
                    return response
            r.sleep()

        raise rospy.ServiceException
