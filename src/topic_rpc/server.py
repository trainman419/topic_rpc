#!/usr/bin/env python
#
# topic_rpc python server implementation
#
# Author: Austin Hendrix

import rospy

class Server(object):
    def __init__(self, name, service_type, callback):
        self.service_name = rospy.resolve_name(name)
        self.service_type = service_type
        self.pub_name = self.service_name + '/response'
        self.sub_name = self.service_name + '/request'
        self.callback = callback

        self.started = False

        if self._checkForOtherServers():
            rospy.logfatal("Another topic_rpc server is already running for " +
                    self.service_name + ", this server will not be started")
            return

        self.pub = rospy.Publisher(self.pub_name, service_type._response_class,
                queue_size=10)
        self.sub = rospy.Subscriber(self.sub_name, service_type._request_class,
                self._callback)

        self.started = True

    def isOk(self):
        return self.started

    def __bool__(self):
        return self.started

    def _callback(self, request):
        response = self.callback(request)
        response.id = request.id
        self.pub.publish(response)

    def _checkForOtherServers(self):
        # check if another service is already running for this service
        # returns true if another service exists, false otherwise
        code, msg, state = rospy.client.get_master().getSystemState()
        if code != 1:
            rospy.logerror("Unable to check for other servers. Assuming none" +
                    " are running")
            return False
        publishers = state[0]
        subscribers = state[1]

        other_sub = False
        other_pub = False

        for pub in publishers:
            if pub[0] == self.pub_name:
                other_pub = True

        for sub in subscribers:
            if sub[0] == self.sub_name:
                other_sub = True

        return other_pub and other_sub
