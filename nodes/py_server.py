#!/usr/bin/env python
#
# topic_rpc 
#
# Author: Austin Hendrix

import rospy
import topic_rpc
from topic_rpc.srv import RpcTest, RpcTestResponse

def handle_request(req):
    return RpcTestResponse('', req.a + req.b)

class TestServ(object):
    def __init__(self, c):
        self.c = c

    def callback(self, req):
        return RpcTestResponse('', req.a + req.b + self.c)


def main():
    rospy.init_node('topic_rpc_py_server')

    server = topic_rpc.Server('rpc_test', RpcTest, handle_request)

    testserv = TestServ(42)

    server2 = topic_rpc.Server('rpc_test2', RpcTest, testserv.callback)

    rospy.spin()

if __name__ == '__main__':
    main()
