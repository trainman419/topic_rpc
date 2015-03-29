#!/usr/bin/env python
#
# topic_rpc test client for python
#
# Author: Austin Hendrix

import rospy
from topic_rpc.srv import RpcTest
import topic_rpc

def main():
    rospy.init_node("py_client")
    client = topic_rpc.Client('rpc_test', RpcTest)
    rospy.loginfo("Waiting for RPC server")
    client.waitForServer()
    rospy.loginfo("RPC server ready")

if __name__ == '__main__':
    main()
