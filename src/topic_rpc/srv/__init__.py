#!/usr/bin/env python
#
# Topic RPC
#
# Temporary python shim to make service request and response types work
# as message types.
#
# In the future, this should be made compatible with message generation and
# there should be a cmake call to generate this shim
#
# Author: Austin Hendrix

from topic_rpc.srv import RpcTest

RpcTestRequest = RpcTest.Request
RpcTestResponse = RpcTest.Response
