"""Shared code of attach.py / detach.py / cancel.py: call one /wheelchair/<cmd> Trigger service and print its response.

Returns at once (the cmd result is in /wheelchair/status); the outcome is the exit code:
    0  accepted
    1  rejected, or no response in time
    2  service not found (apriltag_control not running) or an argparse usage error
"""

import argparse
import os
import sys

import rclpy
from std_srvs.srv import Trigger

DISCOVERY_TIMEOUT = 5.0  # s, wait for apriltag_control's service
RESPONSE_TIMEOUT = 2.0   # s, the service answers at once


def main(action: str) -> None:
    argparse.ArgumentParser(description=f'Call /wheelchair/{action} and print its response.').parse_args()
    rclpy.init()
    node = rclpy.create_node(f'wheelchair_{action}_tool_{os.getpid()}')
    client = node.create_client(Trigger, f'/wheelchair/{action}')
    try:
        if not client.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT):
            code = 2
        else:
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=RESPONSE_TIMEOUT)
            response = future.result()
            if response is None:
                code = 1
            else:
                print(f'{action}: success={response.success} message={response.message}', flush=True)
                code = 0 if response.success else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)
