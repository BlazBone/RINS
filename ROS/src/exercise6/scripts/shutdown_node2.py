#!/usr/bin/python3

import rospy
import subprocess

def shutdown_node2():
    # Get the list of running nodes
    p = subprocess.Popen(['rosnode', 'list'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = p.communicate()
    nodes = out.splitlines()

    # Find the node named 'node2'
    for node in nodes:
        if 'only_parking' in node:
            # Kill the node named 'node2'
            subprocess.call(['rosnode', 'kill', node])
        elif "only_clynders":
            subprocess.call(['rosnode', 'kill', node])

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('shutdown_node2')

    # Register the shutdown function
    rospy.on_shutdown(shutdown_node2)

    # Spin until shutdown
    rospy.spin()
