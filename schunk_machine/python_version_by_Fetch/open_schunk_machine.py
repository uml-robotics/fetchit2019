#! /usr/bin/env python

from __future__ import print_function
import rospy

import actionlib

import fetchit_challenge.msg

def schunk_machine_client():
    client = actionlib.SimpleActionClient('schunk_machine', fetchit_challenge.msg.SchunkMachineAction)

    client.wait_for_server()
    print("Connected to server")
    goal = fetchit_challenge.msg.SchunkMachineGoal(state=0)

    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('schunk_machine_client')
    result = schunk_machine_client()
    print(result)
