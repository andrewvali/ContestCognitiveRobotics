#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def callback(data):
    if data.data == "?":
        print("Unrecognized person! Do you want add a new identity in the database? Yes or Not")
        value = input()
        if value == "Yes":
            name_person = input("Insert the name: ")
            surname_person = input("Insert the surname: ")
            person = {
                "name": name_person,
                "surname": surname_person
            }
            print("Person added: {}".format(person))
        else:
            print("Person not added!")
    else:
        print(data.data)

rospy.init_node('visualization_node',anonymous=True)
rospy.Subscriber('result_topic',String, callback)

try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")


