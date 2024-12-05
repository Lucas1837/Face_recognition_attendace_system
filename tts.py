#!/usr/bin/env python3
import rospy
import subprocess
import shutil
from std_msgs.msg import String

# Function to check if Festival is installed
def check_festival_installed():
    if not shutil.which("festival"):
        rospy.logerr("Festival is not installed or not found in the PATH.")
        exit(1)

# Callback function for the subscriber
def callback(data):
    # Log received message

    # Check if the received text is empty
    
    # Log that the message will be spoken
    rospy.loginfo(f"Speaking: {data.data}")
    
    # Use Festival TTS to speak the received text
    result = subprocess.run(f"echo '{data.data}' | festival --tts", shell=True, capture_output=True)
    
    # Check if there was an error with Festival
    if result.returncode != 0:
        rospy.logerr(f"Scanned Face: {result.stderr.decode()}")

# Main function to initialize the ROS node and subscriber
def tts_listener():
    # Check if Festival is installed
    check_festival_installed()

    # Initialize the ROS node
    rospy.init_node('tts_node', anonymous=True)

    # Use a ROS parameter to allow customization of the topic name
    topic_name = "/name"

    # Subscribe to the topic and set the callback function
    rospy.Subscriber(topic_name, String, callback)
    rospy.loginfo(f"TTS Node is ready")

    # Handle graceful shutdown
    rospy.on_shutdown(lambda: rospy.loginfo("Shutting down TTS Node..."))

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        tts_listener()
    except rospy.ROSInterruptException:
        pass