#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import face_recognition
import cv2
import json

class FaceRecognitionNode:
    def __init__(self):
        #this two variables are used as part of anti-spoofing feature, if you wish to increase the sensitivity of face_recognition, can lower down these two variables
        self.contrast = 2  
        self.brightness = 90    

        self.recognized_faces_pub = rospy.Publisher('/recognized_faces', String, queue_size=10)
        self.video_capture = cv2.VideoCapture(0)  

        if not self.video_capture.isOpened():
            rospy.logerr("Unable to open camera!")
            raise Exception("Camera initialization failed")

        rospy.loginfo("Face Recognition Node Initialized")

    def process_frame(self, frame):

        self.adjusted_frame = cv2.convertScaleAbs(frame, alpha=self.contrast, beta=self.brightness)

        face_locations = face_recognition.face_locations(self.adjusted_frame)

        if face_locations:

            face_encodings = face_recognition.face_encodings(self.adjusted_frame,face_locations)

            face_encodings_list = [encoding.tolist() for encoding in face_encodings]

            str_face_encode = json.dumps(face_encodings_list)
            self.recognized_faces_pub.publish(str(str_face_encode))

            

    def capture_and_recognize(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown() :
            ret, frame = self.video_capture.read()

            if not ret:
                rospy.logwarn("Failed to capture image from camera.")
                continue

            self.process_frame(frame)

            cv2.imshow("Camera Feed", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            rate.sleep()

    def cleanup(self):

        if self.video_capture.isOpened():
            self.video_capture.release()

        cv2.destroyAllWindows()
        rospy.loginfo("Face Recognition Node cleanup complete.")


def run_face_recognition_node():

    rospy.init_node("face_recognition_node")

    try:
        node = FaceRecognitionNode()
        node.capture_and_recognize()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt Exception: Shutting down.")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
    finally:
        node.cleanup()

run_face_recognition_node()
