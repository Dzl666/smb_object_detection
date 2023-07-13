import rospy
from std_msgs.msg import String
import numpy as np
from sklearn.cluster import KMeans

# CLASSES = [11, 24, 25, 39, 56, 74]
CLASSES = [
    "stop sign",
    "backpack",
    "umbrella",
    "bottle",
    "chair",
    "clock",
]

THRES = 1.0

# This list will hold all messages from the 'detection_info' topic
detected_info_messages = []

def detection_info_callback(msg):
    # Append received message to the list

    # TODO change frame to world frame
    detected_info_messages.append(msg.data)

def process_detected_objects_callback(msg):
    # Create an array with all the messages
    all_detections = []
    for m in detected_info_messages:
        all_detections += m.info

    # Find points for each class
    for class in CLASSES:
        msg_of_class = [m if class == m.class_id for m in all_detections]

        # Create numpy array containing all the 3D points
        X = np.zeros((len(msg_of_class), 3))
        for i, m in enumerate(msg_of_class):
            X[i, 0] = m.position.x
            X[i, 1] = m.position.y
            X[i, 2] = m.position.z

        # Perform k-means with increasing number of classes, starting at 1.
        n_clusters = 1
        while True:
            # Do k-means
            kmeans = KMeans(n_clusters=n_clusters, random_state=0, n_init="auto").fit(X)

            # Get the stddev of each cluster, if it is slow, we have finished
            failed = False
            for i in range(n_clusters):
                mask = kmeans.labels_ == i
                points = X[n_clusters, :]
                var = np.var(points, axis=0)
                if np.all(var > THRES):
                    failed = True
                    break

            if failed is True:
                n_clusters += 1
                continue
            else:
                break


def listener():
    # Initiate a node
    rospy.init_node('listener_node')

    # Subscribe to the 'detection_info' topic
    rospy.Subscriber('detection_info', String, detection_info_callback)

    # Subscribe to the 'process_detected_objects' topic
    rospy.Subscriber('process_detected_objects', String, process_detected_objects_callback)

    # Keep the node running until it is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()