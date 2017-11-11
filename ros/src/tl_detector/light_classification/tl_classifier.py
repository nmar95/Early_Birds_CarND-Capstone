from styx_msgs.msg import TrafficLight
from sensor_msgs.msg import Image

import tensorflow as tf
import matplotlib.pyplot as plt
import numpy as np
import visualization_utils as vis_util
import label_map_util
import rospy
from cv_bridge import CvBridge
import cv2
import scipy.misc
import os.path

class TLClassifier(object):
    def __init__(self):
        PATH_TO_MODEL = os.path.abspath('light_classification/frozen_models/frozen_inference_graph.pb')
        PATH_TO_LABELS = os.path.abspath('light_classification/label_map.pbtxt')
        NUM_CLASSES = 3

        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("processed_image",Image)

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_MODEL, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            self.d_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            self.d_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.d_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_d = self.detection_graph.get_tensor_by_name('num_detections:0')
        self.sess = tf.Session(graph=self.detection_graph)
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        rospy.loginfo('trying to classify image')
        with self.detection_graph.as_default():
            # Expand dimension since the model expects image to have shape [1, None, None, 3].
            img_expanded = np.expand_dims(image, axis=0)  
            boxes, scores, classes, num = self.sess.run( [self.d_boxes, self.d_scores, self.d_classes, self.num_d], feed_dict={self.image_tensor: img_expanded})
            outimage = image
            vis_util.visualize_boxes_and_labels_on_image_array(outimage, np.squeeze(boxes), np.squeeze(classes).astype(np.int32), np.squeeze(scores), self.category_index, use_normalized_coordinates=True, line_thickness=8)

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(outimage, "rgb8"))
            except CvBridgeError as e:
                print(e)

            #Basically get the color with the highest score
            color = 4
            if(num > 0):
                if(scores[0][0] > 0.6):
                    color = self.category_index[classes[0][0]]['id']
                else:
                    rospy.loginfo('Not Confident enough')
            
            if color == 1:
                rospy.loginfo('GREEN')
                return TrafficLight.GREEN
            elif color == 2:
                rospy.loginfo('RED')
                return TrafficLight.RED
            elif color == 3:
                rospy.loginfo('YELLOW')
                return TrafficLight.YELLOW
            else:
                rospy.loginfo('UNKNOWN')
                return TrafficLight.UNKNOWN
