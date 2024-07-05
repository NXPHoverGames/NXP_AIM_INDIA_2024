# Copyright 2024 NXP

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

import numpy as np
import cv2
import math

# synapse_msgs is a package defined at ~/cognipilot/cranium/src/.
  # It provides support for ROS2 messages to be used with CogniPilot.
# EdgeVectors is a message type that stores at most two vectors.
  # These vectors represent the shape and magnitude of the road edges.
from synapse_msgs.msg import EdgeVectors

QOS_PROFILE_DEFAULT = 10

PI = math.pi

RED_COLOR = (0, 0, 255)
BLUE_COLOR = (255, 0, 0)
GREEN_COLOR = (0, 255, 0)

VECTOR_IMAGE_HEIGHT_PERCENTAGE = 0.40  # Bottom portion of image to be analyzed for vectors.
VECTOR_MAGNITUDE_MINIMUM = 2.5


class EdgeVectorsPublisher(Node):
	""" Initializes edge vector publisher node with the required publishers and subscriptions.

		Returns:
			None
	"""
	def __init__(self):
		super().__init__('edge_vectors_publisher')

		# Subscription for camera images.
		self.subscription_camera = self.create_subscription(
			CompressedImage,
			'/camera/image_raw/compressed',
			self.camera_image_callback,
			QOS_PROFILE_DEFAULT)

		# Publisher for edge vectors.
		self.publisher_edge_vectors = self.create_publisher(
			EdgeVectors,
			'/edge_vectors',
			QOS_PROFILE_DEFAULT)

		# Publisher for thresh image (for debug purposes).
		self.publisher_thresh_image = self.create_publisher(
			CompressedImage,
			"/debug_images/thresh_image",
			QOS_PROFILE_DEFAULT)

		# Publisher for vector image (for debug purposes).
		self.publisher_vector_image = self.create_publisher(
			CompressedImage,
			"/debug_images/vector_image",
			QOS_PROFILE_DEFAULT)

		self.image_height = 0
		self.image_width = 0
		self.lower_image_height = 0
		self.upper_image_height = 0

	""" Publishes images for debugging purposes.

		Args:
			publisher: ROS2 publisher of the type sensor_msgs.msg.CompressedImage.
			image: image given by an n-dimensional numpy array.

		Returns:
			None
	"""
	def publish_debug_image(self, publisher, image):
		message = CompressedImage()
		_, encoded_data = cv2.imencode('.jpg', image)
		message.format = "jpeg"
		message.data = encoded_data.tobytes()
		publisher.publish(message)

	""" Calculates vector angle in radians.

		Args:
			vector: vector as a list of two coordinates. Ex: [[x1, y1], [x2, xy]].

		Returns:
			theta: vector angle with the x-axis in radians.
	"""
	def get_vector_angle_in_radians(self, vector):
		if ((vector[0][0] - vector[1][0]) == 0):  # Right angle vector.
			theta = PI / 2
		else:
			slope = (vector[1][1] - vector[0][1]) / (vector[0][0] - vector[1][0])
			theta = math.atan(slope)

		return theta

	""" Analyzes the pre-processes image and creates vectors on the road edges, if they exist.
		Computes contours around the objects found in the thresh image.
		Vectors are computed using the minimum and maximum y-value coordinates of contours.
		Vectors of magnitude less than VECTOR_MAGNITUDE_MINIMUM are discarded.

		Args:
			image: original image taken by cam given by an n-dimensional numpy array.
			thresh: pre-processes binary image given by an n-dimensional numpy array.

		Returns:
			vectors: nested list of vectors with their distance from the rover point.
				Note: len(vectors) = 0/1/2.
				Each vector is represented by two coordinates and their distance.
					Ex: [[x1, y1], [x2, y2], distance from rover point].
				Ex: for two vectors = [[[1, 2], [2, 1], 12.0, [[3, 4], [4, 3], 10.0]];
			image: image with vectors drawn in it in blue color, for debugging purposes.
	"""
	def compute_vectors_from_image(self, image, thresh):
		# Draw contours around the objects found in the image.
		contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]

		vectors = []
		for i in range(len(contours)):
			coordinates = contours[i][:, 0, :]

			min_y_value = np.min(coordinates[:, 1])
			max_y_value = np.max(coordinates[:, 1])

			# Get coordinates with minimum and maximum values of y-coords respectively.
			min_y_coords = np.array(coordinates[coordinates[:, 1] == min_y_value])
			max_y_coords = np.array(coordinates[coordinates[:, 1] == max_y_value])

			min_y_coord = min_y_coords[0]
			max_y_coord = max_y_coords[0]

			magnitude = np.linalg.norm(min_y_coord - max_y_coord)
			if (magnitude > VECTOR_MAGNITUDE_MINIMUM):
				# Calculate distance from the rover to the middle point of vector.
				rover_point = [self.image_width / 2, self.lower_image_height]
				middle_point = (min_y_coord + max_y_coord) / 2
				distance = np.linalg.norm(middle_point - rover_point)

				angle = self.get_vector_angle_in_radians([min_y_coord, max_y_coord])
				if angle > 0:
					min_y_coord[0] = np.max(min_y_coords[:, 0])
				else:
					max_y_coord[0] = np.max(max_y_coords[:, 0])

				vectors.append([list(min_y_coord), list(max_y_coord)])
				vectors[-1].append(distance)

			cv2.line(image, min_y_coord, max_y_coord, BLUE_COLOR, 2)

		return vectors, image

	""" Processes the image and creates vectors on the road edges in the image, if they exist.
		Performs image processing such as grayscale, thresholding for separating road edges.
		Chops the image according to VECTOR_IMAGE_HEIGHT_PERCENTAGE before vector analysis.
		Sorts the vectors created on the basis of their distance from the rover.
		Then, chooses one vector each from the left and right half of the image.

		Publishes intermediate images (viewable using foxglove) for debugging purposes.

		Args:
			image: image given by an n-dimensional numpy array.

		Returns:
			final_vectors: nested list of vectors. Note: len(final_vectors) = 0/1/2.
				Each vector is represented by two coordinates. Ex: [[x1, y1], [x2, y2]].
				Ex: for two vectors = [[[23, 171], [0, 180]], [[281, 171], [319, 189]]];
	"""
	def process_image_for_edge_vectors(self, image):
		self.image_height, self.image_width, color_count = image.shape
		self.lower_image_height = int(self.image_height * VECTOR_IMAGE_HEIGHT_PERCENTAGE)
		self.upper_image_height = int(self.image_height - self.lower_image_height)

		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # Convert to grayscale image.
		# Separate black (color of edges) pixels from the rest by applying threshold.
		threshold_black = 25
		thresh = cv2.threshold(gray, threshold_black, 255, cv2.THRESH_BINARY_INV)[1]

		thresh = thresh[self.image_height - self.lower_image_height:]
		image = image[self.image_height - self.lower_image_height:]
		vectors, image = self.compute_vectors_from_image(image, thresh)

		# Sort vectors based on distance from rover, as we only want vectors closest to us.
		vectors = sorted(vectors, key=lambda x: x[2])

		# Split vectors based on whether they lie in the left or right half of the image.
		half_width = self.image_width / 2
		vectors_left = [i for i in vectors if ((i[0][0] + i[1][0]) / 2) < half_width]
		vectors_right = [i for i in vectors if ((i[0][0] + i[1][0]) / 2) >= half_width]

		final_vectors = []
		# Pick one vector each from the left and right half of the image, if they exist.
		for vectors_inst in [vectors_left, vectors_right]:
			if (len(vectors_inst) > 0):
				cv2.line(image, vectors_inst[0][0], vectors_inst[0][1], GREEN_COLOR, 2)
				vectors_inst[0][0][1] += self.upper_image_height
				vectors_inst[0][1][1] += self.upper_image_height
				final_vectors.append(vectors_inst[0][:2])

		self.publish_debug_image(self.publisher_thresh_image, thresh)
		self.publish_debug_image(self.publisher_vector_image, image)

		return final_vectors

	""" Analyzes the image received from /camera/image_raw/compressed to detect road edges.
		Vectors - given by two (x, y) coordinates - represent the approximate road edges.
		The vectors created are published on /edge_vectors. Vector count could be 0/1/2.

		Converts message to an n-dimensional numpy array representation of the image.

		Args:
			message: "docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CompressedImage.html"

		Returns:
			None
	"""
	def camera_image_callback(self, message):
		np_arr = np.frombuffer(message.data, np.uint8)
		image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

		vectors = self.process_image_for_edge_vectors(image)

		vectors_message = EdgeVectors()
		vectors_message.image_height = image.shape[0]
		vectors_message.image_width = image.shape[1]
		vectors_message.vector_count = 0
		if (len(vectors) > 0):
			vectors_message.vector_1[0].x = float(vectors[0][0][0])
			vectors_message.vector_1[0].y = float(vectors[0][0][1])
			vectors_message.vector_1[1].x = float(vectors[0][1][0])
			vectors_message.vector_1[1].y = float(vectors[0][1][1])
			vectors_message.vector_count += 1
		if (len(vectors) > 1):
			vectors_message.vector_2[0].x = float(vectors[1][0][0])
			vectors_message.vector_2[0].y = float(vectors[1][0][1])
			vectors_message.vector_2[1].x = float(vectors[1][1][0])
			vectors_message.vector_2[1].y = float(vectors[1][1][1])
			vectors_message.vector_count += 1
		self.publisher_edge_vectors.publish(vectors_message)

		return


def main(args=None):
	rclpy.init(args=args)

	edge_vectors_publisher = EdgeVectorsPublisher()

	rclpy.spin(edge_vectors_publisher)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	edge_vectors_publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
