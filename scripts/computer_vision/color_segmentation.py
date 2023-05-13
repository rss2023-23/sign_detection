import cv2
import numpy as np
import pdb

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

#Image
HEIGHT = 376
WIDTH = 672
top = np.zeros((int((2/8.0)*HEIGHT), WIDTH), dtype="uint8")
middle = np.ones((int((6/8.0)*HEIGHT), WIDTH), dtype="uint8")
line_follower_mask = np.vstack((top, middle))

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(image, template=None, is_line_follower=False):
	"""
	Implement the sign detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a sign to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the sign, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	if isinstance(image, type(None)):
		return ((0,0),(0,0))
	
	#Reduce image noise
	blurred_image = cv2.GaussianBlur(image, (3,3), 0)
	blurred_image = cv2.erode(blurred_image, (3,3))
	blurred_image = cv2.dilate(blurred_image, (3,3))

	blurred_image = cv2.bitwise_and(blurred_image, blurred_image, mask=line_follower_mask)

	# Convert image to HSV
	image_hsv = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)

	# Create Color Mask
	light_gray = (0, 70, 50)
	dark_gray = (5, 255, 255)
	color_mask = cv2.inRange(image_hsv, light_gray, dark_gray)
	#print(color_mask)

	# Extract out Sign
	filtered_image = cv2.bitwise_and(image, image, mask=color_mask)
	image_print(filtered_image)

	try:
		# Identify Contours
		_, thresholded_image = cv2.threshold(color_mask, 40, 255, cv2.THRESH_BINARY)
		_, contours, _  = cv2.findContours(thresholded_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		
		# Choose contour of largest area
		best_contour = max(contours, key=cv2.contourArea)
		x,y,w,h = cv2.boundingRect(best_contour)
		bounding_box = ((x,y), (x+w, y+h))

		# Visualize Bounding Box
		cv2.rectangle(image,bounding_box[0],bounding_box[1],(0,255,0),2)
		image_print(image)

		# Return bounding box
		return bounding_box
	
	except:
		return ((0,0),(0,0))

cd_color_segmentation(cv2.imread("../..//image.png"))
