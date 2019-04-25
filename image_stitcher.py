# import the necessary packages
from imutils import paths
import numpy as np
import imutils
import cv2

class ImageStitcher:
	def __init__(self):
		pass

	def stitch_images(self, directory, output_path):
		# grab the paths to the input images and initialize our images list
		print("[INFO] loading images...")
		imagePaths = sorted(list(paths.list_images(directory)))
		images = []

		# loop over the image paths, load each one, and add them to our
		# images to stich list
		for imagePath in imagePaths:
			image = cv2.imread(imagePath)
			images.append(image)

		# initialize OpenCV's image sticher object and then perform the image
		# stitching
		print("[INFO] stitching images...")
		stitcher = cv2.createStitcher() if imutils.is_cv3() else cv2.Stitcher_create()
		(status, stitched) = stitcher.stitch(images)

		# if the status is '0', then OpenCV successfully performed image
		# stitching
		if status == 0:
			# write the output stitched image to disk
			cv2.imwrite(output_path, stitched)

			# display the output stitched image to our screen
			cv2.imshow("Stitched", stitched)
			cv2.waitKey(0)

		# otherwise the stitching failed, likely due to not enough keypoints)
		# being detected
		else:
			print("[INFO] image stitching failed ({})".format(status))

if __name__ == "__main__":
	im_st = ImageStitcher()
	im_st.stitch_images("./stitch_images", "output.png")
