# import the necessary packages
from __future__ import division

import numpy as np
import imutils
import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats
import os

class Stitcher:
	def __init__(self):
		# determine if we are using OpenCV v3.X
		self.isv3 = imutils.is_cv3(or_better=True)
		self.stitched_result = []
		self.stitched_depth_result = []
		self.depth_images_path = []
		self.i = 0

	def stitch(self, images, ratio=0.75, reprojThresh=4.0,
		showMatches=False):
		# unpack the images, then detect keypoints and extract
		# local invariant descriptors from them
		(imageB, imageA) = images
		(kpsA, featuresA) = self.detectAndDescribe(imageA)
		(kpsB, featuresB) = self.detectAndDescribe(imageB)
		
		# match features between the two images
		M = self.matchKeypoints(kpsA, kpsB,
			featuresA, featuresB, ratio, reprojThresh)

		# if the match is None, then there aren't enough matched
		# keypoints to create a panorama
		if M is None:
			return None

		# otherwise, apply a perspective warp to stitch the images
		# together
		(matches, H, status) = M
		result = cv2.warpPerspective(imageA, H,
			(imageA.shape[1] + imageB.shape[1], imageA.shape[0]))
		
		result[0:imageB.shape[0], 0:imageB.shape[1]] = imageB
		
		gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
		_,thresh = cv2.threshold(gray,1,255,cv2.THRESH_BINARY)
		contours = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		cnt = contours[0]
		x,y,w,h = cv2.boundingRect(cnt)
		result = result[y:y+h,x:x+w]

		shape_sum = imageA.shape[1] + imageB.shape[1]

		result = self.blend_image(result)
		
		self.stitch_depth(H, x, y, w, h, shape_sum)

		# check to see if the keypoint matches should be visualized
		if showMatches:
			vis = self.drawMatches(imageA, imageB, kpsA, kpsB, matches,
				status)

			# return a tuple of the stitched image and the
			# visualization
			return (result, vis)

		# return the stitched image
		return result
	
	def blend_image(self, result_image):
		a = result_image.copy()
		width = a.shape[1]-1
		height = a.shape[0]-1
		for i, row in enumerate(np.rot90(a,3)):
		        for j, pixel in enumerate(row):
		            if pixel.sum() < 10:
		                a[height-j][i] = np.average(a[height-j][i-6:i-1])
		return a

	def detectAndDescribe(self, image):
		# convert the image to grayscale
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		# check to see if we are using OpenCV 3.X
		if self.isv3:
			# detect and extract features from the image
			descriptor = cv2.xfeatures2d.SIFT_create()
			(kps, features) = descriptor.detectAndCompute(image, None)

		# otherwise, we are using OpenCV 2.4.X
		else:
			# detect keypoints in the image
			detector = cv2.FeatureDetector_create("SIFT")
			kps = detector.detect(gray)

			# extract features from the image
			extractor = cv2.DescriptorExtractor_create("SIFT")
			(kps, features) = extractor.compute(gray, kps)

		# convert the keypoints from KeyPoint objects to NumPy
		# arrays
		kps = np.float32([kp.pt for kp in kps])

		# return a tuple of keypoints and features
		return (kps, features)

	def matchKeypoints(self, kpsA, kpsB, featuresA, featuresB,
		ratio, reprojThresh):
		# compute the raw matches and initialize the list of actual
		# matches
		matcher = cv2.DescriptorMatcher_create("BruteForce")
		rawMatches = matcher.knnMatch(featuresA, featuresB, 2)
		matches = []

		# loop over the raw matches
		for m in rawMatches:
			# ensure the distance is within a certain ratio of each
			# other (i.e. Lowe's ratio test)
			if len(m) == 2 and m[0].distance < m[1].distance * ratio:
				matches.append((m[0].trainIdx, m[0].queryIdx))

		# computing a homography requires at least 4 matches
		if len(matches) > 4:
			# construct the two sets of points
			ptsA = np.float32([kpsA[i] for (_, i) in matches])
			ptsB = np.float32([kpsB[i] for (i, _) in matches])

			# compute the homography between the two sets of points
			(H, status) = cv2.findHomography(ptsA, ptsB, cv2.RANSAC,
				reprojThresh)
			
			np.savetxt("./homographies/homography" + str(self.i) + ".txt", H)

			# return the matches along with the homograpy matrix
			# and status of each matched point
			return (matches, H, status)

		# otherwise, no homograpy could be computed
		return None

	def drawMatches(self, imageA, imageB, kpsA, kpsB, matches, status):
		# initialize the output visualization image
		(hA, wA) = imageA.shape[:2]
		(hB, wB) = imageB.shape[:2]
		vis = np.zeros((max(hA, hB), wA + wB, 3), dtype="uint8")
		vis[0:hA, 0:wA] = imageA
		vis[0:hB, wA:] = imageB

		# loop over the matches
		for ((trainIdx, queryIdx), s) in zip(matches, status):
			# only process the match if the keypoint was successfully
			# matched
			if s == 1:
				# draw the match
				ptA = (int(kpsA[queryIdx][0]), int(kpsA[queryIdx][1]))
				ptB = (int(kpsB[trainIdx][0]) + wA, int(kpsB[trainIdx][1]))
				cv2.line(vis, ptA, ptB, (0, 255, 0), 1)

		# return the visualization
		return vis
	
	def crop_image(self, img,tol=0):
		# img is image data
		# tol  is tolerance
		mask = img>tol
		return img[np.ix_(mask.any(1),mask.any(0))]
	
	def stitch_depth(self, H, x, y, w, h, shape_sum):
		# bar = np.zeros((42, 121))
		if self.i == 0:
			np_img = np.loadtxt('./' + self.depth_images_path[self.i])
			img = np.rot90(np_img, 3)
		else:
			img = self.stitched_depth_result

		if self.i >= len(self.depth_images_path)-1:
			np_img_ = np.loadtxt('./' + self.depth_images_path[self.i])
		else:
			np_img_ = np.loadtxt('./' + self.depth_images_path[self.i + 1])
		img_ = np.rot90(np_img_, 3)

		# homography_path = "./homography_0and3.txt"
		# print("Loading homography in " + homography_path)

		# H = np.loadtxt(homography_path)		
		# h1 = 512
		# w1 = int((img.shape[1] + img_.shape[1]) * (w/shape_sum))

		dst = cv2.warpPerspective(img_,H,(img.shape[1] + img_.shape[1], img.shape[0]))
		dst[0:img.shape[0], 0:img.shape[1]] = img

		# dst = dst[y:y+h1,x:x+w1]
		dst = self.crop_image(dst)
		dst = self.blend_image(dst)
		
		np.savetxt("./depth_stitch" + str(self.i) + ".txt", dst)
		cv2.imwrite("./depth_stitch" + str(self.i) + ".png", dst)
		self.stitched_depth_result = dst
	
	def main(self):
		# load the two images and resize them to have a width of 400 pixels
		# (for faster processing)
		images = [x for x in os.listdir('.') if x.endswith('png')]
		images.sort(key=lambda f: int(filter(str.isdigit, f)))

		self.depth_images_path = [y for y in os.listdir('.') if y.endswith('txt')]
		self.depth_images_path.sort(key=lambda g: int(filter(str.isdigit, g)))

		for img in images:
			if self.i == 0:
				imageA = cv2.imread('./' + images[self.i])
				center = int(imageA.shape[0]/2)
				imageA = imageA[center-int(1304/2):center+int(1304/2), :]
			else:
				imageA = self.stitched_result

			if self.i >= len(images)-1:
				imageB = cv2.imread('./' + images[self.i])
			else:
				imageB = cv2.imread('./' + images[self.i + 1])
			center = int(imageB.shape[0]/2)
			imageB = imageB[center-int(1304/2):center+int(1304/2), :]

			# stitch the images together to create a panorama
			result = st.stitch([imageA, imageB], showMatches=False)
			self.stitched_result = result
			self.i += 1

			# show the images
			cv2.imwrite("stitch" + str(self.i) + ".png", result)
			cv2.waitKey(0)

if __name__ == "__main__":
	st = Stitcher()
	st.main()