#!/usr/bin/env python

from math import pi
import numpy as np

from PIL import Image, ImageDraw
import sys
import cv2

def detect_blob(img_to_mask):

   #used to do the bitwise comparison between the mask and the original image
   # mask = Image.new('RGB', (640,480), 255)
   # ImageDraw.Draw(mask).polygon(polygon, outline = 1, fill = 1)
   # mask_np = np.array(mask)

    #img_original = Image.open(img_to_mask)
    #img_original_np = np.array(img_original)

    #img_original = Image.open(mask)
    #img_original_np = np.array(img_original)

    #img_res_np = np.bitwise_and(mask_np, img_original_np)
    #img_res = Image.fromarray(np.uint8(img_res_np)) 
    #img_res.save('mask_original_bin.png','PNG')
    img_cv = cv2.imread(img_to_mask)
    img = np.asarray(img_cv)[:, :, ::-1]

    im_rgb = img
    im_gray = cv2.cvtColor(im_rgb,cv2.COLOR_BGR2GRAY)

    params = cv2.SimpleBlobDetector_Params()

    # Segmentation Thresholds
    params.minThreshold = 100
    params.maxThreshold = 400

    # Filter by color
    params.filterByColor = True
    params.blobColor = 0

    # Filter by size of the blob.
    params.filterByArea = True
    params.minArea = 20
    params.maxArea = 150

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.7

    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.87

    # Filter by Inertia
    params.filterByInertia = False
    params.minInertiaRatio = 0.01

    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3:
        detector = cv2.SimpleBlobDetector(params)
    else:
        detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs.
    keypoints = detector.detect(im_gray)

    # Draw the key points 
    im_with_keypoints = cv2.drawKeypoints(im_rgb, keypoints, np.array([]), (255, 0, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Show blobs

    cv2.imwrite("blob_detection_test_results.png", cv2.cvtColor(im_with_keypoints, cv2.COLOR_BGR2RGB))
    #print("before publish")


if __name__ == '__main__':
       
    detect_blob(sys.argv[1])
