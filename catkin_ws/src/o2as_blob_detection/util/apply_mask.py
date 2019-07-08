#!/usr/bin/env python

from math import pi
import numpy as np

from PIL import Image, ImageDraw
import sys
import cv2

def mask_apply(img_to_mask, mask_img):

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
    mask_cv = cv2.imread(mask_img,0)
    res = cv2.bitwise_and(img_cv,img_cv, mask = mask_cv)
    cv2.imwrite('image.png',res)

if __name__ == '__main__':
       
    mask_apply(sys.argv[1], sys.argv[2])
