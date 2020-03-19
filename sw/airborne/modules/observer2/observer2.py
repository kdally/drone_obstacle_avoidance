#!/usr/bin/env python

import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import time


def convolve(image, kernel):
    (iH, iW) = image.shape[:2]
    (kH, kW) = kernel.shape[:2]

    # print('kH', kH)
    pad = (kW - 1) // 2
    image = cv2.copyMakeBorder(image, pad, pad, pad, pad, cv2.BORDER_REPLICATE)
    output = np.zeros((iH, iW), dtype="float32")

    # print('pad:', pad)
    # print('output:', output)

    max_k = 0a
    min_k = 1e9

    for y in np.arange(pad, iH + pad):
        for x in np.arange(pad, iW + pad):
            roi = image[y - pad:y + pad + 1, x - pad:x + pad + 1]
            k = (roi * kernel).sum()
            output[y - pad, x - pad] = k
            if k > max_k:
                max_k = k
            if k < min_k:
                min_k = k

    output = (output-min_k) * 255 / (max_k-min_k)
    # output = rescale_intensity(output, in_range=(0, 255))
    output = (output * 255).astype("uint8")

    return output

def color_filter(image, color_filters):
    # cls_pxls = [[]] * len(color_filters)
    dest     = image

    for idx, cf in enumerate(color_filters):
        col = (image[:,:,1]>=cf[0]) & (image[:,:,1]<=cf[1]) & (image[:,:,0]>=cf[2]) \
            & (image[:,:,0]<=cf[3]) & (image[:,:,2]>=cf[4]) & (image[:,:,2]<=cf[5])
        dest[col] = [[255, 255, 255]]

    return dest
    # return cls_pxls, dest


# path = "//home//alejandro//git//paparazzi//drone_images//H_0//orange_pole_202.png"
path = "//home//alejandro//Downloads//mavlab_pics//94615887.jpg"

# FILTER SETTINGS
# color_filters = [orange]
color_filters = [[105, 205, 52, 140, 180, 255], \
                 [0, 150, 0, 100, 0, 100]]


color_filters = [[150, 255, 100, 255, 100, 255]]

# color_filters = [green, orange]
color_filters = [[70, 100, 70, 100, 70, 100], \
                 [20, 205, 52, 140, 80, 220]]
# color_filters = [[0, 150, 0, 100, 0, 100]]


n = 3
xsmallBlur = np.ones((n, n), dtype="float") * (1.0 / (n * n))
smallBlur = np.ones((7, 7), dtype="float") * (1.0 / (7 * 7))
largeBlur = np.ones((21, 21), dtype="float") * (1.0 / (21 * 21))
sharpen = np.array((
	[0, -1, 0],
	[-1, 5, -1],
	[0, -1, 0]), dtype="int")
laplacian = np.array((
	[0, 1, 0],
	[1, -4, 1],
	[0, 1, 0]), dtype="int")
sobelX = np.array((
	[-1, 0, 1],
	[-2, 0, 2],
	[-1, 0, 1]), dtype="int")
sobelY = np.array((
	[-1, -2, -1],
	[0, 0, 0],
	[1, 2, 1]), dtype="int")
kernelBank = (
	("small_blur", smallBlur),
	("large_blur", largeBlur),
	("sharpen", sharpen),
	("laplacian", laplacian),
	("sobel_x", sobelX),
	("sobel_y", sobelY))

kernelBank = (("xsmall_blur", xsmallBlur),
              ("laplacian", laplacian))





# gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
colo = cv2.imread(path, cv2.IMREAD_UNCHANGED)
# yuvc = cv2.imread(path, cv2.COLOR_BGR2YUV)

yuvc = cv2.imread(path, cv2.COLOR_RGB2YUV)

t0 = time.time()

# Apply color filter
filt_img = color_filter(yuvc, color_filters)

# Go to grayscale
# filt_gray = cv2.cvtColor(filt_img, cv2.COLOR_BGR2GRAY)
filt_gray = filt_img[:,:,2]

# cv2.imshow('filtered image', filt_img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
 
# color_filters

# cv2.imshow('image', yuvc)
# cv2.waitKey(0)
# cv2.destroyAllWindows() 
# cv2.imshow('image', colo)
# cv2.waitKey(0)
# cv2.destroyAllWindows() 
# cv2.imshow('image', gray)
# cv2.waitKey(0)
# cv2.destroyAllWindows() 

# cls_pxls, img = color_filter(yuvc, color_filters)
# cv2.imshow('image', img)
# cv2.waitKey(0)
# cv2.destroyAllWindows() 

blur_gray = cv2.filter2D(filt_gray, -1, xsmallBlur)
img_output = cv2.filter2D(blur_gray, -1, laplacian)


t_tot = time.time() - t0
print("Time to run: {} seconds".format(t_tot))
print("Frequency: {} hz".format(1/t_tot))


cv2.imshow("raw", colo)
cv2.imshow("original", filt_img)
cv2.imshow("grey original", filt_gray)
cv2.imshow("1st filter", blur_gray)
cv2.imshow("2nd filter", img_output)

cv2.waitKey(0)
cv2.destroyAllWindows()

# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# for (kernelName, kernel) in kernelBank:
#     t0 = time.time()
#     print("[INFO] applying {} kernel".format(kernelName))
#     # convoleOutput = convolve(gray, kernel)
#     opencvOutput = cv2.filter2D(filt_gray, -1, kernel)
#     opencvOutput = cv2.filter2D(opencvOutput, -1, kernel)
#     # opencvOutput = cv2.filter2D(opencvOutput, -1, kernel)
#     # show the output images
#     cv2.imshow("original", gray)
#     # cv2.imshow("{} - convole".format(kernelName), convoleOutput)
#     cv2.imshow("{} - opencv".format(kernelName), opencvOutput)
#     print("Time to convolve: {} seconds".format(time.time()-t0))
#     # a = input("Next image")
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()























# img = cv2.imread(path, cv2.IMREAD_COLOR) 
# img = cv2.imread(path, 0)

# cv2.imshow('image', img) 
# cv2.waitKey(0) 

# cv2.destoryAllWindows() 