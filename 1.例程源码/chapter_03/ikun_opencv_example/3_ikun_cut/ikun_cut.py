#!/usr/bin/python 
# -*- coding: utf-8 -*-
import numpy as np
import cv2 as cv

'''
作者: 小虎哥哥爱学习
微信: robot4xiihoo
官网: www.xiihoo.com
'''

image = cv.imread("ikun_resized.png")
cv.imshow("分割前", image)

# 创建一个内核，我们将用它来锐化我们的图像 
# 一个二阶导数的近似值，一个非常强大的内核
kernel = np.array([[1, 1, 1], [1, -8, 1], [1, 1, 1]], dtype=np.float32)
# do the laplacian filtering as it is
# well, we need to convert everything in something more deeper then CV_8U
# because the kernel has some negative values,
# and we can expect in general to have a Laplacian image with negative values
# BUT a 8bits unsigned int (the one we are working with) can contain values from 0 to 255
# so the possible negative number will be truncated
imgLaplacian = cv.filter2D(image, cv.CV_32F, kernel)
imgLaplacian = np.clip(imgLaplacian, 0, 255)
imgLaplacian = np.uint8(imgLaplacian)
cv.imshow('分割后', imgLaplacian)



cv.waitKey()
cv.destroyAllWindows()

cv.imwrite("ikun_cut.png",imgLaplacian)