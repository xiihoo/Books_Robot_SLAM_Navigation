#!/usr/bin/python 
# -*- coding: utf-8 -*-
import cv2 as cv

'''
作者: 小虎哥哥爱学习
微信: robot4xiihoo
官网: www.xiihoo.com
'''

image = cv.imread("ikun_filtered.png")

scale = 1.5

width  = int(image.shape[1] * scale)
height = int(image.shape[0] * scale)
dim = (width, height)

resized = cv.resize(image, dim, interpolation = cv.INTER_AREA)

cv.imshow("缩放前", image)
cv.imshow("缩放后", resized)
cv.waitKey()
cv.destroyAllWindows()

cv.imwrite("ikun_resized.png",resized)