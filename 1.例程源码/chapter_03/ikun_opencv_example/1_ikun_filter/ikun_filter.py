#!/usr/bin/python 
# -*- coding: utf-8 -*-
import cv2 as cv

'''
作者: 小虎哥哥爱学习
微信: robot4xiihoo
官网: www.xiihoo.com
'''

image          = cv.imread("ikun.png")
image_filtered = cv.medianBlur(image, 13)

cv.imshow("滤波前", image)
cv.imshow("滤波后", image_filtered)
cv.waitKey()
cv.destroyAllWindows()

cv.imwrite("ikun_filtered.png",image_filtered)