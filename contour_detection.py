import cv2 as cv
import numpy as np
 
img= cv.imread('Resources/doge.jpg')
cv.imshow('Doge' , img)

blank = np.zeros(img.shape, dtype='uint8')


# Contours are boundaries of an object. For object analysis and shape detection.

gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
cv.imshow('Grey', gray)

# canny = cv.Canny(img, 100, 125)
# cv.imshow('Canny edges', canny)

ret, thresh = cv.threshold(gray, 125, 250, cv.THRESH_BINARY)
cv.imshow('Thresh', thresh)

contours, hierarchies = cv.findContours(thresh,cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
print(f'{len(contours)} contour(s) found.')  

# To draw contours to the blank

cv.drawContours(blank, contours, -1, (0,0,255), 2)
cv.imshow('Contours Drawn', blank)


# cv.RETR_LIST ==> All the contours 
# cv.RETR_EXTERNAL ==> Only the external contours
# cv.RETR_TREE ==> All hierarchical contours



cv.waitKey(0)