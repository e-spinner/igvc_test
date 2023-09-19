import numpy as np
import cv2

def order_points(pts):
    """A function that orders points into a rectagle to be used in a perspective warp
    
    :param pts: four ordered pairs of points to order
    :return: the ordered rectangle"""
    #initialize a list of coordinates such as that the following order is followed:
    #   1 - top-left
    #   2 - top-right
    #   3 - bottom-right
    #   4 - bottom-left
    rect = np.zeros((4, 2), dtype = "float32")

    # the top-left point will have the smallest sum, whereas
	# the bottom-right point will have the largest sum
    s = pts.sum(axis = 1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]

    # now, compute the difference between the points, the
    # top-right point will have the smallest difference,
    # whereas the bottom-left will have the largest difference
    diff = np.diff(pts, axis = 1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]

    # return the ordered coordinates
    return rect

def four_point_transform(image, pts):
    """A function that takes and image and warps according to the supplied points

    :param image: the input image
    :param pts: points to warp the image to
    :return: a warped image
    """

    #obtain a consistant order of points and unpack them individually
    rect = (order_points(pts))
    (tl, tr, br, bl) = rect

    # compute the width of the new image which will be either the distance between
    # the tl and tr ot the br and bl x-coordinates
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))

    # compute the height of the new image which will either be the distance between
    # the tr and br or the tl and bl y-coordinates
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))

    # construct set of destination points to obtain a birds-eye-view
    dst = np.array([
        [0,            0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0,            maxHeight -1 ]], dtype= "float32")

    # compute the perspective transform matrix and then apply int
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))

    return warped