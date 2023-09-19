import matplotlib.pyplot as plt
import cv2
import numpy as np
from transform import four_point_transform

# reading in image to cv2, will later be a video
image = cv2.imread("/home/dev/test_ws/src/igvc_test/igvc_test/road.jpg")

def grey(image):
    """Returns image with grayscale applied
    
    :param image: input image
    :return: image with grayscale filter"""
    image = np.asarray(image)
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

def gauss(image):
    """Returns image with gaussian blur applied
    
    :param image: input image
    :return: image with gaussian blur filter"""
    return cv2.GaussianBlur(image, (5, 5), 0)

def canny(image):
    """Returns image with canny edge applied
    
    :param image: input image
    :return: image with canny edge filter"""
    edges = cv2.Canny(image, 50, 150)
    return edges

def region(image, offset):
    """Returns image with a triangle mask with a tip in the center of the image and offset off of the bottom by a certain amount to cut out extra detail
    
    :param image: input image
    :param offset: The number of pixels the mask will be off of the bottom
    :return: image with triangle mask bitwise_anded with the input"""
    height, width = image.shape
    triangle  = np.array([
        [(0, height - offset), (2677, 2252), (width, height - offset)]
    ])
    mask = np.zeros_like(image)

    mask = cv2.fillPoly(mask, triangle, 255)
    mask = cv2.bitwise_and(image, mask)

    triangle = np.array([
        [(1000, height), (2677, 2252), (width - 1000, height)]
    ])
    mask2 = np.zeros_like(image)
    rectangle = np.array([
        [(0, 0), (width, 0), ( width, height), ( 0, height)]
    ])
    mask2= cv2.fillPoly(mask2, rectangle, 255)
    mask2 = cv2.fillPoly(mask2, triangle, 0)
    mask = cv2.bitwise_and(mask2, mask)

    return mask

def make_points(image, average):
    """Defines a line according to an image and outputs it's beginning and end
    
    :param image: image proccesed with canny edge detection
    :param average: an averaged line
    :return: an array with the start and end points of a line"""
    slope, y_int = average
    y1 = image.shape[0]
    y2 = int(y1 * (3/5))
    x1 = int((y1 - y_int) / slope)
    x2 = int((y2 - y_int) / slope)
    return np.array([x1, y1, x2, y2])

def line_average(image, lines):
    """Averages the small line segments from HoughLinesP into one line
    
    :param image: input image
    :param lines: output from HoughLinesP
    :return: an array holding the begin and end points for each line"""
    left = []
    right = []
    for line in lines:
        print(line)
        x1, y1, x2, y2 = line.reshape(4)
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        y_int = parameters[1]
        if slope < 0:
            left.append((slope, y_int))
        else:
            right.append((slope, y_int))

    right_avg = np.average(right, axis = 0)
    left_avg = np.average(left, axis = 0)
    left_line = make_points(image, left_avg)
    right_line = make_points(image, right_avg)
    return np.array([left_line, right_line])

def display_lines(image, lines):
    """creates an image with lines drawn onto them
    
    :param image: the image to display lines onto
    :param lines: averaged lines to draw
    :return: a black image with lines drawn onto them"""
    lines_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line
            cv2.line(lines_image, (x1, y1), (x2, y2), (255, 0, 0), 50)
    return lines_image

pts = np.array([(0, 3000), (2307, 2307), (3080, 2307), (5469, 3000)])
warped = four_point_transform(image, pts)

for (x, y) in pts:
    cv2.circle(image, (x, y), 50, (0, 255, 0), -1)

masked_image = region(canny(gauss(grey(image))), 150)

lines = cv2.HoughLinesP(masked_image,
                        rho=2, 
                        theta=np.pi/180, 
                        threshold=100, 
                        lines=np.array([]), 
                        minLineLength=40, 
                        maxLineGap=5)

averaged_lines = line_average(image, lines)

print("\n\n\n\n\n\n\n")

for line in averaged_lines:
    print(line)

black_lines = display_lines(image, averaged_lines)

lanes = cv2.addWeighted(image, 0.8, black_lines, 1, 1)

plt.imshow(warped)
plt.show()