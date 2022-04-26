import cv2 as cv
import numpy as np
from math import atan2

def getOrientation(pts):
  """
    This function determines the orientation of a contour given the contour points

    :param pts: contour points
    :return: angle of the contour
  """ 
  # Construct a buffer used by the pca analysis
  sz = len(pts)
  data_pts = np.empty((sz, 2), dtype=np.float64)
  for i in range(data_pts.shape[0]):
    data_pts[i,0] = pts[i,0,0]
    data_pts[i,1] = pts[i,0,1]
 
  # Perform PCA analysis
  mean = np.empty((0))
  mean, eigenvectors, _ = cv.PCACompute2(data_pts, mean)

  # Return angle
  return atan2(eigenvectors[0,1], eigenvectors[0,0])

def is_goal_tipped(img, x1, y1, x2, y2): 
  """
    This function determines the state of a goal (tipped or not tipped) given an image and bounding box

    :param img: Full images from camera
    :param x1 y1 x2 y2: Object location identified by YOLOv5 model
    :return: returns -1 if state of goal cannot be determined
             returns one integer (0,1) corresponding to (not tipped, tipped) state
  """ 
  # Resize image to only located goal object
  img = img[y1:y2, x1:x2]

  # Transform image
  gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
  _, bw = cv.threshold(gray, 50, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
  contours, _ = cv.findContours(bw, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
  angles = []

  for i, c in enumerate(contours):
      # Calculate the area of each contour
      area = cv.contourArea(c)
      # Ignore contours that are too small or too large
      if area < 2000 or 100000 < area:
          continue
      # Find orientation of shape
      angles.append(abs(int(np.rad2deg(getOrientation(c)))))
  


  # Determine goal state and return result
  if len(angles) != 1:
    return -1
  elif angles[0] > 45:
    return 1
  else:
    return 0