from typing import Tuple

import numpy as np
import cv2


def get_steer_matrix_left_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:              The shape of the steer matrix.

    Return:
        steer_matrix_left:  The steering (angular rate) matrix for reactive control
                            using the masked left lane markings (numpy.ndarray)
    """

    # TODO: implement your own solution here
    steer_matrix_left = -np.tril(np.ones(shape), k=-15)/np.repeat(np.arange(shape[0]+1, 1, -1), shape[1]).reshape(shape) \
        /np.tile(np.arange(shape[1]+1, 1, -1), shape[0], ).reshape(shape)
    # ---
    return steer_matrix_left


def get_steer_matrix_right_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:               The shape of the steer matrix.

    Return:
        steer_matrix_right:  The steering (angular rate) matrix for reactive control
                             using the masked right lane markings (numpy.ndarray)
    """

    # TODO: implement your own solution here
    steer_matrix_right = np.tril(np.ones(shape), k=-15)/np.repeat(np.arange(shape[0]+1, 1, -1), shape[1]).reshape(shape)\
        /np.tile(np.arange(1, shape[1]+1), shape[0], ).reshape(shape)
    # ---
    return steer_matrix_right


def detect_lane_markings(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Args:
        image: An image from the robot's camera in the BGR color space (numpy.ndarray)
    Return:
        mask_left_edge:   Masked image for the dashed-yellow line (numpy.ndarray)
        mask_right_edge:  Masked image for the solid-white line (numpy.ndarray)
    """
    h, w, _ = image.shape

    imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    sigma = 3
    img_gaussian_filter = cv2.GaussianBlur(img,(0,0), sigma)
    sobelx = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,1,0)
    sobely = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,0,1)
    Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)
    threshold = 40
    mask_mag = (Gmag > threshold)

    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    mask_sobely_pos = (sobely > 0)
    mask_sobely_neg = (sobely < 0)

    width = img.shape[1]
    mask_left = np.ones(sobelx.shape)
    mask_left[:,int(np.floor(width/2)):width + 1] = 0
    mask_right = np.ones(sobelx.shape)
    mask_right[:,0:int(np.floor(width/2))] = 0

    white_lower_hsv = np.array([0, 0, 100])         # CHANGE ME
    white_upper_hsv = np.array([255, 60, 255])   # CHANGE ME
    yellow_lower_hsv = np.array([25, 100, 100])        # CHANGE ME
    yellow_upper_hsv = np.array([30, 255, 255])  # CHANGE ME 

    mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)

    mask_left_edge = mask_left * mask_mag * mask_sobelx_neg * mask_sobely_neg * mask_yellow
    mask_right_edge = mask_right * mask_mag * mask_sobelx_pos * mask_sobely_neg * mask_white
    # mask_left_edge = mask_left * mask_mag * mask_sobelx_neg * mask_sobely_neg
    # mask_right_edge = mask_right * mask_mag * mask_sobelx_pos * mask_sobely_neg

    return mask_left_edge, mask_right_edge
