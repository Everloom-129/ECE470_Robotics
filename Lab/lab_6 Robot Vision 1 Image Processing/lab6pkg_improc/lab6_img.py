#!/usr/bin/env python

'''

lab6pkg_improc/lab6_img.py

@brief: image processing for lab6, including template matching, image filtering, edge detection, contour detection, etc.
@author: Songjie Xiao
@date: Monday 2023/3/20

'''

#########################################################################
#
#  Lab 6-Image Processing
#  Main task is to pick and place all the blocks to your destination.
#  Camera is used to detect blocks. With image processing methods, 
#  robot is able to know the position, shape and orientation of the blocks.
#  Since block's position is based on image coordinate, what we should do
#  is to transform it into robot coordinate.
# 
#  In this lab, you only need to complete the image processing part.
#
#########################################################################

import cv2
import numpy as np

class ImageProcess():
    def __init__(self):

        self.contours_elip = []
        self.contours_rect = []

    def template_process(self, img_path):

        # read image from img_path
        img_temp = cv2.imread(img_path)

        if img_temp is None:
            print('Template Image is None, Please check image path!')
            return
        
        img_copy = img_temp.copy()

        # Bilateral Filter smooths images while preserving edges,
        # by means of a nonlinear combination of nearby image values.
        # cv2.bilateralFilter(src, d, sigmaColor, sigmaSpace)
        # d - Diameter of each pixel neighborhood that is used during filtering.
        img_blur = cv2.bilateralFilter(img_copy, 19, 130, 30)

        # cv2.medianBlur() is used to reduce noise in the image
        img_blur = cv2.medianBlur(img_blur, 9)

        # cv2.cvtColor() is used to convert the image to grayscale
        img_gray = cv2.cvtColor(img_blur, cv2.COLOR_BGR2GRAY)

        # cv2.Sobel() is used to find the gradient of the image
        x_grid = cv2.Sobel(img_gray, cv2.CV_16SC1, 1, 0)
        y_grid = cv2.Sobel(img_gray, cv2.CV_16SC1, 0, 1)
        # cv2.Canny() is used to find the edges of the image
        img_canny = cv2.Canny(x_grid, y_grid, 30, 220)

        # split image into two parts, rectangle and ellipse
        weight, height = img_canny.shape
        img_rect = img_canny[:, :weight]
        img_elip = img_canny[:, weight:]

        # cv2.findContours to find contour
        # variable contours_1 stores all contours, each of which is composed of a series of pixel point 
        # For example: len(contours) contours[0] 
        # rectangle
        self.contours_rect, hierarchy = cv2.findContours(img_rect, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print(len(self.contours_rect))
        # elipse
        self.contours_elip, hierarchy = cv2.findContours(img_elip, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Test code
        # cv2.imshow('grey image', img_gray)
        # cv2.waitKey()
        # cv2.destroyAllWindows()
        # print('contours_rect: ', self.contours_rect)
        # print('length of rectangle: ', len(self.contours_rect))
        # print('example of rectangle: ', self.contours_rect[0])
        # print('contours_elip: ', self.contours_elip)
        # print('length of ellipse: ', len(self.contours_elip))
        # print('example of ellipse: ', self.contours_elip[0])

    def image_process(self, img_path):

        # read image from img_path
        img_src = cv2.imread(img_path)

        if img_src is None:
            print('Source Image is None, Please check image path!')
            return
        
        img_copy = img_src.copy()

        ############## Your Code Start Here ##############
        # TODO: image process including filtering, edge detection, contour detection and so on.
        
        # Important: check contours and make sure the contour is available.
        # For one block, there will be 4 contours, 2 for rectangle or ellipse outside and 2 for arrow inside.
        # For one rectangle edge, there will be 2 contours in image resolution.
        # Tips: use cv2.contourArea(contour) as thres hold to filter out the contour.

        img_blur = cv2.bilateralFilter(img_copy, 5, 130, 30)
        img_blur = cv2.medianBlur(img_blur, 7)
        # img_gray = cv2.cvtColor(img_copy, cv2.COLOR_BGR2GRAY)
        img_gray = cv2.cvtColor(img_blur, cv2.COLOR_BGR2GRAY)
        x_grid = cv2.Sobel(img_gray, cv2.CV_16SC1, 1, 0)
        y_grid = cv2.Sobel(img_gray, cv2.CV_16SC1, 0, 1)
        img_canny = cv2.Canny(x_grid, y_grid, 30, 180)
        
        # cv2.imshow("canny_image", img_canny)
        # cv2.waitKey()
        # cv2.destroyAllWindows()
        
        _contours, hierarchy = cv2.findContours(img_canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # print(_contours)
        threshold_small = 1
        threshold_large = 200000
        contours = []

        for cnt in _contours:
            area = cv2.contourArea(cnt)
            # print("area: ", area)
            if area > threshold_small and area < threshold_large:
                print("area: ", area)
                contours.append(cnt)

        contour_image = np.zeros_like(img_copy)
        cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 2)
        cv2.imshow("filtered contours", contour_image)
        cv2.waitKey()
        cv2.destroyAllWindows()
        # print(contours)

        ############### Your Code End Here ###############

        # length of contours equals to 4 times the number of blocks
        print("length of contours: ", len(contours))

        ############## Your Code Start Here ##############
        # TODO: compute the center of contour and the angle of arrow,
        # as well as match shapes of your block
        center_value = []
        shape = [] # 0 represents rectangle while 1 represents ellipse
        theta = []
        for i in range(len(contours) // 2):
            if i % 2 == 0:# Even
                ############## Your Code Start Here ##############
                # TODO: compute the center of external contour (rectangle/ellipse) and match shapes of your block
                # Tips: ret = cv2.matchShapes(contour1, contour2, 1, 0.0) 

                # cv2.circle(draw_img, (int(center_x), int(center_y)), 7, [0,0,255], -1)

                rect_likelihood = cv2.matchShapes(contours[i * 2], self.contours_rect[1], 1, 0.0)
                elip_likelihood = cv2.matchShapes(contours[i * 2], self.contours_elip[1], 1, 0.0)
                if rect_likelihood < elip_likelihood:
                    shape.append(0)
                else:
                    shape.append(1)

                N = cv2.moments(contours[i * 2])
                x = int(N["m10"] / N["m00"])
                y = int(N["m01"] / N["m00"])
                cv2.circle(img_copy, (int(x), int(y)), 7, [0,0,255], -1)
                center_value.append([x,y])
                print(len(center_value))
                ############### Your Code End Here ###############

            else:
                # TODO: compute the center of internal contour (arrow) and compute the angle of arrow
                N = cv2.moments(contours[i * 2])
                _center_x = int(N["m10"] / N["m00"])
                _center_y = int(N["m01"] / N["m00"])
                # draw a circle on the center point
                cv2.circle(img_copy, (int(_center_x), int(_center_y)), 7, [0,255,0], -1)

                ############## Your Code Start Here ##############
                # TODO: compute the angle of arrow
                # Tips: compute the distance between center point of external contour and every point of internal contour,
                # find the furthest point, then you can compute the angle.
                print("i//2: ", i//2)
                print("center_value: ", center_value)
                x_ex = center_value[i // 2][0]
                print("x_ex: ", x_ex)
                y_ex = center_value[i // 2][1]
                print("y_ex: ", y_ex)

                max_dis = 0
                x = 0
                y = 0
                for cnt in contours[i * 2]:
                    dis = np.sqrt((cnt[0][0] - x_ex) ** 2 + (cnt[0][1] - y_ex) ** 2)
                    if dis > max_dis:
                        x = cnt[0][0]
                        y = cnt[0][1]
                        max_dis = dis

                theta_tmp = np.arctan2(_center_y - y, _center_x - x)

                cv2.line(img_copy, (_center_x, _center_y), (x, y), (128, 0, 0), 2)

                theta.append(theta_tmp)
                

                ############### Your Code End Here ###############

        cv2.imshow('image with center and orientation', img_copy)
        cv2.waitKey()
        cv2.destroyAllWindows()

        return center_value, shape, theta

def main():

    # init image path
    # template image for template matching of rectangle and ellipse
    path_template = "img/template.jpg"
    # two calibrattoion images to calculate the image coordinate
    # with corresponding image coordinate and robot coordinate, 
    # a linear transformation between image coordinate and robot coordinate can be computed
    path_img_cali1 = 'myimg/one rect.png'
    path_img_cali2 = 'myimg/one elip.png'
    # snapshot image saved by your camera
    path_img_snap = 'myimg/all 1.png'

    # init ImageProcess class
    img_process = ImageProcess()

    # template process to get the contour of rectangle and ellipse
    img_process.template_process(path_template)
    print("finished template")
    # image process for calibration images to get the position of blocks
    center_cali1, shape_cali1, theta_cali1 = img_process.image_process(path_img_cali1)
    print("finished 1")

    center_cali2, shape_cali2, theta_cali2 = img_process.image_process(path_img_cali2)
    print("finished 2")

    # image process to get the shape, position and orientation of blocks
    center_snap, shape_snap, theta_snap = img_process.image_process(path_img_snap)
    print("finished many")

if __name__ == '__main__':
	
	main()