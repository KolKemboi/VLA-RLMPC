import cv2
import cvzone

class renderImages:
    def grid (self, front, back, left, right):
        self.front_image = cv2.cvtColor(front, cv2.COLOR_RGB2BGR) 
        self.back_image = cv2.cvtColor(back, cv2.COLOR_RGB2BGR) 
        self.left_image = cv2.cvtColor(left, cv2.COLOR_RGB2BGR) 
        self.right_image = cv2.cvtColor(right, cv2.COLOR_RGB2BGR) 

        grid = cvzone.stackImages(
                [self.front_image, self.back_image,self.left_image, self.right_image]
                ,2,0.5)
        cv2.imshow("front_back_left_right", grid)
        cv2.waitKey(1)
