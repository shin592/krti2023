
import cv2 as cv
import numpy as np

cap = cv.VideoCapture(0)

while True:
    _,frame = cap.read()
    cv.imshow('frame',frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        cv.destroyAllWindows()
        break