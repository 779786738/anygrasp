import cv2
import numpy as np
from PIL import Image

col,row = 400,400
bias = [0,-50]
sizeOfImage = [1280,720]

img = np.zeros((sizeOfImage[1],sizeOfImage[0],3),np.uint8)

ptLeftTop = (int(sizeOfImage[0]/2+bias[0]-col/2),int(sizeOfImage[1]/2+bias[1]-row/2))
ptRB = (ptLeftTop[0]+col,ptLeftTop[1]+row)

p_color = (255,255,255)
thichness = -1 # - fill
cv2.rectangle(img,ptLeftTop,ptRB,p_color,thichness)

gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
ret,im = cv2.threshold(gray,125,255,cv2.THRESH_BINARY)

bi = im.astype(np.bool_)
bi = Image.fromarray(bi)
bi.save("workspace_mask.png")