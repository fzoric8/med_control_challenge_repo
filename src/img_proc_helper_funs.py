import cv2

SAVE_DIR = "path_in_which_we_want_to_save_image "

# image processing
image_np = cv2.imdecode(self.image_array, cv2.COLOR_BGR2GRAY)

# compressing image ((0,0) -> origin of photo from which we resize, fx,fy-> scaling factor)
smaller_img = cv2.resize(image_np, (0, 0), fx=0.5, fy=0.5)
smaller_img = cv2.medianBlur(smaller_img, 5)

# detecting edges with Canny filter
edges = cv2.Canny(smaller_img, 130, 175, apertureSize=3)
edges_not = cv2.bitwise_not(edges)

# changing RGB to BW
gray = cv2.cvtColor(smaller_img, cv2.COLOR_BGR2GRAY)

# Adding median blur to photo
gray = cv2.medianBlur(gray, 5)

# Using probabilistic HouhgLinesP to detect windmill edges (retuns list of (x0,x1,y0,y1) points
# Houghes tutorial -̣> https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_houghlines/py_houghlines.html
minLineLength = 100
minLineGap = 1
lines2 = cv2.HoughLinesP(edges, 1, np.pi/360, minLineLength, minLineGap)

# Applying threshold to images -> https://docs.opencv.org/3.4.0/d7/d4d/tutorial_py_thresholding.html
th = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

# function draw HoughImageP draws found lines on existing image
gray = draw_HoughImageP(lines2, gray)

# HOUGH CIRCLES -> detect circles on image
rows = gray.shape[0]
circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows/8,
                           param1=100, param2=30,
                           minRadius=1, maxRadius=50)

# CORNER DETECTION -> Shi-Tomasi instead of Harris corner detection
corners = cv2.goodFeaturesToTrack(edges, 5, 0.01, 10)
corners = np.int0(corners)
gray = drawCorners(corners, gray, 10)

# save image on local drive
cv2.imwrite("{0}/photo_{1}.png".format(SAVE_DIR, counter), gray)
cv2.imwrite("{0}/canny_{1}.png".format(SAVE_DIR, counter), edges_not)

def drawCorners(corners, img, num_corners):
    """
    
    :param corners: argument gotten from Shi-Tomasi or Harris corner detection
    :param img: image on which we want to draw those corner
    :param num_corners: number of corners we want to draw
    :return: 
    """
    print("Draw corner")
    for i in corners:
        x, y = i.ravel()
        cv2.circle(img, (x, y), num_corners, 255, 2)
    return(img)

def drawHoughCircle(circles, img):
    """
    
    :param circles: argument gotten from Houghes Circle detection
    :param img: image on which we want to draw those circles 
    :return: 
    """
    print(circles)
    if circles is not None:
        circles = np.uint(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            # circle center
            cv2.circle(img, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv2.circle(img, radius, (255, 0, 255), 3)
    return(img)

def draw_HoughImage(lines, img):
    """
    
    :param lines: lines gotten from HoughLines function 
    :param img: image on which we want to draw those lines
    :return: 
    """
    for element in range(len(lines)):
        for rho, theta in lines[element]:
            print(rho)
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 250*(-b))
            y1 = int(y0 + 250*(a))
            x2 = int(x0 - 250*(-b))
            y2 = int(y0 - 250*(a))

        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 1)
    return(img)

def draw_HoughImageP(lines, img):
    """
    
    :param lines: lines gotten from probabilistic HoughLines function 
    :param img: image on which we want to draw those lines 
    :return: 
    """
    for element in range(len(lines)):
        for x1, y1, x2, y2  in lines[element]:
            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), 2)
    return(img)