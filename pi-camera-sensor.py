import diplib as dip
import numpy as np
import matplotlib.pyplot as pp
import cv2
import picamera
import picamera.array
import time
import os

camera = picamera.PiCamera()
camera.resolution = (800, 1952)
#camera.rotation = 180
camera.zoom = (0.25, 0.3, 0.1, 0.2)
time.sleep(5) # let camera warm up and adjust
# set focus
focalValue = 60
os.system("raspistill -t 1")
os.system("i2cset -y 0 0x0c %d %d" % (focalValue,0))

loops = 1
lastDistance = 0
deviation = 0
deviationTotal = 0

while True:
    
    with picamera.array.PiRGBArray(camera) as stream:
        camera.capture(stream, format='bgr') # capture the image in bgr format
        img = stream.array # create a cv2 object (numpy array) from the image captured
        
    # grayscale image
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    #convert image to dip object
    img = dip.Image(img)
    
    calibration_pixels = 801.2423788037772
    pixels_per_mm = 1.984375/calibration_pixels # using 5/64 drill bit as calibration rod
    
    img.SetPixelSize(pixels_per_mm, "mm")

    # find edges
    edges = dip.GradientMagnitude(img)
    
    # binarize
    mask = dip.Threshold(edges)[1]
    mask = dip.Threshold(edges)[0]
    mask = dip.Dilation(mask, 9)  # we want the mask to include the "tails" of the Gaussian
    mask = dip.AreaOpening(mask, filterSize=1000)  # remove small regions

    # measure the two edges
    mask = dip.Label(mask)
    msr = dip.MeasurementTool.Measure(mask, edges, ['Gravity','GreyMajorAxes'])
    # msr[n] is the measurements for object with ID n, if we have two objects, n can be 1 or 2.

    # get distance between edges
    center1 = np.array(msr[1]['Gravity'])
    center2 = np.array(msr[2]['Gravity'])
    print("center1 = ", center1)
    print("center2 = ", center2)
    

    normal1 = np.array(msr[1]['GreyMajorAxes'])[0:2]  # first axis is perpendicular to edge
    normal2 = np.array(msr[2]['GreyMajorAxes'])[0:2]
    normal = (normal1 + normal2) / 2  # we average the two normals, assuming the edges are parallel
    
    print("normal1 full = ", np.array(msr[1]['GreyMajorAxes']))
    
    print("normal2 full = ", np.array(msr[2]['GreyMajorAxes']))
    
    print("normal1 = ", normal1)
    print("normal2 = ", normal2)
    
    distance = abs((center1 - center2) @ normal) # cross product of center1 - center 2 and normal
    units = msr['Gravity'].Values()[0].units
    print("Distance between lines:", distance, units)
    if loops >> 1:
        deviation = (abs(distance - lastDistance) + deviationTotal)/loops
        deviationTotal = deviation + deviationTotal
        print("Average Deviation: ", deviation)
    lastDistance = distance
    loops = loops + 1
    mmpp = img.PixelSize()[0].magnitude
    center1 = center1 / mmpp  # position in pixels
    center2 = center2 / mmpp
    L = 1000
    v = L * np.array([normal[1], -normal[0]])

    pt1 = center1 - v
    pt2 = center1 + v
    pp.plot([pt1[0], pt2[0]], [pt1[1], pt2[1]], 'r--')
    pt1 = center2 - v
    pt2 = center2 + v
    pp.plot([pt1[0], pt2[0]], [pt1[1], pt2[1]], 'r--')
    img.Show()
