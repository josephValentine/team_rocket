def GetHighestX(filt_image):

    #pass in filtered image (needs to be truly filtered down to the colored strip)

    #iterate through the image from right to left until you reach a non black pixel

    #return the x value of that pixel

def GetHighestY(filt_image):

    #pass in filtered image (needs to be truly filtered down to the colored strip)

    #iterate through the image from top to bottom until you reach a non black pixel

    #return the y value of that pixel

def GetLowestX(filt_image):

    #pass in filtered image (needs to be truly filtered down to the colored strip)

    #iterate through the image from left to right until you reach a non black pixel

    #return the x value of that pixel

def GetLowestY(filt_image):

    #pass in filtered image (needs to be truly filtered down to the colored strip)

    #iterate through the image from bottom to top until you reach a non black pixel

    #return the y value of that pixel

def CalcRobotCoords(filt_image):
    x = (GetHighestX(filt_image) + GetLowestX(filt_image))/2
    y = (GetHighestY(filt_image) + GetLowestY(filt_image))/2

    #return x y array

def CalcRobotAngle(filt_image_primary, filt_image_secondary):

    coords_primary = CalcRobotCoords(filt_image_primary)
    coords_secondary = CalcRobotCoords(filt_image_secondary)

    #do some fancy trig to get the angle of the robot

    #angle = arctan((coords_secondary[1]-coords_primary[1])/(coords_secondary[0]-coords_primary[0]))

    #return the angle, 0 - 359
