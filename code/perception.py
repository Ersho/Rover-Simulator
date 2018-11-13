import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(186, 180, 180)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def obstacle_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] < rgb_thresh[0]) \
                & (img[:,:,1] < rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def object_thresh1(img, upper_rgb = (255,255,0), lower_rgb = (86,80,22) ):
    
    """middle_thresh = ((img[:,:,0] < upper_rgb[0]) \
                & (img[:,:,1] < upper_rgb[1]) \
                & (img[:,:,2] < upper_rgb[2])) \
                & ((img[:,:,0] > lower_rgb[0]) \
                & (img[:,:,1] > lower_rgb[1]) \
                & (img[:,:,2] > lower_rgb[2]))
    """
    
    #color_select = np.zeros_like(img[:,:,0])
    color_select = cv2.inRange(img, lower_rgb, upper_rgb)
    #color_select[middle_thresh] = 1
    
    return color_select


def object_thresh(img, lower_rock=np.array([93, 173, 131]), upper_rock=np.array([98, 255, 182])):
    # Create an array of zeros same xy size as img, but single channel
    rock_select = np.zeros_like(img[:, :, 0])
    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Threshold the HSV image to get only yellow colors
    between_thresh = (((hsv[:, :, 0] > lower_rock[0]) & (hsv[:, :, 0] <= upper_rock[0])) &
                      ((hsv[:, :, 1] > lower_rock[1]) & (hsv[:, :, 1] <= upper_rock[1])) &
                      ((hsv[:, :, 2] > lower_rock[2]) & (hsv[:, :, 2] <= upper_rock[2])))
    # Index the array of zeros with the boolean array and set to 1
    rock_select[between_thresh] = 1
    # kernel of 5x5 size
    kernel = np.ones((5, 5), np.uint8)
    # Dilate the image to get a bigger and easier to locate target
    rock_select = cv2.dilate(rock_select, kernel, iterations=3)
    # Return the binary image
    return rock_select
    
    
# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    img = np.asarray(Rover.img)
    xpos, ypos = Rover.pos 
    yaw = Rover.yaw
    roll = Rover.roll
    pitch = Rover.pitch
    world_size = Rover.worldmap.shape[0]
    scale = 10
    # 1) Define source and destination points for perspective transform
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                      [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
    ])
    # 2) Apply perspective transform
    wrap = perspect_transform(img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    obstacles_thresh = obstacle_thresh(wrap)     
    navigable_thresh = color_thresh(wrap)
    rock_thresh = object_thresh(wrap)
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = obstacles_thresh * 255
    Rover.vision_image[:,:,1] = rock_thresh * 255
    Rover.vision_image[:,:,2] = navigable_thresh *255
    # 5) Convert map image pixel values to rover-centric coords
    obstacle_xpix, obstacle_ypix = rover_coords(obstacles_thresh)
    rock_xpix, rock_ypix = rover_coords(rock_thresh)
    navigable_xpix, navigable_ypix = rover_coords(navigable_thresh)
    # 6) Convert rover-centric pixel values to world coordinates
    obstacle_x_world, obstacle_y_world = pix_to_world(obstacle_xpix, obstacle_ypix, xpos, ypos, yaw, world_size, scale)
    rock_x_world, rock_y_world = pix_to_world(rock_xpix, rock_ypix, xpos, ypos, yaw, world_size, scale)
    navigable_x_world, navigable_y_world = pix_to_world(navigable_xpix, navigable_ypix, xpos, ypos, yaw, world_size, scale)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    Rover.worldmap[rock_y_world, rock_x_world, 1] += 11
    Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 10

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    Rover.rock_dists, Rover.rock_angles = to_polar_coords(rock_xpix,rock_ypix)
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(navigable_xpix,navigable_ypix)
    
    
    
    return Rover
