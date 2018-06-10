import numpy as np
import cv2
import time

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
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

def obstacle_thresh(img, rgb_thresh=(130,130,130)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    below_thresh = (img[:,:,0] < rgb_thresh[0]) \
                & (img[:,:,1] < rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[below_thresh] = 1
    # Return the binary image
    return color_select

def rock_thresh(img):
    # Convert RGB to HSV using openCV
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV, 3)
    
    #Define range of  yellow colors in HSV
    lower_yellow = np.array([20,150, 100], dtype='uint8')
    upper_yellow = np.array([50, 255, 255], dtype='uint8')
    
    # Threshhold the HSV image to get only yellow colors
    mask = cv2.inRange(hsv,lower_yellow, upper_yellow)
    return mask


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
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M,(img.shape[1], img.shape[0]))
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    image = Rover.img
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    warped= perspect_transform(image, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    navigable_terrain = color_thresh(warped, (160,160,160))
    obstacles = obstacle_thresh (warped, (130,130,130))
    rock_samples = rock_thresh(warped)
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacles * 255
    Rover.vision_image[:,:,1] = rock_samples * 255                 
    Rover.vision_image[:,:,2] = navigable_terrain * 255 
    
    # 5) Convert map image pixel values to rover-centric coords
    navigable_xpix,navigable_ypix = rover_coords(navigable_terrain)
    obstacles_xpix,obstacles_ypix = rover_coords(obstacles)
    rocks_xpix,rocks_ypix = rover_coords(rock_samples)      
    
    # 6) Convert rover-centric pixel values to world coordinates
    scale = dst_size *2
    xpos, ypos = Rover.pos
    yaw = Rover.yaw
    worldmap_size = Rover.worldmap.shape[0]
    
    navigable_x_world, navigable_y_world = pix_to_world(
            navigable_xpix, navigable_ypix, 
            xpos, ypos, yaw, worldmap_size, scale)
        
    obstacles_x_world, obstacles_y_world = pix_to_world(
            obstacles_xpix, obstacles_ypix, 
            xpos, ypos, yaw, worldmap_size, scale)
        
         
    rocks_x_world, rocks_y_world = pix_to_world(
            rocks_xpix, rocks_ypix, 
            xpos, ypos, yaw, worldmap_size, scale)
    
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    if(Rover.pitch <0.5 or Rover.pitch >359.5) and (Rover.roll <0.5 or Rover.roll>359.5):
        Rover.worldmap[obstacles_y_world, obstacles_x_world, 0] += 1
        Rover.worldmap[rocks_y_world, rocks_x_world, 1] =255
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
                                                    
    
        
    # 8) Convert rover-centric pixel positions to polar coordinates
    dists, angles = to_polar_coords(navigable_xpix, navigable_ypix)                        
                                         
                                                    
    # Update Rover pixel distances and angles
    Rover.nav_dists = dists
    Rover.nav_angles = angles
    
    if len(rocks_xpix) > 5:   
        # if rock is identified, make the rover to navigate it
        rock_dist, rock_angle = to_polar_coords(rocks_xpix, rocks_ypix)
        
        Rover.rock_dist = rock_dist
        Rover.rock_angle = rock_angle
        if not Rover.sample_seen:
            # first frame sample has been seen, thus start the sample timer
            Rover.sample_timer = time.time()
            Rover.sample_seen = True
                                                    
        if Rover.start_pos is None:
            Rover.start_pos = (Rover.pos[0], Rover.pos[1])
            Print('STARTING POSITION IS :', Rover.start_pos)
    return Rover