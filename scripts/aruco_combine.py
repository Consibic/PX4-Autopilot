#!/usr/bin/env python2

"""Python class with methods to generate, identify and interpret ArUco markers"""

# Import needed libraries
import cv2
import numpy as np
import math
# from google.colab.patches import cv2_imshow

      
"""method to generate an image with an ArUco marker based on the given integer ID"""
def generate_marker(marker_id):
    # set the ArUco dictionary as 6x6 (250)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250) 
    # set the pixel length & width of the marker 
    dim = 600
    # set the thickness of the white border in pixels  
    border_px = int(dim/10.)
    # the marker size is the major dimensions minus the border thickness
    marker_sz = dim - 2*border_px

    # declare blank image to store the aruco tag 
    tag = np.ones((marker_sz, marker_sz, 1), dtype="uint8")

    # draw the ArUco marker for the given ID on the blank image   
    cv2.aruco.drawMarker(aruco_dict, marker_id, marker_sz, tag, 1)
    # add white border to the marker for easier recognition
    tag = cv2.copyMakeBorder(tag, border_px, border_px, border_px, border_px, cv2.BORDER_CONSTANT, value=(255)) 

    # plot the generated ArUco tag and move it to (400,300) so it does not get cut off
    winname = "ArUco Tag"
    # cv2_imshow(tag)

    return tag


"""
Generate four markers and organize them in a frame
Inputs:
start_id: starting id for markers
code_size: size of each marker
Outputs:
image_frame: image containing four different markers
"""
def generate_four_markers(start_id, code_size):
    code_width = code_size[0]  # width of each marker
    # size of the final image containing four markers
    frame_size = (int(code_width * 2), int(code_width * 2))
    image_frame = np.full(frame_size, 255)  # initialize final image as white
    # add for markers to the frame
    for i in range(4):
        curr_id = start_id + i
        marker = generate_marker(curr_id)  # generate marker
        marker = cv2.resize(marker, code_size)  # resize to 1/4 of the frame
        # marker = cv2.rotate(marker, cv2.ROTATE_90_COUNTERCLOCKWISE)
        marker = cv2.rotate(marker, cv2.ROTATE_90_CLOCKWISE)
        image_frame[int(np.floor(i / 2) * code_width) : int(np.floor(i / 2) * code_width + code_width), 
                    int(i % 2) * code_width : int(i % 2 * code_width + code_width)] = marker
    # cv2_imshow(image_frame)
    return image_frame


"""
Compute location for markers for rearranging to the Blender unwrap format
Inputs:
size: size of final image
Outputs:
code_loc: list of location of each marker in the final image
code_size: expected size of each marker: (1/16) of the input size
"""
def compute_loc(size):
    code_loc = []  # init list for storing the locations
    # width of each marker
    code_width = int(size / 4)
    code_size = (code_width, code_width)
    centr_line = int(size / 2)  # central line of the final image
    # for four blocks at the center
    for i in range(4):
        h = int(i * code_width)
        w = int(centr_line - code_width / 2)
        sub_img = [(h, w), (h + code_width, w + code_width)]
        code_loc.append(sub_img)
    # for two blocks located at the two sides
    h_side = code_width
    w_left = int(centr_line - code_width * 3 / 2)
    w_right = int(centr_line + code_width / 2)
    sub_img_left = [(h_side, w_left), (h_side + code_width, w_left + code_width)]
    code_loc.append(sub_img_left)
    sub_img_right = [(h_side, w_right), (h_side + code_width, w_right + code_width)]
    code_loc.append(sub_img_right)
    return code_loc, code_size


"""
Combine the images in the format of Blender unwrapping design
Inputs:
- one_side: if only apply marker to one side of the payload (boolean)
- size: size of the output image
- target_path: target path for storing the image
"""
def combine(one_side, size, target_path):
    # totally 1 marker if apply marker to one side, otherwise 6 markers
    num_imgs = 1 if one_side else 6 
    locs, csize = compute_loc(size)  # compute locations for each marker
    image_frame = np.full((size, size), 255)
    img_list = []  # list for storing marker matrices
    # generate markers  
    for i in range(num_imgs):
        img = generate_four_markers(i * 4, csize)
        img_list.append(img)
    # assign markers to corresponding locations in the output image based on 
    # the computed locations
    for i, img_mat in enumerate(img_list):
        if one_side:
            c = 0  # place the image at the top of the payload if only apply marker to one side
        else:
            c = i  # place the image at the corresponding location
        img_mat = img_mat.astype('float32')
        img_resized = cv2.resize(img_mat, csize)
        image_frame[locs[c][0][0]:locs[c][1][0], locs[c][0][1]:locs[c][1][1]] = img_resized
    cv2.imwrite(target_path, image_frame)
    # cv2_imshow(image_frame)


"""
Create a new .dae file using the new group of aruco marks
Inputs:
- orig: full path to the default dae_struct.dae file
- target: full path to the targeting dae file
- image_name: name of the generated image containing aruco marks in the format of unwrapped design
"""
def create_dae(orig, target, image_name):
    f_orig = open(orig)
    f_target = open(target, 'w')
    for line in f_orig:
        if '.png</init_from>' in line:  # replace the component defining the file of the texture image
            new_line = '      <init_from>{0}</init_from>\n'.format(image_name)
            f_target.write(new_line)
        else:
            f_target.write(line)


def test(target_path): # create image
    size = 1500
    combine(False, size, '{0}.png'.format(target_path))
    combine(True, size, '{0}_single_side.png'.format(target_path))


if __name__ == '__main__':
    test('aruco_combined')  # create image
    # create_dae('dae_struct.dae', 'new.dae', 'test.png')  # create dae file based on generated image
