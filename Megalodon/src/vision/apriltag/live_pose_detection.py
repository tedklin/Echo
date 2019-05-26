#!/usr/bin/env python

'''Demonstrate Python wrapper of C apriltag library by running on camera frames.'''
from __future__ import division
from __future__ import print_function

from argparse import ArgumentParser
import cv2
import apriltag

# for some reason pylint complains about members being undefined :(
# pylint: disable=E1101


# --------------
# fx = 
# fy = 
# cx = 
# cy = 
# 
# RUN <python live_pose_detection.py -c -k '(1050.7194990676126, 1052.4153923425188, 540.00000000284456, 359.99999998688185)' -s .127>
# --------------

def main():

    '''Main function.'''

    parser = ArgumentParser(
        description='test apriltag Python bindings')

    # parser.add_argument('device_or_movie', metavar='INPUT', nargs='?', default=0,
    #                     help='Movie to load or integer ID of camera device')

    parser.add_argument('-k', '--camera-params', type=_camera_params,
                        default=None,
                        help='intrinsic parameters for camera (in the form fx,fy,cx,cy)')

    parser.add_argument('-s', '--tag-size', type=float,
                        default=1.0,
                        help='tag size in user-specified units (default=1.0)')

    add_arguments(parser)

    options = parser.parse_args()

    apriltag.add_arguments(parser)

    options = parser.parse_args()

    # try:
    #     cap = cv2.VideoCapture(int(options.device_or_movie))
    # except ValueError:
    #     cap = cv2.VideoCapture(options.device_or_movie)

    cap = cv2.VideoCapture(1)

    window = 'Camera'
    cv2.namedWindow(window)

    # set up a reasonable search path for the apriltag DLL inside the
    # github repo this file lives in;
    #
    # for "real" deployments, either install the DLL in the appropriate
    # system-wide library directory, or specify your own search paths
    # as needed.
    
    detector = apriltag.Detector(options,
                                 searchpath=apriltag._get_demo_searchpath())

    while True:

        success, frame = cap.read()
        if not success:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        detections, dimg = detector.detect(gray, return_image=True)

        num_detections = len(detections)
        print('Detected {} tags.\n'.format(num_detections))

        for i, detection in enumerate(detections):
            print( 'Detection {} of {}:'.format(i+1, num_detections))
            print()
            print(detection.tostring(indent=2))

            #if options.camera_params is not None:
                
            pose, e0, e1 = detector.detection_pose(detection,
                                              options.camera_params,
                                              options.tag_size)

            if _HAVE_CV2:
                _draw_pose(overlay,
                           options.camera_params,
                           options.tag_size,
                           pose)
            
            print(detection.tostring(
                collections.OrderedDict([('Pose',pose),
                                         ('InitError', e0),
                                         ('FinalError', e1)]),
                indent=2))
                
            print()

        overlay = frame // 2 + dimg[:, :, None] // 2

        cv2.imshow(window, overlay)
        k = cv2.waitKey(1)

        if k == 27:
            break

if __name__ == '__main__':
    main()
