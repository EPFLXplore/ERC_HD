## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#####################################################
## librealsense tutorial #1 - Accessing depth data ##
#####################################################

# First import the library
import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt


try:
    width = 640 # 1280
    height = 480 # 720

    # Create a context object. This object owns the handles to all connected realsense devices
    pipeline = rs.pipeline()

    # Configure streams
    config = rs.config()
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)

    # Start streaming
    pipeline.start(config)
    cm = plt.get_cmap('gist_rainbow')

    while True:
        # This call waits until a new coherent set of frames is available on a device
        # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        if not depth: continue

        # Print a simple text-based representation of the image, by breaking it into 10x20 pixel regions and approximating the coverage of pixels within one meter
        # cv2.imshow('Lines', coverage)
        # coverage = [0]*64
        # for y in range(480):
        #     for x in range(width):
        #         dist = depth.get_distance(x, y)
        #         if 0 < dist and dist < 1:
        #             coverage[x//10] += 1

        #     if y%20 is 19:
        #         line = ""
        #         for c in coverage:
        #             line += " .:nhBXWW"[c//25]
        #         coverage = [0]*64
        #         print(line)

        # img = np.zeros((width,height))
        # for y in range(height):
        #     for x in range(width):
        #         dist = depth.get_distance(x, y)
        #         img[x,y] = dist


        # img = np.ones((height, width))
        # for x in range(height):
        #     for y in range(width):
        #         dist = depth.get_distance(y,x)
        #         img[x,y] = dist
        print(type(depth))
        print(dir(depth))
        print(depth.shape)
        img = depth    


        img *= 100  # convert from meters to cm

        # operating depth range of D405 camera is between 7 and 50 cm
        img[img > 50] = 50 
        img[img < 7] = 7

        max_img = img.max()

        img = - (img * 255 /max_img ) + 255

        # print(dir(depth.get_data()))

        # Apply the colormap like a function to any array:
        # print(img)
        # img = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)
        # printBufData(img.shape)
        # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.applyColorMap(img.astype(np.uint8), cv2.COLORMAP_JET)
        # img = np.ones((1000,500))
        # img = np.array(img * 255).astype('uint8')
        # print(img.mean())
        # img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


        # cv2.imshow('depth', depth.as_video_frame() )

        cv2.imshow('depth', np.array(img, dtype = np.uint8 ) )
        c = cv2.waitKey(1)
        if c == 27:
            break
    exit(0)
#except rs.error as e:
#    # Method calls agaisnt librealsense objects may throw exceptions of type pylibrs.error
#    print("pylibrs.error was thrown when calling %s(%s):\n", % (e.get_failed_function(), e.get_failed_args()))
#    print("    %s\n", e.what())
#    exit(1)
except Exception as e:
    print(e)
    pass