"""
File containing methods to get the centers of buttons/plugs for maintenance task

"""

import numpy as np 



def button_centers_0():
    HORIZONTAL_SPACING = 62
    return (np.array([-HORIZONTAL_SPACING, 0., 0.]) / 10).astype(np.float64)


def button_centers_1():
    HORIZONTAL_SPACING = 62
    return (np.array([-HORIZONTAL_SPACING, 0., 0.]) / 10).astype(np.float64)


def button_centers_2():
    HORIZONTAL_SPACING = 85 # in mm 
    VERTICAL_SPACING = 71 # in mm 

    BUTTON_LAYOUT = (2,3)

    button_centers = np.zeros((BUTTON_LAYOUT[0], BUTTON_LAYOUT[1], 3))

    for i in range(BUTTON_LAYOUT[0]):
        for j in range(BUTTON_LAYOUT[1]):
            button_centers[i,j] = np.array([i*HORIZONTAL_SPACING, -j*VERTICAL_SPACING, 0])

    button_centers = button_centers.reshape(-1, *button_centers.shape[-1:])
    print(button_centers.shape)
    return (button_centers / 10).astype(np.float64)

def button_centers_3():
    VERTICAL_SPACING = 71
    return (np.array([-0, VERTICAL_SPACING, 0.]) / 10).astype(np.float64)