# AR TAG based vision:

## Coordinate systems:

[Explained](https://towardsdatascience.com/what-are-intrinsic-and-extrinsic-camera-parameters-in-computer-vision-7071b72fb8ec#:~:text=The%20extrinsic%20matrix%20is%20a,to%20the%20pixel%20coordinate%20system.)

- **World coordinate system**:
  - 3D
  - The coordinates of the object with respect to an aruco tag
- **Camera coordinates system**:
  - 3D
  - The coordinates of the object with respect to the camera
- **Image coordinate system**:
  - 2D
  - The coordinates of the object with respect to the pixels of the camera

## CPO: Control Panel Object

A rectangular object on the control panel

- **corners**: 4 corners in order:
  - top_left
  - top_right
  - bottom_right
  - bottom_left
- **center_hw**: center, height, width
  - 1 point and 2 dimensions
- **top_left_hw**: top left, height, width
  - 1 point and 2 dimensions
