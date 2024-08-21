"""
Perception FSM design considerations:

service to select what is detected


depending on the task publishes each with a header with a timestamp for synchronization.
- object pose
- gripper pose: 2 points
- aruco tag pose 


# contains a list of pipelines that can be active or not
## Pipelines:
- Aruco tag: continously detects and publishes poses of all aruco tags
- object pose : sngle object if seen
    - metal bar
    - probe
    - ethernet cable

Currently each object has its own model. ( maybe later there will be 1 model for all objects) 


# Postprocessing
- measure stone size
- ICP (mask, depth, cad) --> pose
- combine aruco + model for ethernet socket

# filtering / tracking
- Moving average or similar 
- Handle outliers

"""


class PerceptionFSM:

    def __init__(self):
        raise NotImplementedError(
            f"Constructor not implemented for {self.__class__.__name__}"
        )
