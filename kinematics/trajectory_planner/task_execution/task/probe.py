from __future__ import annotations
from .task import *
from .tool_pickup import *
from typing import Optional


class GrabOptions:
    TOP = 0
    SIDE = 1


class ProbeStore(Task):
    NUM_STATIONS = 3
    STATIONS_OCCUPIED = [False]*NUM_STATIONS
    
    def __init__(self, executor: Executor, grab_option: int, station: Optional[int] = None):
        if station is None:
            if all(ProbeStore.STATIONS_OCCUPIED):
                raise IndexError("no available stations")
            station = ProbeStore.STATIONS_OCCUPIED.index(False)
        self.station = station
        self.grab_option = grab_option
        
        self.above_distance = 0.25
        self.descend_distance = 0.08
        
        super().__init__(executor)
    
    def station_pose(self) -> qan.Pose:
        offset = qan.Point(x=-0.008, y=0.008)
        station_poses = [
            qan.Pose(position=qan.Point(x=-0.525 - 0.1, y=0.025, z=0.2) + offset),
            qan.Pose(position=qan.Point(x=-0.525 - 0.05, y=0.025, z=0.2) + offset),
            qan.Pose(position=qan.Point(x=-0.525, y=0.025, z=0.2) + offset),
        ]
        return station_poses[self.station]
    
    def sideGrabAboveStationPose(self) -> qan.Pose:
        eef_pose = qan.Pose()
        probe_position = self.station_pose().point_image([0, 0, self.above_distance])
        eef_pose.orientation = qan.Quaternion.from_axis_angle([0,  0, 1], math.pi/2) * qan.Quaternion.from_axis_angle([1, 0, 0], -math.pi/2)
        probe_to_eef_distance = 0.015
        eef_pose.position = probe_position + eef_pose.orientation.point_image([0, 0, probe_to_eef_distance])
        return eef_pose
    
    def topGrabAboveStationPose(self) -> qan.Pose:
        eef_pose = qan.Pose()
        # eef_pose.orientation = qan.Quaternion.from_axis_angle([0, 1, 0], math.pi) * qan.Quaternion.from_axis_angle([0, 0, 1], math.pi)
        eef_pose.orientation = qan.Quaternion.from_axis_angle([1, 0, 0], math.pi) * qan.Quaternion.from_axis_angle([0, 0, 1], -math.pi/2)
        eef_pose.position = self.station_pose().point_image([0, 0, self.above_distance])
        return eef_pose
    
    def aboveStationPose(self) -> qan.Pose:
        if self.grab_option == GrabOptions.SIDE:
            return self.sideGrabAboveStationPose()
        elif self.grab_option == GrabOptions.TOP:
            return self.topGrabAboveStationPose()
        raise        
    
    def constructCommandChain(self):
        super().constructCommandChain()
        
        probe_to_eef_distance = 0.015
        p = qan.Pose(position=qan.Point(z=pc.fingers.transform_to_eef.position.z - probe_to_eef_distance))
        shape = [0.03, 0.03, 0.3] if self.grab_option == GrabOptions.TOP else [0.03, 0.3, 0.03]
        self.addCommand(
            AttachObjectCommand(pose=pc.GRIPPER_TRANSFORM_CORRECTION @ p, shape=shape, operation=Object.ADD, name="probe")
        )
        
        # self.addCommand(        # TODO: maybe disable collisions for this object
        #     AddObjectCommand(pose=self.station_pose(), shape=[0.05, 0.05, 0.001], name=f"station {self.station}"),
        #     description=f"add station to world"
        # )
        
        # self.addCommand(
        #     PoseCommand(pose=self.aboveStationPose()),
        #     description = "go above station"
        # )
        target = ("side" if self.grab_option == GrabOptions.SIDE else "top") + str(self.station)
        self.addCommand(
            NamedJointTargetCommand(target),
            description="go above station"
        )

        self.addCommand(
            StraightMoveCommand(velocity_scaling_factor=0.2, distance=self.descend_distance),
            pre_operation = lambda cmd: cmd.setAxisFromOrientation(qan.Quaternion(), reverse=True),
            description = "Descend towards station"
        )
        
        # self.constructOpenGripperCommands(low_torque_duration=5)
        
        # self.addCommand(
        #     AttachObjectCommand( operation=Object.REMOVE, name="probe")
        # )
        
        # self.addCommand(
        #     StraightMoveCommand(velocity_scaling_factor=0.2, distance=self.descend_distance),
        #     pre_operation = lambda cmd: cmd.setAxisFromOrientation(self.station_pose().orientation),
        #     description = "Descend towards station"
        # )
        
        # self.constructCloseGripperCommands(low_torque_duration=3)
        