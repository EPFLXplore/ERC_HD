# Task execution API

The central object is the Task. A task contains a list of commands that are executed iteratively.

## Command

Source code can be found in the [command.py](https://github.com/EPFLXplore/ERC_HD/blob/matthias-humble/hd_ws/src/kinematics/trajectory_planner/task_execution/command/command.py) file.  
A `Command` should represent a single "atomic" action to be performed, such as move to a certain pose, open the gripper, but also ask (and wait) for detection.
The `Command` interface contains the following important methods :

### Important methods of the `Command` interface

`Command.execute(self)`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Execute the command, the most import method.  

`Command.done(self)`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Indicates if the command has been executed successfuly and the task can move on to the next command.  

`Command.abort(self)`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Stop the command and all the movement involved as soon as possible.  

`Command.hasFailed(self)`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Indicates whether the command has failed completely and cannot be reatempted.


To create a new command the `Command` class has to be inherited. The `execute` method needs to be overriden (note that it has to contain a call to `super().execute()`).  
The `done` method can be overriden, it returns by default True after one call of the `execute` method.  
The `abort` method can (should) be overriden, it does nothing by default.  

Currently available commands are (note that some are specific to the 2023 arm) :
* [`AddObjectCommand`]() : add / modify / remove an object in the MoveIt world
* [`GripperCommand`]() : apply torque to the gripper for a certain duration
* [`JointSpaceCommand`]() : reach a joint space goal for the arm
* [`NamedJointTargetCommand`]() : reach a named predefined pose for the arm, such as *home*, *zero*, *optimal_view*
* [`PoseCommand`]() : reach a 3d space pose (position + orientation) with the arm
* [`RassorCommand`]() : apply torque to the gripper for a certain duration
* [`RequestDetectionCommand`]() : wait until a new vision detection of the required object has been obtained
* [`StraightMoveCommand`]() : move the arm's end [effector] by a given distance along a given axis
* [`VoltmeterCommand`]() : extend or retract the voltmeter


## BackgroundCommand

Source code can be found in the [command.py](https://github.com/EPFLXplore/ERC_HD/blob/matthias-humble/hd_ws/src/kinematics/trajectory_planner/task_execution/command/command.py) file.  
TODO


## Task

Source code can be found in the [task.py](https://github.com/EPFLXplore/ERC_HD/blob/matthias-humble/hd_ws/src/kinematics/trajectory_planner/task_execution/task/task.py) file.  
In its simplest form, a `Task` is a list of `Command` objects to be executed iteratively. It is also possible to execute commands concurrently using `BackgroundCommand`.  
The execution of every command is preceded by an optional pre-operation taking the command as input, and followed by an optional post-operation taking the command as input. Every command is also associated with a description which is printed at the beginning of execution.

### Important methods of the `Task` interface
  
`Task.constructCommandChain(self)`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Method called in the `__init__` to construct the list of commands.  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; All commands should be added in this function (or a function called by this function), via a call to `Task.addCommand`.  

`Task.addCommand(self, command, pre_operation=None, post_operation=None, description="")`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Only way to add a `Command` to the command chain. This only adds the command to the list and doesn't execute it.  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; The execution order of the commands will be determined by the order in which they are added via this method.  

`Task.execute(self)`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Try to execute all commands in the command chain.  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Returns a boolean indicating if execution was successful.  

`Task.abort(self)`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Stop as soon as possible all action of a task currently being executed.  

`Task.finished(self)`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Return a boolean indicating whether all commands have already been executed.

`Task.declareBackgroundCommand(self, id, command, description="")`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; TODO

`Task.setBackgroundCommandStartPoint(self, id, pre_operation=None, post_operation=None, description="")`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; TODO

`Task.setBackgroundCommandStopPoint(self, id, pre_operation=None, post_operation=None, description="")`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; TODO


To create a new task the `Task` class needs to be inherited. The only method that needs to be overriden is `constructCommandChain`.  
This allows for concise definition of custom tasks where all the execution logic is abstracted in the base `Task` class.

### Some useful methods for writing custom tasks

`Task.scanForObjects(self)`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; TODO  

`Task.getScanPosition(self)`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; TODO  

`Task.getScanOrientation(self)`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; TODO  

`Task.getObjectPose(self)`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; TODO  

`Task.getARTagPose(self)`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; TODO  

`Task.constructStandardDetectionCommands(self, object_name="object", object_box=(0.2, 0.1, 0.0001), extended=True)`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; TODO  

`Task.constructRemoveObjectsCommands(self, object_name="object", extended=True)`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; TODO  

`Task.addControlPanelCommand(self)`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; TODO  

`Task.addObjectAxisCommand(self, object_name="object)`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; TODO  

`Task.constructOpenGripperCommands(self, high_torque=1.0, low_torque=0.1, post_completion_wait=0.0)`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; TODO  

`Task.constructCloseGripperCommands(self, high_torque=1.0, low_torque=0.1, post_completion_wait=0.0)`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; TODO  



Here is an example of how a command can be added using the `Task.addCommand` method.


```python 
self.addCommand(
    PoseCommand(self.executor),
    pre_operation = lambda cmd: cmd.setPose(position=self.getPressPosition(),
                                            orientation=self.getPressOrientation()),
    description = "go in front of button"
)
```
> The added command is a `PoseCommand` which requires a pose to reach. This pose is not known in advance, so this is where the pre-operation comes in handy. `PoseCommand` provides a setter `setPose` which is used here in the lambda expression.  
> While the parameters pre_operation and post_operation can be regular python functions, it is often convenient to use lambda expressions instead.
> > Note that lambda expressions are evaluated lazily so that the calls `self.getPressPosition()` and `self.getPressOrientation()` will only be made when the pre-operation is called, that is right before executing the command, at which point the desired position and orientation will be computable.
> 
> > Lambda expressions don't allow for assignment but one can get around this issue by using setters as in this example.
> 
> > Lambda expressions don't allow for multiline computations but one can get around this issue by using tuples as follows : `lambda ...: (line1, line2, ..., lineK)`. When called, the whole tuple will be returned, and thus evaluated. It also possible to return a specific value as follows : `lambda ...: (line1, line2, ..., lineK, return_value)[-1]` (in this case the whole tuple is still evaluated).
> 
> In this specific example no post-operation is performed.


Currently available tasks are (note that some are specific to the 2023 arm and some are not 100% functional) :
* [`PressButton`]()
* [`AlignPanel`]()
* [`BarMagnetApproach`]()
* [`EthernetApproach`]()
* [`PlugVoltmeterAlign`]()
* [`PlugVoltmeterApproach`]()
* [`RassorSampling`]()
* [`RockSamplingApproach`]()
* [`RockSamplingDrop`]()
* [`RockSamplingComplete`]()

