# How to use

You can run the 3 robots in simulation via these commands. **Do not run this package on live robots yet!**

Option A: Simple demo mode in MoveIt (best for checking simple robot instructions and if your path planning works).

    ```
    roslaunch o2as_moveit_config demo.launch
    ```

Option B: Run a Gazebo simulation with physics and sensors (cameras). This is best for simulating the full setup and camera views.

    ```
    roslaunch o2as_gazebo o2as_gazebo_3_controllers.launch
    roslaunch o2as_moveit_config o2as_moveit_planning_execution.launch sim:=true
    ```

Either option opens rviz. In rviz, you can:

- Select "all_bots" in the Displays box, under "Motion Planning/Planning Request/Planning Group". Then, click the tab "Planning", select "home_all" in the "Select Goal State" area, click "Update", then "Plan and Execute", and the robots will move to the default pose.
- By selecting a different Planning Group under "Motion Planning/Planning Request/Planning Group" in the Displays box, you can plan motions for different robots using the interactive marker. The robot will move in MoveIt and Gazebo when you execute the plan.

In rviz, you can also press Ctrl+O and load "show_tf_frames.rviz" in the o2as_moveit_config/launch folder to get a different view of the robots and frames.

# How to use fewer than 3 robots

Delete the other robots from the config files and scene definition.

TODO: Explain this in a little bit more detail.


# ToDo

- Fix "all_bots" complaining about not being a chain.
- Add controllers for the grippers
- Add the cameras
- Add all the scenes