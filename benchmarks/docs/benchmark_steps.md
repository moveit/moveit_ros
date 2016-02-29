# How to do MoveIt! Benchmarking using ROS Hydro

The documentation for hydro is not available and the wiki page for the same was taken down a long ago.

Follow this to do the benchmarking

Open three terminals

Start with:
```
roslaunch pr2_moveit_config warehouse.launch moveit_warehouse_database_path:=<path to your DB>
```
```
roslaunch pr2_moveit_config planning_context.launch load_robot_description:=true
```
```
roslaunch pr2_moveit_config demo.launch
```

Then: connect to the database by pressing "connect". Now you will be connected to your database. We could also check if the OMPL is loaded.

![ScreenShot](/images/database_connect.png)

* Go to "Scene Objects" tab and load the environment(.scene file or collada model). From "Import From Text" option we can use .scene files

![ScreenShot](/images/import_environment.png)

* Then go to "Planning" tab and select a start start and goal state.
* Now to "Stored States" tab and save the selected start and goal

![ScreenShot](/images/start_goal.png)

* To store the selected scene/environment and query to our database, switch to "Stored Scenes" tab and save the scene. Then the save the query under the saved scene

![ScreenShot](/images/save_scene_query.png)

* Create a config file

Sample .scene files for pr2 is available here https://github.com/isucan/plannerarena

### Sample config file

```sh
[scene]
name=Industrial_benchmark
runs=10
timeout=5
output=<path>
start=pr2_start
query=Motion Plan Request Industrial
goal=pr2_goal

[plugin]
name=ompl_interface/OMPLPlanner
planners=right_arm[BKPIECEkConfigDefault]
```

save the above lines into a file of **.cfg** format

**demo.launch can be closed now**



#### Now, launch the following benchmarking launch file

```sh
roslaunch pr2_moveit_config run_benchmark_ompl.launch cfg:=config_file.cfg
```
#### To generate .CSV file out of the .log file
```rosrun moveit_ros_benchmarks moveit_benchmark_statistics.py file_name.log -c file_name.csv```

#### To generate PDF having box plots
```rosrun moveit_ros_benchmarks moveit_benchmark_statistics.py file_name.log -p file_name.pdf```

