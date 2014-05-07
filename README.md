# pr2_pbd
Programming by Demonstration (PbD) for the PR2.

This is the branch for development to Hydro & Catkin. Please see the [issues page](https://github.com/vovakkk/pr2_pbd/issues) for the TODOs, bugs, and features.

## Running

### Commands on robot (`c1`)
TODO: put these in a launch file
```bash
# terminal 1 [launch robot under hydro]
robot stop
robot claim
roslaunch /etc/ros/hydro/robot.launch # instead of robot start (TODO: this matters?)

# terminal 2 [start moveit]
roslaunch pr2_moveit_config move_group.launch

# terminal 3 [start PbD]
roslaunch pr2_pbd_interaction interaction_node.launch
```

### Commands on desktop
TODO: put these in a launch file
```bash
# terminal 1 [run rosbridge]
roslaunch rosbridge_server rosbridge_websocket.launch

# terminal 2 [run HTTP server]
cd ~/catkin_ws/src/pr2_pbd/pr2_pbd_http; python -m SimpleHTTPServer # runs on 8000

# terminal 3 [run social gaze]
realrobot; roslaunch pr2_social_gaze gaze.launch # (TODO: can this be on Rosie instead?)

# terminal 4 [open a GUI]
google-chrome localhost:8000 # for HTML GUI
rosrun pr2_pbd_gui pr2_pbd_gui # for RQT GUI
```

## Python resources

### Style guides
- [pep8](http://legacy.python.org/dev/peps/pep-0008/) (main style guide)
- [docstrings](http://sphinxcontrib-napoleon.readthedocs.org/en/latest/example_google.html) (note that we use single quotes (') instead of double ("))

### Linting tools
- [sublime linting framework](https://github.com/SublimeLinter/SublimeLinter3), [pylint](https://sublime.wbond.net/packages/SublimeLinter-pylint), [pep8](https://github.com/SublimeLinter/SublimeLinter-pep8) (for linting in sublime)

### Keywords not to use
```python
# Don't use builtin functions; this shows them all.
dir(__builtins__)

# Don't use standard keywords; this shows them all.
import keyword
keyword.kwlist
```
