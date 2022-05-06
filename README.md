# rviz_adas_map_builder
The rviz plugin to modify ADAS map (AISAN format) with Rviz, written in c++, for human beings all over the world. Mainly for [Autoware](https://www.autoware.ai/).

## Features
* By using Rviz, you can add other tools.
* You can modify the HDMap with localizing and checking the actual position of each element. (Can't add element. That function is future work!)
* AISAN vector map builder which officially released is hard to modify the small error of position.
* Waypoints (csv) for Autoware.AI's waypoint follower is also supported.

![samnail](https://github.com/kuriatsu/rviz_hdmap_editor/blob/image/image/rviz_window.png)

## Requirement
* ubuntu
* ros
* QT5 (basic packages, initially installed with ros)


## Install
1. create catkin_ws
```bash
cd /path/to/anydir
mkdir -p catkin_ws/src
cd /catkin_ws/src
catkin_init_workspace
```
2. download and build package
```bash
cd /catkin_ws/src
git clone https://github.com/kuriatsu/rviz_hdmap_editor.git rviz_hdmap_editor
cd ../
catkin_make
```

3. Download example (optional)
Download from [here](https://autoware-ai.s3.us-east-2.amazonaws.com/sample_moriyama_data.tar.gz)

## Usage
1. start rviz
```bash
source /catkin_ws/devel/setup.bash
roscore &
rviz
```

2. Load HDmap and pointclond map and publish /map tf or without them like below
```bash
rosrun tf static_transform_publisher 0 0 0 0 0 0 /map /tf 100
```

3. Change current frame to /map in `global option`

4. Add panel

![panel](https://github.com/kuriatsu/rviz_hdmap_editor/blob/image/image/add_panel.png)

5. You can find the panel at left side.

![panel_added](https://github.com/kuriatsu/rviz_hdmap_editor/blob/image/image/select_panel.png)

6. Show InteractiveMarker and Marker.

7. Push `...` button in the panel and Select HDMap. point.csv, line.csv, vector.csv, and dtlane.csv are necessary.

![select_map](https://github.com/kuriatsu/rviz_hdmap_editor/blob/image/image/select_map.png)

8. Select output folder.

![select_output](https://github.com/kuriatsu/rviz_hdmap_editor/blob/image/image/select_output.png)

9. Push OK

10. Click checkbox which you want to edit. (fig shows when you click `node`)

![edit](https://github.com/kuriatsu/rviz_hdmap_editor/blob/image/image/edit_node.png)

11. Edit points.  

12. Push Save to save your modification.

### Tips
* TopDownOrtho is better to edit. If you want to move along z axis, please use the other option.
* Rotation is supported. Please zoom in and find the circle which signifies that motion.

## Contribution

1. Fork this https://github.com/kuriatsu/rviz_hdmap_editor.git
1. Create your feature branch.
3. Commit your changes (git commit -am "add some feature")
4. Push to the branch (git push origin my-new-feature)
5. Create new Pull Request

## Licence


## Author
[kuriatsu](https://github.com/kuriatsu)

## Change Log
### 2020/11/12
Maintain README.md
Not support adding element to the map. Supports only editing.
It's hard to edit along z axis. Need to be updated.
