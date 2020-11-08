# rviz_adas_map_builder
The rviz plugin to modify ADAS map (AISAN format) with Rviz, written in c++, for human beings all over the world.

## Features
* By using Rviz, you can add other tools.
* You can modify the map with localizing and checking the actual position of each element.
* AISAN vector map builder which officially released is hard to modify the small error of position.

## Requirement
* ubuntu
* ros
* QT5 (basic packages, initially installed with ros)

![samnail](https://github.com/kuriatsu/rviz_hdmap_editor/blob/image/image/rviz_window.png)

## Install
1. create catkin_ws
```bash
cd /path/to/anydir
mkdir -p catkin_ws/src
cd /catkin_ws/src
catkin_init_workspace
```
1. download and build package
```bash
cd /catkin_ws/src
git clone https://github.com/kuriatsu/rviz_hdmap_editor.git rviz_hdmap_editor
cd ../
catkin_make
```

## Usage
1. start rviz
```bash
source /catkin_ws/devel/setup.bash
roscore &
rviz
```

1. Load HDmap and pointclond map and publish /map tf or without them like below
```bash
rosrun tf static_transform_publisher 0 0 0 0 0 0 /world /tf 100
```

1. Add panel

![panel](https://github.com/kuriatsu/rviz_hdmap_editor/blob/image/image/add_panel.png)

1. You can find the panel at right side.

![panel_added](https://github.com/kuriatsu/rviz_hdmap_editor/blob/image/image/select_panel.png)

1. Select HDMap. point.csv, line.csv, vector.csv, and dtlane.csv are necessary. 

![select_map](https://github.com/kuriatsu/rviz_hdmap_editor/blob/image/image/select_map.png)

1. Select output folder.

![select_output](https://github.com/kuriatsu/rviz_hdmap_editor/blob/image/image/select_output.png)

1. Push OK

1. Click checkbox which you want to edit. (fig shows when you click `node`)

![edit](https://github.com/kuriatsu/rviz_hdmap_editor/blob/image/image/edit_node.png)

1. Push Save to save your modification.


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

