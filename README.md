[//]: # (Image References)

[image1]: ./assets/openmanipulator_1.png "openmanipulator"
[image2]: ./assets/openmanipulator_2.png "openmanipulator"
[image3]: ./assets/openmanipulator.png "openmanipulator"
[image4]: ./assets/git_diff.png "git"
[image5]: ./assets/openmanipulator_3.png "openmanipulator"
[image6]: ./assets/openmanipulator_4.png "openmanipulator"
[image7]: ./assets/openmanipulator_5.png "openmanipulator"
[image8]: ./assets/openmanipulator_6.png "openmanipulator"
[image9]: ./assets/ik.png "openmanipulator"
[image10]: ./assets/openmanipulator_7.png "openmanipulator"
[image11]: ./assets/ur_1.png "UR3e"
[image12]: ./assets/ur_2.png "UR3e"
[image13]: ./assets/ur_3.png "UR3e"
[image14]: ./assets/ur_4.png "UR3e"
[image15]: ./assets/ur_5.png "UR3e"
[image16]: ./assets/ur_6.png "UR3e"
[image17]: ./assets/ur_7.png "UR3e"

# 11. - 12. hét - robotkarok

# Hova fogunk eljutni?
<a href="https://youtu.be/mm2vKYH-Jy8"><img width="600" src="./assets/youtube_1.png"></a>  
<a href="https://youtu.be/TDOdKdiD7pk"><img width="600" src="./assets/youtube_2.png"></a>  
<a href="https://youtu.be/llD6eGD8nEM"><img width="600" src="./assets/youtube_3.png"></a>  
<a href="https://youtu.be/BLvH7DzvwUk"><img width="600" src="./assets/youtube_4.png"></a>

# Tartalomjegyzék
1. [Kezdőcsomag](#Kezdőcsomag)  
2. [OpenMANIPULATOR-X](#OpenMANIPULATOR-X)  
2.1. [Első próba](#Első-próba)  
2.2. [Indítás MoveIt-tel](#Indítás-MoveIt-tel)  
2.3. [IKFast plugin](#IKFast-plugin)  
2.4. [Saját node mozgatáshoz](#Saját-node-mozgatáshoz)  
2.5. [Gazebo világba helyezés](#Gazebo-világba-helyezés)  
2.6. [Saját IK](#Saját-IK)  
3. [UR3e robotkar](#UR3e-robotkar)  
3.1. [UR3e gripperrel](#UR3e-gripperrel)  
3.2. [SRDF fájl](#SRDF-fájl)  
3.3. [MoveIt commander](#MoveIt-commander)  

# Kezdőcsomag

A kezdőcsomag ebben az esetben csak a dokumentációt foglalja magában, mert olyan robotkarokkal fogunk foglalkozni, amik rendelkeznek hivatalos ROS csomagokkal, ezeket fogjuk letölteni és kipróbálni!

# OpenMANIPULATOR-X

Az OpenMANIPULATOR-X egy 4 szabadsági fokú robotkar, a Turtlebot 3 hivatalos manipulátora. 
![alt text][image3]

A hivatalos dokumentációja [itt](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_operation/) található.

Ne telepítsünk tárolóból semmilyen OpenMANIPULATOR csomagot, bár Noetic esetén amúgy sem elérhetők tárolóból.

Helyette töltsük le ezeket a git repokat:
```console
git clone https://github.com/MOGI-ROS/robotis_manipulator
git clone https://github.com/MOGI-ROS/dynamixel-workbench
git clone https://github.com/MOGI-ROS/dynamixel-workbench-msgs
git clone https://github.com/MOGI-ROS/DynamixelSDK
git clone https://github.com/MOGI-ROS/open_manipulator_controls
git clone https://github.com/MOGI-ROS/open_manipulator_simulations
git clone https://github.com/MOGI-ROS/open_manipulator_msgs
git clone https://github.com/MOGI-ROS/open_manipulator
```

Ezen kívül használni fogunk még egy gazebo plugint, amire az OpenMANIPULATOR *description*-je is hivatkozik, azonban a hivatalos dokumentációban elfelejtették megemlíteni:
```console
git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins
```

Fordítsuk újra a workspace-t, és máris készen állunk az első próbára.

## Első próba

3 terminál ablakra lesz szükségünk, indítsuk el a következő launch fájlokat.
```console
roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch
```
Ez elindítja a Gazebo szimulációt, vegyük észre, hogy a szimuláció megállítva indul el, épp ahogy az előző leckében láttuk.
![alt text][image7]

```console
roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false
```
Ennek eredményeképp elindul a controller, ami innentől kezdve mozgatni tudja a szimulált karunkat, ha minden rendben ment, akkor a következőt kell látnunk a terminálban:
```console
process[open_manipulator_controller-1]: started with pid [10012]
port_name and baud_rate are set to /dev/ttyUSB0, 1000000
[INFO] Ready to simulate /open_manipulator_controller on Gazebo
```

Ez pedig elindítja az OpenMANIPULATOR grafikus vezérlőpaneljét. Indítsuk el a timert, engedélyezzük az aktuáto, majd küldjük a kart home pozícióba:
```console
roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch
```
![alt text][image8]

Ha esetleg ebbe a hibába futnánk:
```console
[ERROR] [1618922478.206182640, 0.022000000]: Could not load controller 'gripper_position' because controller type 'effort_controllers/JointPositionController' does not exist.
```

Akkor telepítsük az `effort controllers` csomagot:
```console
sudo apt install ros-$(rosversion -d)-effort-controllers
```

## Indítás MoveIt-tel

Az OpenMANIPULATOR fenti iplementációja sajnos nem joint trajectory controller alapú, hanem ROS service hívásokkal tudjuk mozgatni. Ennek ugyan vannak előnyei, hiszen az OpenaMANIPULATOR rengeteg service-t kezel, többek között az inverz kinematikát is kapjuk hozzá.
```console
...
/goal_drawing_trajectory
/goal_joint_space_path
/goal_joint_space_path_from_present
/goal_joint_space_path_to_kinematics_orientation
/goal_joint_space_path_to_kinematics_pose
/goal_joint_space_path_to_kinematics_position
/goal_task_space_path
/goal_task_space_path_from_present
/goal_task_space_path_from_present_orientation_only
/goal_task_space_path_from_present_position_only
/goal_task_space_path_orientation_only
/goal_task_space_path_position_only
/goal_tool_control
...
```

Azonban az `rqt_joint_trajectory_controller`-t sem tudjuk használni vele, és a MoveIt-tel sem kompatibilis. Szerencsére épp emiatt létezik egy joint trajectory controller alapú megolás is, így most csak azzal fogunk foglalkozni, de a service alapú megoldást is próbáljátok ki nyugodtan!

A joint trajectory controller-t a következő módon tudjuk elindítani, ez egyben elindítja a szimulációt is, tehát nincs szükség más launchfájlok indítására.
```console
roslaunch open_manipulator_controllers joint_trajectory_controller.launch
```

Azonnal ki is tudjuk próbálni MoveIt-tel:

![alt text][image1]

És természetesen a `rqt_joint_trajectory_controller` is ugyanúgy működik vele, ahogy korábban láttuk:
```console
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

MoveIt kapcsán hamar észrevesszük, hogy ugyanaz a probléma az OpenMANIPULATOR esetén is, mint az előző fejezetben a saját 4 DoF robotkarunk esetén, a MoveIt KDL pluginja nem támogatja a 6-nál kisebb szabadsági fokú karokat.

## IKFast plugin

Ennek a megoldására készítettem egy IKFast plugint, amit innen tudtok letölteni:
```console
git clone https://github.com/MOGI-ROS/open_manipulator_ikfast_plugin
```

A workspace újrafordítása után a `open_manipulator_controls/open_manipulator_moveit_config/config` mappában lévő `kinematics.yaml` fájlt módosítva tudjuk aktiválni a plugint:

Eredeti:
```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

Módosított:
```yaml
arm:
  kinematics_solver: open_manipulator/IKFastKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

A módosítást természetesen MoveIt Setup Assistant-tal is megcsinálhatjuk!

Ezután próbáljuk is ki a MoveIt-tel:
```console
roslaunch open_manipulator_controllers joint_trajectory_controller.launch
```
![alt text][image6]

## Saját node mozgatáshoz
A joint trajectory controller-nek köszönhetően a már korábban látott módon saját node-ból is egyszerűen tudjuk mozgatni a kar jointjait!

Töltsük le ehhez a következő csomagot:
```console
git clone https://github.com/MOGI-ROS/open_manipulator_tools
```

Indítsuk el a szimulációt, ahogy eddig:
```console
roslaunch open_manipulator_controllers joint_trajectory_controller.launch
```

Majd próbáljuk ki a következő node-okat:

```console
rosrun open_manipulator_tools send_joint_angles.py
rosrun open_manipulator_tools close_gripper.py
rosrun open_manipulator_tools open_gripper.py
```

## Gazebo világba helyezés

Ahhoz, hogy egy Gazebo világba helyezzük a kart, módosítsuk egy kicsit a gyári launch fájlt:  
`open_manipulator_controls/open_manipulator_hw/launch/open_manipulator_gazebo.launch`

```xml
<?xml version="1.0"?>
<launch>
  <arg name="gui" default="true"/>
  <arg name="paused" default="true"/>
  <arg name="use_sim_time" default="true"/>

  <!-- World File -->
  <arg name="world_file" default="$(find open_manipulator_tools)/worlds/world.world"/>

  <!-- Spawn z coordinate -->
  <arg name="z" default="1.02"/>

  <!-- startup simulated world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="paused" value="$(arg paused)"/>
    <arg name="gui" value="$(arg gui)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="world_name" value="$(arg world_file)"/>
  </include>

  <!-- Load the URDF into the ROS Parameter Server -->
  <param name="robot_description"
   command="$(find xacro)/xacro --inorder '$(find open_manipulator_description)/urdf/open_manipulator_robot.urdf.xacro'"/>

  <!-- push robot_description to factory and spawn robot in gazebo -->
  <node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
    args="-urdf -param robot_description -model robot -x 0.0 -y 0.0 -z $(arg z) -Y 0.0 -J joint1 0.0 -J joint2 -1.0 -J joint3 0.3 -J joint4 0.7 -J gripper 0.0 -J gripper_sub 0.0"/>
</launch>
```
A módosítások diff-je:
![alt text][image4]

Ezután már egy egyszerű, az előző fejezethez hasonló, szimulált világban indul a kar.
![alt text][image5]

## Saját IK
Mivel az OpenMANIPULATOR-X felépítése nagyon hasonlít az előző leckében épített karhoz, kis módosítással elkészíthetjük a saját inverz kinematika számoló node-unkat. Nézzük meg az eltéréseket az előző lecke karjához képest!

![alt text][image9]

Nézzük meg működés közben, először indítsuk el a szimulációt:
```console
roslaunch open_manipulator_controllers joint_trajectory_controller.launch
```

Nyissuk ki a grippert:
```console
rosrun open_manipulator_tools open_gripper.py
```

Futtassuk az inverz kinematika node-unkat, ami adott TCP koordinátára viszi a kart.
```console
rosrun open_manipulator_tools inverse_kinematics.py
```
![alt text][image10]

# UR3e robotkar
A következő kar, amit alaposabban megnézünk egy [UR3e robotkar a Universal Robots-tól](https://www.universal-robots.com/products/ur3-robot/), ez szintén megtalálható a tanszéken. Alapesetben a kar szimulációjában nincs benne semmilyen megfogó, ezt nekünk kell majd hozzáadni.

Töltsük le a megfelelő csomagot a szimulációhoz:
```console
git clone -b calibration_devel https://github.com/dudasdavid/universal_robot
```

Fordítsuk újra a workspace-t és próbáljuk ki! Ezúttal is 3 terminálra lesz szükségünk:
```console
roslaunch ur_e_gazebo ur3e.launch limited:=true
```
Ez elindítja a Gazebo szimulációt és a joint trajectory controller-t:
![alt text][image11]

A következő launchfájl elindítja a MoveIt-ot:
```console
roslaunch ur3_e_moveit_config ur3_e_moveit_planning_execution.launch sim:=true limited:=true
```
A harmadik pedig megnyitja az RViz-t:
```console
roslaunch ur3_e_moveit_config moveit_rviz.launch config:=true
```
Ezután már használhatjuk is a MoveIt-ot a karral:
![alt text][image12]

Természetesen végre is hajthatjuk a szimulációban, amit a MoveIt tervezett:
![alt text][image13]

## UR3e gripperrel

Tegyünk egy grippert a UR3e-re, ehhez egy RH-P12-RN grippert fogunk használni a ROBOTIS-tól, a tanszéken található UR3e is ezzel a gripperrel van felszerelve. Elég sok helyen kell módosítanunk a gyári csomagokat, ezért használjunk most egy olyan branchet, ahol ez már be is van állítva!

Menjünk a `src/universal_robot` mappába és:
```console
git checkout rh-p12-rn
```

A gripper miatt töltsük le még a következő csomagokat is:
```console
git clone -b ur3e-gripper https://github.com/dudasdavid/RH-P12-RN-A
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework-msgs
```

Természetesen fordítsuk újra a workspace-t!

A `rh-p12-rn` branchen pontosan ugyanúgy kell elindítanunk a szimulációt, mint az előbb:

```console
roslaunch ur_e_gazebo ur3e.launch limited:=true
```
Ezzel elindul a Gazebo szimuláció, ahol ezúttal már a gripper is része a modellünknek:
![alt text][image14]

Elindítjuk a MoveIt-ot:
```console
roslaunch ur3_e_moveit_config ur3_e_moveit_planning_execution.launch sim:=true limited:=true
```

És végül az RViz-t:
```console
roslaunch ur3_e_moveit_config moveit_rviz.launch config:=true
```
![alt text][image15]

A MoveIt-ban, a már korábban látottakhoz hasonlóan, átválthatunk a gripper planning group-jára, és például bezárhatjuk azt:
![alt text][image16]

Ami természetesen a szimulációban is megtörténik:
![alt text][image17]

## SRDF fájl



## MoveIt commander

sudo apt install ros-melodic-moveit-commander



