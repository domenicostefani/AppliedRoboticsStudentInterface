#!/bin/sh

#------------------------------------------------------------------------------#
# Utiliy for changing maps
# Insert here the path to the "AppliedRoboticsEnvironment" folder
ENV_PATH="/home/lar2019/Desktop/workspace/AppliedRoboticsEnvironment"
# Then call directly the script without changing anything
#------------------------------------------------------------------------------#

outputFile=$ENV_PATH"/src/0_mindstorm_simulator/2_lego_world/models/mindstorm_map/map.sdf"
robotFile=$ENV_PATH"/src/0_mindstorm_simulator/0_sim_common/launch/sim.launch"

write_robot_pos()
{
    POS_X=$1
    POS_Y=$2
    ANGLE=$3

    rm -f $robotFile
    echo '<?xml version="1.0" encoding="UTF-8"?>' >> $robotFile
    echo "<launch>" >> $robotFile
    echo '  <param name="/use_sim_time" value="true"/>' >> $robotFile
    echo "" >> $robotFile
    echo '  <arg name="gui" default="false"/>' >> $robotFile
    echo "" >> $robotFile
    echo "  <!-- Robot pose -->" >> $robotFile
    echo '  <arg name="x" default="'$POS_X'"/>' >> $robotFile
    echo '  <arg name="y" default="'$POS_Y'"/>  ' >> $robotFile
    echo '  <arg name="yaw" default="'$ANGLE'"/>' >> $robotFile
    echo "" >> $robotFile
    echo "  <!-- Load Robot Description -->" >> $robotFile
    echo '  <include file="$(find lego_robot)/launch/robot_description.launch"/>' >> $robotFile
    echo "" >> $robotFile
    echo "  <!-- Launch Gazebo World -->" >> $robotFile
    echo '  <include file="$(find lego_world)/launch/world.launch">' >> $robotFile
    echo '    <arg name="gui" value="$(arg gui)"/>' >> $robotFile
    echo "  </include>" >> $robotFile
    echo "" >> $robotFile
    echo "  <!-- Load Robot in Gazebo -->" >> $robotFile
    echo '  <include file="$(find lego_robot)/launch/spawn_gazebo.launch">' >> $robotFile
    echo '    <arg name="x" value="$(arg x)"/>' >> $robotFile
    echo '    <arg name="y" value="$(arg y)"/>' >> $robotFile
    echo '    <arg name="z" value="0.0"/>' >> $robotFile
    echo '    <arg name="roll" value="0.0"/>' >> $robotFile
    echo '    <arg name="pitch" value="0.0"/>' >> $robotFile
    echo '    <arg name="yaw" value="$(arg yaw)"/>' >> $robotFile
    echo "  </include>" >> $robotFile
    echo "</launch>" >> $robotFile
    echo "" >> $robotFile

}

write_fixed_arena_part()
{
    echo "" >> $outputFile
    echo "    <!-- ################################### -->" >> $outputFile
    echo "    <!-- #####!!!!DO NOT CHANGE THIS!!!##### -->" >> $outputFile
    echo "    <!-- ################################### -->" >> $outputFile
    echo "    <!-- FIXED PART OF THE ARENA -->" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri> model://long_black_stripe</uri>" >> $outputFile
    echo "      <pose> 0.78 0.0 0 0 -0 0 </pose>" >> $outputFile
    echo "      <name>long_stripe_down</name>" >> $outputFile
    echo "    </include>  " >> $outputFile
    echo "  " >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri> model://long_black_stripe</uri>" >> $outputFile
    echo "      <pose> 0.78 1.06 0 0 -0 0 </pose>" >> $outputFile
    echo "      <name>long_stripe_up</name>" >> $outputFile
    echo "    </include> " >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri> model://short_black_stripe</uri>" >> $outputFile
    echo "      <pose> 0.0 0.53 0 0 0 1.5707963268 </pose>" >> $outputFile
    echo "      <name>short_stripe_left</name>" >> $outputFile
    echo "    </include>  " >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri> model://short_black_stripe</uri>" >> $outputFile
    echo "      <pose> 1.56 0.53 0 0 0 1.5707963268 </pose>" >> $outputFile
    echo "      <name>short_stripe_right</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri> model://short_red_stripe</uri>" >> $outputFile
    echo "      <pose> 0.2 -0.1 0 0 0 0 </pose>" >> $outputFile
    echo "      <name>short_red_stripe</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri> model://arena_ground</uri>" >> $outputFile
    echo "      <pose> 0.75 0.5 -0.05 0 -0 0 </pose>" >> $outputFile
    echo "      <name>arena_ground</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "  </model>      " >> $outputFile
    echo "</sdf>" >> $outputFile
}

echo "*** MAPS ***"
echo "0 - default"
echo "1 - default plus obstacle between v1 and v3"
echo "2 - ZigZag Map 1"
echo "3 - ZigZag Map 2 (like 1 with gate moved)"
echo "4 - Ring Map 2 (should require u turn)"
echo "5 - Last year (Narrow passage)"
read -p "Choose the map: " CHOICE

if [ $CHOICE = "0" ]
then
    rm -f $outputFile
    echo "<?xml version='1.0' encoding='UTF-8'?>" >> $outputFile
    echo '<sdf version="1.4">' >> $outputFile
    echo '  <model name="mindstorm_map">' >> $outputFile
    echo "" >> $outputFile
    echo "    <!-- Gate and camera section -->" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://gate</uri>" >> $outputFile
    echo "      <pose>1.3 1.0 0 0 0 1.57</pose>" >> $outputFile
    echo "      <name>gate</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <!-- Victim section -->" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_1</uri>" >> $outputFile
    echo "      <pose>0.8 0.2 0 0 0 1.5</pose>" >> $outputFile
    echo "      <name>victim1</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <!--include>" >> $outputFile
    echo "      <uri>model://victim_2</uri>" >> $outputFile
    echo "      <pose>0.9 0.2 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim2</name>" >> $outputFile
    echo "    </include-->" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_3</uri>" >> $outputFile
    echo "      <pose>1.3 0.2 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim3</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_4</uri>" >> $outputFile
    echo "      <pose>1.3 0.45 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim4</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_5</uri>" >> $outputFile
    echo "      <pose>1.27 0.75 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim5</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "" >> $outputFile
    echo "    <!-- Obstacle section -->    " >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://hexagon_95</uri>" >> $outputFile
    echo "      <pose>0.04 0.45 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_1</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://pentagon_95</uri>" >> $outputFile
    echo "      <pose>0.04 0.6 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_2</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://rectangle_95</uri>" >> $outputFile
    echo "      <pose>0.17 0.45 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_3</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://triangle_95</uri>" >> $outputFile
    echo "      <pose>0.17 0.6 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_4</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://hexagon_142</uri>" >> $outputFile
    echo "      <pose>0.3 0.45 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_5</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://pentagon_142</uri>" >> $outputFile
    echo "      <pose>0.3 0.6 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_6</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://rectangle_142</uri>" >> $outputFile
    echo "      <pose>0.5 0.45 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_7</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://triangle_142</uri>" >> $outputFile
    echo "      <pose>0.5 0.6 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_8</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://hexagon_189</uri>" >> $outputFile
    echo "      <pose>0.9 0.45 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_9</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://pentagon_189</uri>" >> $outputFile
    echo "      <pose>0.9 0.65 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_10</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://rectangle_189</uri>" >> $outputFile
    echo "      <pose>0.7 0.45 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_11</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://triangle_189</uri>" >> $outputFile
    echo "      <pose>0.7 0.65 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_12</name>" >> $outputFile
    echo "    </include>   " >> $outputFile
    write_fixed_arena_part

    echo "MAP written to " $outputFile

    write_robot_pos 0.2 0.2 0

    echo "Robot position written to " $robotFile
elif [ $CHOICE = "1" ]
then
    rm -f $outputFile
    echo "<?xml version='1.0' encoding='UTF-8'?>" >> $outputFile
    echo '<sdf version="1.4">' >> $outputFile
    echo '  <model name="mindstorm_map">' >> $outputFile
    echo "" >> $outputFile
    echo "    <!-- Gate and camera section -->" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://gate</uri>" >> $outputFile
    echo "      <pose>1.3 1.0 0 0 0 1.57</pose>" >> $outputFile
    echo "      <name>gate</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <!-- Victim section -->" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_1</uri>" >> $outputFile
    echo "      <pose>0.8 0.2 0 0 0 1.5</pose>" >> $outputFile
    echo "      <name>victim1</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <!--include>" >> $outputFile
    echo "      <uri>model://victim_2</uri>" >> $outputFile
    echo "      <pose>0.9 0.2 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim2</name>" >> $outputFile
    echo "    </include-->" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_3</uri>" >> $outputFile
    echo "      <pose>1.3 0.2 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim3</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_4</uri>" >> $outputFile
    echo "      <pose>1.3 0.45 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim4</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_5</uri>" >> $outputFile
    echo "      <pose>1.3 0.80 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim5</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "" >> $outputFile
    echo "    <!-- Obstacle section -->    " >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://hexagon_95</uri>" >> $outputFile
    echo "      <pose>0.4 0.1 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_1</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://pentagon_95</uri>" >> $outputFile
    echo "      <pose>0.04 0.6 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_2</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://rectangle_95</uri>" >> $outputFile
    echo "      <pose>0.17 0.45 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_3</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://triangle_95</uri>" >> $outputFile
    echo "      <pose>0.17 0.6 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_4</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://hexagon_142</uri>" >> $outputFile
    echo "      <pose>0.3 0.45 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_5</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://pentagon_142</uri>" >> $outputFile
    echo "      <pose>0.3 0.6 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_6</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://rectangle_142</uri>" >> $outputFile
    echo "      <pose>0.5 0.45 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_7</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://triangle_142</uri>" >> $outputFile
    echo "      <pose>0.5 0.6 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_8</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://hexagon_189</uri>" >> $outputFile
    echo "      <pose>0.9 0.45 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_9</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://pentagon_189</uri>" >> $outputFile
    echo "      <pose>0.9 0.65 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_10</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://rectangle_189</uri>" >> $outputFile
    echo "      <pose>0.7 0.45 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_11</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://triangle_189</uri>" >> $outputFile
    echo "      <pose>0.7 0.65 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_12</name>" >> $outputFile
    echo "    </include>   " >> $outputFile
    write_fixed_arena_part

    echo "MAP written to " $outputFile

    write_robot_pos 0.2 0.2 0

    echo "Robot position written to " $robotFile

elif [ $CHOICE = "2" ]
then
    rm -f $outputFile
    echo "<?xml version='1.0' encoding='UTF-8'?>" >> $outputFile
    echo '<sdf version="1.4">' >> $outputFile
    echo '  <model name="mindstorm_map">' >> $outputFile
    echo "" >> $outputFile
    echo "    <!-- Gate and camera section -->" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://gate</uri>" >> $outputFile
    echo "      <pose>1.3 1.0 0 0 0 1.57</pose>" >> $outputFile
    echo "      <name>gate</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <!-- Victim section -->" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_1</uri>" >> $outputFile
    echo "      <pose>0.25 0.25 0 0 0 1.5</pose>" >> $outputFile
    echo "      <name>victim1</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_2</uri>" >> $outputFile
    echo "      <pose>0.6 0.4 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim2</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_3</uri>" >> $outputFile
    echo "      <pose>0.75 0.83 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim3</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_4</uri>" >> $outputFile
    echo "      <pose>1.22 0.6 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim4</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <!--include>" >> $outputFile
    echo "      <uri>model://victim_5</uri>" >> $outputFile
    echo "      <pose>1.3 0.75 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim5</name>" >> $outputFile
    echo "    </include-->" >> $outputFile
    echo "" >> $outputFile
    echo "" >> $outputFile
    echo "    <!-- Obstacle section -->    " >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://hexagon_95</uri>" >> $outputFile
    echo "      <pose>0.35 0.6 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_1</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://pentagon_95</uri>" >> $outputFile
    echo "      <pose>0.35 0.4 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_2</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://rectangle_95</uri>" >> $outputFile
    echo "      <pose>0.75 0.4 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_3</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://triangle_95</uri>" >> $outputFile
    echo "      <pose>0.75 0.6 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_4</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://hexagon_142</uri>" >> $outputFile
    echo "      <pose>0.75 0.2 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_5</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    write_fixed_arena_part

    echo "MAP written to " $outputFile

    write_robot_pos 0.2 0.8 -1.57

    echo "Robot position written to " $robotFile

elif [ $CHOICE = "3" ]
then
    rm -f $outputFile
    echo "<?xml version='1.0' encoding='UTF-8'?>" >> $outputFile
    echo '<sdf version="1.4">' >> $outputFile
    echo '  <model name="mindstorm_map">' >> $outputFile
    echo "" >> $outputFile
    echo "    <!-- Gate and camera section -->" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://gate</uri>" >> $outputFile
    echo "      <pose>1.3 0.06 0 0 0 -1.57</pose>" >> $outputFile
    echo "      <name>gate</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <!-- Victim section -->" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_1</uri>" >> $outputFile
    echo "      <pose>0.24 0.26 0 0 0 1.5</pose>" >> $outputFile
    echo "      <name>victim1</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_2</uri>" >> $outputFile
    echo "      <pose>0.6 0.4 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim2</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_3</uri>" >> $outputFile
    echo "      <pose>0.75 0.83 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim3</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_4</uri>" >> $outputFile
    echo "      <pose>1.22 0.6 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim4</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <!--include>" >> $outputFile
    echo "      <uri>model://victim_5</uri>" >> $outputFile
    echo "      <pose>1.3 0.75 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim5</name>" >> $outputFile
    echo "    </include-->" >> $outputFile
    echo "" >> $outputFile
    echo "" >> $outputFile
    echo "    <!-- Obstacle section -->    " >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://hexagon_95</uri>" >> $outputFile
    echo "      <pose>0.35 0.6 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_1</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://pentagon_95</uri>" >> $outputFile
    echo "      <pose>0.35 0.4 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_2</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://rectangle_95</uri>" >> $outputFile
    echo "      <pose>0.75 0.4 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_3</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://triangle_95</uri>" >> $outputFile
    echo "      <pose>0.75 0.6 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_4</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://hexagon_142</uri>" >> $outputFile
    echo "      <pose>0.75 0.2 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_5</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    write_fixed_arena_part

    echo "MAP written to " $outputFile

    write_robot_pos 0.2 0.8 -1.57

    echo "Robot position written to " $robotFile

elif [ $CHOICE = "4" ]
then
    rm -f $outputFile
    echo "<?xml version='1.0' encoding='UTF-8'?>" >> $outputFile
    echo '<sdf version="1.4">' >> $outputFile
    echo '  <model name="mindstorm_map">' >> $outputFile
    echo "" >> $outputFile
    echo "    <!-- Gate and camera section -->" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://gate</uri>" >> $outputFile
    echo "      <pose>1.3 1.0 0 0 0 1.57</pose>" >> $outputFile
    echo "      <name>gate</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <!-- Victim section -->" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_1</uri>" >> $outputFile
    echo "      <pose>0.2 0.6 0 0 0 1.5</pose>" >> $outputFile
    echo "      <name>victim1</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_2</uri>" >> $outputFile
    echo "      <pose>0.6 0.6 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim2</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_3</uri>" >> $outputFile
    echo "      <pose>0.5 0.2 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim3</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_4</uri>" >> $outputFile
    echo "      <pose>1.0 0.6 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim4</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <!--include>" >> $outputFile
    echo "      <uri>model://victim_5</uri>" >> $outputFile
    echo "      <pose>1.3 0.75 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim5</name>" >> $outputFile
    echo "    </include-->" >> $outputFile
    echo "" >> $outputFile
    echo "" >> $outputFile
    echo "    <!-- Obstacle section -->    " >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://hexagon_95</uri>" >> $outputFile
    echo "      <pose>0.95 0.75 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_1</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://pentagon_95</uri>" >> $outputFile
    echo "      <pose>0.15 0.25 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_2</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://rectangle_95</uri>" >> $outputFile
    echo "      <pose>0.15 0.15 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_3</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://triangle_95</uri>" >> $outputFile
    echo "      <pose>0.75 0.35 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_4</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://hexagon_142</uri>" >> $outputFile
    echo "      <pose>0.95 0.30 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_5</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://rectangle_142</uri>" >> $outputFile
    echo "      <pose>1.15 0.30 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_5</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://rectangle_189</uri>" >> $outputFile
    echo "      <pose>1.35 0.30 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_5</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    write_fixed_arena_part

    echo "MAP written to " $outputFile

    write_robot_pos 0.8 0.8 3

    echo "Robot position written to " $robotFile
elif [ $CHOICE = "5" ]
then
    rm -f $outputFile
    echo "<?xml version='1.0' encoding='UTF-8'?>" >> $outputFile
    echo '<sdf version="1.4">' >> $outputFile
    echo '  <model name="mindstorm_map">' >> $outputFile
    echo "" >> $outputFile
    echo "    <!-- Gate and camera section -->" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://gate</uri>" >> $outputFile
    echo "      <pose>1.3 0.06 0 0 0 -1.57</pose>" >> $outputFile
    echo "      <name>gate</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <!-- Victim section -->" >> $outputFile
    echo "    <!--include>" >> $outputFile
    echo "      <uri>model://victim_1</uri>" >> $outputFile
    echo "      <pose>0.24 0.26 0 0 0 1.5</pose>" >> $outputFile
    echo "      <name>victim1</name>" >> $outputFile
    echo "    </include-->" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_2</uri>" >> $outputFile
    echo "      <pose>0.4 0.73 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim2</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://victim_3</uri>" >> $outputFile
    echo "      <pose>1.05 0.55 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim3</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>">> $outputFile
    echo "      <uri>model://victim_4</uri>" >> $outputFile
    echo "      <pose>1.25 0.3 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim4</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <!--include>" >> $outputFile
    echo "      <uri>model://victim_5</uri>" >> $outputFile
    echo "      <pose>1.3 0.75 0 0 0 0</pose>" >> $outputFile
    echo "      <name>victim5</name>" >> $outputFile
    echo "    </include-->" >> $outputFile
    echo "" >> $outputFile
    echo "" >> $outputFile
    echo "    <!-- Obstacle section -->    " >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://hexagon_95</uri>" >> $outputFile
    echo "      <pose>0.6 0.25 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_1</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://hexagon_142</uri>" >> $outputFile
    echo "      <pose>0.05 0.4 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_2</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://rectangle_95</uri>" >> $outputFile
    echo "      <pose>0.6 0.9 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_3</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://triangle_95</uri>" >> $outputFile
    echo "      <pose>0.6 0.7 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_4</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    echo "" >> $outputFile
    echo "    <include>" >> $outputFile
    echo "      <uri>model://pentagon_95</uri>" >> $outputFile
    echo "      <pose>0.6 0.1 0 0 0 0</pose>" >> $outputFile
    echo "      <name>obstacle_5</name>" >> $outputFile
    echo "    </include>" >> $outputFile
    write_fixed_arena_part

    echo "MAP written to " $outputFile

    write_robot_pos 1 0.85 0

    echo "Robot position written to " $robotFile

else
    echo "Not an available map"
fi


