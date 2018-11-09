# farmharvestbot_base

## Developer document: 

https://paper.dropbox.com/doc/FBTUG-FarmHarvestBot--AQ5fXbSM9i_eR0umPynqdE_eAg-x45BpJVL07ZnWNeqh2YBt

## User manual: 

https://paper.dropbox.com/doc/FBTUG-FarmHarvestBot--AQj2rarckaDW7FEVtB0RLzl~Ag-fNLXTm8xpJD8plZd3cTOT


## build

    cd ~/catkin_ws/src
    git clone https://github.com/FBTUG/farmharvestbot_base
    cd ~/catkin_ws
    catkin_make

## run

  ~/catkin_ws/src/farmharvestbot_base/10-basic/cli/launch$ roslaunch fhb.launch

## fhb.launch setting

    
    <!-- start switch args -->
    <arg name="cli_enabled" default="true"/>
    <arg name="harvest_enabled" default="true"/>
    <arg name="vision_enabled" default="true"/>
    <arg name="arm_enabled" default="true"/>
    <arg name="fake_arm_enabled" default="false"/>
    <arg name="car_enabled" default="true"/>
    <arg name="sim_enabled" default="false"/>
    <!-- end switch args -->
