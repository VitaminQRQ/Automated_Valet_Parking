clc
clear
close all

start_pos = [2.5,  2.5];
goal_pos  = [40.5, 45.5];
costmap_resolution = 0.2;

MapInfo.map_name = "./maps/map_maze.png";

MapInfo.resolution = 1;
MapInfo.origin = [0, 0]; 
MapInfo.occupancy_thresh = 1; 

AStar = AStarSearch(start_pos, goal_pos, costmap_resolution, MapInfo);

path = AStar.PathPlanning();