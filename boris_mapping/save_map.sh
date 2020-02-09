#!/bin/sh
rosrun map_server map_saver map:=map -f $(rospack find boris_mapping)/maps/saved

