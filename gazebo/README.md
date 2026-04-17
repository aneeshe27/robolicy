# Gazebo Static Scene

This folder now holds only the lightweight static factory scene that was built earlier as a visual reference.

It is no longer the main manipulation path for the project.

## Status

- keep this folder if you want a small local factory-looking Gazebo scene
- do not use it as the basis for the Panda pick-and-place demo
- the active manipulation path is now [external/README.md](/home/aneeshe/projects/robolicy/external/README.md)

## What remains here

- a simple factory-style world at [gazebo/worlds/factory_binpick.world](/home/aneeshe/projects/robolicy/gazebo/worlds/factory_binpick.world)
- static models for tables, bins, trays, and parts

## Why the custom arm path was retired

The custom Panda-style arm experiment in this repo was not physically trustworthy enough for a real demo. Rather than keep iterating on a shaky robot description, the repo now pivots to upstream Gazebo/ROS Franka repos under [external](/home/aneeshe/projects/robolicy/external).
