# Battery-Conscious Robot Tour Guide

Intro to Intelligent Robotics, Project 3: Jordan DeJarnett, Jesse Jennings, Kevin Li, Brad Darby

A TurtleBot 4 tour guide that visits landmark stops, shows commentary for each stop on a nearby computer, and plans its tour around its battery, ensuring it does not die mid-tour: it picks which stops and in what order, replans mid-tour if needed, and returns to its dock to charge between tours.

**Paradigm:** hybrid deliberative/reactive, organized as a three-tiered (3T) architecture.

## Packages

| Package | Layer | Contents |
| --- | --- | --- |
| `tour_guide_interfaces` | — | Custom messages and services (`Stop`, `StopArray`, `EstimateCost`, `PlanTour`) |
| `tour_guide_shared` | Planning | Reusable nodes: `stop_manager`, `energy_model`, `tour_planner`, `sim_battery` |
| `tour_guide` | Executive | `tour_executive`, `narration_display`, launch files, Gazebo world, `stops.yaml` |
| `p1_reactive` | Behavioral | Safety behaviors and arbiter from Projects 1 and 2 |

## Build

```bash
mkdir -p ~/tour_ws/src && cd ~/tour_ws/src
git clone https://github.com/JorDej1/tour-guide.git
cd ~/tour_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

Coming Soon
