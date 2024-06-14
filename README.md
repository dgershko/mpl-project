# mpl-project

This is a project for the Technion's Motion Planning Lab.


### Contents
1. Python interface for Kavraki Lab's VAMP solver, contained in vamp_iface.py
2. Python interface for URX package for controlling UR robots, contained in urx_iface.py
3. Dockerfile definition for running a simulation of the UR robot in a container
4. Example of using these interfaces for solving a motion planning problem in real time


### Installation
---
1. Clone the [vamp repository](https://github.com/KavrakiLab/vamp)
2. Build and install the vamp python package by running the following commands:
```bash
cd vamp
pip install .
```
3. Install the urx python package by running ```pip install urx```

### Containerization
---
There's already an existing Docker image for running the robot's simulation. 
However, it requires some set up to work properly, so for this project we created a Dockerfile which builds on top of the existing image and adds the necessary files and configurations to run the simulation.
To build the image, run the following command:
```bash
docker build -t ur5-sim .
```
To run this image as a container:
```bash
docker network create --subnet=192.168.56.0/24 ursim_net # run this once!
docker run \
  --rm \
  -it \
  -p 5900:5900 \
  -p 6080:6080 \
  -p 30000-30100:30000-30100 \
  -p 50000-50100:50000-50100 \
  --net ursim_net \
  --ip 192.168.56.101 \
  ur5-sim
```
The simulation will be available through vnc at `localhost:5900` and through a web browser at `http://192.168.56.101:6080/vnc.html?host=192.168.56.101&port=6080`

### Usage
---
#### Vamp interface
FILL THIS IN

#### URX interface:
FILL THIS IN
