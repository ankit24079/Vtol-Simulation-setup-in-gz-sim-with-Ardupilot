# Setting up GZ-Sim with ArduPilot

This setup is intended for a **VTOL (Vertical Takeoff and Landing) QuadPlane** model.

---

## 🧩 Prerequisites

* Ubuntu 22.04

---

## ⚙️ Part 1 – Install Required Dependencies

These packages ensure a working development environment for Python, C++, and ROS / Gazebo integration.

```bash
# Update package lists
sudo apt-get update

# Install Git and tools
sudo apt-get install git gitk git-gui

# Install Python tools
sudo apt-get install python3-pip python3-dev

# Install build tools
sudo apt-get install build-essential ccache g++ gawk git make wget

# Install required Python packages
sudo apt-get install python3-pip python3-matplotlib python3-serial python3-wxgtk4.0 python3-lxml python3-scipy python3-opencv python3-numpy
```

---

## 🧱 Part 2 – Gazebo Harmonic Setup

Installs the **Gazebo Harmonic** simulator core and multimedia dependencies.

```bash
# Update apt
sudo apt update

# Install Gazebo Harmonic core and dev files
sudo apt install libgz-sim8-dev rapidjson-dev

# Install multimedia dependencies
sudo apt install libopencv-dev libgstreamer1.0-dev \
libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad \
gstreamer1.0-libav gstreamer1.0-gl

# Install MAVProxy
sudo pip3 install MAVProxy
```

---

## 🪄 Part 3 – Gazebo Dependencies (rosdep)

Use **rosdep** to manage dependencies.

```bash
export GZ_VERSION=harmonic

sudo bash -c 'wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list \
-O /etc/ros/rosdep/sources.list.d/00-gazebo.list'

rosdep update
rosdep resolve gz-harmonic

# Navigate to your ROS workspace before running:
rosdep install --from-paths src --ignore-src -y
```

---

## 🔌 Part 4 – ArduPilot-Gazebo Plugin

This plugin connects **ArduPilot** to **Gazebo**.

```bash
export GZ_VERSION=harmonic
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4

# Add plugin paths
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
source ~/.bashrc
```

---

## ✈️ Part 5 – ArduPilot Setup

```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

---

## 🧰 Part 6 – SITL Models Setup

```bash
git clone https://github.com/ArduPilot/SITL_Models.git

echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:\
$HOME/SITL_Models/Gazebo/models:\
$HOME/SITL_Models/Gazebo/worlds' >> ~/.bashrc
source ~/.bashrc
```

---

## 🤖 Part 7 – ROS 2 Humble & MAVROS Setup

```bash
# Install ROS 2 build tools
sudo apt install python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update

# Install MAVROS
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# Install geographic data
sudo apt install geographiclib-tools
sudo geographiclib-get-geoids egm96-5
```

---

## 🚀 Running the Full Simulation

*(SITL + GZ-Sim + MAVROS)*

Use three separate terminals.

### **Terminal 1 – Launch Gazebo**

```bash
gz sim -v4 -r alti_transition_runway.sdf
```

### **Terminal 2 – Launch ArduPilot SITL**

```bash
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduPlane --model JSON \
--add-param-file=$HOME/SITL_Models/Gazebo/config/alti_transition_quad.param \
--console --map
```

### **Terminal 3 – Start MAVROS Bridge**

```bash
ros2 launch mavros apm.launch fcu_url:=udp://0.0.0.0:14550@127.0.0.1:14550
```

---

## 🧮 Appendix – Customizations

### **A1 – Changing World to Baylands Terrain**

```bash
mkdir -p ~/.gz/models/BaylandsTerrain
tar xf model.tar.gz -C ~/.gz/models/BaylandsTerrain
ls ~/.gz/models/BaylandsTerrain
```

Expected files:

```
model.sdf  materials/  meshes/  metadata.json
```

Copy and edit the world file:

```bash
cp ~/SITL_Models/Gazebo/worlds/alti_transition_runway.sdf \
~/SITL_Models/Gazebo/worlds/baylands_vtol_world.sdf
```

Replace the `<include>` block for `runway` with:

```xml
<include>
  <pose degrees="true">42.44 117 -1.82 0 0 363</pose>
  <uri>model://baylands_terrain</uri>
</include>
```

Launch:

```bash
gz sim -v4 -r ~/SITL_Models/Gazebo/worlds/baylands_vtol_world.sdf
```

---

### **A2 – Adding a Camera to the Model**

Add inside `<world>`:

```xml
<plugin name="gz::sim::systems::Sensors" filename="gz-sim-sensors-system">
  <render_engine>ogre2</render_engine>
</plugin>
```

Then insert the camera XML snippet (from presentation) inside `<model>`.

List topics:

```bash
gz topic -l
```

Enable image streaming:

```bash
gz topic -t /world/runway/model/alti_transition_quad/link/camera_link/sensor/bottom_camera/image/enable_streaming \
-m gz.msgs.Boolean -p "data: 1"
```

View the video:

```bash
gst-launch-1.0 -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

---

### **A3 – Adding Smoke Emitter**

Place textures in:

```
~/SITL_Models/Gazebo/worlds/materials/textures/
├── fog.png
└── fogcolor.png
```

Insert the fog generator XML inside `<world>` and adjust:

* **rate:** e.g., 30
* **particle_size:** 0.5 0.5 0.5
* **size:** increase for larger volume
* **diffuse / color_range_image:** tune for visibility

---

**✅ You’re now ready to simulate VTOL aircraft in Gazebo Harmonic with ArduPilot SITL and ROS 2 integration.**

