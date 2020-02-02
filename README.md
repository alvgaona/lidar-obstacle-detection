# Sensor Fusion Self-Driving Car Course

In this course we will be talking about sensor fusion, whch is the process of taking data from multiple sensors and combining it to give us a better understanding of the world around us.
We will mostly be focusing on two sensors, lidar, and radar.
By the end we will be fusing the data from these two sensors to track multiple cars on the road, estimating their positions and speed.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals.
These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return.
Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal.
Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

**Radar** data is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction.
This ability makes radars a very pratical sensor for doing things like cruise control where its important to know how fast the car infront of you is traveling. Radar sensors are also very affordable and common now of days in newer cars.

**Sensor Fusion** by combing lidar's high resoultion imaging with radar's ability to measure velocity of objects we can get a better understanding of the sorrounding environment than we could using one of the sensors alone.


<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />


## Installation

First, you need to install PCL library and its dependencies.

### Ubuntu 

```bash
sudo apt install libpcl-dev
```

### Windows 

Follow this [link][PCL for Windows].

### MAC

#### Install via Homebrew

1. Install [Homebrew].

2. Update homebrew .

```bash
brew update
```

3. Add homebrew science [tap][Taps].

```bash
brew tap brewsci/science
```

4. View PCL install options.

```bash
brew options pcl
```

5. Install PCL. 

```bash
brew install pcl
```

**IMPORTANT NOTE**: If you're using `pcl-1.9.1` please follow this [link][PCL Bug].

#### Build from Source

[PCL Github]
[PCL Mac Compilation Docs]


## Usage

Download, compile and run the project

```bash
cd
git clone https://github.com/alvgaona/lidar-obstacle-detection.git
cd lidar-obstacle-detection
make build
./environment
```


[Homebrew]: https://brew.sh/
[Taps]: https://docs.brew.sh/Taps
[PCL Bug]: https://github.com/udacity/SFND_Lidar_Obstacle_Detection/issues/23
[PCL Github]: https://github.com/PointCloudLibrary/pcl
[PCL for Windows]: http://www.pointclouds.org/downloads/windows.html
[PCL Mac Compilation Docs]: http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php