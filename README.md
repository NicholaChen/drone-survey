# drone-survey

## Overview
Affordable drone surveying method using free tools and low cost DJI drones.

## Abstract
Drone surveying is being used extensively in topographic mapping, agriculture, and ecological monitoring. This technology is dominated by DJI Enterprise drones which employ Global Navigation Satellite System (GNSS) sensors and use Post Processed Kinematics (PPK) or Real Time Kinematics (RTK) to pinpoint locations with centimetre precision. However, this technology is unavailable to cheaper drones. To overcome this, our research implements the use of an affordable drone, the DJI Mini 4 Pro, with free services to achieve surveying capabilities without the overhead costs associated with DJI Enterprise drones. Our implementation uses Litchi Mission Hub and DJI Fly to fly pre program flight paths while recording video footage and flight logs. These flight logs were decrypted using AirData and combined with the video footage in our custom library that allowed for the footage to be analyzed with GPS coordinates from the flight logs. To test the accuracy, our library was used to calculate the positions of specific visual markers (AprilTags), which were then compared with their real-world placement. Our research shows that affordable consumer drones with free software can be used as an alternative for small scale surveying, expanding the accessibility of drone surveying to those with limited resources. 

## Installation
```bash
python -m pip install --index-url https://test.pypi.org/simple/ --extra-index-url https://pypi.org/simple drone-survey==0.0.2
```

## Usage
More information to come...
```python
import drone_survey
import numpy as np
from pupil_apriltags import Detector
import pyproj

# DJI Mini 4 Pro at 4K/100FPS (slow-mo)
d = drone_survey.DroneData(
    flight_csv_file="...", # AirData csv file path
    video_files=["..."], # video file paths
    fps=100, # video FPS
    resolution=(3840, 2160), # video resolution (width, height)
    compass_heading=0, # compass heading (degrees) [0deg is North, 90deg is East, etc...]
    altitude=2, # drone altitude (m)
    gimbal_pitch=-90, # gimbal pitch (degrees)
    f_x=2.9099e+03, # focal length x (pixels)
    f_y=2.9099e+03, # focal length y (pixels)
    c_x=1920, # optical center x (pixels)
    c_y=1080, # optical center y (pixels)
    distortion=np.array([0.0705, -0.1303, 0, 0, 0]), # lens distortion coefficients
    max_missing_frames=200, # maximum frames without AprilTags allowed while syncing
    timestep=0.001, # time step when syncing (s)
    AprilTagDetector=Detector(
        families="tag36h11", # AprilTag family
        nthreads=12,
        debug=0
    ),
    pyprojTransformer=pyproj.Transformer.from_crs(
        pyproj.CRS("EPSG:4326"), # WGS84
        pyproj.CRS("EPSG:3158"), # UTM zone 14N
        always_xy=True
    ), # WGS84 to UTM zone 14N
    showVideo=False, # show video output while syncing or analyzing
    debug=True, # show debug information
)

d.sync(["..."]) # optionally specify cache file location, one for each video
d.showVideo = True

data = d.analyze(0) # analyze the first video
```
