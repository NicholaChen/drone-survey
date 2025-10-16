# drone-survey 0.1.0

## Overview
Affordable drone surveying method using free tools and low cost DJI drones.

## Installation
```console
python -m pip install --index-url https://test.pypi.org/simple/ --extra-index-url https://pypi.org/simple drone-survey
```

## Usage
More information to come...
```python
import drone_survey
import numpy as np
from pupil_apriltags import Detector
import pyproj
import json

# DJI Mini 4 Pro at 4K/100FPS (slow-mo)
d = drone_survey.DroneData(
    flight_csv_file="...", # AirData flight data CSV file path
    video_files=["..."], # video file paths
    fps=100, # video FPS
    resolution=(3840, 2160), # video resolution (width, height)
    compass_heading=0, # compass heading (degrees) [0deg is North, 90deg is East, etc...]
    altitude=None, # drone altitude (m). If None, uses sonar altitude from CSV file
    gimbal_pitch=-90, # gimbal pitch (degrees)
    f_x=2.9099e+03, # focal length x (pixels)
    f_y=2.9099e+03, # focal length y (pixels)
    c_x=1920, # optical center x (pixels)
    c_y=1080, # optical center y (pixels)
    distortion=np.array([0.0705, -0.1303, 0, 0, 0]), # lens distortion OpenCV coefficients
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
    showCharts=True, # show charts while syncing
    debug=True, # show debug information
)

data = d.sync(["..."]) # optionally specify cache file location, one for each video
with open("sync.json", "w") as json_file:
    json.dump(data, json_file, separators=(',', ':'))

# data = d.detect(0) # detect in the first video without analyzing the results
# with open("detections.json", "w") as json_file: # write detections to JSON file
#     json.dump(data, json_file)

data = d.analyze(0) # analyze the first video
with open("output.json", "w") as json_file: # write output to JSON file
    json.dump(data, json_file, separators=(',', ':'))

d.close()
```

## Acknowledgements
This project makes use of the following open-source Python libraries:

- [NumPy](https://numpy.org/) – BSD License
- [OpenCV-Python](https://opencv.org/) – BSD License
- [Matplotlib](https://matplotlib.org/) – PSF-based License
- [SciPy](https://scipy.org/) – BSD License
- [PyProj](https://pyproj4.github.io/pyproj/stable/) – MIT License
- [pupil_apriltags](https://github.com/pupil-labs/apriltags) – MIT License