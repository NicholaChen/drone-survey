# drone-survey

## Overview
Affordable drone surveying method using free tools and low cost DJI drones.

## Abstract
Drone surveying is being used extensively in topographic mapping, agriculture, and ecological monitoring. This technology is dominated by DJI Enterprise drones which employ Global Navigation Satellite System (GNSS) sensors and use Post Processed Kinematics (PPK) or Real Time Kinematics (RTK) to pinpoint locations with centimetre precision. However, this technology is unavailable to cheaper drones. To overcome this, our research implements the use of an affordable drone, the DJI Mini 4 Pro, with free services to achieve surveying capabilities without the overhead costs associated with DJI Enterprise drones. Our implementation uses Litchi Mission Hub and DJI Fly to fly pre program flight paths while recording video footage and flight logs. These flight logs were decrypted using AirData and combined with the video footage in our custom library that allowed for the footage to be analyzed with GPS coordinates from the flight logs. To test the accuracy, our library was used to calculate the positions of specific visual markers (AprilTags), which were then compared with their real-world placement. Our research shows that affordable consumer drones with free software can be used as an alternative for small scale surveying, expanding the accessibility of drone surveying to those with limited resources. 

## Installation
```bash
python -m pip install --index-url https://test.pypi.org/simple/ --extra-index-url https://pypi.org/simple drone-survey==0.0.2
```

## How to use

## Usage
```python
import drone_survey

# Example usage
```
