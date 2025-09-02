import drone_survey
import numpy as np
from pupil_apriltags import Detector

d = drone_survey.DroneData(
    flight_csv_file="...", # AirData csv file path
    video_files=["..."], # video file paths
    fps=100,
    altitude=2,
    max_missing_frames=200,
    compass_heading=239.4,
    timestep=0.001,
    f_x=2.9099e+03,
    f_y=2.9099e+03,
    distortion=np.array([0.0705, -0.1303, 0, 0, 0]),
    AprilTagDetector=Detector(
        families="tag36h11",
        nthreads=12,
        debug=0
    ),
    debug=True,
    showVideo=False
)

d.sync(["..."]) # optionally specify cache file location, one for each video in the list
d.showVideo = True
data = d.analyze(0)