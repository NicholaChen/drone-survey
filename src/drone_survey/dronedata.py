# written by NC :P

import math
import csv
import numpy as np
import os
import cv2
from matplotlib import pyplot as plt
import pyproj
from pupil_apriltags import Detector
from scipy.interpolate import CubicSpline
from scipy.signal import correlate, correlation_lags


_RED = '\033[31m'
_GREEN = '\033[32m'
_BLUE = '\033[34m'
_YELLOW = '\033[33m'
_CYAN = '\033[36m'
_MAGENTA = '\033[35m'
_WHITE = '\033[37m'
_BLACK = '\033[30m'
_BOLD = '\033[1m'
_ITALIC = '\033[3m'
_UNDERLINE = '\033[4m'
_RESET = '\033[0m'


class DroneData:
    def __init__(self, flight_csv_file: str, 
                 video_files: list[str], 
                 fps: float = 29.97, 
                 resolution: tuple[int, int] = (3840, 2160),
                 compass_heading: float = 0.0,
                 altitude: float = 2.0,
                 gimbal_pitch: float = -90.0,
                 f_x: float = 2000.0,
                 f_y: float = 2000.0,
                 c_x: float = 1920.0,
                 c_y: float = 1080.0,
                 distortion: np.ndarray = np.array([0.0, 0.0, 0.0, 0.0, 0.0]),
                 max_missing_frames: int = 100,
                 timestep: float = 0.001,
                 AprilTagDetector: Detector = Detector(
                    families="tag36h11",
                    debug=0
                 ),
                 pyprojTransformer: pyproj.Transformer = pyproj.Transformer.from_crs(pyproj.CRS("EPSG:4326"), pyproj.CRS("EPSG:3158"), always_xy=True), # WGS84 to UTM zone 14N
                 showVideo: bool = False,
                 showCharts: bool = True,
                 debug: bool = False
                 ):
        """
        Initializes the DroneData class with flight data from a CSV file and video files.
        """

        self.flight_csv_file = flight_csv_file.replace("\\", "/")
        self.video_files = [video_file.replace("\\", "/") for video_file in video_files]
        self.fps = fps
        self.compass_heading = compass_heading % 360.0
        self.altitude = altitude
        self.gimbal_pitch = gimbal_pitch
        self.max_missing_frames = max_missing_frames
        self.timestep = timestep
        self.resolution = resolution

        self.f_x = f_x
        self.f_y = f_y
        self.c_x = c_x
        self.c_y = c_y

        self.distortion = distortion

        self.K = np.array(
            [[self.f_x, 0, self.c_x],
             [0, self.f_y, self.c_y],
             [0, 0, 1]]
        )
        self.optimal_camera_matrix, self.roi = cv2.getOptimalNewCameraMatrix(self.K, self.distortion, self.resolution, 1, self.resolution)
        self.map1, self.map2 = cv2.initUndistortRectifyMap(self.K, self.distortion, None, self.optimal_camera_matrix, self.resolution, cv2.CV_16SC2)

        self.optimal_camera_matrix_inv = np.linalg.inv(self.optimal_camera_matrix)

        self.AprilTagDetector = AprilTagDetector
        self.pyprojTransformer = pyprojTransformer

        self.debug = debug
        self.showCharts = showCharts
        self.showVideo = showVideo

        if not os.path.exists(flight_csv_file):
            raise Exception(f"{_RED}Flight CSV file does not exist: {flight_csv_file}{_RESET}")
        flight_csv_file_reader = csv.DictReader(open(flight_csv_file, "r"))

        last = -1
        lastVideo = "0"

        timestamps = []
        x = []
        y = []

        self.video_start_estimates = [] # used to estimate the start of videos from flight data
        for row in flight_csv_file_reader:
            if int(row["time(millisecond)"]) != last:
                last = int(row["time(millisecond)"])
                timestamps.append(int(row["time(millisecond)"]) / 1000)

                lat = float(row["latitude"])
                lon = float(row["longitude"])

                _x, _y = self.pyprojTransformer.transform(lon, lat)
                
                x.append(_x)
                y.append(_y)

                if row["isVideo"] == "1" and lastVideo == "0": # checks when the video starts (i.e. when isVideo changes from 0 to 1)
                    self.video_start_estimates.append(int(row["time(millisecond)"]) / 1000)
                lastVideo = row["isVideo"]

        x = np.array(x)
        y = np.array(y)

        self.timestamps = np.array(timestamps)
        self.x = CubicSpline(self.timestamps, x)
        self.y = CubicSpline(self.timestamps, y)

        print(f"{_GREEN}Flight CSV File data loaded successfully!{_RESET}")
    
    def sync(self, video_cache_files : list[str] = None):
        '''
        Sync video files with flight data.
        '''

        print(f"{_YELLOW}Syncing videos with flight data{_RESET}")
        if self.debug:  
            print(f"Video start estimates: {self.video_start_estimates}")
        
        video = 0

        self.video_starts = [] # start timestamps for each video (seconds since takeoff)
        self.calibration_finished = [] # length of calibration in seconds for each video
        self.caps = []
        for video_file in self.video_files:
            cap = cv2.VideoCapture(video_file)
            self.caps.append(cap)
            if not cap.isOpened():
                raise Exception(f"{_RED}Could not open video file: {video_file}{_RESET}")
    
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 5)
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))


            video_file_name = os.path.splitext(os.path.basename(video_file))[0]
            video_file_name = "." + video_file_name + ".csv"
            if video_cache_files is not None and video_cache_files is list[str] and video_cache_files[video] is not None and video < len(video_cache_files):
                video_cache_csv_path = video_cache_files[video].replace("\\", "/")
            else:
                video_cache_csv_path = os.path.join(os.path.dirname(video_file), video_file_name).replace("\\", "/")

            print(f"{_BLUE}Syncing video: {video_file}{_RESET}")

            if self.debug:
                print(f"Video cache CSV path: {video_cache_csv_path}")
            missing_frames = 0
            if os.path.exists(video_cache_csv_path):
                with open(video_cache_csv_path, 'r') as f:
                    if self.debug:
                        print(f"Reading video cache CSV: {video_cache_csv_path}")
                    reader = csv.reader(f)
                    april_tags = {}
                    for row in reader:
                        if len(row) < 3:
                            continue
                        if row[0] == '' or row[1] == '' or row[2] == '':
                            missing_frames += 1
                            continue
                        missing_frames = 0
                        april_tags[int(row[0])] = (float(row[1]), float(row[2]))
            else:
                april_tags = {}

            cap.set(cv2.CAP_PROP_POS_FRAMES, max(april_tags.keys(),default=-1) + 1)
            
            print(f"{_YELLOW}Processing video file: {video_file}{_RESET}")

            video_started = False


            if max(april_tags.keys(),default=-1) - 1 < frame_count and self.showVideo and missing_frames < self.max_missing_frames:
                video_started = True
                print(f"Total frames in video: {frame_count}")
                cv2.namedWindow('Drone Footage', cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_NORMAL)

            cancelled = False

            while True:
                if missing_frames >= self.max_missing_frames:
                    break
                ret, frame = cap.read()
                frame_number = int(cap.get(cv2.CAP_PROP_POS_FRAMES)) - 1

                if frame_number >= frame_count:
                    break
                if not ret:
                    raise Exception(f"{_RED}Could not read frame from video file: {video_file}{_RESET}")   

                if self.debug:
                    print(f"{_CYAN}Checking for AprilTags... {_BOLD}[{frame_number + 1}/{frame_count}]{_RESET}", end="")
                else:
                    print(f"{_CYAN}Checking for AprilTags... {_BOLD}[{frame_number + 1}/{frame_count}]{_RESET}")

                gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                d = self.AprilTagDetector.detect(gray_frame)
                if d and d[0].tag_id == 0:
                    for d_ in d:
                        if d_.tag_id == 0:
                            missing_frames = 0
                            point = cv2.undistortPoints(np.array([d_.center], dtype=np.float32), self.K, self.distortion, P=self.optimal_camera_matrix)[0][0]

                            if self.debug:
                                print(" ", f"Detected calibration AprilTag. ({(point[0]-frame.shape[1] / 2) * -1}, {(point[1]-frame.shape[0] / 2)})")

                            with open(video_cache_csv_path, "a") as f:
                                f.write(f"{frame_number},{(point[0]-frame.shape[1] / 2) * -1},{(point[1]-frame.shape[0] / 2)}\n")

                            april_tags[frame_number] = ((point[0]-frame.shape[1] / 2) * -1,(point[1]-frame.shape[0] / 2))

                            break
                else:
                    missing_frames += 1
                    if self.debug:
                        print(" ", f"{_RED}No AprilTags detected. {_BOLD}[{missing_frames}/{self.max_missing_frames}]{_RESET}")

                    with open(video_cache_csv_path, "a") as f:
                        f.write(f"{frame_number},,\n")

                    if missing_frames >= self.max_missing_frames: # Stops if two many frames are missing AprilTags in a row
                        print(f"{_RED}Too many missing frames, stopping detection.{_RESET}")
                        break
                
                if self.showVideo:
                    cv2.imshow('Drone Footage', frame)
                    key = cv2.waitKey(1) & 0xFF

                    if key == ord('q'):
                        print(f"{_YELLOW}Cancelling video processing...{_RESET}")
                        cancelled = True
                        break

                if frame_number >= frame_count - 1:
                    break

            if self.showVideo and video_started:
                cv2.destroyWindow('Drone Footage')
            if cancelled:
                raise Exception(f"{_RED}Cancelled processing video file: {video_file}{_RESET}")
            
            print(f"{_GREEN}AprilTag detection completed for video: {video_file}{_RESET}")

            aprilTags_t = np.array(list(april_tags.keys())) / self.fps

            _c = math.cos(math.radians(-self.compass_heading))
            _s = math.sin(math.radians(-self.compass_heading))
            if april_tags:
                coords = np.array(list(april_tags.values()))
                __x = coords[:, 0]
                __y = coords[:, 1]
                aprilTags_x = __x * _c - __y * _s
                aprilTags_y = __x * _s + __y * _c

            if len(aprilTags_t) >= 2:
                self.aprilTags_x = CubicSpline(aprilTags_t, aprilTags_x)
                self.aprilTags_y = CubicSpline(aprilTags_t, aprilTags_y)
            else:
                raise Exception(f"{_RED}Not enough AprilTags detected in video: {video_file}{_RESET}")
            flight_csv_interpolated_t = np.arange(self.video_start_estimates[video], min(self.video_start_estimates[video] + aprilTags_t[-1], self.timestamps[-1]), self.timestep)
            video_interpolated_t = np.arange(0, aprilTags_t[-1], self.timestep)

            self.x_derivative = self.x.derivative()
            self.y_derivative = self.y.derivative()
            self.aprilTags_x_derivative = self.aprilTags_x.derivative()
            self.aprilTags_y_derivative = self.aprilTags_y.derivative()

            flight_positions = np.array([self.x(flight_csv_interpolated_t) - self.x(self.video_start_estimates[video]), self.y(flight_csv_interpolated_t) - self.y(self.video_start_estimates[video])])  # subtracts the first value to make the starting home point (0,0)
            aprilTag_positions = np.array([self.aprilTags_x(video_interpolated_t), self.aprilTags_y(video_interpolated_t)])

            flight_positions_dot = np.array([self.x_derivative(flight_csv_interpolated_t), self.y_derivative(flight_csv_interpolated_t)])
            aprilTag_positions_dot = np.array([self.aprilTags_x_derivative(video_interpolated_t), self.aprilTags_y_derivative(video_interpolated_t)])
        
            max_flight_positions_dot_0 = np.max(np.abs(flight_positions_dot[0]))
            max_flight_positions_dot_1 = np.max(np.abs(flight_positions_dot[1]))
            max_aprilTag_positions_dot_0 = np.max(np.abs(aprilTag_positions_dot[0]))
            max_aprilTag_positions_dot_1 = np.max(np.abs(aprilTag_positions_dot[1]))

            x_correlation = correlate(flight_positions_dot[0] / max_flight_positions_dot_0, aprilTag_positions_dot[0] / max_aprilTag_positions_dot_0, mode='full')
            y_correlation = correlate(flight_positions_dot[1] / max_flight_positions_dot_1, aprilTag_positions_dot[1] / max_aprilTag_positions_dot_1, mode='full')
            correlation = (x_correlation / np.max(x_correlation)) + (y_correlation / np.max(y_correlation))


            lags = correlation_lags(len(flight_positions_dot[0]), len(aprilTag_positions_dot[0]), mode='full') * self.timestep + self.video_start_estimates[video]
            self.video_starts.append(lags[np.argmax(correlation)])
            self.calibration_finished.append(aprilTags_t[-1])
            print(f"{_GREEN}{self.video_starts[video]} seconds for video {self.video_files[video]}{_RESET}")

            if self.showCharts:
                fig, axs = plt.subplots(2, 1, figsize=(10, 10))
                axs[0].plot(flight_csv_interpolated_t, flight_positions[0], label='Flight X', color='b')
                axs[0].plot([],[], label='Video X', color='r') # Placeholder for Video X
                axs[0].set_title('X Coordinates')
                axs[1].set_xlabel('Time (s)')
                axs[0].set_ylabel('Flight X Coordinate (m)')
                axs[0].legend()

                axs_1 = axs[0].twinx()
                axs_1.plot(video_interpolated_t + self.video_starts[video], aprilTag_positions[0], label='Video X', color='r')
                axs_1.set_ylabel('Video X Coordinate (px)')


                axs[1].plot(flight_csv_interpolated_t, flight_positions[1], label='Flight Y', color='b')
                axs[1].plot([],[], label='Video Y', color='r')  # Placeholder for Video Y
                axs[1].set_title('Y Coordinates')
                axs[1].set_xlabel('Time (s)')
                axs[1].set_ylabel('Flight Y Coordinate (m)')
                axs[1].legend()

                axs_2 = axs[1].twinx()
                axs_2.plot(video_interpolated_t + self.video_starts[video], aprilTag_positions[1], label='Video Y', color='r')
                axs_2.set_ylabel('Video Y Coordinate (px)')


                fig.tight_layout()
                fig.show()



                fig1, axs1 = plt.subplots(2, 1, figsize=(10, 10))
                axs1[0].plot(flight_csv_interpolated_t, flight_positions_dot[0], label='Flight X', color='b')
                axs1[0].plot([],[], label='Video X', color='r') # Placeholder for Video X
                axs1[0].set_title('X Velocity')
                axs1[1].set_xlabel('Time (s)')
                axs1[0].set_ylabel('Flight X Velocity (m/s)')
                axs1[0].legend()

                ax1_1 = axs1[0].twinx()
                ax1_1.plot(video_interpolated_t + self.video_starts[video], aprilTag_positions_dot[0], label='Video X', color='r')
                ax1_1.set_ylabel('Video X Velocity (px/s)')


                axs1[1].plot(flight_csv_interpolated_t, flight_positions_dot[1] / max_flight_positions_dot_1, label='Flight Y', color='b')
                axs1[1].plot([],[], label='Video Y', color='r')  # Placeholder for Video Y
                axs1[1].set_title('Y Velocity')
                axs1[1].set_xlabel('Time (s)')
                axs1[1].set_ylabel('Flight Y Velocity (m/s)')
                axs1[1].legend()

                ax1_2 = axs1[1].twinx()
                ax1_2.plot(video_interpolated_t + self.video_starts[video], aprilTag_positions_dot[1], label='Video Y', color='r')
                ax1_2.set_ylabel('Video Y Velocity (px/s)')


                fig1.tight_layout()
                fig1.show()



                fig3, axs3 = plt.subplots(3, 1, figsize=(10, 10))

                axs3[0].plot(lags, x_correlation / np.max(x_correlation), label='X Correlation', color='b')
                axs3[0].set_title('X Correlation')
                axs3[0].set_xlabel('Video start (s)')
                axs3[0].set_ylabel('Correlation Coefficient')
                axs3[0].legend()
                axs3[1].plot(lags, y_correlation / np.max(y_correlation), label='Y Correlation', color='b')
                axs3[1].set_title('Y Correlation')
                axs3[1].set_xlabel('Video start (s)')
                axs3[1].set_ylabel('Correlation Coefficient')
                axs3[1].legend()
                axs3[2].plot(lags, correlation / 2, label='Correlation', color='b')
                axs3[2].set_title('Correlation')
                axs3[2].set_xlabel('Video start (s)')
                axs3[2].set_ylabel('Correlation Coefficient')
                axs3[2].axvline(self.video_starts[video], color='r', linestyle='--', label='Best Video Start Time')
                axs3[2].axvline(self.video_start_estimates[video], color='g', linestyle=':', label='Video Start Time Estimate')
                axs3[2].legend()

                fig3.tight_layout()
                fig3.show()

                plt.show()
            video += 1

        print(f"Video start times: {self.video_starts}")

    def get_pixel_from_position(self, video: int, timestamp: float, position: tuple[float, float]):
        R = from_euler_zxy(-self.compass_heading, self.gimbal_pitch - 90, 0)

        Cw = np.array([
            self.x(timestamp + self.video_starts[video]),
            self.y(timestamp + self.video_starts[video]),
            self.altitude
        ]).reshape(3, 1)

        Pw = np.array([position[0], position[1], 0]).reshape(3, 1)

        Pc = R @ (Pw - Cw)

        if Pc[2, 0] <= 0:
            raise ValueError("Point is behind the camera (Z <= 0 in camera frame).")

        uv_h = self.optimal_camera_matrix @ Pc
        u = uv_h[0, 0] / uv_h[2, 0]
        v = uv_h[1, 0] / uv_h[2, 0]

        return (u, v)


    def get_position_from_pixel(self, video: int, timestamp: float, pixel: tuple[float, float]):
        R = from_euler_zxy(-self.compass_heading, self.gimbal_pitch - 90, 0)

        uv1 = np.array([pixel[0], pixel[1], 1.0]).reshape(3,1)
        ray_cam = self.optimal_camera_matrix_inv @ uv1

        Z_world = 0.0
        cam_center_world = np.array([
            self.x(timestamp + self.video_starts[video]),
            self.y(timestamp + self.video_starts[video]),
            self.altitude
        ]).reshape(3,1)

        ray_world = R.T @ ray_cam

        scale = (Z_world - cam_center_world[2,0]) / ray_world[2,0]
        Pw = cam_center_world + scale * ray_world

        return Pw.flatten() 
    
    def analyze(self, video: int, function: callable = None):
        '''
        Analyze the video frame by frame.
        '''
        cap = self.caps[video]

        if not cap.isOpened():
            raise Exception(f"{_RED}Could not open video file: {self.video_files[video]}{_RESET}")
        
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(self.calibration_finished[video] * self.fps) + 1)
        if self.showVideo:
            cv2.namedWindow('Drone Footage', cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)

        if self.showCharts:
            fig, ax = plt.subplots(figsize=(8, 6),ncols=1)
            fig.set_dpi(144)
            line, = ax.plot([],[], color="#00000000")
            scat = ax.scatter([], [], c='Red', s=100, alpha=0.5)
            ax.set_xlabel("World X")
            ax.set_ylabel("World Y")
            ax.set_title("Live Detections")

            all = []

            frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

            all_x = []
            all_y = []

            fig.tight_layout()
            fig.canvas.draw()
            mg_plot = np.array(fig.canvas.renderer.buffer_rgba())
            live_detections_img = cv2.cvtColor(mg_plot,cv2.COLOR_RGB2BGR)
            cv2.namedWindow("Live Detections", cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
            cv2.imshow("Live Detections", live_detections_img)

        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            frame_number = int(cap.get(cv2.CAP_PROP_POS_FRAMES)) - 1

            timestamp = frame_number / self.fps

            if self.debug:
                print(f"{_CYAN}Analyzing frame... {_BOLD}[{frame_number + 1}/{frame_count}]{_RESET}", end="")
            else:
                print(f"{_CYAN}Analyzing frame... {_BOLD}[{frame_number + 1}/{frame_count}]{_RESET}")
            dst = cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)
            x, y, w, h = self.roi
            dst = dst[y:y+h, x:x+w]
            
            if function:
                detections = function(dst)
            else:
                gray_frame = cv2.cvtColor(dst, cv2.COLOR_RGB2GRAY)
                detections_ = self.AprilTagDetector.detect(gray_frame)

                detections = [{"x": d.center[0], "y": d.center[1], "id": d.tag_id} for d in detections_]
                
            
            for d in detections:
                world = self.get_position_from_pixel(video, timestamp, (d['x'], d['y']))
                d["world_x"] = world[0]
                d["world_y"] = world[1]

                if self.showCharts:
                    all_x.append(world[0])
                    all_y.append(world[1])
            
            detection = {
                "timestamp": timestamp + self.video_starts[video],
                "frame": frame_number,
                "detections": detections
            }

            if self.debug:
                print(" ", detection)

            all.append(detection)

            if self.showCharts:
                win_w = cv2.getWindowImageRect("Live Detections")[2]
                win_h = cv2.getWindowImageRect("Live Detections")[3]

                dpi = fig.get_dpi()
                fig.set_size_inches(win_w / dpi, win_h / dpi, forward=True)

                line.set_data(all_x, all_y)
                scat.set_offsets(np.c_[all_x, all_y])
                scat.set_sizes([100] * len(all_x))
                ax.relim()
                ax.autoscale_view()
                fig.canvas.draw()

                mg_plot = np.array(fig.canvas.renderer.buffer_rgba())

                live_detections_img = cv2.cvtColor(mg_plot,cv2.COLOR_RGB2BGR)
                live_detections_img = cv2.resize(live_detections_img, (win_w, win_h), interpolation=cv2.INTER_LINEAR)
                cv2.imshow("Live Detections", live_detections_img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
        
            if self.showVideo:
                cv2.imshow('Drone Footage', dst)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
            if frame_number >= frame_count - 1:
                break

        return all


def from_euler_zxy(z, x, y):

    z, x, y = np.radians([z, x, y])

    cz, sz = np.cos(z), np.sin(z)
    cx, sx = np.cos(x), np.sin(x)
    cy, sy = np.cos(y), np.sin(y)


    Rz = np.array([[cz, -sz, 0],
                   [sz,  cz, 0],
                   [ 0,   0, 1]])

    Rx = np.array([[1,  0,   0],
                   [0, cx, -sx],
                   [0, sx,  cx]])

    Ry = np.array([[ cy, 0, sy],
                   [  0, 1,  0],
                   [-sy, 0, cy]])

    R = Rz @ Ry @ Rx
    return R