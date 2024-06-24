# Line Follower Tracker

## Compiling the program

### Obtaining dependencies

This is automated using a shell script `setup.sh` which can be found in the `.devcontainer` folder. Please refer this script for learning more about the steps involved.

#### Required libraries

- OpenCV 4.9.0
- OpenCV modules 4.x

Make sure that these libraries and their dependencies are downloaded, compiled and installed on the system.

### Building the project

The program uses CMake build system, so ensure that CMake and a C++ compiler (gcc) are installed. Start a terminal in the project root directory, create a `build/` folder and navigate to that directory.

```shell
mkdir build && cd build
```

Inside the `build/` folder, create the CMake build system and compile the project.

```shell
cmake .. && make -j$(nproc)
```

The resulting executable `line-follower-tracker` within the same folder is now ready to use.

## Description of CLI option flags

A list of available options with brief descriptions and default values can be viewed by simply running the program without any flags.

```shell
./line-follower-tracker
```

Expected output:

```text
A configuration file is required for tracking.
Line follower tracker using ArUco markers
Usage: line-follower-tracker [params]

        --calibration (value:false)
                Enable calibration mode
        --config
                Configuration file path for the program
        --output
                Output file directory for a new configuration file
```

### `--calibration` : Enable calibration mode

The program has two modes of operation: Tracking and Calibration. The mode can be set using the `--calibration` flag which is either `true` or `false`.

#### Tracking mode

Tracking mode is the main function of the program which tracks, records and evaluates the performance of line followers visually using fiducial markers.

#### Calibration mode

Calibration mode is used for obtaining the intrinsic parameters of the camera. The obtained parameters are then used for pose estimation of fiducial markers in tracking mode.

> Overall accuracy of the program is dependent on this step.

### `--config` : Configuration file path

Read more in the **Configuration** section

### `--output` : Output file path

Optional, depends on the mode:

#### In tracking mode

If specified, recording is enabled and the path specified is used to save the results (CSV file). Else recording is disabled and nothing is saved.

#### In calibration mode

If not specified and a configuration file is given, the intrinsic and distortion parameters obtained from calibration are overwritten to the given configuration file.

If specified along with a configuration file, a new copy of the configuration file with the intrinsic and distortion parameters obtained from calibration is saved to output path.

If specified without a configuration file, a new template configuration file with default parameters is saved to the output path.

## Configuration

The program uses a YAML file to read and store tracking and calibration parameters. Configuration file is needed to provide marker specifications for tracking and calibration modes and for saving the intrinsic parameters from calibration. It needs to be generated and customized for the first time.

### Configuration file structure

```YAML
marker_detection:
   input_source_path: String - Relative/Absolute path to video file
   camera_id: Integer
   show_rejected_markers: Boolean integer (0/1)

line_follower_marker:
   marker_side_meters: Float
   marker_id: Integer
   marker_dictionary_id: Integer - As per aruco::PREDEFINED_DICTIONARY_NAME enum

board_markers:
   marker_side_meters: Float
   marker_seperation_meters_x: Float
   marker_seperation_meters_y: Float
   marker_ids: [ Array of integers ]
   marker_dictionary_id: Integer - As per aruco::PREDEFINED_DICTIONARY_NAME enum

camera_calibration:
   marker_side_meters: Float
   square_side_meters: Float
   squares_quantity_x: Integer
   squares_quantity_y: Integer
   marker_dictionary_id: Integer - As per aruco::PREDEFINED_DICTIONARY_NAME enum
   camera_matrix: !!opencv-matrix
      rows: Integer (Auto-generated)
      cols: Integer (Auto-generated)
      dt: Character (Auto-generated)
      data: [ Array of floats ] (Auto-generated)
   distortion_coefficients: !!opencv-matrix
      rows: Integer (Auto-generated)
      cols: Integer (Auto-generated)
      dt: Character (Auto-generated)
      data: [ Array of floats ] (Auto-generated)
```

> Only the fields that are not auto-generated should be manually entered or modified.

### Generating a template configuration file

Start the program in calibration mode without a configuration file and with an `--output` flag to specify the output directory to which the template configuration file is saved.

```shell
./line-follower-tracker --calibration=true --output=<Desired Directory>
```

The expected output in terminal should be

```text
Creating a new config file in the output directory...
Config file created: <Desired Directory>/config-YYYY-MM-DD-HH-MM-SS.yaml
```

### Steps to customize the configuration file

The board on which the line-follower track is laid requires four aruco markers at each corner to establish a reference coordinate system. The marker specifications such as dictionary, size and the vertical/horizontal spacing between them needs to be specified in the YAML configuration file under the `board_markers` section.

The specifications of the marker used on the line follower also needs to be provided in the configuration file in the `line_follower_markers` section.

This program uses a ChArUco board for calibrating the camera. Parameters of the ChArUCo board such as marker size, dictionary and the layout needs to set in the `camera_calibration` section.

After setting the marker specifications used for the track board, line follower and the calibration board, the next step is to calibrate the camera.

## Camera calibration

Start the program in calibration mode with the customized configuration file. `--output` flag can be used to either save a new copy of configuration file or overwrite the existing one with calibration results.

```shell
./line-follower-tracker --calibration=true --config=<Path to configuration file> --output=<Path to directory for saving a copy>
```

The remaining procedure for camera calibration is as per the [OpenCV documentation](https://docs.opencv.org/4.x/da/d13/tutorial_aruco_calibration.html#:~:text=Calibration%20with%20ChArUco%20Boards).

In short, keep varying the pose of the ChArUco board within the camera view and press the `C` key to capture frames and `Esc` key when done. It is recommended to capture between 25-30 frames.

The final configuration file will have the `camera_matrix` and `distortion_coefficients` parameters in the `camera_calibration` section populated with non-zero values.

## Tracking

Start the program in tracking mode with the configuration file and optionally an output directory to record and save the results.

```shell
./line-follower-tracker --config=<Path to configuration file> --output=<Path to results directory>
```

A small window should pop up with the camera view and visualization of detected markers.

### Closing the program

Hit the `Esc` key while the pop up window is selected to exit the program. Another way is to press `Crtl + C` in the terminal from which the program is executed.

> Clicking the close button on the window does not terminate the program which is expected.

### Make sure that

- The camera is centered, close enough to the track board (it must fill the camera view) without any significant tilt.

- At least three out of four corner markers of the board with track are in the camera view and unobstructed to get the pose of origin for the reference coordinate system.

- There are no unused markers in the camera view from the same dictionaries of board and line follower markers.
