## Goal
Covert verical flow guages from a webcam feed into numerical representation.

## Features
- Read frames from an RTSP video stream
- Process each frame to compute gauge percentages
- Publish results to an MQTT topic at configurable intervals
- Command‑line interface for easy configuration
![input](calibration/undistorted/ch0-1765513908.jpg)
![output](result.png)

## Usage

Run the script with required arguments:

```bash
python FlowMeterReader.py --rtsp <RTSP_URL> --broker <MQTT_BROKER> --topic <MQTT_TOPIC> [--interval <SECONDS>] [--port <PORT>]
```

Arguments:
- `--rtsp` : RTSP stream URL (required)
- `--broker` : MQTT broker address (required)
- `--topic` : MQTT topic to publish gauge percentages (required)
- `--interval` : Seconds between processing frames (default 1.0)
- `--port` : MQTT broker port (default 1883)

The script reads frames from the RTSP stream, processes them to compute gauge percentages, and publishes a JSON payload like:
```json
[{"id":0,"percent":12.3},{"id":1,"percent":45.6}]
```

## Dependencies

- OpenCV (`cv2`)
- NumPy
- Matplotlib
- qreader
- paho-mqtt

Ensure these packages are installed via `pip install opencv-python numpy matplotlib qreader paho-mqtt`.