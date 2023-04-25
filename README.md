# Hardware

Needs ESP32 Cam and IR sensor for hardware installation.

## Installation

- Clone Repo
  
  ```bash
  git clone https://github.com/SmartWasteSegregatorAndRoutePlanner/hardware.git
  ```
  
- Open Sketch in Arduino IDE.

- Update below parameters according to the backend api server configuration.
  - `ssid`
  - `password`
  - `serverName`
  - `serverPort`
  
- Upload updated `main.ino` file to esp32 cam using ftdi module.
