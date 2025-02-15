# ZED-F9P GNSS Module Console Application

This is a console application that connects to the u-blox [ZED-F9P Precision GNSS Module](./timages/IMG_20201119_194246.jpg) via a serial interface.
The application configures the module to operate as either an NTRIP Client or Station.

## Features
- **NTRIP Station Mode**: When configured as a Station, the ZED-F9P enters survey mode. Upon completion, it transmits NTRIP data to the caster platform.
- **NTRIP Client Mode**: When configured as a Client, the application receives NTRIP data, converts it to an NMEA sequence, and combines it with additional GPS data from the u-blox device.

## Compile for Linux/Raspberry Pi
To compile the application for Linux/Raspberry Pi environment (32/64-bit), use the following commands:

dotnet publish -c Release --self-contained -r linux-arm  
dotnet publish -c Release --self-contained -r linux-arm64  

## Parameters

- `-p`, `--com-port` [Port]  
  COM port (e.g., COM#, ttyACM#)  
  **Required**

- `-s`, `--ntrip-server` [rtk2go.com]  
  NTRIP caster server (default: rtk2go.com)

- `-t`, `--ntrip-port` [2101]  
  NTRIP caster port (default: 2101)

- `-m`, `--ntrip-mountpoint` [MountPoint]  
  NTRIP caster mountpoint

- `-w`, `--ntrip-password` [Password]  
  NTRIP caster password

- `-a`, `--rtcm-accuracy-req` [3.000]  
  Minimum RTCM accuracy to complete survey (in meters) (default: 3.000)

- `-y`, `--rtcm-survey-time` [60]  
  Minimum time required to complete survey (in seconds) (default: 60)

- `-o`, `--mode` [Idle/Server/Client]  
  Operation mode for the driver (default: Idle)

- `-n`, `--nmea-socket` [SocketPath]  
  Path to the NMEA socket

- `-r`, `--rtcm-socket` [SocketPath]  
  Path to the RTCM socket

- `-x`, `--sync-socket` [SocketPath]  
  Path to the sync socket

- `-d`, `--debug`  
  Enable debug mode (default: false)

## Running the Application

### Launching on Raspberry Pi
To run the application on Raspberry Pi, use the following commands:

/home/pi/gps-station/f9p/Zedf9p -p /dev/ttyACM0 -s [CASTER_SERVER(rtk2go.com)] -t 2101 -m [MOUNT_POINT] -w [PASSWORD] -a 3.000 -y 60 -o Server -n /tmp/zed-f9p-nmea-data.sock -r /tmp/zed-f9p-rtcm-data.sock -x /tmp/zed-f9p-sync-data.sock -d  

or  

/home/pi/gps-station/f9p/Zedf9p --com-port /dev/ttyACM0 --ntrip-server [CASTER_SERVER(rtk2go.com)] --ntrip-port 2101 --ntrip-mountpoint [MOUNT_POINT] --ntrip-password [PASSWORD] --rtcm-accuracy-req 3.000 --rtcm-survey-time 60 --mode Server --nmea-socket /tmp/zed-f9p-nmea-data.sock --rtcm-socket /tmp/zed-f9p-rtcm-data.sock --sync-socket /tmp/zed-f9p-sync-data.sock --debug  

## License
MIT License.