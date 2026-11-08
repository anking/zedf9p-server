# ZED-F9P GNSS Module Console Application

This is a console application that connects to the u-blox [ZED-F9P Precision GNSS Module](./images/IMG_20201119_194246.jpg) via a serial interface and connects it to an NTRIP caster.

- **Station mode (base):** surveys its position, or uses a fixed one, then streams RTCM corrections to the caster.
- **Client mode (rover):** receives RTCM corrections from the caster, passes them to the module and reports the position and RTK status.
- **Idle mode:** reports the position only.

It is driven by [gps-station](https://github.com/anking/gps-station) and [gps-rider](https://github.com/anking/gps-rider), which start it and listen on the sync socket.

## Requirements

.NET 10 SDK to build. Builds are self-contained, so a Raspberry Pi needs nothing installed.

```sh
dotnet publish Zedf9p/Zedf9p.csproj -c Release --self-contained -r linux-arm64
dotnet publish Zedf9p/Zedf9p.csproj -c Release --self-contained -r linux-arm
```

Tagging a commit `v*` builds both and attaches `linux-arm64.zip` and `linux-arm.zip` to a GitHub release.

## Parameters

| Parameter | Meaning |
| --- | --- |
| `-p`, `--com-port` | Serial port, such as `/dev/ttyACM0`. Required. |
| `-o`, `--mode` | `Idle` (default), `Station` or `Client`. |
| `-s`, `--ntrip-server` | Caster host. Default `rtk2go.com`. |
| `-t`, `--ntrip-port` | Caster port. Default `2101`. |
| `-m`, `--ntrip-mountpoint` | Caster mountpoint. Required in Station and Client mode. |
| `-w`, `--ntrip-password` | Station mode: the mountpoint password (required). Client mode: optional. |
| `-u`, `--ntrip-username` | Client mode: the caster user name. **RTK2go requires a valid email address.** |
| `-a`, `--rtcm-accuracy-req` | Survey: accuracy to reach, in meters. Default `3`. |
| `-y`, `--rtcm-survey-time` | Survey: minimum time, in seconds. Default `60`. |
| `-f`, `--fixed-position` | Station mode: start in fixed mode at `latitude:longitude:height`, skipping the survey. Degrees, degrees, meters above the ellipsoid. Use the `--fixed-position=<value>` form when the latitude is negative. |
| `-x`, `--sync-socket` | Unix socket of the owning process. The driver exits when that process goes away. |
| `-n`, `--nmea-socket` | Unix socket for NMEA data. |
| `-r`, `--rtcm-socket` | Unix socket for a copy of the RTCM stream. |
| `-d`, `--debug` | Debug logging. |

The process exits with code 2 on invalid arguments, including a missing email for RTK2go, and with code 1 on a failure its owner should restart it for.

```sh
# Base
./Zedf9p --mode Station --com-port /dev/ttyACM0 --ntrip-mountpoint MOUNT --ntrip-password PASSWORD --sync-socket /tmp/sync.sock

# Rover
./Zedf9p --mode Client --com-port /dev/ttyACM0 --ntrip-mountpoint MOUNT --ntrip-username you@example.com --sync-socket /tmp/sync.sock
```

## Sync socket

The driver connects to a Unix socket the owner is already listening on.

**Driver to owner:** one JSON object per line, leaving out fields that did not change.

| Field | Unit |
| --- | --- |
| `Latitude`, `Longitude` | degrees |
| `Altitude` | meters above the ellipsoid |
| `Accuracy` | meters: horizontal accuracy, or the mean accuracy during a survey |
| `Heading` | degrees, heading of motion: only meaningful while moving |
| `GroundSpeed` | m/s |
| `FixType` | 0 none, 2 2D, 3 3D, 4 GNSS + dead reckoning, 5 time only |
| `CarrierSolution` | 0 no RTK, 1 RTK float, 2 RTK fixed |
| `Satellites` | count |
| `Errors` | list of the flags below; empty when all is well |
| `ReceiverMode`, `ModuleCurrentSetAccuracy`, `SurveyTime`, `IsSurveyValid` | Station mode: receiver state |
| `LastNtripSent` | Station mode: UTC time of the last correction sent |
| `LastRtcmReceived` | Client mode: UTC time of the last correction received |

Error flags: `NTRIP_CONNECTION_ERROR`, `NTRIP_MOUNTPOINT_DOWN`, `NTRIP_BANNED`, `NTRIP_DATA_RCV_TIMEOUT`, `RTCM_OUTPUT_STALLED`, `GPS_CONNECTION_ERROR`, `GPS_NO_DATA`, `INVALID_FIXED_POSITION`.

**Owner to driver:** one command per line, Station mode only.

```
RESTART_SURVEY:<accuracy in meters>:<minimum time in seconds>
RESTART_FIXED:<latitude>:<longitude>:<height in meters above the ellipsoid>
```

## Connection behaviour

Public casters count failed connections per IP and ban the address, so the driver is careful with them:

- Attempts back off from 15 seconds to 15 minutes. The delay only resets after a connection has lasted a minute.
- After a reply that says the IP is banned, it waits an hour.
- A base connects only while the module is producing corrections, because a source that connects and sends nothing is dropped and counted as bad.
- Losing the caster neither stops the rover's position reports nor restarts the base's survey.

## Tests

```sh
dotnet test --project Zedf9p.Tests/Zedf9p.Tests.csproj
```

`tools/sim` holds a software ZED-F9P and a fake caster for running the real binary without hardware, on Linux:

```sh
dotnet publish Zedf9p/Zedf9p.csproj -c Release --self-contained -r linux-x64 -o out/linux-x64
python3 tools/sim/e2e.py out/linux-x64/Zedf9p
```

To test the Raspberry Pi build on a PC, run it in an emulated container. .NET needs these JIT settings under QEMU, which otherwise crashes:

```sh
dotnet publish Zedf9p/Zedf9p.csproj -c Release --self-contained -r linux-arm64 -o out/linux-arm64
docker run --rm --platform linux/arm64 -e E2E_SERIAL=1 -e DOTNET_TieredCompilation=0 -e DOTNET_EnableWriteXorExecute=0 \
    -v "$PWD:/w" -w /w python:3.13-slim-trixie sh -c "chmod +x out/linux-arm64/Zedf9p && python3 tools/sim/e2e.py out/linux-arm64/Zedf9p"
```

The simulator checks the protocol and the program's behaviour. It says nothing about real RTK performance, which needs the hardware.

## License
MIT License.
