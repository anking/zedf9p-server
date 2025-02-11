using CommandLine;
using Zedf9p.Enums;

public class InputParams
{
    [Option('p', "com-port", Required = true, HelpText = "COM port (e.g., COM#, ttyACM#).")]
    public string Port { get; set; }

    [Option('s', "ntrip-server", Default = "rtk2go.com", HelpText = "NTRIP caster server.")]
    public string NtripServer { get; set; }

    [Option('t', "ntrip-port", Default = 2101, HelpText = "NTRIP caster port.")]
    public int NtripPort { get; set; }

    [Option('m', "ntrip-mountpoint", HelpText = "NTRIP caster mountpoint.")]
    public string NtripMountpoint { get; set; }

    [Option('w', "ntrip-password", HelpText = "NTRIP caster password.")]
    public string NtripPassword { get; set; }

    [Option('a', "rtcm-accuracy-req", Default = 3f, HelpText = "Minimum RTCM accuracy to be accepted before survey completes (in meters).")]
    public float RtcmAccuracy { get; set; }

    [Option('y', "rtcm-survey-time", Default = 60, HelpText = "Minimum time required for the RTCM survey to complete (in seconds).")]
    public int RtcmSurveyTime { get; set; }

    [Option('o', "mode", Default = OperationMode.Idle, HelpText = "Operation mode for the driver (Idle/Server/Client).")]
    public OperationMode Mode { get; set; }

    [Option('n', "nmea-socket", HelpText = "Path to the NMEA socket.")]
    public string NmeaSocketPath { get; set; }

    [Option('r', "rtcm-socket",  HelpText = "Path to the RTCM socket.")]
    public string RtcmSocketPath { get; set; }

    [Option('x', "sync-socket", HelpText = "Path to the sync socket.")]
    public string SyncSocketPath { get; set; }

    [Option('d', "debug", Default = false, HelpText = "Enable debug mode.")]
    public bool Debug { get; set; }
}
