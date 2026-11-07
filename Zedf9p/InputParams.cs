using System;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using CommandLine;
using Zedf9p.Enums;
using Zedf9p.F9p;

public class InputParams
{
    // Same shape RTK2go accepts: a char or digit first, then an address (or "-at-" for clients that cannot type "@").
    private static readonly Regex EmailLike = new(@"^[A-Za-z0-9][A-Za-z0-9+_.\-]*(@|-at-)[A-Za-z0-9\-]+(\.[A-Za-z0-9\-]+)+$", RegexOptions.Compiled);

    [Option('p', "com-port", Required = true, HelpText = "COM port (e.g., COM#, /dev/ttyACM#).")]
    public string Port { get; set; }

    [Option('s', "ntrip-server", Default = "rtk2go.com", HelpText = "NTRIP caster server.")]
    public string NtripServer { get; set; }

    [Option('t', "ntrip-port", Default = 2101, HelpText = "NTRIP caster port.")]
    public int NtripPort { get; set; }

    [Option('m', "ntrip-mountpoint", HelpText = "NTRIP caster mountpoint.")]
    public string NtripMountpoint { get; set; }

    [Option('u', "ntrip-username", HelpText = "NTRIP client user name (Client mode). RTK2go requires a valid email address.")]
    public string NtripUsername { get; set; }

    [Option('w', "ntrip-password", HelpText = "NTRIP caster password. Station mode: the mountpoint password. Client mode: optional.")]
    public string NtripPassword { get; set; }

    [Option('a', "rtcm-accuracy-req", Default = 3f, HelpText = "Minimum RTCM accuracy to be accepted before survey completes (in meters).")]
    public float RtcmAccuracy { get; set; }

    [Option('y', "rtcm-survey-time", Default = 60, HelpText = "Minimum time required for the RTCM survey to complete (in seconds).")]
    public int RtcmSurveyTime { get; set; }

    [Option('f', "fixed-position", HelpText = "Station mode: start in fixed mode at latitude:longitude:height (degrees, degrees, meters above the ellipsoid) instead of surveying. Use --fixed-position=<value> when the latitude is negative.")]
    public string FixedPosition { get; set; }

    [Option('o', "mode", Default = OperationMode.Idle, HelpText = "Operation mode for the driver (Idle/Station/Client).")]
    public OperationMode Mode { get; set; }

    [Option('n', "nmea-socket", HelpText = "Path to the NMEA socket.")]
    public string NmeaSocketPath { get; set; }

    [Option('r', "rtcm-socket", HelpText = "Path to the RTCM socket.")]
    public string RtcmSocketPath { get; set; }

    [Option('x', "sync-socket", HelpText = "Path to the sync socket.")]
    public string SyncSocketPath { get; set; }

    [Option('d', "debug", Default = false, HelpText = "Enable debug mode.")]
    public bool Debug { get; set; }

    /// <summary>
    /// Catches settings that would only fail later as repeated bad caster connections, which can get an IP banned.
    /// </summary>
    public IReadOnlyList<string> Validate()
    {
        var errors = new List<string>();

        if (Mode == OperationMode.Idle)
        {
            return errors;
        }

        if (string.IsNullOrWhiteSpace(NtripServer)) errors.Add("--ntrip-server is required.");
        if (NtripPort <= 0) errors.Add("--ntrip-port must be a valid port.");
        if (string.IsNullOrWhiteSpace(NtripMountpoint)) errors.Add("--ntrip-mountpoint is required.");

        if (Mode == OperationMode.Station)
        {
            if (string.IsNullOrWhiteSpace(NtripPassword)) errors.Add("--ntrip-password is required in Station mode.");

            if (!string.IsNullOrWhiteSpace(FixedPosition) &&
                !global::Zedf9p.F9p.FixedPosition.TryParse(FixedPosition, out _, out var positionError))
            {
                errors.Add($"--fixed-position: {positionError}");
            }
        }

        if (Mode == OperationMode.Client &&
            (NtripServer ?? string.Empty).EndsWith("rtk2go.com", StringComparison.OrdinalIgnoreCase) &&
            !EmailLike.IsMatch(NtripUsername ?? string.Empty))
        {
            errors.Add("--ntrip-username must be a valid email address: RTK2go requires it for rovers and can ban repeated bad connections.");
        }

        return errors;
    }
}
