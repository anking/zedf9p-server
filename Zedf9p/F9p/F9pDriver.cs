using Serilog;
using System;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UBLOX.Enums;
using Zedf9p.DTO;
using Zedf9p.Enums;
using Zedf9p.Exceptions;
using Zedf9p.Interfaces;
using Zedf9p.Ntrip;
using Zedf9p.SerialInterface;
using Zedf9p.Sockets;

namespace Zedf9p.F9p;

internal class F9pDriver : IF9pDriver
{
    const int NTRIP_INCOMING_BUFFER_SIZE = 10000;       //Size of incoming buffer for the connection with NTRIP Caster
    const int SYNC_INCOMING_BUFFER_SIZE = 10000;        //Size of incoming buffer for commands from the owning process

    const byte CLIENT_NAV_FREQUENCY = 5;                //The F9P solves RTK at about 7-8 Hz with all four constellations, asking for more only drops epochs
    const byte STATION_NAV_FREQUENCY = 1;               //One correction epoch per second is the norm for a base, more only multiplies the upload
    const ushort POSITION_POLL_TIMEOUT_MS = 500;

    static readonly TimeSpan NtripReceiveTimeout = TimeSpan.FromSeconds(30);    //Silence from the caster before reconnecting
    static readonly TimeSpan RtcmStallTimeout = TimeSpan.FromSeconds(5);        //Silence from the module before the base stops connecting to the caster
    static readonly TimeSpan GpsSilenceTimeout = TimeSpan.FromSeconds(5);       //No position from the module before reporting an error
    static readonly TimeSpan DeviceRetryDelay = TimeSpan.FromSeconds(5);        //Delay before reopening the module after a failure
    static readonly TimeSpan StationPositionPeriod = TimeSpan.FromSeconds(2);   //How often a running base reports its position

    // Module and its USB connection
    private readonly string _serialPortName;
    private SerialPort _serialPort;
    UBLOX.SFE_UBLOX_GPS _ublox;

    // NTRIP Caster settings
    readonly int _ntripPort;
    readonly string _ntripServer;
    readonly string _ntripMountpoint;
    readonly string _ntripUsername;
    readonly string _ntripPassword;

    // Caster connection of a running base
    NtripConnection _casterConnection;
    readonly RtcmFrameAssembler _rtcmAssembler = new();
    DateTime _lastRtcmFromModule;
    DateTime _nextNtripStatusSendTime;

    //SurveyIn Mode vars
    float _surveyAccuracy;
    int _surveyTime;

    //Fixed Mode vars
    FixedPosition _fixedPosition;

    // Socket for communication with the owning process
    readonly ISocketCommunications _socketCommunications;
    readonly byte[] _syncInBuffer = new byte[SYNC_INCOMING_BUFFER_SIZE];

    //Current operation mode
    OperationMode _driverOperationMode;

    //Current receiver mode (in station configuration)
    ReceiverModeEnum _receiverMode = ReceiverModeEnum.SurveyIn;

    //The task talking to the module, and what stops it
    private Task _deviceTask;
    private CancellationTokenSource _deviceCts;

    //Class that holds current state of errors
    private readonly ErrorFlags _errorFlags;

    // Serial Port handling Interface
    readonly ISerialPortHandler _serialPortHandler;

    readonly ILogger _logger;

    /// <summary>
    /// Main constructor, takes in all the input params for the driver
    /// </summary>
    public F9pDriver(
        InputParams inputParams,
        ISocketCommunications socketCommunications,
        ISerialPortHandler serialPortHandler,
        ILogger logger)
    {
        _serialPortName = inputParams.Port ?? string.Empty;
        _ntripMountpoint = inputParams.NtripMountpoint ?? string.Empty;
        _ntripUsername = inputParams.NtripUsername ?? string.Empty;
        _ntripPassword = inputParams.NtripPassword ?? string.Empty;
        _ntripPort = inputParams.NtripPort;
        _ntripServer = inputParams.NtripServer;
        _surveyAccuracy = inputParams.RtcmAccuracy;
        _surveyTime = inputParams.RtcmSurveyTime;

        if (!string.IsNullOrWhiteSpace(inputParams.FixedPosition) && FixedPosition.TryParse(inputParams.FixedPosition, out var position, out _))
        {
            _fixedPosition = position;
            _receiverMode = ReceiverModeEnum.FixedMode;
        }

        _socketCommunications = socketCommunications;
        _serialPortHandler = serialPortHandler;
        _logger = logger;

        _errorFlags = new ErrorFlags(SendErrors);
    }

    /// <summary>
    /// Runs the driver until cancelled or the owning process goes away
    /// </summary>
    public async Task Initialize(OperationMode mode, CancellationToken cancellationToken)
    {
        _driverOperationMode = mode;
        _logger.Information("Operating mode: {Mode}", mode);

        StartDeviceLoop(cancellationToken);

        try
        {
            await MaintainRunningThreadsAsync(cancellationToken);
        }
        finally
        {
            await StopDeviceLoopAsync();
        }

        _logger.Information("App shutdown");
    }

    /// <summary>
    /// Handles commands from the owning process. Runs on its own so a slow survey or caster never delays a command.
    /// </summary>
    private async Task MaintainRunningThreadsAsync(CancellationToken cancellationToken)
    {
        try
        {
            while (true)
            {
                cancellationToken.ThrowIfCancellationRequested();

                await CheckSyncDataSocketAsync(cancellationToken);

                await Task.Delay(100, cancellationToken);
            }
        }
        catch (OperationCanceledException)
        {
            _logger.Debug("MaintainRunningThreads gracefully stopped.");
        }
    }

    private void StartDeviceLoop(CancellationToken cancellationToken)
    {
        _deviceCts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        var token = _deviceCts.Token;
        _deviceTask = Task.Run(() => RunDeviceLoopAsync(token), token);
    }

    private async Task StopDeviceLoopAsync()
    {
        _deviceCts?.Cancel();

        try
        {
            if (_deviceTask != null) await _deviceTask;
        }
        catch (OperationCanceledException)
        {
            _logger.Debug("Device task stopped.");
        }

        _deviceCts?.Dispose();
        _deviceCts = null;
        _deviceTask = null;
    }

    /// <summary>
    /// Keeps a session with the module going: a failure is logged, reported and retried, never left to die unnoticed.
    /// </summary>
    private async Task RunDeviceLoopAsync(CancellationToken cancellationToken)
    {
        while (!cancellationToken.IsCancellationRequested)
        {
            try
            {
                await RunDeviceSessionAsync(cancellationToken);
            }
            catch (OperationCanceledException) when (cancellationToken.IsCancellationRequested)
            {
                return;
            }
            catch (Exception e)
            {
                _logger.Error(e, "F9P session failed, retrying in {Delay}s: {Message}", DeviceRetryDelay.TotalSeconds, e.Message);
                _errorFlags.Add("GPS_CONNECTION_ERROR");
            }
            finally
            {
                CloseCasterConnection();
                CloseSerialPort();
            }

            await Task.Delay(DeviceRetryDelay, cancellationToken);
        }
    }

    private async Task RunDeviceSessionAsync(CancellationToken cancellationToken)
    {
        _serialPort = await OpenSerialPortAsync(_serialPortName)
            ?? throw new IOException($"Cannot open serial port {_serialPortName}");

        _ublox = new UBLOX.SFE_UBLOX_GPS();
        _ublox.EnableDebugLogging(_logger);

        if (!await _ublox.BeginAsync(_serialPort))
        {
            throw new IOException($"No ZED-F9P answered on {_serialPortName}");
        }

        _errorFlags.Remove("GPS_CONNECTION_ERROR");

        switch (_driverOperationMode)
        {
            case OperationMode.Station:
                await RunStationAsync(cancellationToken);
                break;
            case OperationMode.Client:
                await RunClientAsync(cancellationToken);
                break;
            default:
                await RunPositionLoopAsync(CLIENT_NAV_FREQUENCY, cancellationToken);
                break;
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Client (rover)
    // ---------------------------------------------------------------------------------------------

    private async Task RunClientAsync(CancellationToken cancellationToken)
    {
        _logger.Information("Set nav frequency, " + (await _ublox.setNavigationFrequency(CLIENT_NAV_FREQUENCY) ? "OK" : "Failed"));

        //Set USB output to UBX only, no NMEA Noise
        await _ublox.SetUSBOutputAsync(UBLOX.Constants.COM_TYPE_UBX);

        // The position keeps flowing whatever happens to the corrections
        using var corrections = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        var correctionsTask = Task.Run(() => RunCorrectionsLoopAsync(corrections.Token), corrections.Token);

        try
        {
            await RunPositionLoopAsync(CLIENT_NAV_FREQUENCY, cancellationToken);
        }
        finally
        {
            corrections.Cancel();
            try
            {
                await correctionsTask;
            }
            catch (OperationCanceledException)
            {
                _logger.Debug("Corrections task stopped.");
            }
        }
    }

    /// <summary>
    /// Holds the connection to the caster and hands every correction to the module, waiting longer after each failure.
    /// </summary>
    private async Task RunCorrectionsLoopAsync(CancellationToken cancellationToken)
    {
        var backoff = ReconnectBackoff.ForCaster();

        while (!cancellationToken.IsCancellationRequested)
        {
            var connectedAt = DateTime.MaxValue;
            var delay = TimeSpan.Zero;

            try
            {
                _logger.Information("Connecting to NTRIP caster {Server}:{Port}/{Mountpoint}", _ntripServer, _ntripPort, _ntripMountpoint);
                using var connection = await NtripConnection.OpenClientAsync(_ntripServer, _ntripPort, _ntripMountpoint, _ntripUsername, _ntripPassword, cancellationToken);

                connectedAt = DateTime.UtcNow;
                _logger.Information("NTRIP caster accepted the connection");
                ClearCasterErrors();

                if (connection.InitialData.Length > 0)
                {
                    _ublox.Send(connection.InitialData, connection.InitialData.Length);
                }

                await PumpCorrectionsAsync(connection, cancellationToken);
            }
            catch (OperationCanceledException) when (cancellationToken.IsCancellationRequested)
            {
                return;
            }
            catch (NtripException e)
            {
                _logger.Warning("NTRIP: {Message}", e.Message);
                delay = ReportCasterFailure(e);
            }
            catch (Exception e) when (e is SocketException or IOException or ObjectDisposedException or InvalidOperationException or TimeoutException)
            {
                _logger.Warning("NTRIP connection lost: {Message}", e.Message);
                _errorFlags.Add("NTRIP_CONNECTION_ERROR");
            }

            if (DateTime.UtcNow - connectedAt >= ReconnectBackoff.HealthyConnection)
            {
                backoff.Reset();
            }

            if (delay == TimeSpan.Zero) delay = backoff.NextDelay();

            _logger.Information("Next NTRIP connection attempt in {Seconds}s", delay.TotalSeconds);
            await Task.Delay(delay, cancellationToken);
        }
    }

    private async Task PumpCorrectionsAsync(NtripConnection connection, CancellationToken cancellationToken)
    {
        var buffer = new byte[NTRIP_INCOMING_BUFFER_SIZE];
        var nextStatus = DateTime.MinValue;

        while (true)
        {
            using var receive = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
            receive.CancelAfter(NtripReceiveTimeout);

            int received;
            try
            {
                received = await connection.ReceiveAsync(buffer, receive.Token);
            }
            catch (OperationCanceledException) when (!cancellationToken.IsCancellationRequested)
            {
                _errorFlags.Add("NTRIP_DATA_RCV_TIMEOUT");
                throw new NtripException($"No corrections for {NtripReceiveTimeout.TotalSeconds}s", NtripResponseKind.Rejected);
            }

            if (received == 0)
            {
                throw new NtripException("The caster closed the connection", NtripResponseKind.Rejected);
            }

            _logger.Debug("Sending {Bytes} RTCM bytes to the F9P", received);
            _ublox.Send(buffer, received);
            _errorFlags.Remove("NTRIP_DATA_RCV_TIMEOUT");

            var now = DateTime.UtcNow;
            if (now >= nextStatus)
            {
                _socketCommunications.SendSyncData(new SyncData { LastRtcmReceived = now });
                nextStatus = now.AddSeconds(1);
            }
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Position reporting
    // ---------------------------------------------------------------------------------------------

    private async Task RunPositionLoopAsync(byte frequency, CancellationToken cancellationToken)
    {
        var period = TimeSpan.FromMilliseconds(1000.0 / frequency);
        var lastPosition = DateTime.UtcNow;

        while (true)
        {
            cancellationToken.ThrowIfCancellationRequested();

            var position = await ReadPositionAsync();
            if (position != null)
            {
                _socketCommunications.SendSyncData(position);
                _errorFlags.Remove("GPS_NO_DATA");
                lastPosition = DateTime.UtcNow;
            }
            else if (DateTime.UtcNow - lastPosition > GpsSilenceTimeout)
            {
                _errorFlags.Add("GPS_NO_DATA");
            }

            await Task.Delay(period, cancellationToken);
        }
    }

    /// <summary>
    /// Polls the module once. Returns null when it did not answer, rather than reporting old values as new.
    /// </summary>
    private async Task<SyncData> ReadPositionAsync()
    {
        if (!await _ublox.getHPPOSLLH(POSITION_POLL_TIMEOUT_MS)) return null;
        if (!await _ublox.getPVT(POSITION_POLL_TIMEOUT_MS)) return null;

        return new SyncData
        {
            Latitude = await _ublox.getHighResLatitude() / 10000000D,
            Longitude = await _ublox.getHighResLongitude() / 10000000D,
            Accuracy = await _ublox.getHorizontalAccuracy() / 10000D,   //0.1 mm to m
            Altitude = await _ublox.getAltitude() / 1000D,              //mm to m
            Heading = await _ublox.getHeading() / 100000D,
            GroundSpeed = await _ublox.getGroundSpeed() / 1000D,        //mm/s to m/s
            FixType = await _ublox.getFixType(),
            CarrierSolution = await _ublox.getCarrierSolutionType(),
            Satellites = await _ublox.getSIV(),
        };
    }

    // ---------------------------------------------------------------------------------------------
    // Station (base)
    // ---------------------------------------------------------------------------------------------

    private async Task RunStationAsync(CancellationToken cancellationToken)
    {
        // Set USB output to UBX AND RTCM only, no NMEA Noise
        await _ublox.SetUSBOutputAsync(UBLOX.Constants.COM_TYPE_UBX | UBLOX.Constants.COM_TYPE_RTCM3);

        if (await _ublox.GetProtocolVersion())
        {
            _logger.Information($"Ublox Protocol Version: {await _ublox.GetProtocolVersionHigh()}.{await _ublox.GetProtocolVersionLow()}");
        }

        await _ublox.setNavigationFrequency(STATION_NAV_FREQUENCY);

        // Start from a known state: the module keeps its last mode for as long as it has power
        await DisableRtcmMessagesOnUsbAsync();
        await _ublox.DisableSurveyMode();

        switch (_receiverMode)
        {
            case ReceiverModeEnum.FixedMode:
                await StartFixedAsync();
                break;
            default:
                await StartSurveyAsync(cancellationToken);
                break;
        }

        // Both modes end the same way: corrections from the module go to the caster
        _rtcmAssembler.Reset();
        _lastRtcmFromModule = DateTime.UtcNow;
        _ublox.AttachRTCMHandler(ProcessRtcmStationAsync);
        await EnableRtcmMessagesOnUsbAsync();

        await RunCorrectionsUploadAsync(cancellationToken);
    }

    /// <summary>
    /// Reads the module and keeps the caster connection up. Connects only while corrections are flowing: a source
    /// that connects and sends nothing gets dropped within seconds and then counts against the IP on RTK2go.
    /// </summary>
    private async Task RunCorrectionsUploadAsync(CancellationToken cancellationToken)
    {
        var backoff = ReconnectBackoff.ForCaster();
        var nextAttempt = DateTime.MinValue;
        var nextPosition = DateTime.MinValue;
        var connectedAt = DateTime.MinValue;
        var everConnected = false;
        Task<NtripConnection> connecting = null;

        try
        {
            while (true)
            {
                cancellationToken.ThrowIfCancellationRequested();

                // New RTCM bytes end up in ProcessRtcmStationAsync
                await _ublox.CheckUblox();

                var now = DateTime.UtcNow;
                var rtcmFlowing = now - _lastRtcmFromModule < RtcmStallTimeout;

                if (rtcmFlowing) _errorFlags.Remove("RTCM_OUTPUT_STALLED");
                else _errorFlags.Add("RTCM_OUTPUT_STALLED");

                // Lost connection: only a connection that lasted counts as a fresh start
                if (everConnected && _casterConnection is not { IsConnected: true })
                {
                    CloseCasterConnection();
                    everConnected = false;

                    if (now - connectedAt >= ReconnectBackoff.HealthyConnection) backoff.Reset();

                    var delay = backoff.NextDelay();
                    nextAttempt = now + delay;
                    _errorFlags.Add("NTRIP_CONNECTION_ERROR");
                    _logger.Warning("Connection to the NTRIP caster lost, reconnecting in {Seconds}s", delay.TotalSeconds);
                }

                if (connecting == null && _casterConnection == null && rtcmFlowing && now >= nextAttempt)
                {
                    _logger.Information("Connecting to NTRIP caster {Server}:{Port}/{Mountpoint}", _ntripServer, _ntripPort, _ntripMountpoint);
                    connecting = NtripConnection.OpenSourceAsync(_ntripServer, _ntripPort, _ntripMountpoint, _ntripPassword, cancellationToken);
                }

                if (connecting is { IsCompleted: true })
                {
                    try
                    {
                        _casterConnection = await connecting;
                        connectedAt = now;
                        everConnected = true;
                        ClearCasterErrors();
                        _logger.Information("NTRIP caster accepted the stream");
                    }
                    catch (NtripException e)
                    {
                        _logger.Error("NTRIP: {Message}", e.Message);
                        var delay = ReportCasterFailure(e);
                        if (delay == TimeSpan.Zero) delay = backoff.NextDelay();
                        nextAttempt = now + delay;
                        _logger.Information("Next NTRIP connection attempt in {Seconds}s", delay.TotalSeconds);
                    }
                    finally
                    {
                        connecting = null;
                    }
                }

                if (now >= nextPosition)
                {
                    var position = await ReadPositionAsync();
                    if (position != null) _socketCommunications.SendSyncData(position);
                    nextPosition = now + StationPositionPeriod;
                }

                await Task.Delay(10, cancellationToken);
            }
        }
        finally
        {
            // A connection still being opened must not outlive this session and hold the mountpoint
            if (connecting != null)
            {
                _ = connecting.ContinueWith(t => t.Result.Dispose(), TaskContinuationOptions.OnlyOnRanToCompletion);
            }
        }
    }

    /// <summary>
    /// Handler for RTCM data coming from the module. Whole frames go to the caster as soon as they are complete.
    /// </summary>
    private Task ProcessRtcmStationAsync(byte incoming)
    {
        // Send rtcm info thru socket to the UI http server app if needed
        if (_socketCommunications.IsRtcmDataSocketConnected())
        {
            _socketCommunications.SendRtcmData(incoming);
        }

        var frameLength = _rtcmAssembler.Add(incoming);
        if (frameLength > 0)
        {
            _lastRtcmFromModule = DateTime.UtcNow;
            SendFrameToCaster(frameLength);
        }

        return Task.CompletedTask;
    }

    private void SendFrameToCaster(int frameLength)
    {
        var connection = _casterConnection;
        if (connection == null) return;

        try
        {
            connection.Send(_rtcmAssembler.Buffer, frameLength);

            //Do not send this more than once a second to sync
            var now = DateTime.UtcNow;
            if (now >= _nextNtripStatusSendTime)
            {
                _socketCommunications.SendSyncData(new SyncData { LastNtripSent = now });
                _nextNtripStatusSendTime = now.AddSeconds(1);
            }
        }
        catch (Exception e) when (e is SocketException or ObjectDisposedException)
        {
            // The upload loop sees the closed connection and schedules the reconnect
            _logger.Warning("Sending to the NTRIP caster failed: {Message}", e.Message);
            connection.Dispose();
        }
    }

    /// <summary>
    /// Enables Survey mode on f9p device and waits for it to complete
    /// </summary>
    private async Task StartSurveyAsync(CancellationToken cancellationToken)
    {
        if (_surveyAccuracy <= 0 || _surveyTime <= 0) throw new InvalidDataException("Survey accuracy or minimum time not provided, cannot continue...");

        _logger.Information("Enable Survey Mode, minimum " + _surveyTime + "sec and minimum accuracy " + _surveyAccuracy + " meters");

        if (!await _ublox.EnableSurveyMode((ushort)_surveyTime, _surveyAccuracy)) throw new IOException("Unable to start the survey on the module");

        // Read receiver mode
        var surveyMode = await _ublox.GetTmode3() ?? throw new IOException("Unable to read the receiver mode from the module");
        _logger.Information("Accuracy: " + surveyMode.GetAccuracyLimit() + "m");
        _logger.Information("Mode: " + surveyMode.GetMode().ToString());
        _socketCommunications.SendSyncData(new SyncData()
        {
            ModuleCurrentSetAccuracy = surveyMode.GetAccuracyLimit(),
            ReceiverMode = surveyMode.GetMode(),
            IsSurveyValid = false,
        });

        _logger.Information("Start requesting survey status...");

        // Wait for survey to complete
        while (_ublox.svin.valid == false)
        {
            cancellationToken.ThrowIfCancellationRequested();

            // Query module for SVIN status with 2000ms timeout (req can take a long time)
            if (await _ublox.getSurveyStatus(2000))
            {
                // Copy before the position poll below reuses the module's buffers
                var survey = new SyncData()
                {
                    Accuracy = _ublox.svin.meanAccuracy,
                    SurveyTime = _ublox.svin.observationTime,
                    IsSurveyValid = _ublox.svin.valid,
                };

                var position = await ReadPositionAsync();
                if (position != null)
                {
                    position.Accuracy = null; // Accuracy means the survey's mean accuracy until it completes
                    _socketCommunications.SendSyncData(position);
                }

                _socketCommunications.SendSyncData(survey);

                _logger.Information("Time elapsed: {Seconds}s Accuracy: {Accuracy}m Is Valid?: {Valid}", survey.SurveyTime, survey.Accuracy, survey.IsSurveyValid);
            }
            else
            {
                _logger.Information("SVIN request failed");
            }

            await Task.Delay(100, cancellationToken);
        }

        _logger.Information("Base survey complete! RTCM can now be broadcast");
    }

    /// <summary>
    /// Enables Fixed mode on f9p device
    /// </summary>
    private async Task StartFixedAsync()
    {
        _logger.Information("Enable Fixed Mode at {Position}", _fixedPosition.ToString());

        var moduleResponse = await _ublox.enableFixedMode(
            _fixedPosition.LatitudeE7, _fixedPosition.LongitudeE7, _fixedPosition.HeightCm,
            _fixedPosition.LatitudeHp, _fixedPosition.LongitudeHp, _fixedPosition.HeightHp);

        if (!moduleResponse) throw new IOException("Unable to set the fixed position on the module");

        var surveyMode = await _ublox.GetTmode3() ?? throw new IOException("Unable to read the receiver mode from the module");
        _socketCommunications.SendSyncData(new SyncData()
        {
            ReceiverMode = surveyMode.GetMode(),
        });

        var position = await ReadPositionAsync();
        if (position != null) _socketCommunications.SendSyncData(position);

        _logger.Information("Base setting complete! RTCM can now be broadcast");
    }

    private async Task EnableRtcmMessagesOnUsbAsync()
    {
        _logger.Information("Enable RTCM Messaging on USB");

        var moduleResponse = await _ublox.EnableRTCMmessage(UBLOX.Constants.UBX_RTCM_1005, UBLOX.Constants.COM_PORT_USB, 1);
        moduleResponse &= await _ublox.EnableRTCMmessage(UBLOX.Constants.UBX_RTCM_1074, UBLOX.Constants.COM_PORT_USB, 1);
        moduleResponse &= await _ublox.EnableRTCMmessage(UBLOX.Constants.UBX_RTCM_1084, UBLOX.Constants.COM_PORT_USB, 1);
        moduleResponse &= await _ublox.EnableRTCMmessage(UBLOX.Constants.UBX_RTCM_1094, UBLOX.Constants.COM_PORT_USB, 1);
        moduleResponse &= await _ublox.EnableRTCMmessage(UBLOX.Constants.UBX_RTCM_1124, UBLOX.Constants.COM_PORT_USB, 1);
        moduleResponse &= await _ublox.EnableRTCMmessage(UBLOX.Constants.UBX_RTCM_1230, UBLOX.Constants.COM_PORT_USB, 10);

        if (!moduleResponse) throw new IOException("Unable to enable RTCM output on the module");
    }

    private async Task DisableRtcmMessagesOnUsbAsync()
    {
        _logger.Information("Disable RTCM Messaging on USB");

        var moduleResponse = await _ublox.disableRTCMmessage(UBLOX.Constants.UBX_RTCM_1005, UBLOX.Constants.COM_PORT_USB);
        moduleResponse &= await _ublox.disableRTCMmessage(UBLOX.Constants.UBX_RTCM_1074, UBLOX.Constants.COM_PORT_USB);
        moduleResponse &= await _ublox.disableRTCMmessage(UBLOX.Constants.UBX_RTCM_1084, UBLOX.Constants.COM_PORT_USB);
        moduleResponse &= await _ublox.disableRTCMmessage(UBLOX.Constants.UBX_RTCM_1094, UBLOX.Constants.COM_PORT_USB);
        moduleResponse &= await _ublox.disableRTCMmessage(UBLOX.Constants.UBX_RTCM_1124, UBLOX.Constants.COM_PORT_USB);
        moduleResponse &= await _ublox.disableRTCMmessage(UBLOX.Constants.UBX_RTCM_1230, UBLOX.Constants.COM_PORT_USB);

        if (!moduleResponse) throw new IOException("Unable to disable RTCM output on the module");
    }

    // ---------------------------------------------------------------------------------------------
    // Commands from the owning process
    // ---------------------------------------------------------------------------------------------

    /// <summary>
    /// Check incoming data thru the sync socket for commands coming from the frontend
    /// </summary>
    private async Task CheckSyncDataSocketAsync(CancellationToken cancellationToken)
    {
        // Left running, this process would keep the serial port and the mountpoint away from its replacement
        if (_socketCommunications.IsSyncDataSocketClosedByPeer())
        {
            throw new SyncSocketException("The owning process closed the sync socket, the driver must shut down");
        }

        if (!_socketCommunications.IsSyncDataSocketConnected()) return;

        var syncRcvLen = _socketCommunications.ReceiveSyncData(_syncInBuffer);
        if (syncRcvLen <= 0) return;

        foreach (var syncMessage in Encoding.ASCII.GetString(_syncInBuffer, 0, syncRcvLen).Split('\n', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries))
        {
            _logger.Information("Sync Message Received: " + syncMessage);

            try
            {
                await ProcessSyncCommandAsync(new SyncIncomingCommand(syncMessage), cancellationToken);
            }
            catch (Exception e) when (e is UnknownSyncCommandException or FormatException or NotSupportedException or IndexOutOfRangeException or ArgumentException)
            {
                // A bad command from the UI must not take the running base down
                _logger.Warning("Ignoring sync command: {Message}", e.Message);
            }
        }
    }

    private async Task ProcessSyncCommandAsync(SyncIncomingCommand command, CancellationToken cancellationToken)
    {
        if (_driverOperationMode != OperationMode.Station)
        {
            _logger.Warning("Ignoring {Command}: the receiver mode only applies to Station mode", command.Type);
            return;
        }

        if (command.Type == SyncIncomingCommandType.RESTART_SURVEY)
        {
            var accuracy = command.GetValue<float>();
            var time = command.GetValue<int>(1);
            if (accuracy <= 0 || time <= 0 || time > ushort.MaxValue) throw new FormatException($"Invalid survey settings: {accuracy}m, {time}s");

            _surveyAccuracy = accuracy;
            _surveyTime = time;
            _receiverMode = ReceiverModeEnum.SurveyIn;
        }
        else if (command.Type == SyncIncomingCommandType.RESTART_FIXED)
        {
            if (!FixedPosition.TryParse(command.GetValue<string>(), command.GetValue<string>(1), command.GetValue<string>(2), out var position, out var error))
            {
                // The base keeps running as it was: a position that is wrong by a digit would ruin every rover fix
                _errorFlags.Add("INVALID_FIXED_POSITION");
                throw new FormatException($"Invalid fixed position: {error}");
            }

            _errorFlags.Remove("INVALID_FIXED_POSITION");
            _fixedPosition = position;
            _receiverMode = ReceiverModeEnum.FixedMode;
        }
        else
        {
            return;
        }

        // The session owns the serial port and the caster connection: end it, then start again with the new settings
        await StopDeviceLoopAsync();
        StartDeviceLoop(cancellationToken);
    }

    // ---------------------------------------------------------------------------------------------
    // Shared helpers
    // ---------------------------------------------------------------------------------------------

    /// <summary>
    /// Raises the flag for a failed caster connection. Returns how long the reply says to wait, or zero to use the normal backoff.
    /// </summary>
    private TimeSpan ReportCasterFailure(NtripException e)
    {
        switch (e.Response)
        {
            case NtripResponseKind.Banned:
                _errorFlags.Add("NTRIP_BANNED");
                return ReconnectBackoff.BannedDelay;
            case NtripResponseKind.SourceTable:
                _errorFlags.Add("NTRIP_MOUNTPOINT_DOWN");
                return TimeSpan.Zero;
            default:
                _errorFlags.Add("NTRIP_CONNECTION_ERROR");
                return TimeSpan.Zero;
        }
    }

    private void ClearCasterErrors()
    {
        _errorFlags.Remove("NTRIP_CONNECTION_ERROR");
        _errorFlags.Remove("NTRIP_MOUNTPOINT_DOWN");
        _errorFlags.Remove("NTRIP_BANNED");
    }

    /// <summary>
    /// Send error flags to UI, this error handler is being called by the "ErrorFlags" class then new error comes up
    /// </summary>
    private void SendErrors(ErrorFlags errorFlags)
    {
        _socketCommunications.SendSyncData(new SyncData() { Errors = errorFlags.Errors });
    }

    private void CloseCasterConnection()
    {
        _casterConnection?.Dispose();
        _casterConnection = null;
    }

    private void CloseSerialPort()
    {
        try
        {
            _serialPortHandler.ClosePort(_serialPort);
        }
        catch (IOException e)
        {
            _logger.Warning("Closing the serial port failed: {Message}", e.Message);
        }

        _serialPort = null;
    }

    private async Task<SerialPort> OpenSerialPortAsync(
        string portName,
        int baudRate = 115200,
        Parity parity = Parity.None,
        int dataBits = 8,
        StopBits stopBits = StopBits.One)
    {
        _logger.Information("Create serial port connection for f9p module...");
        return await _serialPortHandler.OpenPortAsync(portName, TimeSpan.FromSeconds(5), baudRate, parity, dataBits, stopBits);
    }

    /// <summary>
    /// Disconnects the owning process's sockets. The device task has already closed the port and the caster connection.
    /// </summary>
    public void Cleanup()
    {
        _socketCommunications.CleanupSockets();
    }
}
