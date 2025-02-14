using Serilog;
using System;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UBLOX.Enums;
using Zedf9p.DTO;
using Zedf9p.Enums;
using Zedf9p.Exceptions;
using Zedf9p.Interfaces;
using Zedf9p.SerialInterface;
using Zedf9p.Sockets;
using Zedf9p.Utils;

namespace Zedf9p.F9p;

internal class F9pDriver : IF9pDriver
{
    const int RTCM_OUTGOING_BUFFER_SIZE = 500;          //The size of the RTCM correction data varies but in general it is approximately 2000 bytes every second (~2500 bytes every 10th second when 1230 is transmitted).
    const int NTRIP_INCOMING_BUFFER_SIZE = 10000;       //Size of incoming buffer for the connection with NTRIP Caster
    const int SYNC_INCOMING_BUFFER_SIZE = 10000;        //Size of incoming buffer for the connection with NTRIP Caster
    const int NMEA_OUTGOING_STRING_MAXLEN = 300;        //Size of incoming buffer for the connection with NTRIP Caster

    const int CLIENT_NAV_FREQUENCY = 20;                //Sets how often GPS module will spit out data
    const int SERVER_NAV_FREQUENCY = 4;                 //Sets how often GPS module will spit out data

    const int NTRIP_RECEIVE_TIMEOUT = 30;               //How many seconds we wait until we dicede there is an Ntrip receive timeout

    // TEST CONSTANTS
    const bool NTRIP_SERVER_SIMULATION = false;         // If set to true the driver will not connect to NTRIP
    const bool IGNORE_DISCONNECTED_SOCKET = true;       // True is a testvalue

    // Ublox f9p gps module
    UBLOX.SFE_UBLOX_GPS _ublox;

    //Gps Module Settings       
    private readonly string _serialPortName;                  //port the module is connected to (COM#) or ttyACM#
    private SerialPort _serialPort;                     //instance of serial port for communication with module

    // NTRIP Caster settings
    Socket _ntripCasterSocket;   //socket for connection with NTRIP caster
    readonly byte[] _ntripOutBuffer = new byte[RTCM_OUTGOING_BUFFER_SIZE];
    readonly byte[] _ntripInBuffer = new byte[NTRIP_INCOMING_BUFFER_SIZE];
    int _rtcmFrame = 0;           //number of currect rtcm frame (for incoming rtcm data buffer control)
    readonly int _ntripPort;              //caster port
    readonly string _ntripServer;         //caster server        
    readonly string _ntripMountpoint;     //caster mountpoint
    readonly string _ntripPassword;       //caster password

    //SurveyIn Mode vars
    float _surveyAccuracy;         //Minimum accuracy to be accepted before survey completes in meters (3.000F)/float
    int _surveyTime;         //minimum time required for the survey to complete

    //Fixed Mode vars
    int _latitude = 406028960;
    int _longitude = -800735223;
    int _altitude = 284000;

    // f9p socket for interapplication communication with node express server
    readonly ISocketCommunications _socketCommunications;

    //initialize buffer for sync channel
    readonly byte[] _syncInBuffer = new byte[SYNC_INCOMING_BUFFER_SIZE];

    //initialize buffer for nmea channel
    string _nmeaOutBuffer = "";

    //Current operation mode
    OperationMode _driverOperationMode;
    public void SetMode(OperationMode mode) => _driverOperationMode = mode;
    public OperationMode GetMode() => _driverOperationMode;

    //Current receiver mode (in server configuration)
    ReceiverModeEnum _receiverMode = ReceiverModeEnum.SurveyIn;

    //Reference to currently running main thread
    private Task _driverRunningThreadTask;

    //Cancellation token for main running thread
    CancellationTokenSource _cancellationTokenSourceForDriverThreadTask;

    //Indicator of next NTRIP packet send status (cannot send them to often)
    long _nextNtripStatusSendTime;

    //Class that holds current state of errors
    private readonly ErrorFlags _errorFlags;

    // Serial Port handling Interface
    readonly ISerialPortHandler _serialPortHandler;

    readonly ILogger _logger;


    /// <summary>
    /// Main constructor, takes in all the input params for the driver
    /// </summary>
    /// <param name="inputParams"></param>
    public F9pDriver(
        InputParams inputParams,
        ISocketCommunications socketCommunications,
        ISerialPortHandler serialPortHandler,
        ILogger logger)
    {
        // Using null-coalescing operator for optional fields (to avoid null values)
        _serialPortName = inputParams.Port ?? string.Empty; // Default to empty string if null
        _ntripMountpoint = inputParams.NtripMountpoint ?? string.Empty; // Default to empty string if null
        _ntripPassword = inputParams.NtripPassword ?? string.Empty; // Default to empty string if null
        _ntripPort = inputParams.NtripPort; // No need for null check, int has default value
        _ntripServer = inputParams.NtripServer; // Assumed default value in InputParams
        _surveyAccuracy = inputParams.RtcmAccuracy; // Assumed default value in InputParams
        _surveyTime = inputParams.RtcmSurveyTime; // Assumed default value in InputParams

        _socketCommunications = socketCommunications;
        _serialPortHandler = serialPortHandler;
        _logger = logger;

        _errorFlags = new ErrorFlags(SendErrors);
    }

    /// <summary>
    /// Initialize driver and connect sockets
    /// </summary>
    /// <param name="mode">Mode of operation</param>
    /// <returns></returns>
    public async Task Initialize(OperationMode mode, CancellationToken cancellationToken)
    {
        // Set current mode
        _driverOperationMode = mode;
        _logger.Debug($"Operating mode: {_driverOperationMode}");

        _cancellationTokenSourceForDriverThreadTask = new CancellationTokenSource();
        var runTaskCancellationToken = _cancellationTokenSourceForDriverThreadTask.Token;

        try
        {
            if (_driverOperationMode == OperationMode.Station)
            {
                // Start NTRIP server (base station)
                _driverRunningThreadTask = Task.Run(() => ConfigureGpsAsNtripStationAsync(runTaskCancellationToken), runTaskCancellationToken);
            }
            else if (_driverOperationMode == OperationMode.Client)
            {
                // Start NTRIP client (rover)
                _driverRunningThreadTask = Task.Run(() => ConfigureGpsAsNtripClientAsync(runTaskCancellationToken), runTaskCancellationToken);
            }

            // Start monitoring Sync Channel in a separate tracked task
            await MaintainRunningThreadsAsync(cancellationToken);
        }
        catch (OperationCanceledException)
        {
            _logger.Information("Initialization was canceled.");
        }
        catch (Exception ex)
        {
            _logger.Error(ex, "Error occurred during initialization.");
        }
        finally
        {
            _logger.Debug("Cancelling Driver Thread Subtask");
            _cancellationTokenSourceForDriverThreadTask?.Cancel();
        }
    }

    private async Task MaintainRunningThreadsAsync(CancellationToken cancellationToken)
    {
        try
        {
            while (true)
            {
                cancellationToken.ThrowIfCancellationRequested();

                // Check sync socket periodically
                await CheckSyncDataSocketAsync(cancellationToken);

                await Task.Delay(100, cancellationToken);
            }
        }
        catch (OperationCanceledException)
        {
            _logger.Debug("MaintainRunningThreads gracefully stopped.");
        }
        catch (Exception ex)
        {
            _logger.Fatal(ex, "MaintainRunningThreads encountered an unexpected error.");
        }
    }

    /// <summary>
    /// Once module is configured for a specific operation mode this will keep it running
    /// </summary>
    /// <returns></returns>
    public async Task RunAsync(CancellationToken cancellationToken)
    {
        _logger.Information("Starting 'Run' process as " + _driverOperationMode.ToString());

        //Clear any errors from setup
        _errorFlags.ClearErrors();

        //Running server operations
        if (_driverOperationMode == OperationMode.Station)
        {
            _logger.Debug("Server operation start-up...");

            // ATTACH NMEA HANDLER
            //_myGPS.attachNMEAHandler(processNmea_Server);

            // ATTACH RTCM HANDLER            
            //_myGPS.attachRTCMHandler(processRtcm_Server);

            if (await _ublox.GetProtocolVersion())
            {
                _logger.Information($"Ublox Protocol Version: {await _ublox.GetProtocolVersionHigh()}.{await _ublox.GetProtocolVersionLow()}");
            }

            // Request receiver mode every 3 seconds
            using var timer = new System.Timers.Timer(3000)
            {
                AutoReset = true
            };

            timer.Elapsed += async (sender, e) =>
            {
                //_logger.Information($"Timer run in thread {Thread.CurrentThread.ManagedThreadId}");

                // Get receiver mode
                // var tmode3 = await _myGPS.getTmode3();
                // if (tmode3 != null)
                // {
                //     _logger.Information(tmode3.getMode());
                //     _socketCommunications.SendSyncData($"RECEIVER_MODE:{tmode3.getMode()}");
                // }
            };
            timer.Start();

            try
            {
                // Receive RTCM updates and send them to proper channels
                while (_ublox != null)
                {
                    _logger.Debug("246");

                    cancellationToken.ThrowIfCancellationRequested();

                    // If base station configured successfully, start pulling data from it
                    await _ublox.CheckUblox(); // See if new data is available. Process bytes as they come in.
                    await Task.Delay(10, cancellationToken);
                }
            }
            catch (OperationCanceledException)
            {
                _logger.Information($"Run thread {Environment.CurrentManagedThreadId} was cancelled.");
            }
            finally
            {
                timer.Stop();
                _logger.Information($"Run thread {Environment.CurrentManagedThreadId} exiting...");
            }
        }
        // Running client operations
        else if (_driverOperationMode == OperationMode.Client)
        {
            //_logger.Information("attaching nmea handler...");

            //ATTACH NMEA HANDLER
            //_myGPS.attachNMEAHandler(processNmea_Client);        //this is not needed, GPS coords will be passed thru UBX        

            //increase nav message output frequency
            _logger.Information("Set nav frequency, " + (await _ublox.setNavigationFrequency(CLIENT_NAV_FREQUENCY) ? "OK" : "Failed"));
            _logger.Information("Current update rate: " + await _ublox.getNavigationFrequency());


            //Set USB output to UBX only, no NMEA Noise
            await _ublox.SetUSBOutputAsync(UBLOX.Constants.COM_TYPE_UBX);

            //SEND RTCM to module when it becomes available
            new Task(() =>
            {
                var lastRtcmDataReceived = Time.millis();

                while (_ntripCasterSocket != null && _ntripCasterSocket.Connected && _socketCommunications.IsSyncDataSocketConnected())
                {
                    if (_ntripCasterSocket.Available > 0)
                    {
                        var ntripRcvLen = _ntripCasterSocket.Receive(_ntripInBuffer);

                        if (ntripRcvLen > 0)
                        {
                            _logger.Information("Sending RTCM info to F9p module...");

                            _ublox.Send(_ntripInBuffer, ntripRcvLen);

                            lastRtcmDataReceived = Time.millis();
                        }

                        _errorFlags.Remove("NTRIP_DATA_RCV_TIMEOUT");
                    }

                    //if no RTCM data received for longer than 10 minutes send error to UI
                    if (Time.millis() > lastRtcmDataReceived + NTRIP_RECEIVE_TIMEOUT * 1000)
                    {
                        _errorFlags.Add("NTRIP_DATA_RCV_TIMEOUT");
                    }

                    Thread.Sleep(10);
                }

                _logger.Information("Sockets needed for RTCM communication disconnected");
            }).Start();


            //Output nav data to console every nav cycle
            while (_socketCommunications.IsSyncDataSocketConnected())
            {
                await SendPositionDataAsync(["lat", "lon", "alt", "acc", "head"]);

                await Task.Delay(1100 / CLIENT_NAV_FREQUENCY, _cancellationTokenSourceForDriverThreadTask.Token);
            }

            _logger.Information("Driver Exited...");
        }
        else if (_driverOperationMode == OperationMode.Idle)
        {

            //Output nav data to console every nav cycle
            while (_socketCommunications.IsSyncDataSocketConnected())
            {
                await SendPositionDataAsync(["lat", "lon", "alt", "acc", "head"]);

                await Task.Delay(1100 / CLIENT_NAV_FREQUENCY);
            }
        }
    }

    /// <summary>
    /// Quesries module for most recent position information and sends all necessary data to UI
    /// <param name="dataPoints">All data point that needed to be outputed (lat, lon, alt, acc, head)</param>
    /// </summary>
    private async Task SendPositionDataAsync(string[] dataPoints)
    {
        var syncData = new SyncData
        {
            Latitude = dataPoints.Contains("lat") ? await _ublox.getHighResLatitude() / 10000000D : 0,
            Longitude = dataPoints.Contains("lon") ? await _ublox.getHighResLongitude() / 10000000D : 0,
            Altitude = dataPoints.Contains("alt") ? await _ublox.getAltitude() / 1000 : 0,
            Accuracy = dataPoints.Contains("acc") ? await _ublox.getPositionAccuracy() : 0,
            Heading = dataPoints.Contains("head") ? await _ublox.getHeading() / 100000D : 0
        };

        _socketCommunications.SendSyncData(syncData);
    }

    /// <summary>
    /// Send error flags to UI, this error handler is being called by the "ErrorFlags" class then new error comes up
    /// </summary>
    private void SendErrors(ErrorFlags errorFlags)
    {
        if (_socketCommunications.IsSyncDataSocketConnected())
        {
            _socketCommunications.SendSyncData(new SyncData() { Errors = errorFlags });
        }
        else
        {
            _logger.Information("Cannot send error output, sync socket is closed...");
        }
    }

    /// <summary>
    /// Destructof of class, disconnects all sockets
    /// </summary>
    /// <returns></returns>
    public void Cleanup()
    {
        // Deisconnect interprocess comm sockets
        _socketCommunications.CleanupSockets();

        // Disconect NTRIP Caster socket
        _ntripCasterSocket?.Disconnect(false);
        _ntripCasterSocket?.Close();

        // Close COM Port
        CloseSerialPort(_serialPort);
    }

    /// <summary>
    /// Configure f9p module as ntrip client (will consume data from NTRIP server and adjust coordinates)
    /// </summary>
    /// <returns></returns>
    private async Task ConfigureGpsAsNtripClientAsync(CancellationToken cancellationToken)
    {
        _serialPort = await OpenSerialPortAsync(_serialPortName);

        //_logger.Information("Create serial port connection for f9p module...");
        _ublox = new UBLOX.SFE_UBLOX_GPS();
        await _ublox.BeginAsync(_serialPort);

        //setup for debugging(needs to be a different serial port)
        //_myGPS.enableDebugging(serialPort, true);

        //Try reconnecting to NTRIP Caster 10 times every 10 seconds if connection fails 
        Threading.RetryHelper<NtripException>(() =>
        {
            //Update position data in UI during these retries
            SendPositionDataAsync(new[] { "lat", "lon", "alt", "acc", "head" }).GetAwaiter().GetResult();

            //start sending data to ntrip caster
            ConnectNtripCasterSocketAsync().GetAwaiter().GetResult();

            //send credentials to ntrip caster
            string ntripWelcomeMessage = "GET /" + _ntripMountpoint + " /HTTP/1.0\r\n";
            ntripWelcomeMessage += "User-Agent: NTRIP SOLVIT/1.0.1\r\n";
            //ntripWelcomeMessage += "Accept: */\r\n*";
            //ntripWelcomeMessage += "Connection: close \r\n";
            //ntripWelcomeMessage += "\r\n";

            _logger.Information("Trying to authenticate to NTRIP caster");

            //Send auth request
            _ntripCasterSocket.Send(Encoding.ASCII.GetBytes(ntripWelcomeMessage));

            //Receive respone with ICY 200 OK or ERROR - Bad Password
            var rcvLen = _ntripCasterSocket.Receive(_ntripInBuffer);
            var ntripResponseMessage = Encoding.ASCII.GetString(_ntripInBuffer, 0, rcvLen);

            if (ntripResponseMessage.Contains("ICY 200 OK\r\n"))
            {
                _logger.Information("Authentication passed!");
            }
            else if (ntripResponseMessage.Contains("SOURCETABLE 200 OK\r\n"))
            {
                throw new NtripException("Authentication passed. Sourcetable received but no RTCM data. base station is down?");
            }
            else
            {
                //send error response back to sync socket
                _errorFlags.Add("NTRIP_CONNECTION_ERROR");

                _logger.Information("NTRIP Response: \"" + ntripResponseMessage.Trim() + "\"");
                throw new NtripException("NTRIP Authentication error or station down");
            }

        }, 10, new TimeSpan(0, 0, 10), () => _logger.Information("trying to reconnect in 10 seconds..."));

        await RunAsync(cancellationToken);
    }

    /// <summary>
    /// Configure f9p module as ntrip client (will be sending out RTCM data to ntrip caster)
    /// </summary>
    /// <returns></returns>
    private async Task ConfigureGpsAsNtripStationAsync(CancellationToken cancellationToken)
    {
        try
        {
            _serialPort = await OpenSerialPortAsync(_serialPortName);

            // Set up UBLOX GPS
            _ublox = new UBLOX.SFE_UBLOX_GPS();
            _ublox.EnableDebugLogging(_logger);
            await _ublox.BeginAsync(_serialPort);

            // Set USB output to UBX AND RTCM only, no NMEA Noise
            await _ublox.SetUSBOutputAsync(UBLOX.Constants.COM_TYPE_UBX | UBLOX.Constants.COM_TYPE_RTCM3);

            // Engage proper receiver mode
            switch (_receiverMode)
            {
                case ReceiverModeEnum.SurveyIn:
                    await StartSurveyAsync(_surveyTime, _surveyAccuracy, cancellationToken);
                    break;
                case ReceiverModeEnum.FixedMode:
                    await StartFixedAsync(_latitude, _longitude, _altitude);
                    break;
            }

            // Check cancellation before continuing
            cancellationToken.ThrowIfCancellationRequested();

            // Connect to NTRIP caster socket
            _ntripCasterSocket = await ConnectNtripCasterSocketAsync();

            // Authenticate with NTRIP
            await AuthenticateNtripConnectionAsync(_ntripCasterSocket, _ntripMountpoint, _ntripPassword);

            // Run main logic
            await RunAsync(cancellationToken);
        }
        catch (OperationCanceledException)
        {
            _logger.Information("Configuration of GPS as NTRIP Station was canceled.");
        }
        finally
        {
            // Ensure serial port is closed
            CloseSerialPort(_serialPort);
            _logger.Debug("Serial port closed.");
        }
    }

    /// <summary>
    /// Opens the connection to the NTRIP (creates a socket)
    /// </summary>
    /// <returns></returns>
    private async Task<Socket> ConnectNtripCasterSocketAsync()
    {
        _logger.Debug("Connecting Ntrip Caster Socket...");

        if (NTRIP_SERVER_SIMULATION)
        {
            _logger.Debug("NTRIP SIMULATION IS ENABLED");
            return null;
        }

        if (_ntripServer != null && _ntripMountpoint != null && _ntripPassword != null && _ntripPort != 0)
        {
            _logger.Debug("Get DNS host entry...");

            //parse host entry to get a IP address for connection
            var hostEntry = await Dns.GetHostEntryAsync(_ntripServer);

            //Connect to server
            var ntripCasterSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);

            _logger.Information("Trying to connect to NTRIP caster");

            ntripCasterSocket.Connect(new IPEndPoint(hostEntry.AddressList[0], _ntripPort));

            return ntripCasterSocket;
        }
        else
        {
            throw new Exception("NTRIP configuration or credentials not provided");
        }
    }

    private async Task AuthenticateNtripConnectionAsync(Socket ntripCasterSocket, string mountPoint, string password)
    {
        // AUTHENTICATE ON NTRIP CASTER AS SERVER
        if (!NTRIP_SERVER_SIMULATION)
        {
            // Send credentials to ntrip caster
            string ntripWelcomeMessage = $"SOURCE {password} /{mountPoint}\r\n" +
                                         $"Source-Agent: NTRIP NtripServerPOSIX/Revision: 9109\r\n\r\n";

            // Send auth request
            _logger.Debug("Sending NTRIP welcome message...");
            ntripCasterSocket.Send(Encoding.ASCII.GetBytes(ntripWelcomeMessage));

            // Receive respone with ICY 200 OK or ERROR - Bad Password
            var rcvLen = ntripCasterSocket.Receive(_ntripInBuffer);
            var ntripResponseMessage = Encoding.ASCII.GetString(_ntripInBuffer, 0, rcvLen); //take first 30 bytes from response 

            _logger.Debug($"NTRIP Response: {ntripResponseMessage}");

            if (ntripResponseMessage.Equals("ICY 200 OK\r\n"))
            {
                _logger.Information("NTRIP Authentication passed!");
            }
            else
            {
                _logger.Error("NTRIP Authentiction error: " + ntripResponseMessage);
                throw new Exception("NTRIP Authentication error");
            }
        }
    }

    // Handler for NMEA data coming from the module when creating client (-client)
    private async Task processNmea_ClientAsync(char incoming)
    {
        processNmea_StationAsync(incoming);
    }

    /// <summary>
    /// handler for NMEA data coming from the module when creating server (-server)
    /// </summary>
    /// <param name="incoming">Incoming byte</param>
    /// <returns></returns>
    private async Task processNmea_StationAsync(char incoming)
    {
        _nmeaOutBuffer += incoming;

        //if (incoming == '\n') _logger.Information("NMEA NEWLINE RECEIVED");
        //if (_nmeaOutBuffer.Length >= NMEA_OUTGOING_STRING_MAXLEN) _logger.Information("NMEA MAXLEN ACHIEVED");


        if (incoming == '\n' || _nmeaOutBuffer.Length >= NMEA_OUTGOING_STRING_MAXLEN)
        {
            //send nmea info thru socket to the http server app
            _socketCommunications.SendNmeaData(_nmeaOutBuffer);
            _nmeaOutBuffer = "";
        }
    }

    /// <summary>
    /// Handler for RTCM data coming from the module when creating station (-server)
    /// </summary>
    /// <param name="incoming">Incoming byte</param>
    /// <returns></returns>
    private async Task ProcessRtcm_StationAsync(byte incoming)
    {

        // Send rtcm info thru socket to the UI http server app if needed
        if (_socketCommunications.IsRtcmDataSocketConnected())
        {
            _socketCommunications.SendRtcmData(incoming);
        }

        //push RTCM bute into the receive buffer until it reaches maximum defined size
        _ntripOutBuffer[_rtcmFrame++] = incoming;

        //once max buffer size is reached send rtcm data to the RTCM caster thru the socket via NTRIP protocol
        if (_rtcmFrame == RTCM_OUTGOING_BUFFER_SIZE)
        {
            _rtcmFrame = 0; //reset buffer

            if (NTRIP_SERVER_SIMULATION)
            {
                if (Time.millis() > _nextNtripStatusSendTime)
                {
                    //_socketCommunications.SendSyncData("NTRIP_SENT");
                    _nextNtripStatusSendTime = Time.millis() + 1000;
                }
                _logger.Information("Sending RTCM to NTRIP Server");
                Thread.Sleep(250); return;
            }

            //check if ntrip socket is defined and connected
            if (_ntripCasterSocket != null && _ntripCasterSocket.Connected)
            {
                _logger.Information("Sending RTCM to NTRIP Caster");

                //Do not send this more than once a second to sync
                if (Time.millis() > _nextNtripStatusSendTime)
                {
                    _socketCommunications.SendSyncData(new SyncData() { LastNtripSent = DateTime.Now });
                    _nextNtripStatusSendTime = Time.millis() + 1000;
                }

                _ntripCasterSocket.Send(_ntripOutBuffer);
            }
            else
            {
                //check if socket was closed
                _logger.Information("Ntrip Socket disconnected?");
            }
        }
    }

    /// <summary>
    /// Check incoming data thru the sync socket for commands coming from the frontend
    /// </summary>
    /// <returns></returns>
    private async Task CheckSyncDataSocketAsync(CancellationToken cancellationToken)
    {
        // Check if socket is connected
        if (_socketCommunications.IsSyncDataSocketConnected())
        {
            // Check if anything in available in a socket
            var syncRcvLen = _socketCommunications.ReceiveSyncData(_syncInBuffer);

            if (syncRcvLen > 0)
            {
                var syncMessage = Encoding.ASCII.GetString(_syncInBuffer, 0, syncRcvLen);

                _logger.Information("Sync Message Received: " + syncMessage);

                try
                {
                    var command = new SyncIncomingCommand(syncMessage);

                    await ProcessSyncCommandAsync(command, cancellationToken);
                }
                catch (UnknownSyncCommandException e)
                {
                    // Spit out errors if anything dyring command parsing
                    _logger.Information(e.Message);
                }
            }
        }
        else
        {
            if (!IGNORE_DISCONNECTED_SOCKET)
                throw new SyncSocketException("Sync Socket Disconnected, cannot continue, driver must shutdown...");
        }
    }

    private async Task ProcessSyncCommandAsync(SyncIncomingCommand command, CancellationToken cancellationToken)
    {
        if (command.Type == SyncIncomingCommandType.RESTART_SURVEY)
        {
            // Receive new params from UI
            _surveyAccuracy = command.GetValue<float>();
            _surveyTime = command.GetValue<int>(1);
            _receiverMode = ReceiverModeEnum.SurveyIn;

            // Disable receiver survey mode bewfore re-enabling with new settings
            if (await _ublox.DisableReceiver()) _logger.Information("Receiver disabled");

            StopCurrentDriverThread();          

            // Reset cancellation token
            _cancellationTokenSourceForDriverThreadTask = new CancellationTokenSource();

            // Enable receiver with new settings
            _driverRunningThreadTask = Task.Run(() => ConfigureGpsAsNtripStationAsync(_cancellationTokenSourceForDriverThreadTask.Token),
                _cancellationTokenSourceForDriverThreadTask.Token);

        }
        else if (command.Type == SyncIncomingCommandType.RESTART_FIXED)
        {

            // Receive new params from UI
            _latitude = int.Parse(command.GetValue<string>().Replace(".", ""));
            _longitude = int.Parse(command.GetValue<string>(1).Replace(".", ""));
            _altitude = command.GetValue<int>(2);
            _receiverMode = ReceiverModeEnum.FixedMode;

            // Disable receiver survey mode bewfore re-enabling with new settings
            if (await _ublox.DisableReceiver()) _logger.Information("Receiver disabled");

            StopCurrentDriverThread();

            // Reset cancellation token
            _cancellationTokenSourceForDriverThreadTask = new CancellationTokenSource();

            // Enable receiver with new settings
            _driverRunningThreadTask = Task.Run(() => ConfigureGpsAsNtripStationAsync(_cancellationTokenSourceForDriverThreadTask.Token),
                _cancellationTokenSourceForDriverThreadTask.Token);

        }
    }

    private void StopCurrentDriverThread()
    {
        try
        {
            // Stop main running thread
            _cancellationTokenSourceForDriverThreadTask.Cancel();

            // Wait for running thread to stop
            _driverRunningThreadTask.Wait();
        }
        catch (AggregateException ex)
        {
            // Handle other exceptions that may be thrown by Wait
            foreach (var innerEx in ex.InnerExceptions)
            {
                if (innerEx is TaskCanceledException)
                {
                    _logger.Information("Driver thread was canceled during the wait.");
                }
                else
                {
                    _logger.Error(innerEx, "An error occurred while waiting for the driver thread to stop.");
                }
            }
        }
        catch (TaskCanceledException)
        {
            _logger.Information("Driver thread was already canceled or canceled during the wait.");
        }
        catch (Exception ex)
        {
            _logger.Error(ex, "An unexpected error occurred while stopping the driver thread.");
        }
    }

    /// <summary>
    /// Enables Survey mode on f9p device
    /// </summary>
    /// <param name="rtcmSurveyTime"></param>
    /// <param name="rtcmAccuracy"></param>
    /// <returns></returns>
    private async Task StartSurveyAsync(int rtcmSurveyTime, float rtcmAccuracy, CancellationToken cancellationToken)
    {
        if (_surveyAccuracy == 0 || _surveyTime == 0) throw new InvalidDataException("Survey accuracy or minimum time not provided, cannot continue...");

        _logger.Information("Enable Survey Mode, minimum " + rtcmSurveyTime + "sec and minimum accuracy " + rtcmAccuracy + " meters");

        var moduleResponse = await _ublox.EnableSurveyMode((ushort)rtcmSurveyTime, rtcmAccuracy);
        if (!moduleResponse) throw new Exception("710 Unable to properly configure module");

        await DisableRtcmMessagesOnUsbAsync();
        _ublox.AttachRTCMHandler(null);

        // Read receiver mode
        var surveyMode = await _ublox.GetTmode3();
        _logger.Information("Accuracy: " + surveyMode.GetAccuracyLimit() + "m");
        _logger.Information("Mode: " + surveyMode.GetMode().ToString());
        _socketCommunications.SendSyncData(new SyncData()
        {
            ModuleCurrentSetAccuracy = surveyMode.GetAccuracyLimit(),
            ReceiverMode = surveyMode.GetMode(),
        });

        _logger.Information("Start requesting survey status...");

        // Set proper navigation frequency for base
        await _ublox.setNavigationFrequency(4);

        // Wait for survey to complete
        while (_ublox.svin.valid == false)
        {
            // Exit if cancellation requested
            cancellationToken.ThrowIfCancellationRequested();

            // Check sync data socket for updates
            await CheckSyncDataSocketAsync(cancellationToken);

            // Query module for SVIN status with 2000ms timeout (req can take a long time)
            moduleResponse = await _ublox.getSurveyStatus(2000);

            // If module response if always true
            if (moduleResponse)
            {
                // Send data back to UI
                await SendPositionDataAsync(["lat", "lon", "alt"]); //add regular data
                _socketCommunications.SendSyncData(new SyncData()
                {
                    Accuracy = _ublox.svin.meanAccuracy,
                    SurveyTime = _ublox.svin.observationTime,
                    IsSurveyValid = _ublox.svin.valid,
                });

                // Show output in a console
                var output = "Time elapsed: " + _ublox.svin.observationTime + "s";
                output += " Accuracy: " + _ublox.svin.meanAccuracy + "m";
                output += " Is Valid?: " + _ublox.svin.valid;
                _logger.Information(output);
            }
            else
            {
                _logger.Information("SVIN request failed");
            }

            await _ublox.CheckUblox(); //See if new data is available in COM PORT and consume it

            await Task.Delay(100, cancellationToken);
        }

        // Send location data again in case survey has started as valid already
        {
            await SendPositionDataAsync(["lat", "lon", "alt", "acc"]);
        }

        _logger.Information("Base survey complete! RTCM can now be broadcast");

        await EnableRtcmMessagesOnUsbAsync();
        _ublox.AttachRTCMHandler(ProcessRtcm_StationAsync);
    }

    /// <summary>
    /// Enables Fixed mode on f9p device
    /// </summary>
    /// <param name="latitude"></param>
    /// <param name="longitude"></param>
    /// <param name="altitude"></param>
    /// <returns></returns>
    private async Task StartFixedAsync(int latitude, int longitude, int altitude)
    {

        if (latitude == 0 || longitude == 0 || altitude == 0) throw new InvalidDataException("Latitude, longitude or altitude not provided, cannot continue...");

        await DisableRtcmMessagesOnUsbAsync();

        _logger.Information("Enable Fixed Mode, Lat: " + latitude + " Lon: " + longitude + " Alt: " + altitude);

        var moduleResponse = await _ublox.enableFixedMode(latitude, longitude, altitude); //Enable Survey in, 60 seconds, 5.0m  

        var surveyMode = await _ublox.GetTmode3();
        _socketCommunications.SendSyncData(new SyncData()
        {
            ReceiverMode = surveyMode.GetMode()
        });
        ///_socketCommunications.SendSyncData("RECEIVER_MODE:" + surveyMode.GetMode());

        if (!moduleResponse) throw new Exception("Unable to properly configure module");

        await SendPositionDataAsync(["lat", "lon", "alt", "acc"]);

        _logger.Information("Base setting complete! RTCM can now be broadcast");

        await EnableRtcmMessagesOnUsbAsync();
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

        if (!moduleResponse) throw new Exception("Unable to properly configure module");
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

        if (!moduleResponse) throw new Exception("833 Unable to properly configure module");
    }

    private void CloseSerialPort(SerialPort serialPort)
    {
        _logger.Information($"Closing Serial Port{serialPort.PortName}");
        serialPort.Close();
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
}
