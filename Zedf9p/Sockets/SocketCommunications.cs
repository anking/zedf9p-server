using Newtonsoft.Json;
using Serilog;
using System;
using Zedf9p.DTO;

namespace Zedf9p.Sockets
{
    internal class SocketCommunications : ISocketCommunications
    {
        private string NmeaDataSocketLocation { get; set; }
        private string RtcmDataSocketLocation { get; set; }
        private string SyncDataSocketLocation { get; set; }

        private AsyncSocket _nmeaDataSocket;
        private AsyncSocket _rtcmDataSocket;
        private AsyncSocket _syncDataSocket;
        private readonly ILogger _logger;

        // Position and error updates come from different tasks: keep each JSON line whole
        private readonly object _syncSendLock = new();

        // Constructor to accept socket paths
        public SocketCommunications(string nmeaSocketPath, string rtcmSocketPath, string syncSocketPath, ILogger logger)
        {
            NmeaDataSocketLocation = nmeaSocketPath;
            RtcmDataSocketLocation = rtcmSocketPath;
            SyncDataSocketLocation = syncSocketPath;
            _logger = logger;

            _logger.Information($"Socket paths set: NMEA: {NmeaDataSocketLocation}, RTCM: {RtcmDataSocketLocation}, Sync: {SyncDataSocketLocation}");
        }

        // Connect to f9p socket for interprocess communication
        public void ConnectDataSockets()
        {
            _logger.Information("Connecting to interprocess sockets...");

            // Client socket for HTTP server running on Node
            TryConnectSocket(SyncDataSocketLocation, ref _syncDataSocket, "Sync");
            TryConnectSocket(NmeaDataSocketLocation, ref _nmeaDataSocket, "NMEA");
            TryConnectSocket(RtcmDataSocketLocation, ref _rtcmDataSocket, "RTCM");
        }

        private void TryConnectSocket(string socketPath, ref AsyncSocket socket, string socketName)
        {
            if (string.IsNullOrWhiteSpace(socketPath))
            {
                _logger.Information($"{socketName} socket is disabled (no path provided).");
                return;
            }

            // A configured socket that cannot be reached is a startup failure: the owner restarts this process
            socket = new AsyncSocket(socketPath, _logger).Connect();
            _logger.Information($"{socketName} socket connected successfully.");
        }

        // Check if the Sync data socket is connected
        public bool IsSyncDataSocketConnected()
        {
            return _syncDataSocket?.IsConnected() ?? false;
        }

        public bool IsSyncDataSocketClosedByPeer()
        {
            return _syncDataSocket?.IsClosedByPeer() ?? false;
        }

        // Check if the RTCM data socket is connected
        public bool IsRtcmDataSocketConnected()
        {
            return _rtcmDataSocket?.IsConnected() ?? false;
        }

        // Check if the NMEA data socket is connected
        public bool IsNmeaDataSocketConnected()
        {
            return _nmeaDataSocket?.IsConnected() ?? false;
        }

        public void CleanupSockets()
        {
            // in the end, disconnect and close the socket to cleanup
            _rtcmDataSocket?.Disconnect();
            _nmeaDataSocket?.Disconnect();
            _syncDataSocket?.Disconnect();
        }

        public void SendSyncData(SyncData syncData)
        {
            string jsonData = JsonConvert.SerializeObject(syncData, new JsonSerializerSettings
            {
                NullValueHandling = NullValueHandling.Ignore // Ignore null values
            }) + "\n"; // Append newline as separator

            lock (_syncSendLock)
            {
                SendData(_syncDataSocket, "Sync", jsonData);
            }
        }

        public void SendNmeaData(string data) => SendData(_nmeaDataSocket, "NMEA", data);

        public void SendRtcmData(byte data) => SendData(_rtcmDataSocket, "RTCM", data);

        private void SendData<T>(AsyncSocket socket, string socketName, T data)
        {
            if (data == null || (data is byte[] byteArray && byteArray.Length == 0))
            {
                _logger.Warning($"Attempted to send empty or null data to {socketName} socket.");
                return;
            }

            // A socket without a path is disabled on purpose, nothing to report
            if (socket == null) return;

            if (!socket.IsConnected())
            {
                _logger.Warning($"{socketName} socket is not connected. Cannot send data.");
                return;
            }

            try
            {
                if (data is string strData)
                {
                    _logger.Debug($"Sending to {socketName} socket: {strData}");
                    socket.SendLine(strData);
                }
                else if (data is byte byteData)
                {
                    socket.Send(byteData);
                }
                else
                {
                    _logger.Warning($"Unsupported data type for {socketName} socket: {data.GetType()}");
                }
            }
            catch (Exception ex)
            {
                _logger.Error(ex, $"Failed to send data to {socketName}: {data}");
            }
        }

        /// <summary>
        /// Returns length of data received
        /// </summary>
        /// <param name="_syncInBuffer"></param>
        /// <returns></returns>
        public int ReceiveSyncData(byte[] syncInBuffer)
        {
            if (_syncDataSocket.BytesAvailable > 0)
            {
                return _syncDataSocket.Receive(syncInBuffer);
            }

            return 0;
        }
    }
}
