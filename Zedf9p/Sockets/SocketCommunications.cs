﻿using Serilog;
using System;
using System.Net.Sockets;

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

        // Constructor to accept socket paths
        public SocketCommunications(string nmeaSocketPath, string rtcmSocketPath, string syncSocketPath, ILogger logger)
        {
            NmeaDataSocketLocation = nmeaSocketPath ?? throw new ArgumentNullException(nameof(nmeaSocketPath));
            RtcmDataSocketLocation = rtcmSocketPath ?? throw new ArgumentNullException(nameof(rtcmSocketPath));
            SyncDataSocketLocation = syncSocketPath ?? throw new ArgumentNullException(nameof(syncSocketPath));
            _logger = logger;

            _logger.Information($"Socket paths set: NMEA: {NmeaDataSocketLocation}, RTCM: {RtcmDataSocketLocation}, Sync: {SyncDataSocketLocation}");
        }

        // Connect to f9p socket for interprocess communication
        public void ConnectDataSockets()
        {
            try
            {
                _logger.Information("Connecting to interprocess sockets...");

                // Client socket for HTTP server running on Node
                _syncDataSocket = new AsyncSocket(SyncDataSocketLocation, _logger).Connect();
                _nmeaDataSocket = new AsyncSocket(NmeaDataSocketLocation, _logger).Connect();
                _rtcmDataSocket = new AsyncSocket(RtcmDataSocketLocation, _logger).Connect();
            }
            catch (SocketException e)
            {
                // Unable to connect to the socket, spit out error in a console
                _logger.Error("Socket connection error: " + e.Message);
            }
        }

        // Check if the Sync data socket is connected
        public bool IsSyncDataSocketConnected()
        {
            return _syncDataSocket?.IsConnected() ?? false;
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

        public void SendSyncData(string data) => SendData(_syncDataSocket, "Sync", data);

        public void SendNmeaData(string data) => SendData(_nmeaDataSocket, "NMEA", data);

        public void SendRtcmData(byte data) => SendData(_rtcmDataSocket, "RTCM", data);

        private void SendData<T>(AsyncSocket socket, string socketName, T data)
        {
            if (data == null || (data is byte[] byteArray && byteArray.Length == 0))
            {
                _logger.Warning($"Attempted to send empty or null data to {socketName} socket.");
                return;
            }

            if (socket == null)
            {
                _logger.Error($"{socketName} socket is NULL. Cannot send data.");
                return;
            }

            _logger.Debug($"Checking {socketName} socket connection...");

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
                    _logger.Debug($"Sending to {socketName} socket: {byteData}");
                    socket.Send(byteData);
                }
                else
                {
                    _logger.Warning($"Unsupported data type for {socketName} socket: {data.GetType()}");
                }

                _logger.Information($"{socketName} data sent successfully.");
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
