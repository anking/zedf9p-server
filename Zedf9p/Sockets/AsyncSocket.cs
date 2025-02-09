using Serilog;
using System;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using Zedf9p.Unix;
using Zedf9p.Utils;

namespace Zedf9p.Sockets
{
    class AsyncSocket(string location, ILogger logger)
    {
        private readonly string _socketLocation = location;                 //location of the interprocess socket(used to communicate with nodejs server)
        Socket _socket;                                                     //instance of .net core socket
        readonly ILogger _logger = logger;
        private const int MaxRetries = 10;                                  // Maximum number of retries before giving up
        private readonly TimeSpan RetryDelay = TimeSpan.FromSeconds(1);     // Delay between retries (1 second)


        public int BytesAvailable
        {
            get => _socket.Available;
        }

        /// <summary>
        /// Connect to f9p socket for interprocess communication
        /// </summary>
        public AsyncSocket Connect()
        {
            // Using the RetryHelper to handle retries for the connection
            Action connectAction = () =>
            {
                _logger.Information("Trying to connect to interprocess socket " + _socketLocation);

                // Check if the socket file exists
                if (!File.Exists(_socketLocation))
                {
                    _logger.Warning($"Socket file does not exist at {_socketLocation}. Will retry after {RetryDelay.TotalSeconds} seconds.");
                    throw new FileNotFoundException($"Socket file does not exist at {_socketLocation}");
                }

                // Set socket parameters for interprocess socket
                _socket = new Socket(AddressFamily.Unix, SocketType.Stream, ProtocolType.IP)
                {
                    SendTimeout = 500
                };

                // Connect to the Unix socket endpoint
                _socket.Connect(new UnixEndPoint(_socketLocation)); // Connect to the socket
                _logger.Information("Successfully connected to socket at " + _socketLocation);
            };

            // Use RetryHelper to try the connect action with retries
            connectAction.RetryHelper<Exception>(MaxRetries, RetryDelay, () =>
            {
                // This action will be called after each failed retry
                _logger.Warning($"Retrying connection attempt to {_socketLocation}");
            });

            return this; // Return the current instance after a successful connection or after retries
        }

        /// <summary>
        /// Disconnects the socket and releases any associated resources.
        /// </summary>
        public void Disconnect()
        {
            if (_socket == null)
            {
                _logger.Information("Socket is already null, no disconnect needed.");
                return;
            }

            try
            {
                _logger.Information("Disconnecting socket...");

                // Disconnect the socket, false indicates this connection cannot be reused
                _socket.Disconnect(false);

                // Properly close and dispose of the socket
                _socket.Close();
                _socket.Dispose();

                _logger.Information("Socket successfully disconnected and disposed.");
            }
            catch (Exception ex)
            {
                // Log any errors encountered during the disconnection process
                _logger.Error($"Error disconnecting socket: {ex.Message}");
            }
            finally
            {
                // Nullify the socket after disconnecting to avoid using an invalid reference
                _socket = null;
            }
        }

        /// <summary>
        /// Checks if the socket is connected and can be used.
        /// </summary>
        /// <returns>true if the socket is connected, otherwise false</returns>
        public bool IsConnected()
        {
            return _socket?.Connected ?? false;
        }

        /// <summary>
        /// Sends ASCII text into the socket.
        /// </summary>
        /// <param name="text">Text string</param>
        /// <returns>Number of bytes sent to the socket, or -1 if the socket is not connected or an error occurred</returns>
        public int Send(string text)
        {
            if (_socket == null || !_socket.Connected)
            {
                _logger.Warning("Socket is not connected.");
                return -1;  // Sentinel value to indicate failure
            }

            try
            {
                var data = Encoding.ASCII.GetBytes(text);
                return _socket.Send(data);
            }
            catch (SocketException e)
            {
                _logger.Error($"SocketException while sending data: {e.Message}");
            }
            catch (ObjectDisposedException e)
            {
                _logger.Error($"Socket is disposed: {e.Message}");
            }
            catch (Exception e)
            {
                _logger.Error($"Unexpected error while sending data: {e.Message}");
            }

            return -1;  // Return a sentinel value in case of failure
        }

        /// <summary>
        /// Sends ASCII text string into socket terminated by \r\n
        /// </summary>
        /// <param name="text">Text string</param>
        /// <returns>Number of bytes sent to socket, or null if socket is not connected</returns>
        public int SendLine(string text) => Send(text + "\r\n");

        /// <summary>
        /// Sends a single byte into the socket.
        /// </summary>
        /// <param name="incoming">Byte to be sent</param>
        /// <returns>Number of bytes sent to the socket, or -1 if socket is not connected or an error occurred</returns>
        public int Send(byte incoming)
        {
            if (_socket == null || !_socket.Connected)
            {
                _logger.Warning("Socket is not connected.");
                return -1;  // Sentinel value to indicate failure
            }

            try
            {
                return _socket.Send([incoming]);
            }
            catch (SocketException e)
            {
                _logger.Error($"SocketException while sending data: {e.Message}");
            }
            catch (ObjectDisposedException e)
            {
                _logger.Error($"Socket is disposed: {e.Message}");
            }
            catch (Exception e)
            {
                _logger.Error($"Unexpected error while sending data: {e.Message}");
            }

            return -1;  // Return a sentinel value in case of failure
        }

        /// <summary>
        /// Receive from socket buffer
        /// </summary>
        /// <param name="text">Text string</param>
        /// <returns>Number of bytes sent to socket</returns>
        public int Receive(byte[] buffer) => _socket.Receive(buffer);

    }
}
