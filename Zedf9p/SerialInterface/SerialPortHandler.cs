using System;
using System.IO.Ports;
using System.Threading.Tasks;
using Serilog;

namespace Zedf9p.SerialInterface
{
    public class SerialPortHandler(ILogger logger) : ISerialPortHandler
    {
        private readonly ILogger _logger = logger;

        /// <summary>
        /// Opens a serial port connection asynchronously.
        /// </summary>
        /// <param name="portName">The name of the serial port.</param>
        /// <param name="timeout">Timeout for opening the port.</param>
        /// <returns>Returns the opened SerialPort instance or null if failed.</returns>
        public async Task<SerialPort> OpenPortAsync(
            string portName,
            TimeSpan timeout,
            int baudRate = 115200,
            Parity parity = Parity.None,
            int dataBits = 8,
            StopBits stopBits = StopBits.One)
        {
            try
            {
                var serialPort = new SerialPort(portName, 115200, Parity.None, 8, StopBits.One)
                {
                    ReadTimeout = 500,
                    WriteTimeout = 500
                };

                // Attempt to open the serial port in a separate task with a timeout
                var openPortTask = Task.Run(() =>
                {
                    try
                    {
                        _logger.Information($"Trying to open port {portName}...");
                        serialPort.Open();
                        _logger.Information("Serial port opened successfully.");
                    }
                    catch (Exception ex)
                    {
                        _logger.Error($"Failed to open serial port: {ex.Message}");
                        throw;
                    }
                });

                var completedTask = await Task.WhenAny(openPortTask, Task.Delay(timeout));

                if (completedTask == openPortTask && serialPort.IsOpen)
                {
                    return serialPort;
                }
                else
                {
                    _logger.Error($"Timeout reached while trying to open port {portName}.");
                    serialPort.Dispose();
                    return null;
                }
            }
            catch (Exception ex)
            {
                _logger.Error($"Exception occurred while trying to open port {portName}: {ex.Message}");
                return null;
            }
        }
    }
}