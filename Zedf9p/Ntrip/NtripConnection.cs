using System;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Zedf9p.Exceptions;

namespace Zedf9p.Ntrip
{
    /// <summary>
    /// One authenticated connection to an NTRIP caster, as a source (base) or a client (rover).
    /// </summary>
    internal sealed class NtripConnection : IDisposable
    {
        private static readonly TimeSpan HandshakeTimeout = TimeSpan.FromSeconds(15);
        private const int SendTimeoutMs = 5000;
        private const int ReplyBufferSize = 4096;

        private readonly Socket _socket;
        private bool _disposed;

        /// <summary>Stream bytes that arrived in the same read as the caster's reply.</summary>
        public byte[] InitialData { get; }

        private NtripConnection(Socket socket, byte[] initialData)
        {
            _socket = socket;
            InitialData = initialData;
        }

        public static Task<NtripConnection> OpenSourceAsync(string host, int port, string mountpoint, string password, CancellationToken cancellationToken) =>
            OpenAsync(host, port, NtripProtocol.BuildSourceRequest(mountpoint, password), cancellationToken);

        public static Task<NtripConnection> OpenClientAsync(string host, int port, string mountpoint, string username, string password, CancellationToken cancellationToken) =>
            OpenAsync(host, port, NtripProtocol.BuildClientRequest(mountpoint, username, password), cancellationToken);

        private static async Task<NtripConnection> OpenAsync(string host, int port, string request, CancellationToken cancellationToken)
        {
            var socket = new Socket(SocketType.Stream, ProtocolType.Tcp)
            {
                NoDelay = true,
                SendTimeout = SendTimeoutMs,
            };
            socket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.KeepAlive, true);

            using var handshake = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
            handshake.CancelAfter(HandshakeTimeout);

            try
            {
                await socket.ConnectAsync(host, port, handshake.Token);
                await socket.SendAsync(Encoding.ASCII.GetBytes(request), SocketFlags.None, handshake.Token);

                var reply = new byte[ReplyBufferSize];
                var received = await socket.ReceiveAsync(reply, SocketFlags.None, handshake.Token);
                if (received == 0)
                {
                    throw new NtripException("The caster closed the connection without replying", NtripResponseKind.Rejected);
                }

                var text = Encoding.ASCII.GetString(reply, 0, received);
                var kind = NtripProtocol.Classify(text);
                if (kind != NtripResponseKind.Ok)
                {
                    throw new NtripException($"The caster refused the connection ({kind}): {FirstLine(text)}", kind);
                }

                // Everything after the status line is treated as stream data; the F9P ignores stray CR/LF bytes
                var lineEnd = text.IndexOf("\r\n", StringComparison.Ordinal);
                var initialData = lineEnd < 0 ? [] : reply.Skip(lineEnd + 2).Take(received - lineEnd - 2).ToArray();

                return new NtripConnection(socket, initialData);
            }
            catch (OperationCanceledException) when (!cancellationToken.IsCancellationRequested)
            {
                socket.Dispose();
                throw new NtripException($"Timed out connecting to {host}:{port}", NtripResponseKind.Rejected);
            }
            catch (SocketException e)
            {
                socket.Dispose();
                throw new NtripException($"Cannot reach {host}:{port}: {e.Message}", null);
            }
            catch
            {
                socket.Dispose();
                throw;
            }
        }

        private static string FirstLine(string text)
        {
            var end = text.IndexOf('\r');
            var line = end < 0 ? text : text[..end];
            return line.Length > 120 ? line[..120] : line;
        }

        /// <summary>
        /// False once the caster has closed the connection. A socket only reports this when asked.
        /// </summary>
        public bool IsConnected
        {
            get
            {
                if (_disposed) return false;
                try
                {
                    return _socket.Connected && !(_socket.Poll(0, SelectMode.SelectRead) && _socket.Available == 0);
                }
                catch (Exception e) when (e is SocketException or ObjectDisposedException)
                {
                    return false;
                }
            }
        }

        public void Send(byte[] buffer, int count) => _socket.Send(buffer, 0, count, SocketFlags.None);

        public ValueTask<int> ReceiveAsync(Memory<byte> buffer, CancellationToken cancellationToken) =>
            _socket.ReceiveAsync(buffer, SocketFlags.None, cancellationToken);

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;

            try { _socket.Shutdown(SocketShutdown.Both); }
            catch (Exception e) when (e is SocketException or ObjectDisposedException) { /* Already gone */ }

            _socket.Dispose();
        }
    }
}
