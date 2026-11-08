using System;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Xunit;
using Zedf9p.Exceptions;
using Zedf9p.Ntrip;

namespace Zedf9p.Tests;

public class NtripConnectionTests
{
    /// <summary>A caster that gives one reply and reports what the client sent.</summary>
    private static async Task<(int Port, Task<string> Request)> StartCasterAsync(byte[] reply, CancellationToken cancellationToken)
    {
        var listener = new TcpListener(IPAddress.Loopback, 0);
        listener.Start();
        var port = ((IPEndPoint)listener.LocalEndpoint).Port;

        var request = Task.Run(async () =>
        {
            using var client = await listener.AcceptTcpClientAsync(cancellationToken);
            var stream = client.GetStream();
            var buffer = new byte[4096];
            var text = new StringBuilder();
            while (!text.ToString().Contains("\r\n\r\n"))
            {
                var read = await stream.ReadAsync(buffer, cancellationToken);
                if (read == 0) break;
                text.Append(Encoding.ASCII.GetString(buffer, 0, read));
            }

            await stream.WriteAsync(reply, cancellationToken);
            await stream.FlushAsync(cancellationToken);
            listener.Stop();
            await Task.Delay(200, cancellationToken);
            return text.ToString();
        }, cancellationToken);

        await Task.Yield();
        return (port, request);
    }

    [Fact]
    public async Task Client_connects_and_keeps_stream_bytes_that_follow_the_status_line()
    {
        var ct = TestContext.Current.CancellationToken;
        var stream = new byte[] { 0xD3, 0x00, 0x01, 0xAA, 0x00, 0x00, 0x00 };
        var reply = Encoding.ASCII.GetBytes("ICY 200 OK\r\n").Concat(stream).ToArray();
        var (port, request) = await StartCasterAsync(reply, ct);

        using var connection = await NtripConnection.OpenClientAsync("127.0.0.1", port, "MOUNT", "rover@example.com", "none", ct);

        Assert.StartsWith("GET /MOUNT HTTP/1.0\r\n", await request);
        Assert.Equal(stream, connection.InitialData);
    }

    [Fact]
    public async Task Source_connects_on_an_icy_reply()
    {
        var ct = TestContext.Current.CancellationToken;
        var (port, request) = await StartCasterAsync(Encoding.ASCII.GetBytes("ICY 200 OK\r\n"), ct);

        using var connection = await NtripConnection.OpenSourceAsync("127.0.0.1", port, "MOUNT", "secret", ct);

        Assert.StartsWith("SOURCE secret /MOUNT\r\n", await request);
        Assert.Empty(connection.InitialData);
    }

    [Theory]
    [InlineData("SOURCETABLE 200 OK\r\n\r\n", NtripResponseKind.SourceTable)]
    [InlineData("ERROR - Bad Password\r\n", NtripResponseKind.Rejected)]
    [InlineData("HTTP/1.1 403 Forbidden\r\n\r\nbanned", NtripResponseKind.Banned)]
    public async Task A_refusal_reports_what_the_caster_said(string reply, NtripResponseKind expected)
    {
        var ct = TestContext.Current.CancellationToken;
        var (port, _) = await StartCasterAsync(Encoding.ASCII.GetBytes(reply), ct);

        var e = await Assert.ThrowsAsync<NtripException>(() => NtripConnection.OpenClientAsync("127.0.0.1", port, "MOUNT", "a@b.co", "", ct));

        Assert.Equal(expected, e.Response);
    }

    [Fact]
    public async Task An_unreachable_caster_is_an_ntrip_failure_with_no_reply()
    {
        var ct = TestContext.Current.CancellationToken;
        var listener = new TcpListener(IPAddress.Loopback, 0);
        listener.Start();
        var port = ((IPEndPoint)listener.LocalEndpoint).Port;
        listener.Stop();

        var e = await Assert.ThrowsAsync<NtripException>(() => NtripConnection.OpenSourceAsync("127.0.0.1", port, "MOUNT", "secret", ct));

        Assert.Null(e.Response);
    }

    [Fact]
    public async Task IsConnected_turns_false_when_the_caster_closes()
    {
        var ct = TestContext.Current.CancellationToken;
        var (port, request) = await StartCasterAsync(Encoding.ASCII.GetBytes("ICY 200 OK\r\n"), ct);

        using var connection = await NtripConnection.OpenSourceAsync("127.0.0.1", port, "MOUNT", "secret", ct);
        await request; // The caster closes its side when this completes

        for (var i = 0; i < 50 && connection.IsConnected; i++)
        {
            await Task.Delay(100, ct);
        }

        Assert.False(connection.IsConnected);
    }
}
