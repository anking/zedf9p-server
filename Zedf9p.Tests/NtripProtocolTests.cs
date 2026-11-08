using System;
using System.Text;
using Xunit;
using Zedf9p.Ntrip;

namespace Zedf9p.Tests;

public class NtripProtocolTests
{
    [Fact]
    public void Client_request_is_well_formed_and_carries_the_email_as_user_name()
    {
        var request = NtripProtocol.BuildClientRequest("MOUNT", "rover@example.com", "none");

        Assert.StartsWith("GET /MOUNT HTTP/1.0\r\n", request);
        var expected = Convert.ToBase64String(Encoding.ASCII.GetBytes("rover@example.com:none"));
        Assert.Contains($"Authorization: Basic {expected}\r\n", request);
        Assert.EndsWith("\r\n\r\n", request);
    }

    [Fact]
    public void Client_request_without_a_user_has_no_authorization_header()
    {
        Assert.DoesNotContain("Authorization", NtripProtocol.BuildClientRequest("MOUNT", "", ""));
    }

    [Fact]
    public void Source_request_matches_ntrip_rev1()
    {
        var request = NtripProtocol.BuildSourceRequest("MOUNT", "secret");

        Assert.StartsWith("SOURCE secret /MOUNT\r\n", request);
        Assert.EndsWith("\r\n\r\n", request);
    }

    [Theory]
    [InlineData("ICY 200 OK\r\n", NtripResponseKind.Ok)]
    [InlineData("ICY 200 OK\r\n\r\n\xD3", NtripResponseKind.Ok)]
    [InlineData("HTTP/1.1 200 OK\r\nContent-Type: gnss/data\r\n\r\n", NtripResponseKind.Ok)]
    [InlineData("SOURCETABLE 200 OK\r\nContent-Type: text/plain\r\n\r\n", NtripResponseKind.SourceTable)]
    [InlineData("HTTP/1.1 200 OK\r\nContent-Type: gnss/sourcetable\r\n\r\n", NtripResponseKind.SourceTable)]
    [InlineData("HTTP/1.1 403 Forbidden\r\n\r\nYour IP has been banned", NtripResponseKind.Banned)]
    [InlineData("ERROR - Bad Password\r\n", NtripResponseKind.Rejected)]
    [InlineData("HTTP/1.1 401 Unauthorized\r\n\r\n", NtripResponseKind.Rejected)]
    [InlineData("", NtripResponseKind.Rejected)]
    public void Replies_are_classified(string reply, NtripResponseKind expected)
    {
        Assert.Equal(expected, NtripProtocol.Classify(reply));
    }
}
