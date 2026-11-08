using System;
using System.Collections.Generic;
using System.Linq;
using Newtonsoft.Json;
using Xunit;
using Zedf9p.DTO;
using Zedf9p.Enums;
using Zedf9p.Exceptions;
using Zedf9p.F9p;
using Zedf9p.Interfaces;
using Zedf9p.Ntrip;

namespace Zedf9p.Tests;

public class FixedPositionTests
{
    [Fact]
    public void Trailing_zeros_do_not_change_the_position()
    {
        // The old parser removed the decimal point, so 40.123456 became 4.0123456 degrees
        Assert.True(FixedPosition.TryParse("40.123456", "-80.1234567", "300", out var position, out _));

        Assert.Equal(401234560, position.LatitudeE7);
        Assert.Equal(-801234567, position.LongitudeE7);
        Assert.Equal(30000, position.HeightCm);
    }

    [Fact]
    public void Height_is_converted_from_meters_to_centimeters_with_a_remainder()
    {
        Assert.True(FixedPosition.TryParse("1", "1", "300.2567", out var position, out _));

        Assert.Equal(30025, position.HeightCm);
        Assert.Equal(67, position.HeightHp);
    }

    [Fact]
    public void Digits_past_the_seventh_decimal_go_to_the_remainder()
    {
        Assert.True(FixedPosition.TryParse("40.123456789", "-80.123456789", "1", out var position, out _));

        Assert.Equal(401234567, position.LatitudeE7);
        Assert.Equal(89, position.LatitudeHp);
        Assert.Equal(-801234567, position.LongitudeE7);
        Assert.Equal(-89, position.LongitudeHp);
    }

    [Fact]
    public void The_combined_form_is_accepted()
    {
        Assert.True(FixedPosition.TryParse("-33.8688:151.2093:58.5", out var position, out _));

        Assert.Equal(-338688000, position.LatitudeE7);
        Assert.Equal(5850, position.HeightCm);
    }

    [Theory]
    [InlineData("91", "0", "0")]
    [InlineData("-91", "0", "0")]
    [InlineData("0", "181", "0")]
    [InlineData("10", "10", "10000")]
    [InlineData("0", "0", "100")]
    [InlineData("abc", "10", "10")]
    [InlineData("10", "", "10")]
    [InlineData("10,5", "10", "10")]
    [InlineData("1e1", "10", "10")]
    public void Bad_values_are_refused(string latitude, string longitude, string height)
    {
        Assert.False(FixedPosition.TryParse(latitude, longitude, height, out _, out var error));
        Assert.False(string.IsNullOrEmpty(error));
    }

    [Theory]
    [InlineData("1:2")]
    [InlineData("")]
    [InlineData(null)]
    public void A_malformed_combined_form_is_refused(string text)
    {
        Assert.False(FixedPosition.TryParse(text, out _, out _));
    }
}

public class RtcmFrameAssemblerTests
{
    private static byte[] Frame(int payloadLength)
    {
        var frame = new byte[payloadLength + 6];
        frame[0] = 0xD3;
        frame[1] = (byte)(payloadLength >> 8);
        frame[2] = (byte)payloadLength;
        return frame;
    }

    [Fact]
    public void A_frame_completes_on_its_last_byte()
    {
        var assembler = new RtcmFrameAssembler();
        var frame = Frame(10);

        var lengths = frame.Select(assembler.Add).ToArray();

        Assert.All(lengths.Take(frame.Length - 1), l => Assert.Equal(0, l));
        Assert.Equal(frame.Length, lengths.Last());
        Assert.Equal(frame, assembler.Buffer.Take(frame.Length));
    }

    [Fact]
    public void Consecutive_frames_are_separated_and_noise_between_them_is_dropped()
    {
        var assembler = new RtcmFrameAssembler();
        var bytes = Frame(5).Concat(new byte[] { 0x01, 0x02 }).Concat(Frame(1023)).ToArray();

        var completed = bytes.Select(assembler.Add).Where(l => l > 0).ToArray();

        Assert.Equal([11, 1029], completed);
    }

    [Fact]
    public void Reset_drops_a_partial_frame()
    {
        var assembler = new RtcmFrameAssembler();
        foreach (var b in Frame(10).Take(5)) assembler.Add(b);

        assembler.Reset();

        Assert.Equal(11, Frame(5).Select(assembler.Add).ToArray().Last());
    }
}

public class ReconnectBackoffTests
{
    [Fact]
    public void The_delay_doubles_up_to_the_maximum_and_resets()
    {
        var backoff = new ReconnectBackoff(TimeSpan.FromSeconds(15), TimeSpan.FromSeconds(50));

        Assert.Equal(15, backoff.NextDelay().TotalSeconds);
        Assert.Equal(30, backoff.NextDelay().TotalSeconds);
        Assert.Equal(50, backoff.NextDelay().TotalSeconds);
        Assert.Equal(50, backoff.NextDelay().TotalSeconds);

        backoff.Reset();

        Assert.Equal(15, backoff.NextDelay().TotalSeconds);
    }
}

public class InputParamsTests
{
    private static InputParams Client(string username, string server = "rtk2go.com") => new()
    {
        Port = "/dev/ttyACM0",
        Mode = OperationMode.Client,
        NtripServer = server,
        NtripPort = 2101,
        NtripMountpoint = "MOUNT",
        NtripUsername = username,
    };

    [Theory]
    [InlineData("me@example.com")]
    [InlineData("me-at-example.com")]
    [InlineData("first.last+tag@sub.example.co")]
    public void Rtk2go_accepts_an_email_as_the_rover_user(string username)
    {
        Assert.Empty(Client(username).Validate());
    }

    [Theory]
    [InlineData(null)]
    [InlineData("")]
    [InlineData("rover")]
    [InlineData("[me]@example.com")]
    public void Rtk2go_refuses_anything_else_before_it_counts_as_a_bad_connection(string username)
    {
        Assert.Contains(Client(username).Validate(), e => e.Contains("email"));
    }

    [Fact]
    public void Other_casters_can_go_without_a_user()
    {
        Assert.Empty(Client(null, "caster.example.net").Validate());
    }

    [Fact]
    public void A_station_needs_a_mountpoint_and_a_password()
    {
        var problems = new InputParams { Mode = OperationMode.Station, NtripServer = "rtk2go.com", NtripPort = 2101 }.Validate();

        Assert.Equal(2, problems.Count);
    }

    [Fact]
    public void A_bad_fixed_position_is_refused_at_startup()
    {
        var parameters = new InputParams
        {
            Mode = OperationMode.Station,
            NtripServer = "rtk2go.com",
            NtripPort = 2101,
            NtripMountpoint = "MOUNT",
            NtripPassword = "secret",
            FixedPosition = "95:10:300",
        };

        Assert.Contains(parameters.Validate(), e => e.Contains("--fixed-position"));
    }

    [Fact]
    public void Idle_needs_no_caster()
    {
        Assert.Empty(new InputParams { Mode = OperationMode.Idle }.Validate());
    }
}

public class SyncProtocolTests
{
    [Fact]
    public void Errors_are_sent_as_a_flat_list()
    {
        List<string> sent = null;
        var flags = new ErrorFlags(f => sent = f.Errors);

        flags.Add("A").Add("B").Add("A").Remove("B");

        Assert.Equal(["A"], sent);

        var json = JsonConvert.SerializeObject(new SyncData { Errors = sent }, new JsonSerializerSettings { NullValueHandling = NullValueHandling.Ignore });
        Assert.Equal("{\"Errors\":[\"A\"]}", json);
    }

    [Fact]
    public void Removing_an_absent_flag_sends_nothing()
    {
        var sends = 0;
        new ErrorFlags(_ => sends++).Remove("A");

        Assert.Equal(0, sends);
    }

    [Fact]
    public void Commands_parse_their_values_independently_of_culture()
    {
        var command = new SyncIncomingCommand("RESTART_SURVEY:2.5:120\r\n");

        Assert.Equal(SyncIncomingCommandType.RESTART_SURVEY, command.Type);
        Assert.Equal(2.5f, command.GetValue<float>());
        Assert.Equal(120, command.GetValue<int>(1));
    }

    [Theory]
    [InlineData("HELLO")]
    [InlineData("")]
    [InlineData("99")]
    public void Unknown_commands_are_refused(string text)
    {
        Assert.Throws<UnknownSyncCommandException>(() => new SyncIncomingCommand(text));
    }
}
