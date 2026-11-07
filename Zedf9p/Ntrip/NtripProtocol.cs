using System;
using System.Text;

namespace Zedf9p.Ntrip
{
    public enum NtripResponseKind
    {
        Ok,
        SourceTable,    // The caster answered with its table: the mountpoint is not streaming right now
        Banned,
        Rejected,
    }

    /// <summary>
    /// Wire format of the NTRIP Rev1 handshake, kept apart from the sockets so it can be tested.
    /// </summary>
    internal static class NtripProtocol
    {
        public const string ClientAgent = "NTRIP Zedf9p/3.0";
        public const string SourceAgent = "NTRIP NtripServerPOSIX/Revision: 9109";

        public static string BuildClientRequest(string mountpoint, string username, string password)
        {
            var request = new StringBuilder();
            request.Append($"GET /{mountpoint} HTTP/1.0\r\n");
            request.Append($"User-Agent: {ClientAgent}\r\n");
            request.Append("Accept: */*\r\n");
            request.Append("Connection: close\r\n");

            if (!string.IsNullOrEmpty(username))
            {
                var credentials = Encoding.ASCII.GetBytes($"{username}:{password ?? string.Empty}");
                request.Append($"Authorization: Basic {Convert.ToBase64String(credentials)}\r\n");
            }

            request.Append("\r\n");
            return request.ToString();
        }

        public static string BuildSourceRequest(string mountpoint, string password) =>
            $"SOURCE {password} /{mountpoint}\r\nSource-Agent: {SourceAgent}\r\n\r\n";

        public static NtripResponseKind Classify(string response)
        {
            if (string.IsNullOrEmpty(response)) return NtripResponseKind.Rejected;
            if (response.Contains("SOURCETABLE 200 OK", StringComparison.OrdinalIgnoreCase)) return NtripResponseKind.SourceTable;
            if (response.Contains("banned", StringComparison.OrdinalIgnoreCase)) return NtripResponseKind.Banned;
            if (response.StartsWith("ICY 200 OK", StringComparison.OrdinalIgnoreCase)) return NtripResponseKind.Ok;

            // A Rev2 caster answers a stream request with a regular HTTP success
            if (response.StartsWith("HTTP/1.", StringComparison.OrdinalIgnoreCase) && response.Contains(" 200 ", StringComparison.Ordinal))
            {
                return response.Contains("gnss/sourcetable", StringComparison.OrdinalIgnoreCase)
                    ? NtripResponseKind.SourceTable
                    : NtripResponseKind.Ok;
            }

            return NtripResponseKind.Rejected;
        }
    }
}
