using System;
using System.Globalization;

namespace Zedf9p.F9p
{
    /// <summary>
    /// A surveyed base position in the units UBX-CFG-TMODE3 expects: latitude and longitude in 1e-7 degrees
    /// plus a 1e-9 degree remainder, height above the ellipsoid in cm plus a 0.1 mm remainder.
    /// </summary>
    public readonly record struct FixedPosition(int LatitudeE7, sbyte LatitudeHp, int LongitudeE7, sbyte LongitudeHp, int HeightCm, sbyte HeightHp)
    {
        private const NumberStyles NumberFormat = NumberStyles.AllowLeadingSign | NumberStyles.AllowDecimalPoint;

        /// <summary>Parses "latitude:longitude:height", in degrees, degrees and meters.</summary>
        public static bool TryParse(string text, out FixedPosition position, out string error)
        {
            position = default;
            var parts = (text ?? string.Empty).Split(':');
            if (parts.Length != 3)
            {
                error = "expected latitude:longitude:height";
                return false;
            }

            return TryParse(parts[0], parts[1], parts[2], out position, out error);
        }

        public static bool TryParse(string latitude, string longitude, string height, out FixedPosition position, out string error)
        {
            position = default;

            if (!TryParseDecimal(latitude, -90m, 90m, out var lat))
            {
                error = "latitude must be a decimal number of degrees between -90 and 90";
                return false;
            }

            if (!TryParseDecimal(longitude, -180m, 180m, out var lon))
            {
                error = "longitude must be a decimal number of degrees between -180 and 180";
                return false;
            }

            if (!TryParseDecimal(height, -500m, 9000m, out var meters))
            {
                error = "height must be a decimal number of meters above the ellipsoid, between -500 and 9000";
                return false;
            }

            if (lat == 0m && lon == 0m)
            {
                error = "latitude and longitude are both zero";
                return false;
            }

            var (latE7, latHp) = Split(lat * 1_000_000_000m);
            var (lonE7, lonHp) = Split(lon * 1_000_000_000m);
            var (heightCm, heightHp) = Split(meters * 10_000m);

            position = new FixedPosition(latE7, latHp, lonE7, lonHp, heightCm, heightHp);
            error = null;
            return true;
        }

        private static bool TryParseDecimal(string text, decimal minimum, decimal maximum, out decimal value) =>
            decimal.TryParse(text?.Trim(), NumberFormat, CultureInfo.InvariantCulture, out value) && value >= minimum && value <= maximum;

        // Splits a value in the finest unit into the main field (100 units each) and the -99..99 remainder.
        private static (int Main, sbyte Remainder) Split(decimal fineUnits)
        {
            var total = (long)Math.Round(fineUnits, MidpointRounding.AwayFromZero);
            var main = total / 100;
            return ((int)main, (sbyte)(total - main * 100));
        }

        public override string ToString()
        {
            static string Degrees(int e7, sbyte hp) => ((e7 * 100L + hp) / 1_000_000_000m).ToString("0.000000000", CultureInfo.InvariantCulture);
            var meters = ((HeightCm * 100L + HeightHp) / 10_000m).ToString("0.0000", CultureInfo.InvariantCulture);
            return $"{Degrees(LatitudeE7, LatitudeHp)}, {Degrees(LongitudeE7, LongitudeHp)}, {meters} m";
        }
    }
}
