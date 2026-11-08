using System;
using System.Collections.Generic;
using UBLOX.Enums;

namespace Zedf9p.DTO
{
    /// <summary>
    /// One update on the sync socket, sent as a JSON line. Fields that did not change are left out.
    /// </summary>
    public class SyncData
    {
        public double? Latitude { get; set; }           // degrees
        public double? Longitude { get; set; }          // degrees
        public double? Altitude { get; set; }           // meters above the ellipsoid
        public double? Accuracy { get; set; }           // meters: horizontal accuracy, or the mean accuracy during a survey
        public double? Heading { get; set; }            // degrees, heading of motion: only meaningful while moving
        public double? GroundSpeed { get; set; }        // m/s

        public byte? FixType { get; set; }              // 0 none, 2 2D, 3 3D, 4 GNSS + dead reckoning, 5 time only
        public byte? CarrierSolution { get; set; }      // 0 no RTK, 1 RTK float, 2 RTK fixed
        public byte? Satellites { get; set; }

        public List<string> Errors { get; set; }

        public float? ModuleCurrentSetAccuracy { get; set; }
        public ReceiverModeEnum? ReceiverMode { get; set; }
        public ushort? SurveyTime { get; set; }
        public bool? IsSurveyValid { get; set; }
        public DateTime? LastNtripSent { get; set; }    // UTC: Station mode, last correction sent to the caster
        public DateTime? LastRtcmReceived { get; set; } // UTC: Client mode, last correction received from the caster
    }
}
