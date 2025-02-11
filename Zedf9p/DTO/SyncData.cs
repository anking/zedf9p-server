using UBLOX.Enums;

namespace Zedf9p.DTO
{
    public class SyncData
    {
        public double? Latitude { get; set; }
        public double? Longitude { get; set; }
        public double? Altitude { get; set; }
        public double? Accuracy { get; set; }
        public double? Heading { get; set; }

        public object Errors { get; set; }
        public float? ModuleCurrentSetAccuracy { get; set; }
        public ReceiverModeEnum? ReceiverMode { get; set; }
        public ushort? SurveyTime { get; set; }
        public bool? IsSurveyValid { get; set; }
    }
}
