using System;
using System.Linq;

namespace UBLOX.Models
{

    //-=-=-=-=- UBX binary specific variables
    class ubxPacket
    {

        public byte cls;
        public byte id;
        public ushort len;          //Length of the payload. Does not include cls, id, or checksum bytes
        public ushort counter;      //Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
        public ushort startingSpot; //The counter value needed to go past before we begin recording into payload array
        public byte[] payload;
        public byte checksumA; //Given to us from module. Checked against the rolling calculated A/B checksums.
        public byte checksumB;
        public Enums.SfeUbloxPacketValidity valid;           //Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
        public Enums.SfeUbloxPacketValidity classAndIDmatch; // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID
    };


    // Struct to hold the results returned by getGeofenceState (returned by UBX-NAV-GEOFENCE)
    class geofenceState
    {

        public byte status;    // Geofencing status: 0 - Geofencing not available or not reliable; 1 - Geofencing active
        public byte numFences; // Number of geofences
        public byte combState; // Combined (logical OR) state of all geofences: 0 - Unknown; 1 - Inside; 2 - Outside
        public byte[] states = new byte[4]; // Geofence states: 0 - Unknown; 1 - Inside; 2 - Outside
    };

    // Struct to hold the current geofence parameters
    class geofenceParams
    {

        public byte numFences; // Number of active geofences
        public int[] lats = new int[4];   // Latitudes of geofences (in degrees * 10^-7)
        public int[] longs = new int[4];  // Longitudes of geofences (in degrees * 10^-7)
        public uint[] rads = new uint[4];  // Radii of geofences (in m * 10^-2)
    };

    //Survey-in specific controls
    class svinStructure
    {
        public bool active;
        public bool valid;
        public ushort observationTime;
        public float meanAccuracy;
    };

    //Relative Positioning Info in NED frame specific controls
    class frelPosInfoStructure
    {
        public ushort refStationID;

        public float relPosN;
        public float relPosE;
        public float relPosD;

        public long relPosLength;
        public long relPosHeading;

        public byte relPosHPN;
        public byte relPosHPE;
        public byte relPosHPD;
        public byte relPosHPLength;

        public float accN;
        public float accE;
        public float accD;

        public bool gnssFixOk;
        public bool diffSoln;
        public bool relPosValid;
        public byte carrSoln;
        public bool isMoving;
        public bool refPosMiss;
        public bool refObsMiss;
    };

    class deadReckData
    {
        public byte version;
        public byte fusionMode;

        public byte xAngRateVald;
        public byte yAngRateVald;
        public byte zAngRateVald;
        public byte xAccelVald;
        public byte yAccelVald;
        public byte zAccelVald;

        public int xAngRate;
        public int yAngRate;
        public int zAngRate;

        public int xAccel;
        public int yAccel;
        public int zAccel;

        // The array size is based on testing directly on M8U and F9R
        public uint rawData;
        public uint rawDataType;
        public uint rawTStamp;

        public uint[] data = new uint[Constants.DEF_NUM_SENS];
        public uint[] dataType = new uint[Constants.DEF_NUM_SENS];
        public uint[] dataTStamp = new uint[Constants.DEF_NUM_SENS];
    };

    class indivImuData
    {

        public byte numSens;

        public byte senType;
        public bool isUsed;
        public bool isReady;
        public byte calibStatus;
        public byte timeStatus;

        public byte freq; // Hz

        public bool badMeas;
        public bool badTag;
        public bool missMeas;
        public bool noisyMeas;
    };

    class vehicleAttitude
    {
        // All values in degrees
        public int roll;
        public int pitch;
        public int heading;
        public uint accRoll;
        public uint accPitch;
        public uint accHeading;
    };

    class moduleQueried
    {
        public uint gpsiTOW = 1;
        public uint gpsYear = 1;
        public uint gpsMonth = 1;
        public uint gpsDay = 1;
        public uint gpsHour = 1;
        public uint gpsMinute = 1;
        public uint gpsSecond = 1;
        public uint gpsDateValid = 1;
        public uint gpsTimeValid = 1;
        public uint gpsNanosecond = 1;

        public uint all = 1;
        public uint longitude = 1;
        public uint latitude = 1;
        public uint altitude = 1;
        public uint altitudeMSL = 1;
        public uint SIV = 1;
        public uint fixType = 1;
        public uint carrierSolution = 1;
        public uint groundSpeed = 1;
        public uint headingOfMotion = 1;
        public uint pDOP = 1;
        public uint versionNumber = 1;
    };

    class highResModuleQueried
    {
        public ushort all = 1;
        public ushort timeOfWeek = 1;
        public ushort highResLatitude = 1;
        public ushort highResLongitude = 1;
        public ushort elipsoid = 1;
        public ushort meanSeaLevel = 1;
        public ushort geoidSeparation = 1; // Redundant but kept for backward-compatibility
        public ushort horizontalAccuracy = 1;
        public ushort verticalAccuracy = 1;
        public ushort elipsoidHp = 1;
        public ushort meanSeaLevelHp = 1;
        public ushort highResLatitudeHp = 1;
        public ushort highResLongitudeHp = 1;
    };


    /// <summary>
    /// OBJECT MODEL REQUESTS DTO's
    /// </summary>

    public class UbloxDataPacket
    {
        public byte[] payload { get; set; }
    }

    public class SurveyMode : UbloxDataPacket
    {
        /// <summary>
        /// Get receiver mode. 0-disabled, 1-survey-in, 2-fixed mode
        /// </summary>
        /// <returns></returns>
        public Enums.ReceiverModeEnum getMode() => (Enums.ReceiverModeEnum)BitConverter.ToInt16(payload.Skip(2).Take(2).ToArray());

        /// <summary>
        /// Get receiver accuracy in meters
        /// </summary>
        /// <returns></returns>
        public float getAccuracyLimit() => BitConverter.ToInt32(payload.Skip(28).Take(4).ToArray()) / 10000F; //Accuracy contained in 0.1mm in a module, convert to meters
    }
}