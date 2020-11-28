namespace UBLOX.Enums
{
    // Global Status Returns
    public enum SfeUbloxStatus
    {
        SFE_UBLOX_STATUS_SUCCESS,
        SFE_UBLOX_STATUS_FAIL,
        SFE_UBLOX_STATUS_CRC_FAIL,
        SFE_UBLOX_STATUS_TIMEOUT,
        SFE_UBLOX_STATUS_COMMAND_NACK, // Indicates that the command was unrecognised, invalid or that the module is too busy to respond
        SFE_UBLOX_STATUS_OUT_OF_RANGE,
        SFE_UBLOX_STATUS_INVALID_ARG,
        SFE_UBLOX_STATUS_INVALID_OPERATION,
        SFE_UBLOX_STATUS_MEM_ERR,
        SFE_UBLOX_STATUS_HW_ERR,
        SFE_UBLOX_STATUS_DATA_SENT,     // This indicates that a 'set' was successful
        SFE_UBLOX_STATUS_DATA_RECEIVED, // This indicates that a 'get' (poll) was successful
        SFE_UBLOX_STATUS_I2C_COMM_FAILURE,
        SFE_UBLOX_STATUS_DATA_OVERWRITTEN // This is an error - the data was valid but has been or _is being_ overwritten by another packet
    };

    // ubxPacket validity
    public enum SfeUbloxPacketValidity
    {
        SFE_UBLOX_PACKET_VALIDITY_NOT_VALID,
        SFE_UBLOX_PACKET_VALIDITY_VALID,
        SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED,
        SFE_UBLOX_PACKET_NOTACKNOWLEDGED // This indicates that we received a NACK
    };

    // Identify which packet buffer is in use:
    // packetCfg (or a custom packet), packetAck or packetBuf
    public enum SfeUbloxPacketBuffer
    {
        SFE_UBLOX_PACKET_PACKETCFG,
        SFE_UBLOX_PACKET_PACKETACK,
        SFE_UBLOX_PACKET_PACKETBUF
    };

    public enum DynModel // Possible values for the dynamic platform model, which provide more accuract position output for the situation. Description extracted from ZED-F9P Integration Manual
    {
        DYN_MODEL_PORTABLE = 0, //Applications with low acceleration, e.g. portable devices. Suitable for most situations.
                                // 1 is not defined
        DYN_MODEL_STATIONARY = 2, //Used in timing applications (antenna must be stationary) or other stationary applications. Velocity restricted to 0 m/s. Zero dynamics assumed.
        DYN_MODEL_PEDESTRIAN,     //Applications with low acceleration and speed, e.g. how a pedestrian would move. Low acceleration assumed.
        DYN_MODEL_AUTOMOTIVE,     //Used for applications with equivalent dynamics to those of a passenger car. Low vertical acceleration assumed
        DYN_MODEL_SEA,            //Recommended for applications at sea, with zero vertical velocity. Zero vertical velocity assumed. Sea level assumed.
        DYN_MODEL_AIRBORNE1g,     //Airborne <1g acceleration. Used for applications with a higher dynamic range and greater vertical acceleration than a passenger car. No 2D position fixes supported.
        DYN_MODEL_AIRBORNE2g,     //Airborne <2g acceleration. Recommended for typical airborne environments. No 2D position fixes supported.
        DYN_MODEL_AIRBORNE4g,     //Airborne <4g acceleration. Only recommended for extremely dynamic environments. No 2D position fixes supported.
        DYN_MODEL_WRIST,          // Not supported in protocol versions less than 18. Only recommended for wrist worn applications. Receiver will filter out arm motion.
        DYN_MODEL_BIKE,           // Supported in protocol versions 19.2
    };
    //Depending on the sentence type the processor will load characters into different arrays
    public enum SentenceTypes
    {
        NONE = 0,
        NMEA,
        UBX,
        RTCM
    };
    //currentSentence = NONE;

    //Depending on the ubx binary response class, store binary responses into different places
    public enum ClassTypes
    {
        CLASS_NONE = 0,
        CLASS_ACK,
        CLASS_NOT_AN_ACK
    };
    //ubxFrameClass = CLASS_NONE;

    public enum CommTypes
    {
        COMM_TYPE_I2C = 0,
        COMM_TYPE_SERIAL,
        COMM_TYPE_SPI
    };

    public enum ReceiverModeEnum
    {
        Disabled = 0,
        SurveyIn = 1,
        FixedMode = 2
    }
}