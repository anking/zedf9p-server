namespace Zedf9p.Ntrip
{
    /// <summary>
    /// Collects RTCM3 bytes into whole frames so each message goes to the caster as soon as it is complete.
    /// A fixed-size send buffer holds the tail of an epoch back until the next epoch fills it, which at 1 Hz
    /// makes corrections up to a second older than they need to be.
    /// </summary>
    internal sealed class RtcmFrameAssembler
    {
        private const byte Preamble = 0xD3;
        private const int HeaderLength = 3;
        private const int CrcLength = 3;
        private const int MaxPayloadLength = 1023;

        private int _count;
        private int _frameLength;

        public byte[] Buffer { get; } = new byte[HeaderLength + MaxPayloadLength + CrcLength];

        /// <summary>
        /// Adds one byte. Returns the frame length when it completes a frame held in <see cref="Buffer"/>, otherwise 0.
        /// </summary>
        public int Add(byte incoming)
        {
            if (_count == 0 && incoming != Preamble) return 0;

            Buffer[_count++] = incoming;

            if (_count == HeaderLength)
            {
                _frameLength = (((Buffer[1] & 0x03) << 8) | Buffer[2]) + HeaderLength + CrcLength;
            }

            if (_count >= HeaderLength && _count == _frameLength)
            {
                _count = 0;
                return _frameLength;
            }

            return 0;
        }

        public void Reset() => _count = 0;
    }
}
