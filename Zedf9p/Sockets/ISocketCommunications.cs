using Zedf9p.DTO;

namespace Zedf9p.Sockets
{
    public interface ISocketCommunications
    {
        void ConnectDataSockets();

        // Method to clean up and close the data sockets
        void CleanupSockets();

        void SendSyncData(SyncData syncData);

        void SendNmeaData(string data);

        void SendRtcmData(byte data);

        int ReceiveSyncData(byte[] syncInBuffer);

        bool IsSyncDataSocketConnected();

        // True when the process on the other end of the sync socket has gone away
        bool IsSyncDataSocketClosedByPeer();

        bool IsRtcmDataSocketConnected();

        bool IsNmeaDataSocketConnected();
    }
}
