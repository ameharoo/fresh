#pragma once


#include "../mesh_base_interface.h"
#include "../mesh_controller.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <cstring>
#include <unordered_map>


namespace NsWifiEspNowInterface
{
    struct MacAddr {
        std::array<ubyte, 6> raw{};

        MacAddr(const ubyte (&mac_)[6]) {memcpy(raw.data(), mac_, 6);}
        MacAddr() = default;
    } __attribute__((packed));

    inline bool operator==(const MacAddr& src, const MacAddr& dst) {
        return src.raw == dst.raw;
    }

    inline bool operator!=(const MacAddr& src, const MacAddr& dst) {
        return !(src == dst);
    }

    using SessionManager = SimpleSecureMeshInterfaceSessionManager<MacAddr>;

    class EspNowPeerManager
    {
    public:
        bool add_peer(const ubyte* mac, ubyte channel);

        bool remove_peer(const ubyte* mac);
    };
}

namespace std {
    template<>
    struct hash<NsWifiEspNowInterface::MacAddr> {
        inline std::size_t operator()(const NsWifiEspNowInterface::MacAddr& mac) const {
            return (*(u32le*)mac.raw.data()) ^ (*(u32le*)(&mac.raw[2]));
        }
    };
}


class WifiEspNowMeshInterface : public MeshInterface
{
public:
    static const int MAX_QUEUED_RX_PACKETS = 8;
    static const int FAR_MTU = 250;

    NsWifiEspNowInterface::EspNowPeerManager peer_manager;
    NsWifiEspNowInterface::SessionManager session_manager;
    QueueHandle_t rx_queue{};
    NsWifiEspNowInterface::MacAddr self_addr;

    WifiEspNowMeshInterface();

    void check_packets() override;

    bool accept_near_packet(MeshPhyAddrPtr phy_addr, const MeshProto::MeshPacket* packet, uint size) override;

    MeshProto::MeshPacket* alloc_near_packet(MeshProto::MeshPacketType type, uint size) const override;

    MeshProto::MeshPacket* realloc_near_packet(MeshProto::MeshPacket* packet,
                                               MeshProto::MeshPacketType old_type,
                                               MeshProto::MeshPacketType new_type,
                                               uint new_size) const override;

    void free_near_packet(MeshProto::MeshPacket* packet) const override;

    void send_packet(MeshPhyAddrPtr phy_addr, const MeshProto::MeshPacket* packet, uint size) override;

    MeshInterfaceProps get_props() override;

    void send_hello(MeshPhyAddrPtr phy_addr) override;

    void write_addr_bytes(MeshPhyAddrPtr phy_addr, void* out_buf) const override;

    ~WifiEspNowMeshInterface() override;

    MeshProto::far_addr_t derive_far_addr_uint32();
};
