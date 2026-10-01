#include "bsps_protocol.h"
#include <string.h>

#if defined(_WIN32)
    #include <winsock2.h>
#else
    #include <arpa/inet.h>
#endif

static inline uint64_t hton64(uint64_t val) {
    #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        return (((uint64_t)htonl((uint32_t)val)) << 32) | htonl((uint32_t)(val >> 32));
    #else
        return val;
    #endif
}

static inline uint32_t float_to_network(float f) {
    uint32_t i;
    memcpy(&i, &f, sizeof(float));
    return htonl(i);
}

static inline float network_to_float(uint32_t net_i) {
    uint32_t host_i = ntohl(net_i);
    float f;
    memcpy(&f, &host_i, sizeof(float));
    return f;
}

int BSPS_Serialize_SpatialState(const BspsSpatialStatePacket_t* in_pkt, uint8_t* out_buf, size_t buf_len) {
    if (buf_len < sizeof(BspsSpatialStatePacket_t)) return -1;
    BspsSpatialStatePacket_t* net_pkt = (BspsSpatialStatePacket_t*)out_buf;

    net_pkt->header.magic        = htons(BSPS_MAGIC);
    net_pkt->header.version      = BSPS_VERSION;
    net_pkt->header.packet_type  = BSPS_TYPE_SPATIAL_STATE;
    net_pkt->header.sender_id    = htons(in_pkt->header.sender_id);
    net_pkt->header.sequence     = htonl(in_pkt->header.sequence);
    net_pkt->header.timestamp_ns = hton64(in_pkt->header.timestamp_ns);

    net_pkt->target_frame_hash   = htonl(in_pkt->target_frame_hash);
    net_pkt->source_frame_hash   = htonl(in_pkt->source_frame_hash);

    for (int i = 0; i < 3; i++) {
        net_pkt->position[i]         = (float)float_to_network(in_pkt->position[i]);
        net_pkt->linear_velocity[i]  = (float)float_to_network(in_pkt->linear_velocity[i]);
        net_pkt->angular_velocity[i] = (float)float_to_network(in_pkt->angular_velocity[i]);
    }
    for (int i = 0; i < 4; i++) {
        net_pkt->orientation[i]      = (float)float_to_network(in_pkt->orientation[i]);
    }
    net_pkt->variance_rms = (float)float_to_network(in_pkt->variance_rms);
    net_pkt->status_flags = in_pkt->status_flags;

    return (int)sizeof(BspsSpatialStatePacket_t);
}

bool BSPS_Deserialize_SpatialState(const uint8_t* in_buf, size_t buf_len, BspsSpatialStatePacket_t* out_pkt) {
    if (buf_len < sizeof(BspsSpatialStatePacket_t)) return false;
    const BspsSpatialStatePacket_t* net_pkt = (const BspsSpatialStatePacket_t*)in_buf;

    if (ntohs(net_pkt->header.magic) != BSPS_MAGIC) return false;
    if (net_pkt->header.version != BSPS_VERSION) return false;

    out_pkt->header.magic        = ntohs(net_pkt->header.magic);
    out_pkt->header.version      = net_pkt->header.version;
    out_pkt->header.packet_type  = net_pkt->header.packet_type;
    out_pkt->header.sender_id    = ntohs(net_pkt->header.sender_id);
    out_pkt->header.sequence     = ntohl(net_pkt->header.sequence);
    out_pkt->header.timestamp_ns = hton64(net_pkt->header.timestamp_ns);

    out_pkt->target_frame_hash   = ntohl(net_pkt->target_frame_hash);
    out_pkt->source_frame_hash   = ntohl(net_pkt->source_frame_hash);

    uint32_t raw_float;
    for (int i = 0; i < 3; i++) {
        memcpy(&raw_float, &net_pkt->position[i], 4);
        out_pkt->position[i] = network_to_float(raw_float);

        memcpy(&raw_float, &net_pkt->linear_velocity[i], 4);
        out_pkt->linear_velocity[i] = network_to_float(raw_float);

        memcpy(&raw_float, &net_pkt->angular_velocity[i], 4);
        out_pkt->angular_velocity[i] = network_to_float(raw_float);
    }
    for (int i = 0; i < 4; i++) {
        memcpy(&raw_float, &net_pkt->orientation[i], 4);
        out_pkt->orientation[i] = network_to_float(raw_float);
    }
    memcpy(&raw_float, &net_pkt->variance_rms, 4);
    out_pkt->variance_rms = network_to_float(raw_float);
    out_pkt->status_flags = net_pkt->status_flags;

    return true;
}
