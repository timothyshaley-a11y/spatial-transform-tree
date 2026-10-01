#ifndef BSPS_PROTOCOL_H
#define BSPS_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define BSPS_MAGIC              0x5350U
#define BSPS_VERSION            0x01U
#define BSPS_TYPE_SPATIAL_STATE 0x01U
#define BSPS_TYPE_SYNC_PING     0x10U
#define BSPS_TYPE_SYNC_PONG     0x11U

#pragma pack(push, 1)

typedef struct {
    uint16_t magic;
    uint8_t  version;
    uint8_t  packet_type;
    uint16_t sender_id;
    uint32_t sequence;
    uint64_t timestamp_ns;
} BspsHeader_t;

typedef struct {
    BspsHeader_t header;
    uint32_t target_frame_hash;
    uint32_t source_frame_hash;
    float    position[3];
    float    orientation[4];
    float    linear_velocity[3];
    float    angular_velocity[3];
    float    variance_rms;
    uint8_t  status_flags;
    uint8_t  reserved[3];
} BspsSpatialStatePacket_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

int BSPS_Serialize_SpatialState(const BspsSpatialStatePacket_t* in_pkt, uint8_t* out_buf, size_t buf_len);
bool BSPS_Deserialize_SpatialState(const uint8_t* in_buf, size_t buf_len, BspsSpatialStatePacket_t* out_pkt);

#ifdef __cplusplus
}
#endif

#endif // BSPS_PROTOCOL_H
