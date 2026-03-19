#ifndef MSG_H
#define MSG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#if defined(__GNUC__)
#define PACKED __attribute__((packed))
#else
#define PACKED
#endif

#pragma pack(push, 1)

typedef struct PACKED {
    float position;
} state_msg_t;

typedef struct PACKED {
    float command;
} command_msg_t;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif // MSG_H