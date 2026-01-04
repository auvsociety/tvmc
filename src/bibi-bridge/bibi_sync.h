#ifndef BIBI_SYNC_H
#define BIBI_SYNC_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define SLOT_SIZE 256

#define HEADER_SIZE 12

#define MAX_PAYLOAD_SIZE (SLOT_SIZE - HEADER_SIZE)

#define SYNC_BYTE 170

#define MAX_MSG_SIZE 244

#define IMU_MSG_SIZE 36

#define ORIENTATION_MSG_SIZE 12

#define DEPTH_MSG_SIZE 4

#define THRUSTER_PWM_SIZE 24

#define LED_CMD_SIZE 2

#define CALIBRATION_CMD_SIZE 1

typedef struct BibiByteTopic BibiByteTopic;

typedef struct BibiRegistry BibiRegistry;

typedef struct BibiTypedTopic BibiTypedTopic;

struct BibiRegistry *bibi_registry_new(void);

void bibi_registry_free(struct BibiRegistry *registry);

struct BibiByteTopic *bibi_registry_get_byte_topic(struct BibiRegistry *registry,
                                                   const char *name,
                                                   uintptr_t capacity);

void bibi_byte_topic_free(struct BibiByteTopic *topic);

uint64_t bibi_byte_topic_publish(struct BibiByteTopic *topic, const uint8_t *data, uintptr_t len);

int32_t bibi_byte_topic_try_receive(struct BibiByteTopic *topic,
                                    uint8_t *out_data,
                                    uintptr_t *out_len,
                                    uintptr_t max_len);

int32_t bibi_byte_topic_peek_latest(struct BibiByteTopic *topic,
                                    uint8_t *out_data,
                                    uintptr_t *out_len,
                                    uint64_t *out_epoch,
                                    uintptr_t max_len);

uintptr_t bibi_byte_topic_len(struct BibiByteTopic *topic);

bool bibi_byte_topic_is_empty(struct BibiByteTopic *topic);

uint64_t bibi_byte_topic_latest_epoch(struct BibiByteTopic *topic);

struct BibiTypedTopic *bibi_registry_get_typed_topic(struct BibiRegistry *registry,
                                                     const char *name,
                                                     uintptr_t capacity,
                                                     uintptr_t msg_size);

void bibi_typed_topic_free(struct BibiTypedTopic *topic);

uint64_t bibi_typed_topic_publish(struct BibiTypedTopic *topic, const uint8_t *data);

int32_t bibi_typed_topic_try_receive(struct BibiTypedTopic *topic, uint8_t *out_data);

int32_t bibi_typed_topic_peek_latest(struct BibiTypedTopic *topic,
                                     uint8_t *out_data,
                                     uint64_t *out_epoch);

#endif /* BIBI_SYNC_H */
