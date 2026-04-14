#ifndef IMU_H_
#define IMU_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

#define IMU_PACKET_SIZE       44U
#define IMU_SYNC_BYTE         0x0EU
#define IMU_MSG_ID_NAV        0xA2U
#define IMU_MSG_ID_CTRL       0xA1U

#pragma pack(push, 1)
typedef struct {
    uint8_t sync;
    uint8_t msg_id;
    int16_t angular_rate_x;
    int16_t angular_rate_y;
    int16_t angular_rate_z;
    int16_t linear_accel_x;
    int16_t linear_accel_y;
    int16_t linear_accel_z;
    uint16_t status_word;
    int32_t delta_angle_x;
    int32_t delta_angle_y;
    int32_t delta_angle_z;
    int32_t delta_velocity_x;
    int32_t delta_velocity_y;
    int32_t delta_velocity_z;
    uint16_t checksum;
} imu_raw_packet_t;
#pragma pack(pop)

typedef struct {
    float angular_rate_x;
    float angular_rate_y;
    float angular_rate_z;
    float linear_accel_x;
    float linear_accel_y;
    float linear_accel_z;
    uint16_t status_word;
    float delta_angle_x;
    float delta_angle_y;
    float delta_angle_z;
    float delta_velocity_x;
    float delta_velocity_y;
    float delta_velocity_z;
    uint32_t timestamp;
} IMU_Data_t;

uint16_t IMU_CalculateChecksum(const uint8_t *data, uint16_t length);
bool IMU_ParsePacket(const uint8_t *packet_data, IMU_Data_t *imu_data);
bool IMU_IsValidMessageID(uint8_t msg_id);
bool IMU_IsControlPacket(uint8_t msg_id);
bool IMU_IsNavigationPacket(uint8_t msg_id);

void IMU_Init(UART_HandleTypeDef *huart);
bool IMU_PollNewData(IMU_Data_t *out_data);
void IMU_HandleUartError(UART_HandleTypeDef *huart);
void IMU_UART4_IdleIrqHandler(void);

#endif /* IMU_H_ */
