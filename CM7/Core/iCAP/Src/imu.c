#include "imu.h"
#include <stddef.h>

uint16_t IMU_CalculateChecksum(const uint8_t *data, uint16_t length)
{
  uint16_t sum = 0U;

  if ((data == NULL) || (length < IMU_PACKET_SIZE))
  {
    return 0U;
  }

  for (uint16_t i = 0U; i < (IMU_PACKET_SIZE - 2U); i += 2U)
  {
    uint16_t word = (uint16_t)data[i] | ((uint16_t)data[i + 1U] << 8);
    sum = (uint16_t)(sum + word);
  }

  return sum;
}

bool IMU_ParsePacket(const uint8_t *packet_data, IMU_Data_t *imu_data)
{
  const imu_raw_packet_t *packet = (const imu_raw_packet_t *)packet_data;
  const float angular_rate_lsb = 1.0f / 2048.0f;
  const float linear_accel_lsb = (1.0f / 32.0f) * 0.3048f;
  const float delta_angle_lsb = 1.0f / 8589934592.0f;
  const float delta_velocity_lsb = (1.0f / 134217728.0f) * 0.3048f;
  uint16_t calculated_checksum;
  uint16_t received_checksum;

  if ((packet_data == NULL) || (imu_data == NULL))
  {
    return false;
  }

  if ((packet->sync != IMU_SYNC_BYTE) || (packet->msg_id != IMU_MSG_ID_NAV))
  {
    return false;
  }

  calculated_checksum = IMU_CalculateChecksum(packet_data, IMU_PACKET_SIZE);
  received_checksum = (uint16_t)packet_data[IMU_PACKET_SIZE - 2U] |
                      ((uint16_t)packet_data[IMU_PACKET_SIZE - 1U] << 8);

  if (calculated_checksum != received_checksum)
  {
    return false;
  }

  imu_data->angular_rate_x = (float)packet->angular_rate_x * angular_rate_lsb;
  imu_data->angular_rate_y = (float)packet->angular_rate_y * angular_rate_lsb;
  imu_data->angular_rate_z = (float)packet->angular_rate_z * angular_rate_lsb;
  imu_data->linear_accel_x = (float)packet->linear_accel_x * linear_accel_lsb;
  imu_data->linear_accel_y = (float)packet->linear_accel_y * linear_accel_lsb;
  imu_data->linear_accel_z = (float)packet->linear_accel_z * linear_accel_lsb;
  imu_data->status_word = packet->status_word;
  imu_data->delta_angle_x = (float)packet->delta_angle_x * delta_angle_lsb;
  imu_data->delta_angle_y = (float)packet->delta_angle_y * delta_angle_lsb;
  imu_data->delta_angle_z = (float)packet->delta_angle_z * delta_angle_lsb;
  imu_data->delta_velocity_x = (float)packet->delta_velocity_x * delta_velocity_lsb;
  imu_data->delta_velocity_y = (float)packet->delta_velocity_y * delta_velocity_lsb;
  imu_data->delta_velocity_z = (float)packet->delta_velocity_z * delta_velocity_lsb;
  imu_data->timestamp = 0U;

  return true;
}

bool IMU_IsValidMessageID(uint8_t msg_id)
{
  return (msg_id == IMU_MSG_ID_CTRL) || (msg_id == IMU_MSG_ID_NAV);
}

bool IMU_IsControlPacket(uint8_t msg_id)
{
  return (msg_id == IMU_MSG_ID_CTRL);
}

bool IMU_IsNavigationPacket(uint8_t msg_id)
{
  return (msg_id == IMU_MSG_ID_NAV);
}
