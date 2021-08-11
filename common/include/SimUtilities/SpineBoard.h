/*! @file SpineBoard.h
 *  @brief Spine Board Code, used to simulate the SpineBoard.
 *
 *  This is mostly a copy of the exact code that runs on the SpineBoard
 */

#ifndef PROJECT_SPINEBOARD_H
#define PROJECT_SPINEBOARD_H

#include "cTypes.h"
#include "spi_data_t.hpp"
#include "spi_command_t.hpp"

/*!
 * Command to spine board
 */
struct SpiCommand
{
  float q_des_abad[4];
  float q_des_hip[4];
  float q_des_knee[4];

  float qd_des_abad[4];
  float qd_des_hip[4];
  float qd_des_knee[4];

  float kp_abad[4];
  float kp_hip[4];
  float kp_knee[4];

  float kd_abad[4];
  float kd_hip[4];
  float kd_knee[4];

  float tau_abad_ff[4];
  float tau_hip_ff[4];
  float tau_knee_ff[4];

  int32_t flags[4];
  void makeSpiCommand(spi_command_t msg)
  {
      for(int i=0;i<4;i++)
      {
           q_des_abad[i] = msg.q_des_abad[i];
           q_des_hip[i] = msg.q_des_hip[i];
           q_des_knee[i] = msg.q_des_knee[i];

           qd_des_abad[i] = msg.qd_des_abad[i];
           qd_des_hip[i] = msg.qd_des_hip[i];
           qd_des_knee[i] = msg.qd_des_knee[i];

           kp_abad[i] = msg.kp_abad[i];
           kp_hip[i] = msg.kp_hip[i];
           kp_knee[i] = msg.kp_knee[i];

           kd_abad[i] = msg.kd_abad[i];
           kd_hip[i] = msg.kd_hip[i];
           kd_knee[i] = msg.kp_knee[i];

           tau_abad_ff[i] = msg.tau_abad_ff[i];
           tau_hip_ff[i] = msg.tau_hip_ff[i];
           tau_knee_ff[i] = msg.tau_knee_ff[i];
           flags[i] = msg.flags[i];
      }
  }
  void buildLCM(spi_command_t *msg)
  {
      for(int i=0;i<4;i++)
      {
          msg->q_des_abad[i] = q_des_abad[i];
          msg->q_des_hip[i] = q_des_hip[i];
          msg->q_des_knee[i] = q_des_knee[i];

          msg->qd_des_abad[i] = qd_des_abad[i];
          msg->qd_des_hip[i] = qd_des_hip[i];
          msg->qd_des_knee[i] = qd_des_knee[i];

          msg->kp_abad[i] = kp_abad[i];
          msg->kp_hip[i] = kp_hip[i];
          msg->kp_knee[i] = kp_knee[i];

          msg->kd_abad[i] = kd_abad[i];
          msg->kd_hip[i] = kd_hip[i];
          msg->kd_knee[i] = kp_knee[i];

          msg->tau_abad_ff[i] = tau_abad_ff[i];
          msg->tau_hip_ff[i] = tau_hip_ff[i];
          msg->tau_knee_ff[i] = tau_knee_ff[i];
          msg->flags[i] = flags[i];
      }

  }
};

/*!
 * Data from spine board
 */
struct SpiData
{
  float q_abad[4];
  float q_hip[4];
  float q_knee[4];
  float qd_abad[4];
  float qd_hip[4];
  float qd_knee[4];
  int32_t flags[4];
  int32_t spi_driver_status;
  void makeSpiData(spi_data_t *msg)
  {
      for(int i=0;i<4;i++)
      {
           q_abad[i] = msg->q_abad[i];
           q_hip[i] = msg->q_hip[i];
           q_knee[i] = msg->q_knee[i];
           qd_abad[i] = msg->qd_abad[i];
           qd_hip[i] = msg->qd_hip[i];
           qd_knee[i] = msg->q_knee[i];
           flags[i] = msg->flags[i];
      }
      spi_driver_status = msg->spi_driver_status;
  }
  void buildLCM(spi_data_t *msg)
  {
      for(int i=0;i<4;i++)
      {
          msg->q_abad[i] = q_abad[i];
          msg->q_hip[i] = q_hip[i];
          msg->q_knee[i] = q_knee[i];
          msg->qd_abad[i] = qd_abad[i];
          msg->qd_hip[i] = qd_hip[i];
          msg->qd_knee[i] = q_knee[i];
          msg->flags[i] = flags[i];
      }
      msg->spi_driver_status = spi_driver_status;

  }
};

/*!
 * Spine board control logic
 */
class SpineBoard {
 public:
  SpineBoard() {}
  void init(float side_sign, s32 board);
  void run();
  void resetData();
  void resetCommand();
  SpiCommand* cmd = nullptr;
  SpiData* data = nullptr;
  float torque_out[3];

 private:
  float side_sign;
  s32 board_num;
//  const float max_torque[3] = {17.f, 17.f, 26.f};  // TODO CHECK WITH BEN
//  const float wimp_torque[3] = {6.f, 6.f, 6.f};    // TODO CHECK WITH BEN

  const float max_torque[3] = {144.f, 144.f, 144.f};  // TODO CHECK WITH BEN
  const float wimp_torque[3] = {48.f, 48.f, 48.f};    // TODO CHECK WITH BEN
  const float disabled_torque[3] = {0.f, 0.f, 0.f};
  const float q_limit_p[3] = {1.5f, 5.0f, 0.f};
  const float q_limit_n[3] = {-1.5f, -5.0f, 0.f};
  const float kp_softstop = 100.f;
  const float kd_softstop = 0.4f;
  s32 iter_counter = 0;
};

#endif  // PROJECT_SPINEBOARD_H
