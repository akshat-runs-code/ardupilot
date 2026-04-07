#include "AP_Mount_config.h"

#if HAL_MOUNT_CADDX_ENABLED

#include "AP_Mount_CADDX.h"
#include <AP_HAL/AP_HAL.h>
#include "AP_AHRS/AP_AHRS.h"
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>

#define AP_MOUNT_CADDX_RESEND_MS   1000    // resend angle targets to gimbal once per second
#define SET_ATTITUDE_HEADER1 0x69
#define SET_ATTITUDE_HEADER2 0x96
#define BUF_SIZE 14
#define AXIS_MIN 0
#define AXIS_MAX 4096

// update mount position - should be called periodically
void AP_Mount_CADDX::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // change to RC_TARGETING mode if RC input has changed
    set_rctargeting_on_rcinput_change();

    // flag to trigger sending target angles to gimbal
  //  bool resend_now = false;

    // update based on mount mode
    switch (get_mode()) {
        // move mount to a "retracted" position.  To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT: {
            const Vector3f &target = _params.retract_angles.get();
            mnt_target.angle_rad.set(target*DEG_TO_RAD, false);
            mnt_target.target_type = MountTargetType::ANGLE;
            //target_angle = AP::ahrs().get_pitch();
            break;
        }

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL: {
            const Vector3f &target = _params.neutral_angles.get();
            mnt_target.angle_rad.set(target*DEG_TO_RAD, false);
            mnt_target.target_type = MountTargetType::ANGLE;
            //target_angle = AP::ahrs().get_pitch();
            break;
        }

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // mnt_target should have already been filled in by set_angle_target() or set_rate_target()
           // resend_now = true;
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING: {
            // update targets using pilot's RC inputs
            MountTarget rc_target;
            get_rc_target(mnt_target.target_type, rc_target);
            switch (mnt_target.target_type) {
            case MountTargetType::ANGLE:
                mnt_target.angle_rad = rc_target;
                //target_angle = rc_target.pitch;
                break;
            case MountTargetType::RATE:
                mnt_target.rate_rads = rc_target;
                break;
            }
            //resend_now = true;
            break;
        }

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if (get_angle_target_to_roi(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
                //target_angle = -1*mnt_target.angle_rad.pitch;
               // resend_now = true;
            }
            break;

        // point mount to Home location
        case MAV_MOUNT_MODE_HOME_LOCATION:
            if (get_angle_target_to_home(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
                //target_angle = -1 * mnt_target.angle_rad.pitch;
               // resend_now = true;
            }
            break;

        // point mount to another vehicle
        case MAV_MOUNT_MODE_SYSID_TARGET:
            if (get_angle_target_to_sysid(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
                //target_angle = -1*mnt_target.angle_rad.pitch;
               // resend_now = true;
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }

    // update angle targets from angle rates
    if (mnt_target.target_type == MountTargetType::RATE) {
        update_angle_target_from_rate(mnt_target.rate_rads, mnt_target.angle_rad);
    }

    // resend target angles at least once per second
    // resend_now = resend_now || ((AP_HAL::millis() - _last_send_ms) > AP_MOUNT_CADDX_RESEND_MS);
    // if (resend_now) {
    //     send_target_angles(mnt_target.angle_rad);
    // }
}


// get attitude as a quaternion.  returns true on success
bool AP_Mount_CADDX::get_attitude_quaternion(Quaternion& att_quat)
{
    // gimbal does not provide attitude so simply return targets
    att_quat.from_euler(mnt_target.angle_rad.roll, mnt_target.angle_rad.pitch, mnt_target.angle_rad.get_bf_yaw());
    return true;
}

void AP_Mount_CADDX::update_fast(){
    //uint32_t now = AP_HAL::micros();
    //uint32_t delay = now - _last_send_ms;
     //gcs().send_text(MAV_SEVERITY_INFO,"%lu\r\n",delay);
    //_last_send_ms = AP_HAL::micros();
    send_target_angles(mnt_target.angle_rad);
}

// send_target_angles
void AP_Mount_CADDX::send_target_angles(const MountTarget& angle_target_rad)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }
    if (_uart->txspace() < BUF_SIZE) {
        return;
    }

    //create packets of uint8_t for faster transmission
    uint8_t data[BUF_SIZE] = {}; // [target_pitch, target_roll, target_yaw, drone_pitch,drone_roll, drone_yaw] two bytes(upper, lower) for each

    int16_t target_pitch = mnt_target.angle_rad.pitch * 1000;
    int16_t target_roll = mnt_target.angle_rad.roll * 1000;
    int16_t target_yaw = mnt_target.angle_rad.yaw * 1000;
    int16_t drone_pitch = AP::ahrs().get_pitch() * 1000;
    int16_t drone_roll = AP::ahrs().get_roll() * 1000;
    int16_t drone_yaw = AP::ahrs().get_yaw() * 1000;

    data[0] = SET_ATTITUDE_HEADER1;
    data[1] = SET_ATTITUDE_HEADER2;
    data[2] = (target_pitch >> 8) & 0xFF;
    data[3] = target_pitch & 0xFF;
    data[4] = (target_roll >> 8) & 0xFF;
    data[5] = target_roll & 0xFF;
    data[6] = (target_yaw >> 8) & 0xFF;
    data[7] = target_yaw & 0xFF;
    data[8] = (drone_pitch >> 8) & 0xFF;
    data[9] = drone_pitch & 0xFF;
    data[10] = (drone_roll >> 8) & 0xFF;
    data[11] = drone_roll & 0xFF;
    data[12] = (drone_yaw >> 8) & 0xFF;
    data[13] = drone_yaw & 0xFF;

    _uart->write(data, sizeof(data));

    // gcs().send_text(MAV_SEVERITY_INFO,"%.d,%.d\n",target_pitch,drone_pitch);
    // // ensure we have enough space to send the packet
    // if (_uart->txspace() < SET_ATTITUDE_BUF_SIZE) {
    //     return;
    // }

    // // calculate roll, pitch, yaw angles in range 0 to 4096
    // const float scalar = AXIS_MAX / M_2PI;
    // const uint16_t roll_target_cmd = constrain_uint16(wrap_2PI(angle_target_rad.roll) * scalar, AXIS_MIN, AXIS_MAX);
    // const uint16_t pitch_target_cmd = constrain_uint16(wrap_2PI(angle_target_rad.pitch) * scalar, AXIS_MIN, AXIS_MAX);
    // const uint16_t yaw_target_cmd = constrain_uint16(wrap_2PI(angle_target_rad.get_bf_yaw()) * scalar, AXIS_MIN, AXIS_MAX);

    // // prepare packet to send to gimbal
    // uint8_t set_attitude_cmd_buf[SET_ATTITUDE_BUF_SIZE] {};

    // // first two bytes hold the header
    // set_attitude_cmd_buf[0] = SET_ATTITUDE_HEADER1;
    // set_attitude_cmd_buf[1] = SET_ATTITUDE_HEADER2;

    // // byte 2's lower 3 bits are mode
    // // lower 5 bits are sensitivity but always left as zero
    // uint8_t mode = (uint8_t)LockMode::TILT_LOCK | (uint8_t)LockMode::ROLL_LOCK;
    // if (angle_target_rad.yaw_is_ef) {
    //     mode |= (uint8_t)LockMode::YAW_LOCK;
    // }
    // set_attitude_cmd_buf[2] = mode & 0x07;

    // // byte 3's lower 4 bits are reserved
    // // upper 4 bits are roll's lower 4 bits
    // set_attitude_cmd_buf[3] = (roll_target_cmd << 4) & 0xF0;

    // // byte 4 is roll's upper 8 bits
    // set_attitude_cmd_buf[4] = (roll_target_cmd >> 4) & 0xFF;

    // // byte 5 is pitch's lower 8 bits
    // set_attitude_cmd_buf[5] = pitch_target_cmd & 0xFF;

    // // byte 6's lower 4 bits are pitch's upper 4 bits
    // // upper 4 bits are yaw's lower 4 bits
    // set_attitude_cmd_buf[6] = (pitch_target_cmd >> 8) & 0x0F;
    // set_attitude_cmd_buf[6] |= (yaw_target_cmd << 4) & 0xF0;

    // // byte 7 is yaw's upper 8 bits
    // set_attitude_cmd_buf[7] = (yaw_target_cmd >> 4) & 0xFF;

    // // calculate CRC
    // const uint16_t crc16 = crc16_ccitt(set_attitude_cmd_buf, sizeof(set_attitude_cmd_buf) - 2, 0);
    // set_attitude_cmd_buf[8] = HIGHBYTE(crc16);
    // set_attitude_cmd_buf[9] = LOWBYTE(crc16);

    // // send packet to gimbal
    //_uart->write(set_attitude_cmd_buf, sizeof(set_attitude_cmd_buf));

    // store time of send
   
}

#endif // HAL_MOUNT_CADDX_ENABLED