/*
* The ASCII protocol is a simpler, human readable alternative to the main native
* protocol.
* In the future this protocol might be extended to support selected GCode commands.
* For a list of supported commands see doc/ascii-protocol.md
*/

/* Includes ------------------------------------------------------------------*/

#include "odrive_main.h"
#include "communication.h"
#include "ascii_protocol.hpp"
#include <utils.hpp>
#include <fibre/cpp_utils.hpp>

#include "autogen/type_info.hpp"
#include "communication/interface_can.hpp"

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private constant data -----------------------------------------------------*/

#define MAX_LINE_LENGTH 256
#define TO_STR_INNER(s) #s
#define TO_STR(s) TO_STR_INNER(s)

/* Private variables ---------------------------------------------------------*/

static Introspectable root_obj = ODriveTypeInfo<ODrive>::make_introspectable(odrv);

/* Private function prototypes -----------------------------------------------*/
/* Function implementations --------------------------------------------------*/

// @brief Sends a line on the specified output.
template<typename ... TArgs>
void respond(StreamSink& output, bool include_checksum, const char * fmt, TArgs&& ... args) {
    char response[64]; // Hardcoded max buffer size. We silently truncate the output if it's too long for the buffer.
    size_t len = snprintf(response, sizeof(response), fmt, std::forward<TArgs>(args)...);
    len = std::min(len, sizeof(response));
    output.process_bytes((uint8_t*)response, len, nullptr); // TODO: use process_all instead
    if (include_checksum) {
        uint8_t checksum = 0;
        for (size_t i = 0; i < len; ++i)
            checksum ^= response[i];
        len = snprintf(response, sizeof(response), "!%u", checksum);
        len = std::min(len, sizeof(response));
        output.process_bytes((uint8_t*)response, len, nullptr);
    }
    output.process_bytes((const uint8_t*)"\n", 1, nullptr);
}

float serial_get_float(char *buf, int *decimal_pos, int decimal_idx)
{
    bool is_negative = false;
    float reading;
    for (int idx_start = decimal_pos[decimal_idx - 1]; idx_start < decimal_pos[decimal_idx]; idx_start++)
    {
        is_negative |= (buf[idx_start + 3] == 45);
        if (is_negative)
            break;
    }
    int first_part_length = decimal_pos[decimal_idx] - decimal_pos[decimal_idx - 1];

    // abs bigger than ten
    // 12.34-12.34
    // 0123456789
    // 8 - 2 = 6
    // 12.3412.34
    // 0123456789
    // 7 - 2 = 5

    // abs bigger than one hundred
    // 12.34-123.4
    // 0123456789
    // 9 - 2 = 7
    // 12.34123.4
    // 0123456789
    // 8 - 2 = 6
    bool abs_bigger_than_ten = (is_negative && first_part_length == 6) || (!is_negative && first_part_length == 5);
    bool abs_bigger_than_one_hundred = (is_negative && first_part_length == 7) || (!is_negative && first_part_length == 6);
    reading = 1 * (buf[decimal_pos[decimal_idx] - 1] - 48) + (buf[decimal_pos[decimal_idx] + 1] - 48) * 0.1f + (buf[decimal_pos[decimal_idx] + 2] - 48) * 0.01f;
    if (abs_bigger_than_ten)
        reading += 10 * (buf[decimal_pos[decimal_idx] - 2] - 48);
    else if (abs_bigger_than_one_hundred)
    {
        reading += 10 * (buf[decimal_pos[decimal_idx] - 2] - 48) + 
                  100 * (buf[decimal_pos[decimal_idx] - 3] - 48);
    }
    if (is_negative)
        reading *= -1;
    return reading;
}

void set_axis_limits(Axis* axis, float desired_torque_limit, float desired_velocity_limit)
{
    axis->controller_.config_.vel_limit = desired_velocity_limit;
    axis->motor_.config_.torque_lim = desired_torque_limit;
}

void set_axis_targets(Axis* axis, driverCmd axis_cmd, float desired_pos, float desired_vel, float desired_tau)
{
    // assign the desired values if no errors detected
    if (axis->joint_mode_ != jointMode::JOINTMODE_ERROR)
    {
        switch (axis_cmd)
        {
            case driverCmd::TURN_OFF:
            {
                axis->requested_state_ = Axis::AXIS_STATE_IDLE;
                break;
            }
            case driverCmd::TORQUE_CONTROL:
            {
                axis->joint_mode_ = jointMode::JOINTMODE_TORQUE_CONTROL;
                axis->requested_state_ = Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
                axis->controller_.config_.control_mode = Controller::CONTROL_MODE_TORQUE_CONTROL;
                set_axis_limits(axis, 3.1f, 20.f);   // limit vel to 20*2*pi/13 = 9.67 rad/sec, limit torque to 13*3.1 = 40.3 Nm
                axis->controller_.input_torque_ = desired_tau;
                axis->watchdog_feed();
                break;
            }
            case driverCmd::POSITION_CONTROL_NORMAL:
            {
                axis->joint_mode_ = jointMode::JOINTMODE_POSITION_CONTROL_NORMAL;
                axis->requested_state_ = Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
                axis->controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
                set_axis_limits(axis, 3.1f, 20.f);   // limit vel to 20*2*pi/13 = 9.67 rad/sec, limit torque to 13*3.1 = 40.3 Nm
                axis->controller_.input_pos_ = desired_pos;
                axis->controller_.input_vel_ = desired_vel;
                axis->controller_.input_torque_ = desired_tau;
                axis->watchdog_feed();
                break;
            }
            case driverCmd::POSITION_CONTROL_LOW_TORQUE:
            {
                axis->joint_mode_ = jointMode::JOINTMODE_POSITION_CONTROL_LOW_TORQUE;
                axis->requested_state_ = Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
                axis->controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
                set_axis_limits(axis, 1.0f, 5.f);   // limit vel to 5*2*pi/13 = 2.417 rad/sec, limit torque to 0.3*3.1 = 3.9 Nm
                axis->controller_.input_pos_ = desired_pos;
                axis->controller_.input_vel_ = desired_vel;
                axis->controller_.input_torque_ = desired_tau;
                axis->watchdog_feed();
                break;
            }
            case driverCmd::VELOCITY_CONTROL_LOW_TORQUE:
            {
                axis->joint_mode_ = jointMode::JOINTMODE_VELOCITY_CONTROL_LOW_TORQUE;
                axis->requested_state_ = Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
                axis->controller_.config_.control_mode = Controller::CONTROL_MODE_VELOCITY_CONTROL;
                set_axis_limits(axis, 1.0f, 5.f);   // limit vel to 5*2*pi/13 = 2.417 rad/sec, limit torque to 0.3*3.1 = 3.9 Nm
                axis->controller_.input_vel_ = desired_vel;
                axis->controller_.input_torque_ = desired_tau;
                axis->watchdog_feed();
                break;
            }
            case driverCmd::FULL_CALIBRATION:
            {
                axis->requested_state_ = Axis::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
                break;
            }
            case driverCmd::CLEAR_ERROR:
            {
                axis->clear_errors();
                axis->requested_state_ = Axis::AXIS_STATE_IDLE;
                axis->controller_.input_pos_ = 0;
                axis->controller_.input_vel_ = 0;
                axis->controller_.input_torque_ = 0;
                break;
            }
            case driverCmd::EMPTY:
            {
                // do nothing
                break;
            }
        }
    }
    else
    {
        axis->requested_state_ = Axis::AXIS_STATE_IDLE; // Turn off motor if joint_mode_ is JOINTMODE_ERROR
    }
}

// @brief Executes an ASCII protocol command
// @param buffer buffer of ASCII encoded characters
// @param len size of the buffer
void ASCII_protocol_process_line(const uint8_t* buffer, size_t len, StreamSink& response_channel) {
    static_assert(sizeof(char) == sizeof(uint8_t));

    // scan line to find beginning of checksum and prune comment
    uint8_t checksum = 0;
    size_t checksum_start = SIZE_MAX;
    for (size_t i = 0; i < len; ++i) {
        if (buffer[i] == ';') { // ';' is the comment start char
            len = i;
            break;
        }
        if (checksum_start > i) {
            if (buffer[i] == '!') {
                checksum_start = i + 1;
            } else {
                checksum ^= buffer[i];
            }
        }
    }

    // copy everything into a local buffer so we can insert null-termination
    char cmd[MAX_LINE_LENGTH + 1];
    if (len > MAX_LINE_LENGTH) len = MAX_LINE_LENGTH;
    memcpy(cmd, buffer, len);
    cmd[len] = 0; // null-terminate

    // checksum validation
    bool use_checksum = (checksum_start < len);
    if (use_checksum) {
        unsigned int received_checksum;
        int numscan = sscanf((const char *)cmd + checksum_start, "%u", &received_checksum);
        // wrong checksum -- error
        if ((numscan < 1) || (received_checksum != checksum))
        {
            respond(response_channel, true, "%u%u%.2f%.2f%.2f%.2f%.2f%.2f%.2f", 
                    int(jointMode::JOINTMODE_CONTROL_MSG_ERROR), int(jointMode::JOINTMODE_CONTROL_MSG_ERROR),
                    double(std::clamp(vbus_voltage, 0.f, 30.f)),
                    double(0),double(0),double(0),double(0),double(0),double(0));
            return;
        }
        len = checksum_start - 1; // prune checksum and asterisk
        cmd[len] = 0; // null-terminate
    }
    else    // checksum not detected -- error
    {
        respond(response_channel, true, "%u%u%.2f%.2f%.2f%.2f%.2f%.2f%.2f", 
                int(jointMode::JOINTMODE_CONTROL_MSG_ERROR), int(jointMode::JOINTMODE_CONTROL_MSG_ERROR),
                double(std::clamp(vbus_voltage, 0.f, 30.f)),
                double(0),double(0),double(0),double(0),double(0),double(0));
        return;
    }

    // Process control signal from serial
    int decimal_num = 0;
    int decimal_pos[6] = {0,0,0,0,0,0};
    for (size_t i = 0; i < len; i++)
    {
        if (decimal_num > 6)
        {
            return;
        }
        if (cmd[i] == 46)
        {
            decimal_pos[decimal_num] = i;
            decimal_num++;
        }
    }
    if (decimal_num == 5)   // cmds should contain 6 floats
    {
        Axis* a0 = axes[0];
        Axis* a1 = axes[1];
        driverCmd a0_cmd = driverCmd(cmd[0]-48);
        driverCmd a1_cmd = driverCmd(cmd[1]-48);
        float a0_desired_pos = 0;
        // 012345
        // 001.23       3
        if (decimal_pos[0] == 3)
        {
            a0_desired_pos = (cmd[2] - 48) + (cmd[4] - 48) * 0.1f + (cmd[5] - 48) * 0.01f;
        }
        else if (decimal_pos[0] == 4)
        {
        // 0123456
        // 00-1.23      4
            if (cmd[2] == 45)
            {
                a0_desired_pos = ((cmd[3] - 48) + (cmd[5] - 48) * 0.1f + (cmd[6] - 48) * 0.01f) * -1;
            }
        // 0123456
        // 0012.34      4
            else
            {
                a0_desired_pos = 10 * (cmd[2] - 48) + (cmd[3] - 48) + (cmd[5] - 48) * 0.1f + (cmd[6] - 48) * 0.01f;
            }
        }
        else if (decimal_pos[0] == 5)
        {
        // 01234567
        // 00-12.34     5
            if (cmd[2] == 45)
            {
                a0_desired_pos = (10 * (cmd[3] - 48) + (cmd[4] - 48) + (cmd[6] - 48) * 0.1f + (cmd[7] - 48) * 0.01f) * -1;
            }
        // 01234567
        // 00123.45     5
            else
            {
                a0_desired_pos = 100 * (cmd[2] - 48) + 10 * (cmd[3] - 48) + (cmd[4] - 48) + (cmd[6] - 48) * 0.1f + (cmd[7] - 48) * 0.01f;
            }
        }
        // 012345678
        // 00-123.45    6
        else if (decimal_pos[0] == 6)
        {
            a0_desired_pos = (100 * (cmd[3] - 48) + 10 * (cmd[4] - 48) + (cmd[5] - 48) + 0.1f * (cmd[7] - 48) + 0.01f * (cmd[8] - 48)) * -1;
        }
        a0_desired_pos = a0_desired_pos / (2*M_PI);
        float a0_desired_vel = serial_get_float(cmd, decimal_pos, 1) / (2*M_PI);
        float a0_desired_tau = serial_get_float(cmd, decimal_pos, 2);
        float a1_desired_pos = serial_get_float(cmd, decimal_pos, 3) / (2*M_PI);
        float a1_desired_vel = serial_get_float(cmd, decimal_pos, 4) / (2*M_PI);
        float a1_desired_tau = serial_get_float(cmd, decimal_pos, 5);

        set_axis_targets(a0, a0_cmd, a0_desired_pos, a0_desired_vel, a0_desired_tau);
        set_axis_targets(a1, a1_cmd, a1_desired_pos, a1_desired_vel, a1_desired_tau);
        if (a0->joint_mode_ != jointMode::JOINTMODE_ERROR && a1->joint_mode_ != jointMode::JOINTMODE_ERROR)
        {
            respond(response_channel, true, "%u%u%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", 
                    int(a0->joint_mode_), int(a1->joint_mode_),
                    double(std::clamp(vbus_voltage, float(0), float(30))),
                    double(std::clamp(a0->encoder_.pos_estimate_*2*M_PI, -999.f, 999.f)), 
                    double(std::clamp(a0->encoder_.vel_estimate_*2*M_PI, -999.f, 999.f)), 
                    double(std::clamp(a0->motor_.current_control_.Iq_measured * a0->motor_.config_.torque_constant, -999.f, 999.f)),
                    double(std::clamp(a1->encoder_.pos_estimate_*2*M_PI, -999.f, 999.f)), 
                    double(std::clamp(a1->encoder_.vel_estimate_*2*M_PI, -999.f, 999.f)), 
                    double(std::clamp(a1->motor_.current_control_.Iq_measured * a1->motor_.config_.torque_constant, -999.f, 999.f)));
        }
        else if (a0->joint_mode_ == jointMode::JOINTMODE_ERROR && a1->joint_mode_ != jointMode::JOINTMODE_ERROR)
        {
            respond(response_channel, true, "%u%u%.2f,%u,%u,%u,%.2f,%.2f,%.2f", 
                    int(a0->joint_mode_), int(a1->joint_mode_),
                    double(std::clamp(vbus_voltage, float(0), float(30))),
                    std::clamp(int(a0->motor_.error_), 0, 255),
                    std::clamp(int(a0->encoder_.error_), 0, 255),
                    std::clamp(int(a0->controller_.error_), 0, 255),
                    double(std::clamp(a1->encoder_.pos_estimate_*2*M_PI, -999.f, 999.f)), 
                    double(std::clamp(a1->encoder_.vel_estimate_*2*M_PI, -999.f, 999.f)), 
                    double(std::clamp(a1->motor_.current_control_.Iq_measured * a1->motor_.config_.torque_constant, -999.f, 999.f)));
        }
        else if (a0->joint_mode_ != jointMode::JOINTMODE_ERROR && a1->joint_mode_ == jointMode::JOINTMODE_ERROR)
        {
            respond(response_channel, true, "%u%u%.2f,%.2f,%.2f,%.2f,%u,%u,%u", 
                    int(a0->joint_mode_), int(a1->joint_mode_),
                    double(std::clamp(vbus_voltage, float(0), float(30))),
                    double(std::clamp(a0->encoder_.pos_estimate_*2*M_PI, -999.f, 999.f)), 
                    double(std::clamp(a0->encoder_.vel_estimate_*2*M_PI, -999.f, 999.f)), 
                    double(std::clamp(a0->motor_.current_control_.Iq_measured * a0->motor_.config_.torque_constant, -999.f, 999.f)),
                    std::clamp(int(a1->motor_.error_), 0, 255),
                    std::clamp(int(a1->encoder_.error_), 0, 255),
                    std::clamp(int(a1->controller_.error_), 0, 255));
        }
        else if (a0->joint_mode_ == jointMode::JOINTMODE_ERROR && a1->joint_mode_ == jointMode::JOINTMODE_ERROR)
        {
            respond(response_channel, true, "%u%u%.2f,%u,%u,%u,%u,%u,%u", 
                    int(a0->joint_mode_), int(a1->joint_mode_),
                    double(std::clamp(vbus_voltage, float(0), float(30))),
                    std::clamp(int(a0->motor_.error_), 0, 255),
                    std::clamp(int(a0->encoder_.error_), 0, 255),
                    std::clamp(int(a0->controller_.error_), 0, 255),
                    std::clamp(int(a1->motor_.error_), 0, 255),
                    std::clamp(int(a1->encoder_.error_), 0, 255),
                    std::clamp(int(a1->controller_.error_), 0, 255));
        }
    }
}

void ASCII_protocol_parse_stream(const uint8_t* buffer, size_t len, StreamSink& response_channel) {
    static uint8_t parse_buffer[MAX_LINE_LENGTH];
    static bool read_active = true;
    static uint32_t parse_buffer_idx = 0;

    while (len--) {
        // if the line becomes too long, reset buffer and wait for the next line
        if (parse_buffer_idx >= MAX_LINE_LENGTH) {
            read_active = false;
            parse_buffer_idx = 0;
        }

        // Fetch the next char
        uint8_t c = *(buffer++);
        bool is_end_of_line = c == '\n';
        if (is_end_of_line) {
            if (read_active)
                ASCII_protocol_process_line(parse_buffer, parse_buffer_idx, response_channel);
            parse_buffer_idx = 0;
            read_active = true;
        } else {
            if (read_active) {
                parse_buffer[parse_buffer_idx++] = c;
            }
        }
    }
}