#ifndef __SYNTA_H__
#define __SYNTA_H__

#include <stdio.h>
#include <stdint.h>
#include "stm32f10x.h"

#define ADDR_APP 0x8000000
#define ADDR_DATA 0x8003400

#define BUF_SIZE 64
#define FIRMWARE_VERSION 1281
#define AXIS_RA '1'
#define AXIS_DEC '2'
#define RA 0
#define DEC 1
#define MAX_STEP_NUM 30

#define MODE_SLEW 1
#define MODE_GOTO 2
#define MODE_SLEW_HS 3
#define MODE_GOTO_HS 0

#define STOP_MOVING 0x01        //stop the axis moveing completely
#define STOP_CHANGING 0x02      //just stop to change speed
#define AXIS_DECELLERATE 0x04

#define AXIS_STOPED 0x0100
#define AXIS_DIR 0x0200
#define AXIS_GOTO 0x0010
#define AXIS_ENERGISED 0x0001
#define AXIS_STOPING 0x1000

#define ST4_RA_NONE 0
#define ST4_RA_P 1
#define ST4_RA_M 2
#define ST4_DEC_NONE 0
#define ST4_DEC_P 1
#define ST4_DEC_M 2

#define FIND_STOP_INDEX(s_n,idx) ((s_n * 2) - 1 - idx)

#ifdef DEBUG
#define DEBUG_PRINTF(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINTF(fmt, ...)
#endif

struct axis_data_ {
    uint8_t revert;
    uint8_t mode;
    uint8_t motor_mode;
    uint8_t direction;
    uint16_t speed;
    uint16_t sidereal_scaler;
    uint32_t steps_per_axis;
    uint32_t steps_per_worm;
    uint32_t sidereal_step_rate;
    uint32_t high_speed_multiplier;
    uint32_t max_speed;
    uint32_t position;
    uint32_t status;
    uint32_t goto_position;
    //uint32_t init_speed;
    float guiding_speed_factor;
    int8_t (*stop)(void);
    int8_t (*emergency_stop)(void);
    int8_t (*enable)(void);
    int8_t (*action)(void);
};

struct save_axis_data_ {
    uint8_t revert;
    uint8_t motor_mode;
    uint16_t sidereal_scaler;
    uint32_t steps_per_axis;
    uint32_t steps_per_worm;
    uint32_t sidereal_step_rate;
    uint32_t high_speed_multiplier;
    uint32_t max_speed;
    float guiding_speed_factor;
};

struct tim_control_ {
    uint16_t flag;
    uint16_t prescaler;
    uint16_t period;
    uint16_t ccr;
    uint32_t steps;
};

extern uint8_t axis;
extern __IO uint8_t cmd_buf[BUF_SIZE];
extern __IO uint8_t f_cmd_buf[BUF_SIZE];
extern volatile struct axis_data_ axis_data[2];
extern __IO uint8_t tim_idx[MAX_STEP_NUM * 2];
extern uint16_t ra_step_num, dec_step_num;
extern struct save_axis_data_ save_axis_data[2];

struct cmd_ {
    char cmd;
    uint16_t in;
    uint16_t out;
};

extern uint32_t response_data;
extern char response[BUF_SIZE];

int16_t get_response_len(char cmd);
int16_t get_cmddata_len(char cmd);
int8_t vavid_cmd(char cmd);
uint8_t char_to_nibble(char s);
char nibble_to_char(uint8_t d);
void prepare_response(char *response, uint8_t cmd, uint32_t response_data);
int8_t calc_speed(uint8_t axis, uint8_t mode, uint16_t target_IVal, uint16_t *step_num, volatile struct tim_control_ *tim);
void init_tim_idx(uint16_t s_n);
void calc_guiding_speed(void);
void init_axis_data(void);

#endif
