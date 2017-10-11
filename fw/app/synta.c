#include "synta.h"
#include "hw_config.h"

#ifdef MOUNT_ENABLE
__IO uint8_t cmd_buf[BUF_SIZE];
char response[BUF_SIZE];
uint8_t axis;
uint32_t response_data;
float ra_speed_factor, dec_speed_factor;
struct save_axis_data_ save_axis_data[2];


struct cmd_ cmds[] = {
    {'e', 0, 6},
    {'a', 0, 6},
    {'b', 0, 6},
    {'g', 0, 2},
    {'s', 0, 6},
    {'K', 0, 0},
    {'E', 6, 0},
    {'j', 0, 6},
    {'f', 0, 3},
    {'G', 2, 0},
    {'H', 6, 0},
    {'I', 6, 0},
    {'M', 6, 0},
    {'J', 0, 0},
    {'F', 0, 0},
    {'L', 0, 0},
    {'Z', 6, 0},
    {'Y', 60, 0},
    {'X', 0, 2},
    {'W', 0, sizeof(save_axis_data)},
    {'V', sizeof(save_axis_data), 2},
    {'A', 0, 0},
    {'U', 0, 0},
    {'T', 0, 4},
    {'S', 0, 0},
    {'R', 0, 16},
    {'Q', 0, 2},
    {0, 0, 0},
};

volatile struct tim_control_ ra_tim_control[MAX_STEP_NUM + 1];
volatile struct tim_control_ dec_tim_control[MAX_STEP_NUM + 1];
volatile struct tim_control_ ra_guide_tim[2], dec_guide_tim;
TIM_TypeDef *axis_tim[2] = {TIM1, TIM2};
__IO uint8_t tim_idx[MAX_STEP_NUM * 2];

extern __IO uint8_t ra_stop_changing, dec_stop_changing;
extern __IO uint8_t ra_idx, dec_idx;
extern __IO uint8_t ra_need_to_stop, dec_need_to_stop;
extern __IO uint32_t ra_steps, dec_steps;
extern __IO uint8_t ra_guiding, dec_guiding;

uint16_t ra_step_num, dec_step_num;
uint8_t curve_steps[MAX_STEP_NUM + 1]; 

void init_axis_data(void)
{
    uint16_t i;

    if (*(uint32_t*)ADDR_DATA == 0xFFFFFFFF) {
        save_axis_data[RA].motor_mode = axis_data[RA].motor_mode; 
        save_axis_data[RA].sidereal_scaler = axis_data[RA].sidereal_scaler; 
        save_axis_data[RA].steps_per_axis = axis_data[RA].steps_per_axis; 
        save_axis_data[RA].steps_per_worm = axis_data[RA].steps_per_worm; 
        save_axis_data[RA].sidereal_step_rate = axis_data[RA].sidereal_step_rate; 
        save_axis_data[RA].high_speed_multiplier = axis_data[RA].high_speed_multiplier; 
        save_axis_data[RA].max_speed = axis_data[RA].max_speed; 
        save_axis_data[RA].guiding_speed_factor = axis_data[RA].guiding_speed_factor; 
        save_axis_data[RA].revert = axis_data[RA].revert; 
        save_axis_data[DEC].motor_mode = axis_data[DEC].motor_mode; 
        save_axis_data[DEC].sidereal_scaler = axis_data[DEC].sidereal_scaler; 
        save_axis_data[DEC].steps_per_axis = axis_data[DEC].steps_per_axis; 
        save_axis_data[DEC].steps_per_worm = axis_data[DEC].steps_per_worm; 
        save_axis_data[DEC].sidereal_step_rate = axis_data[DEC].sidereal_step_rate; 
        save_axis_data[DEC].high_speed_multiplier = axis_data[DEC].high_speed_multiplier; 
        save_axis_data[DEC].max_speed = axis_data[DEC].max_speed; 
        save_axis_data[DEC].guiding_speed_factor = axis_data[DEC].guiding_speed_factor; 
        save_axis_data[DEC].revert = axis_data[DEC].revert; 
        FLASH_Unlock();
        FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
        FLASH_ErasePage(ADDR_DATA);
        for (i = 0; i < sizeof(save_axis_data) / 2; i++)
            FLASH_ProgramHalfWord(ADDR_DATA + i * 2, ((uint16_t*)save_axis_data)[i]);
        FLASH_Lock();
    } else {
        for (i = 0; i < sizeof(save_axis_data) / 2; i++)
            ((uint16_t*)save_axis_data)[i] = ((uint16_t*)ADDR_DATA)[i];
        axis_data[RA].motor_mode = save_axis_data[RA].motor_mode; 
        axis_data[RA].sidereal_scaler = save_axis_data[RA].sidereal_scaler; 
        axis_data[RA].steps_per_axis = save_axis_data[RA].steps_per_axis; 
        axis_data[RA].steps_per_worm = save_axis_data[RA].steps_per_worm; 
        axis_data[RA].sidereal_step_rate = save_axis_data[RA].sidereal_step_rate; 
        axis_data[RA].high_speed_multiplier = save_axis_data[RA].high_speed_multiplier; 
        axis_data[RA].max_speed = save_axis_data[RA].max_speed; 
        axis_data[RA].guiding_speed_factor = save_axis_data[RA].guiding_speed_factor; 
        axis_data[RA].revert = save_axis_data[RA].revert; 
        axis_data[DEC].motor_mode = save_axis_data[DEC].motor_mode; 
        axis_data[DEC].sidereal_scaler = save_axis_data[DEC].sidereal_scaler; 
        axis_data[DEC].steps_per_axis = save_axis_data[DEC].steps_per_axis; 
        axis_data[DEC].steps_per_worm = save_axis_data[DEC].steps_per_worm; 
        axis_data[DEC].sidereal_step_rate = save_axis_data[DEC].sidereal_step_rate; 
        axis_data[DEC].high_speed_multiplier = save_axis_data[DEC].high_speed_multiplier; 
        axis_data[DEC].max_speed = save_axis_data[DEC].max_speed; 
        axis_data[DEC].guiding_speed_factor = save_axis_data[DEC].guiding_speed_factor; 
        axis_data[DEC].revert = save_axis_data[DEC].revert; 
    }
}

void init_tim_idx(uint16_t s_n)
{
    uint8_t i;

    for (i = 0; i < s_n; i++)
        tim_idx[i] = i + 1;

    for (;i < s_n * 2; i++)
        tim_idx[i] = s_n * 2 - i - 1;

    for (i = 0; i <= s_n; i++)
        curve_steps[i] = i;
}

int8_t vavid_cmd(char cmd)
{
    int8_t i;

    for (i = 0; cmds[i].cmd != 0; i++)
        if (cmd == cmds[i].cmd)
            return 1;

    return -1;
}

int16_t get_response_len(char cmd)
{
    int8_t i, len = -1;

    for(i = 0; cmds[i].cmd != 0; i++)
        if(cmd == cmds[i].cmd)
            len = cmds[i].out + 2;

    return len;
}

int16_t get_cmddata_len(char cmd)
{
    int8_t i, len = -1;

    for(i = 0; cmds[i].cmd != 0; i++)
        if(cmd == cmds[i].cmd)
            len = cmds[i].in;

    return len;
}

uint8_t char_to_nibble(char s)
{
    return s > '9' ? 0x0A + (s - 'A') : s - '0';
}

char nibble_to_char(uint8_t d)
{
    return d > 9 ? 'A' + (d - 10) : d + '0';
}

void prepare_response(char *response, uint8_t cmd, uint32_t response_data)
{
    int16_t len;
    len = get_response_len(cmd);
    DEBUG_PRINTF("\n\rLen=%d\n\r", len);

    if(len == -1) {
        response[0] = '!';
        response[1] = '\r';
#ifdef DEBUG
        response[2] = '\n';
        response[3] = '\0';
#endif
    } else if(len == 2) {
        response[0] = '=';
        response[1] = '\r';
#ifdef DEBUG
        response[2] = '\n';
        response[3] = '\0';
#endif
    } else if(len == 4) {
        response[0] = '=';
        response[1] = nibble_to_char((uint8_t)((response_data >> 4) & 0x0000000f));
        response[2] = nibble_to_char((uint8_t)(response_data & 0x0000000f));
        response[3] = '\r';
#ifdef DEBUG
        response[4] = '\n';
        response[5] = '\0';
#endif
    } else if(len == 5) {
        response[0] = '=';
        response[1] = nibble_to_char((uint8_t)((response_data >> 8) & 0x0000000f));
        response[2] = nibble_to_char((uint8_t)((response_data >> 4) & 0x0000000f));
        response[3] = nibble_to_char((uint8_t)(response_data & 0x0000000f));
        response[4] = '\r';
#ifdef DEBUG
        response[5] = '\n';
        response[6] = '\0';
#endif
    } else if(len == 8) {
        response[0] = '=';
        response[1] = nibble_to_char((uint8_t)((response_data >> 4) & 0x0000000f));
        response[2] = nibble_to_char((uint8_t)(response_data & 0x0000000f));
        response[3] = nibble_to_char((uint8_t)((response_data >> 12) & 0x0000000f));
        response[4] = nibble_to_char((uint8_t)((response_data >> 8) & 0x0000000f));
        response[5] = nibble_to_char((uint8_t)((response_data >> 20) & 0x0000000f));
        response[6] = nibble_to_char((uint8_t)((response_data >> 16) & 0x0000000f));
        response[7] = '\r';
#ifdef DEBUG
        response[8] = '\n';
        response[9] = '\0';
#endif
    } else if (len == 6) {
        response[0] = '=';
        response[5] = '\r';
#ifdef DEBUG
        response[6] = '\n';
        response[7] = '\0';
#endif
    } else if (len == (sizeof(save_axis_data)+2)) {
        response[0] = '=';
        response[sizeof(save_axis_data) + 1] = '\r';
#ifdef DEBUG
        response[sizeof(save_axis_data) + 2] = '\n';
        response[sizeof(save_axis_data) + 3] = '\0';
#endif
    }
}

int8_t ra_stop(void)
{
    if (ra_guiding != ST4_RA_NONE) {
        TIM_Cmd(axis_tim[RA], DISABLE);
        ra_guiding = ST4_RA_NONE;
        ra_motor_enable(0);
        axis_data[RA].status |= AXIS_STOPED;
    } else if (!(axis_data[RA].status & (AXIS_STOPED | AXIS_STOPING)))
        ra_need_to_stop = TRUE;
    return 1;
}

int8_t dec_stop(void)
{
    if (dec_guiding != ST4_DEC_NONE) {
        TIM_Cmd(axis_tim[DEC], DISABLE);
        dec_guiding = ST4_DEC_NONE;
        dec_motor_enable(0);
        axis_data[DEC].status |= AXIS_STOPED;
    } else if (!(axis_data[DEC].status & (AXIS_STOPED | AXIS_STOPING)))
        dec_need_to_stop = TRUE;
    return 1;
}

int8_t ra_emergency_stop(void)
{
    TIM_Cmd(axis_tim[RA], DISABLE);
    axis_data[RA].status |= AXIS_STOPED;
    ra_motor_enable(0);
    axis_data[RA].status &= ~AXIS_ENERGISED;
    return 1;
}

int8_t dec_emergency_stop(void)
{
    TIM_Cmd(axis_tim[DEC], DISABLE);
    axis_data[DEC].status |= AXIS_STOPED;
    dec_motor_enable(0);
    axis_data[DEC].status &= ~AXIS_ENERGISED;
    return 1;
}

int8_t ra_enable(void)
{
    ra_motor_mode(axis_data[RA].motor_mode);
    //ra_motor_enable(1);
    axis_data[RA].status |= (AXIS_STOPED | AXIS_STOPING | AXIS_ENERGISED);
    axis_data[RA].speed = 0;

    return 1;
}

int8_t dec_enable(void)
{
    dec_motor_mode(axis_data[DEC].motor_mode);
    //dec_motor_enable(1);
    axis_data[DEC].status |= (AXIS_STOPED | AXIS_STOPING | AXIS_ENERGISED);
    axis_data[DEC].speed = 0;
    return 1;
}

int8_t ra_action(void)
{
    axis_data[RA].status &= ~(AXIS_STOPED | AXIS_STOPING);
    ra_motor_dir(axis_data[RA].direction);
    if (axis_data[RA].speed != 0) {
        calc_speed(RA, axis_data[RA].mode, axis_data[RA].speed, &ra_step_num, &ra_tim_control[0]);
        axis_data[RA].speed = 0;
    } else
        calc_speed(RA, axis_data[RA].mode, axis_data[RA].sidereal_scaler/axis_data[RA].high_speed_multiplier, &ra_step_num, &ra_tim_control[0]);

    ra_motor_enable(1);
    TIM_ITConfig(axis_tim[RA], TIM_IT_Update, ENABLE);
    TIM_Cmd(axis_tim[RA], DISABLE);
    ra_steps = 0;
    ra_idx = 0;
    axis_tim[RA]->PSC = ra_tim_control[tim_idx[ra_idx]].prescaler;
    axis_tim[RA]->ARR = ra_tim_control[tim_idx[ra_idx]].period;
    axis_tim[RA]->CCR1 = ra_tim_control[tim_idx[ra_idx]].ccr;
    axis_tim[RA]->CNT = 0;
    ra_stop_changing = FALSE;
    TIM_Cmd(axis_tim[RA], ENABLE);

    return 1;
}

int8_t dec_action(void)
{
    axis_data[DEC].status &= ~(AXIS_STOPED | AXIS_STOPING);
    dec_motor_dir(axis_data[DEC].direction);
    if (axis_data[DEC].speed != 0) {
        calc_speed(DEC, axis_data[DEC].mode, axis_data[DEC].speed, &dec_step_num, &dec_tim_control[0]);
        axis_data[DEC].speed = 0;
    } else
        calc_speed(DEC, axis_data[DEC].mode, axis_data[DEC].sidereal_scaler/axis_data[DEC].high_speed_multiplier, &dec_step_num, &dec_tim_control[0]);

    dec_motor_enable(1);
    TIM_ITConfig(axis_tim[DEC], TIM_IT_Update, ENABLE);
    TIM_Cmd(axis_tim[DEC], DISABLE);
    dec_steps = 0;
    dec_idx = 0;
    axis_tim[DEC]->PSC = dec_tim_control[tim_idx[dec_idx]].prescaler;
    axis_tim[DEC]->ARR = dec_tim_control[tim_idx[dec_idx]].period;
    axis_tim[DEC]->CCR1 = dec_tim_control[tim_idx[dec_idx]].ccr;
    axis_tim[DEC]->CNT = 0;
    dec_stop_changing = FALSE;
    TIM_Cmd(axis_tim[DEC], ENABLE);

    return 1;
}

/* ========================================================================== */
/**
 * @brief   calc_speed calculate speed data from current speed to target speed
 *
 * @param   axis            target axis
 * @param   target_speed    IVal of target speed
 * @param   tim             speed data struct
 *
 * @return  1 -> ok, 0 -> error
 */
/* ========================================================================== */
int8_t calc_speed(uint8_t axis, uint8_t mode, uint16_t target_IVal, uint16_t *step_num, volatile struct tim_control_ *tim)
{
    uint16_t i, j;
    uint32_t tmp;
    uint32_t changing_step = 0;
    double freq, step;

    if (target_IVal < (axis_data[axis].sidereal_scaler / axis_data[axis].max_speed))
        target_IVal = axis_data[axis].sidereal_scaler / axis_data[axis].max_speed;

    freq = axis_data[axis].sidereal_step_rate / (double)target_IVal;

    tmp = axis_data[axis].sidereal_scaler / target_IVal;

    if (tmp >= 1 && tmp < 5)
        *step_num = 2;
    else if (tmp > 5 && tmp < 10)
        *step_num = 3;
    else if (tmp >= 10 && tmp < 20)
        *step_num = 4;
    else if (tmp >= 20 && tmp < 30)
        *step_num = 8;
    else if (tmp >= 30 && tmp < 150)
        *step_num = 15;
    else if (tmp >= 150)
        *step_num = 30;

    DEBUG_PRINTF("IVal=%d, target_freq=%ld, tmp=%ld, step_num=%d\n\r", target_IVal, (uint32_t)freq, tmp, *step_num);

    init_tim_idx(*step_num);

    step = freq / (double)*step_num;

    for(i = 1; i <= *step_num; i++) {
        tmp = (SystemCoreClock / (step * i) + 0.5);
        for(j = 1; j < 0xFFFF; j++) {
            if(tmp / j < 0xFFFF) {
                tim[i].prescaler = j - 1;
                tim[i].period = ((double)tmp / j + 0.5);
                tim[i].ccr = tim[i].period >> 1;
                tim[i].steps = curve_steps[i];
                tim[i].flag &= ~(AXIS_DECELLERATE|STOP_CHANGING);
                changing_step += curve_steps[i];
                break;
            }
        }

        if (mode == MODE_GOTO || mode == MODE_GOTO_HS) {
            if (axis_data[axis].goto_position < (changing_step * 2)) {
                tim[i].steps = axis_data[axis].goto_position - (changing_step - curve_steps[i]) * 2;
                tim[i].flag |= AXIS_DECELLERATE;
                axis_data[axis].goto_position = 0;
                break;
            } else if (i == *step_num) {
                tim[i].steps = axis_data[axis].goto_position - (changing_step - curve_steps[i]) * 2;
                axis_data[axis].goto_position = 0;
            }
        } else if (i == *step_num) {
            tim[i].flag |= STOP_CHANGING;
        }
    }

#ifdef DEBUG
    for (j = 0; j < i; j++)
        printf("psc=%d, period=%d, ccr=%d, steps=%ld\n\r",
                tim[j].prescaler, tim[j].period, tim[j].ccr, tim[j].steps);
#endif

    return 1;
}

void calc_guiding_speed(void)
{
    uint16_t i;
    uint32_t tmp;
    float freq;

    /* ST4_RA_P */
    freq = axis_data[RA].sidereal_step_rate * (1 + axis_data[RA].guiding_speed_factor) / axis_data[RA].sidereal_scaler;
    tmp = SystemCoreClock / freq;
    for(i = 1; i < 0xFFFF; i++) {
        if(tmp / i < 0xFFFF) {
            ra_guide_tim[0].prescaler = i - 1;
            ra_guide_tim[0].period = tmp / i;
            ra_guide_tim[0].ccr = ra_guide_tim[0].period >> 1;
            break;
        }
    }

    /* ST4_RA_M */
    freq = axis_data[RA].sidereal_step_rate * (1 - axis_data[RA].guiding_speed_factor) / axis_data[RA].sidereal_scaler;
    tmp = SystemCoreClock / freq;
    for(i = 1; i < 0xFFFF; i++) {
        if(tmp / i < 0xFFFF) {
            ra_guide_tim[1].prescaler = i - 1;
            ra_guide_tim[1].period = tmp / i;
            ra_guide_tim[1].ccr = ra_guide_tim[1].period >> 1;
            break;
        }
    }

    /* ST4_DEC */
    freq = axis_data[DEC].sidereal_step_rate * axis_data[DEC].guiding_speed_factor / axis_data[DEC].sidereal_scaler;
    tmp = SystemCoreClock / freq;
    for(i = 1; i < 0xFFFF; i++) {
        if(tmp / i < 0xFFFF) {
            dec_guide_tim.prescaler = i - 1;
            dec_guide_tim.period = tmp / i;
            dec_guide_tim.ccr = dec_guide_tim.period >> 1;
            break;
        }
    }
}

volatile struct axis_data_ axis_data[2] = {
    //RA
    {0, MODE_SLEW, 4, 0, 0, 6201, 3888000, 21600, 279809, 200, 250, 0, 0x0001, 0, 0.5, ra_stop, ra_emergency_stop, ra_enable, ra_action},
    //DEC
    {0, MODE_SLEW, 4, 0, 0, 6201, 3888000, 21600, 279809, 200, 250, 0, 0x0001, 0, 0.5, dec_stop, dec_emergency_stop, dec_enable, dec_action},
};
#endif
