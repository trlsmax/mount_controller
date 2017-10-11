#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "libserialport.h"
#include "iup.h"

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

struct save_axis_data_ save_axis_data[2];

Ihandle *dlg, *dlg_open, *vbox_main, *frame_s, *frame_u;
Ihandle *gbox_s, *hbox_com;
Ihandle *btn_refresh_com, *lst_com, *btn_com;
Ihandle *lbl_ra, *lbl_dec, *lbl_w, *lbl_m, *lbl_mode, *lbl_s,
        *lbl_h, *lbl_max, *btn_update;
Ihandle *txt_ra_w, *txt_ra_m, *txt_ra_s, *txt_ra_h,
        *txt_ra_max, *lst_ra_mode;
Ihandle *txt_dec_w, *txt_dec_m, *txt_dec_s, *txt_dec_h,
        *txt_dec_max, *lst_dec_mode;
Ihandle *lbl_aval, *txt_aval_ra, *txt_aval_dec;
Ihandle *lbl_sval, *txt_sval_ra, *txt_sval_dec;
Ihandle *lbl_bval, *txt_bval_ra, *txt_bval_dec;
Ihandle *lbl_ival, *txt_ival_ra, *txt_ival_dec;
Ihandle *btn_open, *btn_upgrade, *txt_path, *pb, *txt_sn;
Ihandle *lbl_gr, *lst_gr_ra, *lst_gr_dec;
Ihandle *btn_bl, *lbl_sn;

Ihandle *btn_advc, *advc_dlg, *lbl_advc_ra, *lbl_advc_dec,
        *btn_advc_ra, *btn_advc_dec, *txt_advc_val_ra, *txt_advc_val_dec,
        *btn_advc_ra_con, *btn_advc_dec_con,
        *btn_advc_to_aval_ra, *btn_advc_to_aval_dec, *btn_advc_ok, *btn_advc_ra_stop, *btn_advc_dec_stop,
        *btn_advc_to_sval_ra, *btn_advc_to_sval_dec, *tgl_ra, *tgl_dec, *lbl_revert; 

int port_idx, open_flag = -1;
struct sp_port **list;
struct sp_port_config *cfg = NULL;
uint32_t aval_ra, aval_dec;
uint32_t bval_ra, bval_dec;
uint32_t sval_ra, sval_dec;
uint32_t ival_ra, ival_dec;

static char byteMap[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };
static int byteMapLen = sizeof(byteMap);

/* Utility function to convert nibbles (4 bit values) into a hex character representation */
static char nibbleToChar(uint8_t nibble)
{
	if(nibble < byteMapLen) return byteMap[nibble];
	return '*';
}

/* Convert a buffer of binary values into a hex string representation */
char *bytesToHexString(uint8_t *bytes, size_t buflen)
{
	char *retval;
	int i;
	
	retval = malloc(buflen*2 + 1);
	for(i=0; i<buflen; i++) {
		retval[i*2] = nibbleToChar(bytes[i] >> 4);
		retval[i*2+1] = nibbleToChar(bytes[i] & 0x0f);
	}
    retval[buflen*2] = '\0';
	return retval;
}

int cb_txt_chk_num(Ihandle *ih, int c, char *new_value)
{
    /* only number and backspace */
    if(c != 0 && (c < 0x30 || c > 0x39) && c != '.')
        return IUP_IGNORE;

    return IUP_DEFAULT;
}

int cb_txt_changed_ra(Ihandle *self)
{
    uint16_t i;
    float w, m, s, mode;
    double tmp, delta, delta_n, sidereal;

    w = IupGetFloat(txt_ra_w, "VALUE");
    m = IupGetFloat(txt_ra_m, "VALUE");
    s = IupGetFloat(txt_ra_s, "VALUE");
    mode = IupGetFloatId(lst_ra_mode, "", IupGetInt(lst_ra_mode, "VALUE"));

    sval_ra = 360 * m * mode / s;
    aval_ra = sval_ra * w;
    sidereal = aval_ra / 86164.0905;

    delta_n = 100.0;
    for(i = 1; i < 65535; i++) {
        delta = modf(sidereal * i, &tmp);
        if(delta < delta_n) {
            delta_n = delta;
            if(i > 2000)
                break;
            else
                ival_ra = i;
        }
    }

    bval_ra = sidereal * ival_ra;

    IupSetInt(txt_aval_ra, "VALUE", (int)aval_ra);
    IupSetInt(txt_sval_ra, "VALUE", (int)sval_ra);
    IupSetInt(txt_bval_ra, "VALUE", (int)bval_ra);
    IupSetInt(txt_ival_ra, "VALUE", (int)ival_ra);

    return IUP_DEFAULT;
}

int cb_txt_changed_dec(Ihandle *self)
{
    uint16_t i;
    float w, m, s, mode;
    double tmp, delta, delta_n, sidereal;

    w = IupGetFloat(txt_dec_w, "VALUE");
    m = IupGetFloat(txt_dec_m, "VALUE");
    s = IupGetFloat(txt_dec_s, "VALUE");
    mode = IupGetFloatId(lst_dec_mode, "", IupGetInt(lst_dec_mode, "VALUE"));

    sval_dec = 360 * m * mode / s;
    aval_dec = sval_dec * w;
    sidereal = aval_dec / 86164.0905;

    delta_n = 100.0;
    for(i = 1; i < 65535; i++) {
        delta = modf(sidereal * i, &tmp);
        if(delta < delta_n) {
            delta_n = delta;
            if(i > 2000)
                break;
            else
                ival_dec = i;
        }
    }

    bval_dec = sidereal * ival_dec;

    IupSetInt(txt_aval_dec, "VALUE", (int)aval_dec);
    IupSetInt(txt_sval_dec, "VALUE", (int)sval_dec);
    IupSetInt(txt_bval_dec, "VALUE", (int)bval_dec);
    IupSetInt(txt_ival_dec, "VALUE", (int)ival_dec);

    return IUP_DEFAULT;
}

static int cb_btn_open(Ihandle *self)
{
    IupPopup(dlg_open, IUP_CENTER, IUP_CENTER);

    return IUP_DEFAULT;
}

static int cb_open_ok(Ihandle *ih, const char *file_name, const char *status)
{
    if(strcmp(status, "OK") == 0) {
        if(file_name == NULL)
            return IUP_IGNORE;
        else {
            printf("file selected:%s\n", file_name);
            printf("status:%s\n", status);
            IupSetAttribute(txt_path, "VALUE", file_name);
            return IUP_DEFAULT;
        }
    }

    return IUP_DEFAULT;
}

static int cb_btn_upgrade(Ihandle *self)
{
    /* declare a file pointer */
    FILE    *infile;
    uint8_t    *buffer;
    uint8_t *cmd_buf;
    char read_buf[5];
    uint16_t i,j;
    uint32_t    numbytes, chksum, chksum2;

    /* open an existing file for reading */
    infile = fopen(IupGetAttribute(txt_path, "VALUE"), "rb");

    /* quit if the file does not exist */
    if(infile == NULL) {
        IupMessage("Error", "Can't read firmware file.");
        return IUP_IGNORE;
    }

    /* Get the number of bytes */
    fseek(infile, 0L, SEEK_END);
    numbytes = ftell(infile);

    if (numbytes % 60 > 0)
        numbytes += 60 - (numbytes % 60) + 1;

    /* reset the file position indicator to
    the beginning of the file */
    fseek(infile, 0L, SEEK_SET);

    /* grab sufficient memory for the
    buffer to hold the text */
    buffer = (uint8_t*)calloc(numbytes, sizeof(uint8_t));

    /* memory error */
    if(buffer == NULL) {
        IupMessage("错误", "无法申请内存.");
        return IUP_IGNORE;
    }

    cmd_buf = (uint8_t*)calloc(64, sizeof(uint8_t));
    /* memory error */
    if(cmd_buf == NULL) {
        IupMessage("错误", "无法申请内存.");
        return IUP_IGNORE;
    }
    cmd_buf[0] = ':';
    cmd_buf[1] = 'Z';
    cmd_buf[2] = '1';

    /* copy all the text into the buffer */
    fread(buffer, sizeof(char), numbytes, infile);
    fclose(infile);

    /* confirm we have read the file by
    outputing it to the console */
    fprintf(stderr, "The file size is : %ud\n", numbytes);

    numbytes >>= 1;
    *(uint16_t*)&cmd_buf[3] = numbytes;
    for (chksum=0,i=0; i < numbytes; i++)
        chksum += ((uint16_t*)buffer)[i];
    fprintf(stderr, "chksum:0x%ux\n", chksum);
    *(uint32_t*)&cmd_buf[5] = chksum;

    cmd_buf[9] = '\r';
    cmd_buf[10] = '\0';
    fprintf(stderr, "cmd:%s\n", cmd_buf);
    fprintf(stderr, "port: %s\n", sp_get_port_name(list[port_idx]));
    if (sp_blocking_write(list[port_idx], cmd_buf, 10, 1000) != 10)
        IupMessage("错误", "无法向EQMAX发送命令");
    sp_flush(list[port_idx], SP_BUF_OUTPUT);
    j = sp_blocking_read(list[port_idx], read_buf, 2, 1000);

    IupSetFloat(pb, "MIN", (double)0);
    IupSetFloat(pb, "MAX", (double)numbytes);
    IupSetFloat(pb, "VALUE", (double)0);
    cmd_buf[0] = ':';
    cmd_buf[1] = 'Y';
    cmd_buf[2] = '1';
    cmd_buf[63] = '\r';
    i = 0;
    do {
        for (j = 0; i < numbytes && j < 30; i++, j++) {
            ((uint16_t *)&cmd_buf[3])[j] = ((uint16_t *)buffer)[i];
        }
        if (sp_blocking_write(list[port_idx], cmd_buf, 64, 1000) != 64)
            IupMessage("错误", "无法向EQMAX发送命令");
        sp_flush(list[port_idx], SP_BUF_OUTPUT);
        j = sp_blocking_read(list[port_idx], read_buf, 2, 1000);
        //fprintf(stderr, "Got return : %d\n", j);
        IupSetFloat(pb, "VALUE", (double)i);
        IupRedraw(pb,0);
    } while (i < numbytes);

    cmd_buf[0] = ':';
    cmd_buf[1] = 'X';
    cmd_buf[2] = '1';
    cmd_buf[3] = '\r';
    if (sp_blocking_write(list[port_idx], cmd_buf, 4, 1000) != 4)
        IupMessage("错误", "无法向EQMAX发送命令");
    sp_flush(list[port_idx], SP_BUF_OUTPUT);
    j = sp_blocking_read(list[port_idx], read_buf, 4, 10000);
    read_buf[j] = '\0';
    fprintf(stderr, "Got return : [%d]%s\n", j, read_buf);
    if (j == 4 && read_buf[0] == '=' && read_buf[1] == '0' && read_buf[2] == '1') {
        IupMessage("固件升级", "固件升级完成. 请重启EQMAX.");
        sp_close(list[port_idx]);
        open_flag = 0;
        IupSetAttribute(btn_com, "TITLE", "打开");
        IupSetAttribute(btn_update, "ACTIVE", "NO");
        IupSetAttribute(btn_upgrade, "ACTIVE", "NO");
        IupSetAttribute(btn_advc, "ACTIVE", "NO");
        IupSetAttribute(lst_com, "ACTIVE", "YES");
        IupSetAttribute(btn_refresh_com, "ACTIVE", "YES");
        printf("Serial closed\n");
    } else
        IupMessage("固件升级", "固件升级失败，请重试");

    /* free the memory we used for the buffer */
    free(buffer);
    free(cmd_buf);

    return IUP_DEFAULT;
}

static int cb_btn_com(Ihandle *self)
{
    char cmd_buf[60], read_buf[60], *sn;
    uint16_t i;

    if (sp_new_config(&cfg) != SP_OK)
        fprintf(stderr, "Fail to get new config.\n");

    port_idx = IupGetInt(lst_com, "VALUE") - 1;

    if (open_flag > 0) {
        sp_close(list[port_idx]);
        open_flag = 0;
        IupSetAttribute(btn_com, "TITLE", "打开");
        IupSetAttribute(btn_update, "ACTIVE", "NO");
        IupSetAttribute(btn_upgrade, "ACTIVE", "NO");
        IupSetAttribute(btn_advc, "ACTIVE", "NO");
        IupSetAttribute(lst_com, "ACTIVE", "YES");
        IupSetAttribute(btn_refresh_com, "ACTIVE", "YES");
        IupSetAttribute(btn_bl, "ACTIVE", "NO");
        printf("Serial closed\n");
    } else {
        if (sp_open(list[port_idx], SP_MODE_READ|SP_MODE_WRITE) != SP_OK) {
            IupMessagef ("错误", "无法打开串口: %s\n", sp_get_port_name(list[port_idx]));
            return IUP_DEFAULT;
        }

        sp_set_config_baudrate(cfg, 9600);
        sp_set_config_bits(cfg, 8);
        sp_set_config_parity(cfg, SP_PARITY_NONE);
        sp_set_config_stopbits(cfg, 1);
        sp_set_config_xon_xoff(cfg, SP_XONXOFF_DISABLED);

        if (sp_set_config(list[port_idx], cfg) != SP_OK)
            fprintf(stderr, "Fail to set config\n");

        open_flag = 1;
        IupSetAttribute(btn_com, "TITLE", "关闭");
        //IupSetAttribute(btn_update, "ACTIVE", "YES");
        //IupSetAttribute(btn_upgrade, "ACTIVE", "YES");
        IupSetAttribute(lst_com, "ACTIVE", "NO");
        IupSetAttribute(btn_refresh_com, "ACTIVE", "NO");
        //IupSetAttribute(btn_advc, "ACTIVE", "YES");
        printf("Serial Opened\n");

        cmd_buf[0] = ':';
        cmd_buf[1] = 'R';
        cmd_buf[2] = '1';
        cmd_buf[3] = '\r';
        if (sp_blocking_write(list[port_idx], cmd_buf, 4, 1000) != 4)
            IupMessage("错误", "无法向EQMAX发送命令");
        sp_flush(list[port_idx], SP_BUF_OUTPUT);
        i = sp_blocking_read(list[port_idx], read_buf, 18, 1000);
        fprintf(stderr, "got data back %d/%s\n\r", i, (char*)&read_buf[0]);
        if (i != 18) {
            IupMessage("错误", "无法连接EQMAX");
            sp_close(list[port_idx]);
            open_flag = 0;
            IupSetAttribute(btn_com, "TITLE", "打开");
            IupSetAttribute(btn_update, "ACTIVE", "NO");
            IupSetAttribute(btn_upgrade, "ACTIVE", "NO");
            IupSetAttribute(lst_com, "ACTIVE", "YES");
            IupSetAttribute(btn_refresh_com, "ACTIVE", "YES");
            printf("Serial closed\n");
            return IUP_DEFAULT;
        }
        printf("\n\r");

        if (read_buf[1] != 0) {
            //bootloader
            IupSetAttribute(btn_open, "ACTIVE", "YES");
            IupSetAttribute(btn_upgrade, "ACTIVE", "YES");
            sn = bytesToHexString(&read_buf[1], 16);
            fprintf(stderr, "Length of sn = %d.\n\r", strlen(sn));
            IupSetAttribute(txt_sn, "VALUE", sn);
        } else { //app Firmware
            IupSetAttribute(btn_bl, "ACTIVE", "YES");
            if (read_buf[2] == 1 || read_buf[2] == 3) {
                cmd_buf[0] = ':';
                cmd_buf[1] = 'W';
                cmd_buf[2] = '1';
                cmd_buf[3] = '\r';
                if (sp_blocking_write(list[port_idx], cmd_buf, 4, 1000) != 4)
                    IupMessage("错误", "无法向EQMAX发送命令");
                sp_flush(list[port_idx], SP_BUF_OUTPUT);
                i = sp_blocking_read(list[port_idx], read_buf, sizeof(save_axis_data) + 2, 1000);
                fprintf(stderr, "got data back %d/%c\n\r", i, read_buf[0]);
                if (i != sizeof(save_axis_data) + 2) {
                    IupMessage("错误", "无法连接EQMAX");
                    sp_close(list[port_idx]);
                    open_flag = 0;
                    IupSetAttribute(btn_com, "TITLE", "打开");
                    IupSetAttribute(btn_update, "ACTIVE", "NO");
                    IupSetAttribute(lst_com, "ACTIVE", "YES");
                    IupSetAttribute(btn_refresh_com, "ACTIVE", "YES");
                    IupSetAttribute(btn_advc, "ACTIVE", "YES");
                    IupSetAttribute(btn_bl, "ACTIVE", "NO");
                    printf("Serial closed\n");
                    return IUP_DEFAULT;
                }
                printf("\n\r");
                
                IupSetAttribute(btn_update, "ACTIVE", "YES");
                IupSetAttribute(btn_advc, "ACTIVE", "YES");
        
                for (i = 0; i < sizeof(save_axis_data); i++) {
                    ((uint8_t*)save_axis_data)[i] = ((uint8_t*)&read_buf[1])[i];
                    printf("0x%02x ", ((uint8_t*)&read_buf[1])[i]);
                    if (i != 0 && i % 8 == 0)
                        printf("\n\r");
                }

                IupSetInt(txt_ra_h, "VALUE", save_axis_data[0].high_speed_multiplier);
                IupSetInt(txt_ra_max, "VALUE", save_axis_data[0].max_speed);
                IupSetInt(txt_aval_ra, "VALUE", save_axis_data[0].steps_per_axis);
                IupSetInt(txt_sval_ra, "VALUE", save_axis_data[0].steps_per_worm);
                IupSetInt(txt_bval_ra, "VALUE", save_axis_data[0].sidereal_step_rate);
                IupSetInt(txt_ival_ra, "VALUE", save_axis_data[0].sidereal_scaler);
                IupSetInt(lst_ra_mode, "VALUE", log(save_axis_data[0].motor_mode)/log(2) + 1);
                IupSetInt(lst_gr_ra, "VALUE", (uint16_t)(save_axis_data[0].guiding_speed_factor * 10));
                IupSetAttribute(tgl_ra, "VALUE", save_axis_data[0].revert == 1 ? "YES" : "NO");

                IupSetInt(txt_dec_h, "VALUE", save_axis_data[1].high_speed_multiplier);
                IupSetInt(txt_dec_max, "VALUE", save_axis_data[1].max_speed);
                IupSetInt(txt_aval_dec, "VALUE", save_axis_data[1].steps_per_axis);
                IupSetInt(txt_sval_dec, "VALUE", save_axis_data[1].steps_per_worm);
                IupSetInt(txt_bval_dec, "VALUE", save_axis_data[1].sidereal_step_rate);
                IupSetInt(txt_ival_dec, "VALUE", save_axis_data[1].sidereal_scaler);
                IupSetInt(lst_dec_mode, "VALUE", log(save_axis_data[1].motor_mode)/log(2) + 1);
                IupSetInt(lst_gr_dec, "VALUE", (uint16_t)(save_axis_data[1].guiding_speed_factor * 10));
                IupSetAttribute(tgl_dec, "VALUE", save_axis_data[1].revert == 1 ? "YES" : "NO");
            }
        }
    }
    
    return IUP_DEFAULT;
}

int cb_btn_refresh_com(Ihandle *self)
{
    uint16_t i;

    IupSetAttribute(lst_com, "REMOVEITEM", "ALL");
    sp_free_port_list(list);
    if(sp_list_ports(&list) != SP_OK) {
        fprintf(stderr, "Can't list serial ports.\n");
    } else {
        printf("List serial ports OK.\n");
        for(i = 0; list[i] != NULL; i++) {
            printf("Port[%2d]:%s.\n", i, sp_get_port_name(list[i]));
            IupSetAttributeId(lst_com, "", i + 1, sp_get_port_name(list[i]));
        }
        IupSetAttribute(lst_com, "VALUE", "1");
    }

    return IUP_DEFAULT;
}

int cb_btn_update(Ihandle *self)
{
    char cmd_buf[60], read_buf[10];
    uint16_t i;

    if (IupGetInt(txt_aval_ra, "VALUE") == 0 ||
        IupGetInt(txt_sval_ra, "VALUE") == 0 ||
        IupGetInt(txt_bval_ra, "VALUE") == 0 ||
        IupGetInt(txt_ival_ra, "VALUE") == 1 ||
        IupGetInt(txt_aval_dec, "VALUE") == 0 ||
        IupGetInt(txt_sval_dec, "VALUE") == 0 ||
        IupGetInt(txt_bval_dec, "VALUE") == 0 ||
        IupGetInt(txt_ival_dec, "VALUE") == 1) {
        IupMessage("错误", "请先完整填写所有参数");
        return IUP_DEFAULT;
    }

    save_axis_data[0].motor_mode = IupGetIntId(lst_ra_mode, "", IupGetInt(lst_ra_mode, "VALUE"));
    save_axis_data[0].sidereal_scaler = IupGetInt(txt_ival_ra, "VALUE");
    save_axis_data[0].steps_per_axis = IupGetInt(txt_aval_ra, "VALUE");
    save_axis_data[0].steps_per_worm = IupGetInt(txt_sval_ra, "VALUE");
    save_axis_data[0].sidereal_step_rate = IupGetInt(txt_bval_ra, "VALUE");
    save_axis_data[0].high_speed_multiplier = IupGetInt(txt_ra_h, "VALUE");
    save_axis_data[0].max_speed = IupGetInt(txt_ra_max, "VALUE");
    save_axis_data[0].guiding_speed_factor = IupGetFloat(lst_gr_ra, "VALUE")/10;
    save_axis_data[0].revert = (uint8_t)IupGetInt(tgl_ra, "VALUE");

    save_axis_data[1].motor_mode = IupGetIntId(lst_dec_mode, "", IupGetInt(lst_dec_mode, "VALUE"));
    save_axis_data[1].sidereal_scaler = IupGetInt(txt_ival_dec, "VALUE");
    save_axis_data[1].steps_per_axis = IupGetInt(txt_aval_dec, "VALUE");
    save_axis_data[1].steps_per_worm = IupGetInt(txt_sval_dec, "VALUE");
    save_axis_data[1].sidereal_step_rate = IupGetInt(txt_bval_dec, "VALUE");
    save_axis_data[1].high_speed_multiplier = IupGetInt(txt_dec_h, "VALUE");
    save_axis_data[1].max_speed = IupGetInt(txt_dec_max, "VALUE");
    save_axis_data[1].guiding_speed_factor = IupGetFloat(lst_gr_dec, "VALUE")/10;
    save_axis_data[1].revert = (uint8_t)IupGetInt(tgl_dec, "VALUE");

    cmd_buf[0] = ':';
    cmd_buf[1] = 'V';
    cmd_buf[2] = '1';
    for (i = 0; i < sizeof(save_axis_data); i++)
        ((uint8_t*)&cmd_buf[3])[i] = ((uint8_t*)save_axis_data)[i];
    cmd_buf[sizeof(save_axis_data) + 3] = '\r';

    i = sp_blocking_write(list[port_idx], cmd_buf, (sizeof(save_axis_data) + 4), 1000);
    if (i != sizeof(save_axis_data) + 4)
    {
        IupMessage("Error", "Fail to write cmd");
        return IUP_DEFAULT;
    }
    sp_flush(list[port_idx], SP_BUF_OUTPUT);
    i = sp_blocking_read(list[port_idx], read_buf, 4, 1000);
    printf("Got data : [%d]0x%02x 0x%02x 0x%02x 0x%02x\n\r", i, read_buf[0], read_buf[1], read_buf[2], read_buf[3]);
    if (i == 4 && read_buf[0] == '=' && read_buf[1] == '0' && read_buf[2] == '1' && read_buf[3] == '\r')
        IupMessage("完成", "参数更新完成");

    return IUP_DEFAULT;
}

int cb_txt_aval_ra_change(Ihandle *self)
{
    uint16_t i;
    double tmp, delta, delta_n, sidereal;

    aval_ra = IupGetFloat(txt_aval_ra, "VALUE");
    sidereal = aval_ra / 86164.0905;

    delta_n = 100.0;
    for(i = 1; i < 65535; i++) {
        delta = modf(sidereal * i, &tmp);
        if(delta < delta_n) {
            delta_n = delta;
            if(i > 2000)
                break;
            else
                ival_ra = i;
        }
    }

    bval_ra = sidereal * ival_ra;

    IupSetInt(txt_aval_ra, "VALUE", (int)aval_ra);
    IupSetInt(txt_bval_ra, "VALUE", (int)bval_ra);
    IupSetInt(txt_ival_ra, "VALUE", (int)ival_ra);

    return IUP_DEFAULT;
}

int cb_txt_aval_dec_change(Ihandle *self)
{
    uint16_t i;
    double tmp, delta, delta_n, sidereal;

    aval_dec = IupGetFloat(txt_aval_dec, "VALUE");
    sidereal = aval_dec / 86164.0905;

    delta_n = 100.0;
    for(i = 1; i < 65535; i++) {
        delta = modf(sidereal * i, &tmp);
        if(delta < delta_n) {
            delta_n = delta;
            if(i > 2000)
                break;
            else
                ival_dec = i;
        }
    }

    bval_dec = sidereal * ival_dec;

    IupSetInt(txt_aval_dec, "VALUE", (int)aval_dec);
    IupSetInt(txt_bval_dec, "VALUE", (int)bval_dec);
    IupSetInt(txt_ival_dec, "VALUE", (int)ival_dec);

    return IUP_DEFAULT;
}

int cb_txt_sval_dec_change(Ihandle *self)
{
    uint16_t i;
    float w;
    double tmp, delta, delta_n, sidereal;

    sval_dec = IupGetFloat(txt_sval_dec, "VALUE");
    w = IupGetFloat(txt_dec_w, "VALUE");

    aval_dec = sval_dec * w;
    sidereal = aval_dec / 86164.0905;

    delta_n = 100.0;
    for(i = 1; i < 65535; i++) {
        delta = modf(sidereal * i, &tmp);
        if(delta < delta_n) {
            delta_n = delta;
            if(i > 2000)
                break;
            else
                ival_dec = i;
        }
    }

    bval_dec = sidereal * ival_dec;

    IupSetInt(txt_aval_dec, "VALUE", (int)aval_dec);
    IupSetInt(txt_sval_dec, "VALUE", (int)sval_dec);
    IupSetInt(txt_bval_dec, "VALUE", (int)bval_dec);
    IupSetInt(txt_ival_dec, "VALUE", (int)ival_dec);

    return IUP_DEFAULT;
}

int cb_txt_sval_ra_change(Ihandle *self)
{
    uint16_t i;
    float w;
    double tmp, delta, delta_n, sidereal;

    sval_ra = IupGetFloat(txt_sval_ra, "VALUE");
    w = IupGetFloat(txt_ra_w, "VALUE");

    aval_ra = sval_ra * w;
    sidereal = aval_ra / 86164.0905;

    delta_n = 100.0;
    for(i = 1; i < 65535; i++) {
        delta = modf(sidereal * i, &tmp);
        if(delta < delta_n) {
            delta_n = delta;
            if(i > 2000)
                break;
            else
                ival_ra = i;
        }
    }

    bval_ra = sidereal * ival_ra;

    IupSetInt(txt_aval_ra, "VALUE", (int)aval_ra);
    IupSetInt(txt_sval_ra, "VALUE", (int)sval_ra);
    IupSetInt(txt_bval_ra, "VALUE", (int)bval_ra);
    IupSetInt(txt_ival_ra, "VALUE", (int)ival_ra);


    return IUP_DEFAULT;
}

int cb_btn_advc(Ihandle *self)
{
    IupPopup(advc_dlg, IUP_CENTER, IUP_CENTER);
    return IUP_DEFAULT;
}

int cb_btn_advc_ra(Ihandle *self)
{
    char cmd_buf[60], read_buf[10];
    int8_t i;
    cmd_buf[0] = ':';
    cmd_buf[1] = 'U';
    cmd_buf[2] = '1';
    cmd_buf[3] = '\r';

    if (i = sp_blocking_write(list[port_idx], cmd_buf, 4, 1000) != 4) {
        IupMessage("错误", "无法向EQMAX发送命令");
        return IUP_DEFAULT;
    }
    sp_blocking_read(list[port_idx], read_buf, 2, 1000);

    return IUP_DEFAULT;
}

int cb_btn_advc_ra_stop(Ihandle *self)
{
    char cmd_buf[60], read_buf[10];
    int8_t i, j;
    uint32_t val;
    cmd_buf[0] = ':';
    cmd_buf[1] = 'T';
    cmd_buf[2] = '1';
    cmd_buf[3] = '\r';

    if (i = sp_blocking_write(list[port_idx], cmd_buf, 4, 1000) != 4) {
        IupMessage("错误", "无法向EQMAX发送命令");
        return IUP_DEFAULT;
    }
    sp_flush(list[port_idx], SP_BUF_OUTPUT);

    if (i = sp_blocking_read(list[port_idx], read_buf, 6, 1000) != 6) {
        IupMessage("错误", "无法读取结果");
        return IUP_DEFAULT;
    }

    if (read_buf[0] == '=' && read_buf[5] == '\r') {
        for (i = 0; i < 4; i++)
            ((uint8_t*)&val)[i] = read_buf[i+1];
        IupSetInt(txt_advc_val_ra, "VALUE", val);
    } else {
        IupMessage("错误", "无法读取结果");
        return IUP_DEFAULT;
    }

    return IUP_DEFAULT;
}

int cb_btn_advc_dec(Ihandle *self)
{
    char cmd_buf[60], read_buf[10];
    int8_t i;
    cmd_buf[0] = ':';
    cmd_buf[1] = 'U';
    cmd_buf[2] = '2';
    cmd_buf[3] = '\r';

    if (i = sp_blocking_write(list[port_idx], cmd_buf, 4, 1000) != 4) {
        IupMessage("错误", "无法向EQMAX发送命令");
        return IUP_DEFAULT;
    }
    sp_blocking_read(list[port_idx], read_buf, 2, 1000);

    return IUP_DEFAULT;
}

int cb_btn_advc_dec_stop(Ihandle *self)
{
    char cmd_buf[60], read_buf[10];
    int8_t i, j;
    uint32_t val;
    cmd_buf[0] = ':';
    cmd_buf[1] = 'T';
    cmd_buf[2] = '2';
    cmd_buf[3] = '\r';

    if (i = sp_blocking_write(list[port_idx], cmd_buf, 4, 1000) != 4) {
        IupMessage("错误", "无法向EQMAX发送命令");
        return IUP_DEFAULT;
    }
    sp_flush(list[port_idx], SP_BUF_OUTPUT);

    if (i = sp_blocking_read(list[port_idx], read_buf, 6, 1000) != 6) {
        IupMessage("错误", "无法读取结果");
        return IUP_DEFAULT;
    }

    if (read_buf[0] == '=' && read_buf[5] == '\r') {
        for (i = 0; i < 4; i++)
            ((uint8_t*)&val)[i] = read_buf[i+1];
        IupSetInt(txt_advc_val_dec, "VALUE", val);
    } else {
        IupMessage("错误", "无法读取结果");
        return IUP_DEFAULT;
    }

    return IUP_DEFAULT;
}

int cb_btn_advc_ra_con(Ihandle *self)
{
    char cmd_buf[60], read_buf[10];
    int8_t i;
    cmd_buf[0] = ':';
    cmd_buf[1] = 'S';
    cmd_buf[2] = '1';
    cmd_buf[3] = '\r';

    if (i = sp_blocking_write(list[port_idx], cmd_buf, 4, 1000) != 4) {
        IupMessage("错误", "无法向EQMAX发送命令");
        return IUP_DEFAULT;
    }
    sp_blocking_read(list[port_idx], read_buf, 2, 1000);
    return IUP_DEFAULT;
}

int cb_btn_advc_dec_con(Ihandle *self)
{
    char cmd_buf[60], read_buf[10];
    int8_t i;
    cmd_buf[0] = ':';
    cmd_buf[1] = 'S';
    cmd_buf[2] = '2';
    cmd_buf[3] = '\r';

    if (i = sp_blocking_write(list[port_idx], cmd_buf, 4, 1000) != 4) {
        IupMessage("错误", "无法向EQMAX发送命令");
        return IUP_DEFAULT;
    }
    sp_blocking_read(list[port_idx], read_buf, 2, 1000);
    return IUP_DEFAULT;
}

int cb_btn_advc_to_aval_ra(Ihandle *self)
{
    int aval;
    uint16_t i;
    double tmp, delta, delta_n, sidereal;

    if ((aval = IupGetInt(txt_advc_val_ra, "VALUE")) > 0) {
        aval_ra = aval;
        sidereal = aval_ra / 86164.0905;

        delta_n = 100.0;
        for(i = 1; i < 65535; i++) {
            delta = modf(sidereal * i, &tmp);
            if(delta < delta_n) {
                delta_n = delta;
                if(i > 2000)
                    break;
                else
                    ival_ra = i;
            }
        }

        bval_ra = sidereal * ival_ra;

        IupSetInt(txt_aval_ra, "VALUE", (int)aval_ra);
        IupSetInt(txt_bval_ra, "VALUE", (int)bval_ra);
        IupSetInt(txt_ival_ra, "VALUE", (int)ival_ra);
    }
    return IUP_DEFAULT;
}

int cb_btn_advc_to_sval_ra(Ihandle *self)
{
    int sval;
    uint16_t i;
    float w;
    double tmp, delta, delta_n, sidereal;

    if ((sval = IupGetInt(txt_advc_val_ra, "VALUE")) > 0) {
        sval_ra = sval;
        w = IupGetFloat(txt_ra_w, "VALUE");

        aval_ra = sval_ra * w;
        sidereal = aval_ra / 86164.0905;

        delta_n = 100.0;
        for(i = 1; i < 65535; i++) {
            delta = modf(sidereal * i, &tmp);
            if(delta < delta_n) {
                delta_n = delta;
                if(i > 2000)
                    break;
                else
                    ival_ra = i;
            }
        }

        bval_ra = sidereal * ival_ra;

        IupSetInt(txt_aval_ra, "VALUE", (int)aval_ra);
        IupSetInt(txt_sval_ra, "VALUE", (int)sval_ra);
        IupSetInt(txt_bval_ra, "VALUE", (int)bval_ra);
        IupSetInt(txt_ival_ra, "VALUE", (int)ival_ra);
    }

    return IUP_DEFAULT;
}

int cb_btn_advc_to_aval_dec(Ihandle *self)
{
    int aval;
    uint16_t i;
    double tmp, delta, delta_n, sidereal;

    if ((aval = IupGetInt(txt_advc_val_dec, "VALUE")) > 0) {
        aval_dec = aval;
        sidereal = aval_dec / 86164.0905;

        delta_n = 100.0;
        for(i = 1; i < 65535; i++) {
            delta = modf(sidereal * i, &tmp);
            if(delta < delta_n) {
                delta_n = delta;
                if(i > 2000)
                    break;
                else
                    ival_dec = i;
            }
        }

        bval_dec = sidereal * ival_dec;

        IupSetInt(txt_aval_dec, "VALUE", (int)aval_dec);
        IupSetInt(txt_bval_dec, "VALUE", (int)bval_dec);
        IupSetInt(txt_ival_dec, "VALUE", (int)ival_dec);
    }

    return IUP_DEFAULT;
}

int cb_btn_advc_to_sval_dec(Ihandle *self)
{
    int sval;
    uint16_t i;
    float w;
    double tmp, delta, delta_n, sidereal;

    if ((sval = IupGetInt(txt_advc_val_dec, "VALUE")) > 0) {
        sval_dec = sval;
        w = IupGetFloat(txt_dec_w, "VALUE");

        aval_dec = sval_dec * w;
        sidereal = aval_dec / 86164.0905;

        delta_n = 100.0;
        for(i = 1; i < 65535; i++) {
            delta = modf(sidereal * i, &tmp);
            if(delta < delta_n) {
                delta_n = delta;
                if(i > 2000)
                    break;
                else
                    ival_dec = i;
            }
        }

        bval_dec = sidereal * ival_dec;

        IupSetInt(txt_aval_dec, "VALUE", (int)aval_dec);
        IupSetInt(txt_sval_dec, "VALUE", (int)sval_dec);
        IupSetInt(txt_bval_dec, "VALUE", (int)bval_dec);
        IupSetInt(txt_ival_dec, "VALUE", (int)ival_dec);
    }

    return IUP_DEFAULT;
}

int cb_btn_bl(Ihandle *self)
{
    char cmd_buf[60], read_buf[10];
    int8_t i;
    cmd_buf[0] = ':';
    cmd_buf[1] = 'Q';
    cmd_buf[2] = '1';
    cmd_buf[3] = '\r';

    if (i = sp_blocking_write(list[port_idx], cmd_buf, 4, 1000) != 4) {
        IupMessage("错误", "无法向EQMAX发送命令");
        return IUP_DEFAULT;
    }

    sp_close(list[port_idx]);
    open_flag = 0;
    IupSetAttribute(btn_com, "TITLE", "打开");
    IupSetAttribute(btn_update, "ACTIVE", "NO");
    IupSetAttribute(btn_upgrade, "ACTIVE", "NO");
    IupSetAttribute(btn_advc, "ACTIVE", "NO");
    IupSetAttribute(lst_com, "ACTIVE", "YES");
    IupSetAttribute(btn_refresh_com, "ACTIVE", "YES");
    IupSetAttribute(btn_bl, "ACTIVE", "NO");
    printf("Serial closed\n");

    return IUP_DEFAULT;
}

int main(int argc, char **argv)
{
    uint16_t i;

    IupOpen(&argc, &argv);

    /* set serial connection */
    btn_refresh_com = IupButton("刷新", NULL);
    IupSetCallback(btn_refresh_com, "ACTION", (Icallback)cb_btn_refresh_com);
    btn_com = IupButton("打开", NULL);
    IupSetAttribute(btn_com, "ACTIVE", "YES");
    IupSetCallback(btn_com, "ACTION", (Icallback)cb_btn_com);
    lst_com = IupList(NULL);
    IupSetAttributes(lst_com, "DROPDOWN=YES,VALUE=1");
    hbox_com = IupHbox(btn_refresh_com, btn_com, lst_com, NULL);
    IupSetAttributes(hbox_com, "ALIGNMENT=ACENTER,GAP=3,MARGIN=5x5,NORMALIZESIZE=HORIZONTAL");

    if(sp_list_ports(&list) != SP_OK) {
        fprintf(stderr, "Can't list serial ports.\n");
    } else {
        printf("List serial ports OK.\n");
        for(i = 0; list[i] != NULL; i++) {
            printf("Port[%2d]:%s.\n", i, sp_get_port_name(list[i]));
            IupSetAttributeId(lst_com, "", i + 1, sp_get_port_name(list[i]));
        }
        IupSetAttribute(lst_com, "VALUE", "1");
    }

    btn_update = IupButton("更新", NULL);
    IupSetAttribute(btn_update, "ACTIVE", "NO");
    IupSetCallback(btn_update, "ACTION", (Icallback)cb_btn_update);

    /* set ra label */
    lbl_ra = IupLabel("RA");
    lbl_dec = IupLabel("DEC");
    lbl_w = IupLabel("蜗轮蜗杆减速比:");
    lbl_m = IupLabel("电机减速比:");
    lbl_s = IupLabel("电机步距角:");
    lbl_mode = IupLabel("驱动细分系数:");
    lbl_h = IupLabel("高速倍率:");
    lbl_max = IupLabel("最大速度倍率:");

    IupSetAttribute(lbl_w, "MINSIZE", "140");
    IupSetAttribute(lbl_m, "MINSIZE", "140");
    IupSetAttribute(lbl_s, "MINSIZE", "140");
    IupSetAttribute(lbl_mode, "MINSIZE", "140");
    IupSetAttribute(lbl_h, "MINSIZE", "140");
    IupSetAttribute(lbl_max, "MINSIZE", "140");

    lbl_aval = IupLabel("轴总步数:");
    IupSetAttribute(lbl_aval, "MINSIZE", "140");

    txt_aval_ra = IupText(NULL);
    //IupSetAttribute(lbl_aval_ra, "MINSIZE", "46");
    IupSetCallback(txt_aval_ra, "ACTION", (Icallback)cb_txt_chk_num);
    IupSetCallback(txt_aval_ra, "VALUECHANGED_CB", (Icallback)cb_txt_aval_ra_change);

    txt_aval_dec = IupText(NULL);
    //IupSetAttribute(lbl_aval_dec, "MINSIZE", "46");
    IupSetCallback(txt_aval_dec, "ACTION", (Icallback)cb_txt_chk_num);
    IupSetCallback(txt_aval_dec, "VALUECHANGED_CB", (Icallback)cb_txt_aval_dec_change);

    lbl_sval = IupLabel("蜗杆总步数:");
    IupSetAttribute(lbl_sval, "MINSIZE", "140");

    txt_sval_ra = IupText(NULL);
    //IupSetAttribute(lbl_sval_ra, "MINSIZE", "46");
    IupSetCallback(txt_sval_ra, "ACTION", (Icallback)cb_txt_chk_num);
    IupSetCallback(txt_sval_ra, "VALUECHANGED_CB", (Icallback)cb_txt_sval_ra_change);

    txt_sval_dec = IupText(NULL);
    //IupSetAttribute(lbl_sval_dec, "MINSIZE", "46");
    IupSetCallback(txt_sval_dec, "ACTION", (Icallback)cb_txt_chk_num);
    IupSetCallback(txt_sval_dec, "VALUECHANGED_CB", (Icallback)cb_txt_sval_dec_change);

    lbl_bval = IupLabel("恒星速步数:");
    IupSetAttribute(lbl_bval, "MINSIZE", "140");

    txt_bval_ra = IupText(NULL);
    //IupSetAttribute(lbl_bval_ra, "MINSIZE", "46");
    IupSetCallback(txt_bval_ra, "ACTION", (Icallback)cb_txt_chk_num);

    txt_bval_dec = IupText(NULL);
    //IupSetAttribute(lbl_bval_dec, "MINSIZE", "46");
    IupSetCallback(txt_bval_dec, "ACTION", (Icallback)cb_txt_chk_num);

    lbl_ival = IupLabel("恒星速系数:");
    IupSetAttribute(lbl_ival, "MINSIZE", "140");

    txt_ival_ra = IupText(NULL);
    //IupSetAttribute(lbl_ival_ra, "MINSIZE", "46");
    IupSetCallback(txt_ival_ra, "ACTION", (Icallback)cb_txt_chk_num);

    txt_ival_dec = IupText(NULL);
    //IupSetAttribute(lbl_ival_dec, "MINSIZE", "46");
    IupSetCallback(txt_ival_dec, "ACTION", (Icallback)cb_txt_chk_num);

    /* set ra txt */
    txt_ra_w = IupText(NULL);
    txt_ra_m = IupText(NULL);
    txt_ra_s = IupText(NULL);
    txt_ra_h = IupText(NULL);
    txt_ra_max = IupText(NULL);

    IupSetCallback(txt_ra_w, "ACTION", (Icallback)cb_txt_chk_num);
    IupSetCallback(txt_ra_m, "ACTION", (Icallback)cb_txt_chk_num);
    IupSetCallback(txt_ra_s, "ACTION", (Icallback)cb_txt_chk_num);
    IupSetCallback(txt_ra_h, "ACTION", (Icallback)cb_txt_chk_num);
    IupSetCallback(txt_ra_max, "ACTION", (Icallback)cb_txt_chk_num);

    IupSetCallback(txt_ra_w, "VALUECHANGED_CB", (Icallback)cb_txt_changed_ra);
    IupSetCallback(txt_ra_m, "VALUECHANGED_CB", (Icallback)cb_txt_changed_ra);
    IupSetCallback(txt_ra_s, "VALUECHANGED_CB", (Icallback)cb_txt_changed_ra);
    //IupSetCallback(txt_ra_h, "VALUECHANGED_CB", (Icallback)cb_txt_changed_ra);
    //IupSetCallback(txt_ra_max, "VALUECHANGED_CB", (Icallback)cb_txt_changed_ra);

    /* set dec txt */
    txt_dec_w = IupText(NULL);
    txt_dec_m = IupText(NULL);
    txt_dec_s = IupText(NULL);
    txt_dec_h = IupText(NULL);
    txt_dec_max = IupText(NULL);

    IupSetCallback(txt_dec_w, "ACTION", (Icallback)cb_txt_chk_num);
    IupSetCallback(txt_dec_m, "ACTION", (Icallback)cb_txt_chk_num);
    IupSetCallback(txt_dec_s, "ACTION", (Icallback)cb_txt_chk_num);
    IupSetCallback(txt_dec_h, "ACTION", (Icallback)cb_txt_chk_num);
    IupSetCallback(txt_dec_max, "ACTION", (Icallback)cb_txt_chk_num);

    IupSetCallback(txt_dec_w, "VALUECHANGED_CB", (Icallback)cb_txt_changed_dec);
    IupSetCallback(txt_dec_m, "VALUECHANGED_CB", (Icallback)cb_txt_changed_dec);
    IupSetCallback(txt_dec_s, "VALUECHANGED_CB", (Icallback)cb_txt_changed_dec);

    lst_ra_mode = IupList(NULL);
    lst_dec_mode = IupList(NULL);
    IupSetAttributes(lst_ra_mode, "DROPDOWN=YES,SIZE=46,1=1,2=2,3=4,4=8,5=16,VALUE=1");
    IupSetAttributes(lst_dec_mode, "DROPDOWN=YES,SIZE=46,1=1,2=2,3=4,4=8,5=16,VALUE=1");
    IupSetCallback(lst_ra_mode, "VALUECHANGED_CB", (Icallback)cb_txt_changed_ra);
    IupSetCallback(lst_dec_mode, "VALUECHANGED_CB", (Icallback)cb_txt_changed_dec);

    lbl_gr = IupLabel("导星速率:");
    IupSetAttribute(lbl_gr, "MINSIZE", "140");
    lst_gr_ra = IupList(NULL);
    IupSetAttributes(lst_gr_ra, "DROPDOWN=YES,SIZE=46,1=0.1x,2=0.2x,3=0.3x,4=0.4x,5=0.5x,6=0.6x,7=0.7x,8=0.8x,9=0.9x,VALUE=5");
    lst_gr_dec = IupList(NULL);
    IupSetAttributes(lst_gr_dec, "DROPDOWN=YES,SIZE=46,1=0.1x,2=0.2x,3=0.3x,4=0.4x,5=0.5x,6=0.6x,7=0.7x,8=0.8x,9=0.9x,VALUE=5");
    btn_open = IupButton("打开", NULL);
    IupSetAttribute(btn_open, "MAXSIZE", "10x");
    IupSetAttribute(btn_open, "ACTIVE", "NO");
    IupSetCallback(btn_open, "ACTION", (Icallback)cb_btn_open);
    btn_upgrade = IupButton("升级", NULL);
    IupSetAttribute(btn_upgrade, "ACTIVE", "NO");
    IupSetCallback(btn_upgrade, "ACTION", (Icallback)cb_btn_upgrade);
    txt_path = IupText(NULL);
    IupSetAttribute(txt_path, "EXPAND", "YES");
    pb = IupProgressBar();
    IupSetAttribute(pb, "EXPAND", "YES");
    txt_sn = IupText(NULL);
    IupSetAttribute(txt_sn,"VISIBLECOLUMNS", "20");

    btn_advc = IupButton("高级", NULL);
    IupSetAttribute(btn_advc, "ACTIVE", "NO");
    IupSetCallback(btn_advc, "ACTION", (Icallback)cb_btn_advc);
    lbl_advc_ra = IupLabel("RA");
    IupSetAttributes(lbl_advc_ra, "MINSIZE=40");
    lbl_advc_dec = IupLabel("DEC");
    IupSetAttributes(lbl_advc_dec, "MINSIZE=40");
    btn_advc_ra = IupButton("开始", NULL);
    IupSetCallback(btn_advc_ra, "ACTION", (Icallback)cb_btn_advc_ra);
    btn_advc_dec = IupButton("开始", NULL);
    IupSetCallback(btn_advc_dec, "ACTION", (Icallback)cb_btn_advc_dec);
    btn_advc_ra_stop = IupButton("停止", NULL);
    IupSetCallback(btn_advc_ra_stop, "ACTION", (Icallback)cb_btn_advc_ra_stop);
    btn_advc_dec_stop = IupButton("停止", NULL);
    IupSetCallback(btn_advc_dec_stop, "ACTION", (Icallback)cb_btn_advc_dec_stop);
    btn_advc_ra_con = IupButton("继续", NULL);
    IupSetCallback(btn_advc_ra_con, "ACTION", (Icallback)cb_btn_advc_ra_con);
    btn_advc_dec_con = IupButton("继续", NULL);
    IupSetCallback(btn_advc_dec_con, "ACTION", (Icallback)cb_btn_advc_dec_con);
    txt_advc_val_ra = IupText("");
    IupSetAttributes(txt_advc_val_ra, "MINSIZE=100");
    txt_advc_val_dec = IupText("");
    IupSetAttributes(txt_advc_val_dec, "MINSIZE=100");
    btn_advc_to_aval_ra = IupButton("TO AVAL", NULL);
    IupSetCallback(btn_advc_to_aval_ra, "ACTION", (Icallback)cb_btn_advc_to_aval_ra);
    btn_advc_to_aval_dec = IupButton("TO AVAL", NULL);
    IupSetCallback(btn_advc_to_aval_dec, "ACTION", (Icallback)cb_btn_advc_to_aval_dec);
    btn_advc_to_sval_ra = IupButton("TO_SVAL", NULL);
    IupSetCallback(btn_advc_to_sval_ra, "ACTION", (Icallback)cb_btn_advc_to_sval_ra);
    btn_advc_to_sval_dec = IupButton("TO_SVAL", NULL);
    IupSetCallback(btn_advc_to_sval_dec, "ACTION", (Icallback)cb_btn_advc_to_sval_dec);
    lbl_revert = IupLabel("反向");
    IupSetAttribute(lbl_revert, "MINSIZE", "40");
    tgl_ra = IupToggle("RA", NULL);

    tgl_dec = IupToggle("DEC", NULL);
    Ihandle *frame_steps = IupFrame(
                            IupVbox(
                                IupSetAttributes(IupHbox(lbl_advc_ra, btn_advc_ra, txt_advc_val_ra, btn_advc_ra_stop, btn_advc_ra_con, btn_advc_to_aval_ra, btn_advc_to_sval_ra, NULL), "ALIGNMENT=ACENTER,NORMALIZESIZE=HORIZONTAL"),
                                IupSetAttributes(IupHbox(lbl_advc_dec, btn_advc_dec, txt_advc_val_dec, btn_advc_dec_stop, btn_advc_dec_con, btn_advc_to_aval_dec, btn_advc_to_sval_dec, NULL), "ALIGNMENT=ACENTER,NORMALIZESIZE=HORIZONTAL"),
                                NULL
                                )
            );
    IupSetAttribute(frame_steps, "TITLE", "测试步数");
    Ihandle *frame_revert = IupFrame(IupHbox(lbl_revert, tgl_ra, tgl_dec, NULL));
    IupSetAttribute(frame_revert, "TITLE", "轴反向");
    IupSetAttribute(frame_revert, "EXPAND", "YES");

    advc_dlg = IupDialog(
            IupSetAttributes(IupVbox(
                    frame_steps,
                    frame_revert,
                    NULL), "MARGIN=10x10,GAP=5,ALIGNMENT=ACENTER")
            );
    IupSetAttribute(advc_dlg, "TITLE", "高级设置");
    IupSetAttribute(advc_dlg, "RESIZE", "NO");

    btn_bl = IupButton("Bootloader", NULL);
    IupSetCallback(btn_bl, "ACTION", (Icallback)cb_btn_bl);
    IupSetAttribute(btn_bl, "ACTIVE", "NO");
    lbl_sn = IupLabel("SN: ");

    gbox_s = IupGridBox(
                 IupFill(), lbl_ra, lbl_dec,
                 lbl_w, txt_ra_w, txt_dec_w,
                 lbl_m, txt_ra_m, txt_dec_m,
                 lbl_s, txt_ra_s, txt_dec_s,
                 lbl_h, txt_ra_h, txt_dec_h,
                 lbl_max, txt_ra_max, txt_dec_max,
                 lbl_mode, lst_ra_mode, lst_dec_mode,
                 lbl_gr, lst_gr_ra, lst_gr_dec,
                 lbl_aval, txt_aval_ra, txt_aval_dec,
                 lbl_sval, txt_sval_ra, txt_sval_dec,
                 lbl_bval, txt_bval_ra, txt_bval_dec,
                 lbl_ival, txt_ival_ra, txt_ival_dec,
                 btn_bl, btn_advc, btn_update,
                 NULL);
    IupSetAttributes(gbox_s, "ALIGNMENTLIN=ACENTER,ALIGNMENTCOL=ACENTER,NUMDIV=3, SIZECOL=2,SIZELIN=3,MARGIN=10x10,GAPLIN=5,GAPCOL=5");

    frame_s = IupFrame(gbox_s);
    frame_u = IupFrame(
                  IupSetAttributes(IupVbox(
                                       IupSetAttributes(IupHbox(lbl_sn, txt_sn, NULL),"ALIGNMENT=ACENTER"),
                                       IupSetAttributes(IupHbox(btn_open, txt_path, btn_upgrade, NULL),"ALIGNMENT=ACENTER,GAP=3,NORMALIZESIZE=HORIZONTAL"),
                                       pb,
                                       NULL), "MARGIN=5x5")
              );

    IupSetAttribute(frame_s, "TITLE", "参数设置");
    IupSetAttribute(frame_u, "TITLE", "固件升级");

    vbox_main = IupVbox(hbox_com, frame_s, frame_u, NULL);
    IupSetAttribute(vbox_main, "MARGIN", "10x10");
    IupSetAttribute(vbox_main, "GAP", "10");

    dlg = IupDialog(vbox_main);
    IupSetAttributes(dlg, "TITLE=\"EQMAX Utils\",RESIZE=NO");

    dlg_open = IupFileDlg();
    IupSetAttribute(dlg_open, "DIALOGTYPE", "打开");
    IupSetAttribute(dlg_open, "EXTFILTER", "Firmware|*.bin|");
    IupSetAttribute(dlg_open, "TITLE", "打开固件");
    IupSetCallback(dlg_open, "FILE_CB", (Icallback)cb_open_ok);


    IupShowXY(dlg, IUP_CENTER, IUP_CENTER);
    IupMainLoop();

    IupClose();
    sp_free_port_list(list);
    sp_free_config(cfg);
    return EXIT_SUCCESS;
}



