/*
 * $Id$
 *
 * Copyright (C) 2003-2006  Haiyang Chao
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** \file osam_imu_ugear.c
 *  \brief Communication with any IMU through serial communication (originally gps uart)
 * This file is first generated by Haiyang Chao on 20080507 for communication.
 */

#include <stdlib.h>
#include <string.h>

#include "ins_osam_ugear.h"
#include "gps.h"
#include "gps_ubx.h"
#include "latlong.h"
#include "sys_time.h"
#include "airframe.h"
#include "subsystems/nav.h"
#include "estimator.h"
#include "xsens_protocol.h"

#define UNINIT      0
#define GOT_SYNC1   1
#define GOT_SYNC2   2
#define GOT_ID          3
#define GOT_LEN         4
#define GOT_PAYLOAD     5
#define GOT_CHECKSUM1   6
#define GOT_CHECKSUM2   7

#define UGEAR_SYNC1 0x93
#define UGEAR_SYNC2 0xE0
/*#define UGEAR_MAX_PAYLOAD 24*/
#define UGEAR_MAX_PAYLOAD 40
#define IMU_PACKET_SIZE 12
#define RAD2DEG 57.3
#define WrapUp(x) (x < 0 ? x+3600 : x)

/* from main_ap.c */
extern bool_t launch;

/* variable defined for dlsetting generated by Haiyang 20080717*/
bool_t imu1_ir0 = 0;
float fd_alpha = 0;
float fi_alpha = 0;
float ins_roll_neutral;
float ins_pitch_neutral;

/* variable definition copied from gps_ubx.c 20080508*/
uint16_t gps_week;
uint32_t gps_itow;
int32_t gps_alt;
uint16_t gps_gspeed;
int16_t gps_climb;
int16_t gps_course;
int32_t gps_utm_east, gps_utm_north;
uint8_t gps_utm_zone;
uint8_t gps_mode;
volatile bool_t gps_msg_received;
bool_t gps_pos_available;
uint8_t ugear_id, ugear_class;
int32_t gps_lat, gps_lon;
uint16_t gps_reset;

uint16_t gps_PDOP;
uint32_t gps_Pacc, gps_Sacc;
uint8_t gps_numSV;
/* variable definition copied from gps_ubx.c20080508*/

/* variable definition copied from gps.c20080515*/
uint8_t gps_nb_ovrn;
struct svinfo gps_svinfos[GPS_NB_CHANNELS];
uint8_t gps_nb_channels;

/* variable definition copied from gps.c20080515*/

//uint16_t last_gps_msg_t;	/** cputime of the last gps message */

volatile bool_t ugear_msg_received;

static uint8_t  ugear_status;
static uint8_t  ugear_type;
static uint8_t  ugear_len;
static uint8_t  ugear_msg_idx;
static uint8_t ck_a, ck_b;

int16_t ugear_phi;
int16_t ugear_psi;
int16_t ugear_theta;

int16_t gps_ve;
int16_t gps_vn;
int16_t gps_vd;
/* added 20080522 for debugging*/
int16_t ugear_debug1;
int16_t ugear_debug2;
int16_t ugear_debug3;
int32_t ugear_debug4;
int32_t ugear_debug5;
int32_t ugear_debug6;

//bool_t gps_verbose_downlink;

uint8_t ugear_msg_buf[UGEAR_MAX_PAYLOAD] __attribute__ ((aligned));

/*The following definition is for ugear data 20080509, modified on 0512 Haiyang*/
struct imu {
   int32_t phi,the,psi;          /* attitudes             */
};

struct gps {
   int32_t lon,lat,alt;
   int16_t ve,vn,vd;
};

struct imu imupacket;
struct gps gpspacket;
/*The above definition is for ugear data 20080509 Haiyang*/

void ugear_init( void ) {
  ugear_status = UNINIT;
  ugear_phi = 0;
  ugear_psi = 0;
  ugear_theta = 0;
  ugear_debug2 = 0;
  ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
  ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
}

void parse_ugear( uint8_t c ) {
  /*checksum go first*/
  if (ugear_status < GOT_PAYLOAD) {
    ck_a += c;
    ck_b += ck_a;
  }
  switch (ugear_status) {
  case UNINIT:
    if (c == UGEAR_SYNC1)
      ugear_status++;
    break;
  case GOT_SYNC1:
    if (c != UGEAR_SYNC2)
      goto error;
    ck_a = 0;
    ck_b = 0;
    ugear_status++;
    break;
  case GOT_SYNC2:
    if (ugear_msg_received) {
      /* Previous message has not yet been parsed: discard this one */
      goto error;
    }
    ugear_type = c;
    ugear_status++;
    //ugear_theta = 30; // for debug
    if (ugear_type > 2)
    goto restart;
    break;
  case GOT_ID:
    ugear_len = c;
    ugear_msg_idx = 0;
    ugear_status++;
    break;
  case GOT_LEN:
    ugear_msg_buf[ugear_msg_idx] = c;
    ugear_msg_idx++;
    if (ugear_msg_idx >= ugear_len) {
      ugear_status++;
    }
    break;
  case GOT_PAYLOAD:
    if (c != ck_a)
      goto error;
    ugear_status++;
    break;
  case GOT_CHECKSUM1:
    if (c != ck_b)
      goto error;
    ugear_msg_received = TRUE;
    UgearToggleLed();
    goto restart;
    break;
  }
  return;
error:
restart:
  ugear_status = UNINIT;
  return;
}
/*
void decode_imupacket( struct imu *data, uint8_t* buffer){

    data->phi = (double)((((signed char)buffer[25])<<8)|buffer[26]);
    data->theta = (double)((((signed char)buffer[27])<<8)|buffer[28]);
    data->psi = (double)((((signed char)buffer[29])<<8)|buffer[30]);

}
*/
void parse_ugear_msg( void ){

    float ins_phi, ins_psi, ins_theta;

    switch (ugear_type){
        case 0:  /*gps*/
            ugear_debug1 = ugear_debug1+1;
            gps_lat = UGEAR_NAV_POSLLH_LAT(ugear_msg_buf);
            gps_lon = UGEAR_NAV_POSLLH_LON(ugear_msg_buf);

            nav_utm_zone0 = (gps_lon/10000000+180) / 6 + 1;
            latlong_utm_of(RadOfDeg(gps_lat/1e7), RadOfDeg(gps_lon/1e7), nav_utm_zone0);
            gps_utm_east = latlong_utm_x * 100;
            gps_utm_north = latlong_utm_y * 100;

            gps_alt = UGEAR_NAV_POSLLH_HEIGHT(ugear_msg_buf);
            gps_utm_zone = nav_utm_zone0;

            gps_gspeed = UGEAR_NAV_VELNED_GSpeed(ugear_msg_buf);
                gps_climb = - UGEAR_NAV_POSLLH_VD(ugear_msg_buf);
                gps_course = UGEAR_NAV_VELNED_Heading(ugear_msg_buf)/10000; /*in decdegree */
                gps_PDOP = UGEAR_NAV_SOL_PDOP(ugear_msg_buf);
                gps_Pacc = UGEAR_NAV_SOL_Pacc(ugear_msg_buf);
                gps_Sacc = UGEAR_NAV_SOL_Sacc(ugear_msg_buf);
                gps_numSV = UGEAR_NAV_SOL_numSV(ugear_msg_buf);
            gps_week = 0; // FIXME
            gps_itow = UGEAR_NAV_VELNED_ITOW(ugear_msg_buf);

            //ugear_debug2 = gps_climb;
            //ugear_debug4 = (int32_t)(UGEAR_NAV_VELNED_GSpeed(ugear_msg_buf));
            //ugear_debug5 = UGEAR_NAV_VELNED_GSpeed(ugear_msg_buf);
            //ugear_debug6 = (int16_t)estimator_phi*100;

            gps_mode = 3;  /*force GPSfix to be valided*/
            gps_pos_available = TRUE; /* The 3 UBX messages are sent in one rafale */
            break;
        case 1:  /*IMU*/
            ugear_debug2 = ugear_debug2+1;
            ugear_phi = UGEAR_IMU_PHI(ugear_msg_buf);
            ugear_psi = UGEAR_IMU_PSI(ugear_msg_buf);
            ugear_theta = UGEAR_IMU_THE(ugear_msg_buf);
            ugear_debug4 = (int32_t)ugear_phi;
            ugear_debug5 = (int32_t)ugear_theta;
            ugear_debug6 = (int32_t)ugear_psi;
            ugear_debug3 = 333;
            ins_phi  = (float)ugear_phi/10000 - ins_roll_neutral;
            ins_psi = 0;
            ins_theta  = (float)ugear_theta/10000 - ins_pitch_neutral;
#ifndef INFRARED
            EstimatorSetAtt(ins_phi, ins_psi, ins_theta);
#endif
            break;
        case 2:  /*GPS status*/
//			ugear_debug1 = 2;
                gps_nb_channels = XSENS_GPSStatus_nch(ugear_msg_buf);
            uint8_t is;
            for(is = 0; is < Min(gps_nb_channels, 16); is++) {
                uint8_t ch = XSENS_GPSStatus_chn(ugear_msg_buf,is);
                    if (ch > 16) continue;
                    gps_svinfos[ch].svid = XSENS_GPSStatus_svid(ugear_msg_buf, is);
                    gps_svinfos[ch].flags = XSENS_GPSStatus_bitmask(ugear_msg_buf, is);
                    gps_svinfos[ch].qi = XSENS_GPSStatus_qi(ugear_msg_buf, is);
                    gps_svinfos[ch].cno = XSENS_GPSStatus_cnr(ugear_msg_buf, is);
                    gps_svinfos[ch].elev = 0;
                    gps_svinfos[ch].azim = 0;
            }
            break;
        case 3:  /*servo*/
            break;
        case 4:  /*health*/
            break;

    }

}

/* add the following function only to get rid of compilation error in datalink.c 20080608 Haiyang*/
void ubxsend_cfg_rst(uint16_t bbr , uint8_t reset_mode) {
}

void ugear_event( void ) {
    if (UgearBuffer()){
        ReadUgearBuffer();
    }
    if (ugear_msg_received){
        parse_ugear_msg();
        ugear_msg_received = FALSE;
        if (gps_pos_available){
            //gps_downlink();
            gps_verbose_downlink = !launch;
            UseGpsPosNoSend(estimator_update_state_gps);
            gps_msg_received_counter = gps_msg_received_counter+1;
            #ifdef GX2
            if (gps_msg_received_counter == 1){
                gps_send();
                gps_msg_received_counter = 0;
            }
            #endif
            #ifdef XSENSDL
            if (gps_msg_received_counter == 25){
                gps_send();
                gps_msg_received_counter = 0;
            }
            #endif
            gps_pos_available = FALSE;
        }
    }
}
