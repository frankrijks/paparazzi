/*
 * Copyright (C) Frank
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/randomwaypoint/randomwaypoint.c"
 * @author Frank
 * Creating and moving a random waypoint
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include "modules/randomwaypoint/randomwaypoint.h"
#include "modules/computer_vision/cv/resize.h"
#include "modules/computer_vision/lib/v4l/video.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"

int mysdf = 0;
int metersx = 4;
int metersy = 4;

int mock_alspaalinmiddenzit() 
{
  time_t t;

  srand((unsigned) time(&t));
  
  if ((mysdf % 1000) == 0)
  {
    mysdf=0;
    metersx = 2-rand()%4;
    metersy = 2-rand()%4;

    waypoints[WP_RNDM].x = metersx * 256; // integer position: unit are meters devided by 2^8 = (1<<POS_FRAC) = 256
    waypoints[WP_RNDM].y = metersy * 256;

    printf("mock_alspaalinmiddenzit %d\n",metersx);
  }
  mysdf=mysdf+1;
  return 0;
}


    // WP_RNDM is the name defined in the flightplan.xml (which is then compiled to a header file flight_plan.h in ./var/aircraft/AIRCRAFT_NAME/ap/generated)
    //meters=srand()%8;
    //waypoints[WP_RNDM].x = meters * 256; // integer position: unit are meters devided by 2^8 = (1<<POS_FRAC) = 256
    //waypoints[WP_RNDM].y = meters * 256;
    //waypoints[WP_RNDM].z = -2; 
    //waypoints[WP_RNDM].z = 0; 


//int alspaalinmiddenzit() 
//{
//
// int DOWNSIZE_FACTOR = 4; 
// volatile uint8_t computervision_thread_status = 0;
//
//   //The real stuff, still todo
//  
//  //Video input
//  struct vid_struct vid;
//        vid.device = (char*)"/dev/video2";
//        vid.w=1280;
//        vid.h=720;
//        vid.n_buffers=0;
//  if (video_init(&vid)<0) {
//    printf("Error initialising video\n");
//    computervision_thread_status = -1;
//    return 0;
//  }
  
//  // Pull image from video feed
//  struct img_struct* img_new = video_create_image(&vid);
// 
//  // Resize image
//  struct img_struct small;
//  small.w = vid.w / DOWNSIZE_FACTOR;
//  small.h = vid.h / DOWNSIZE_FACTOR;
//  small.buf = (uint8_t*)malloc(small.w*small.h*2);
//  
//  resize_uyuv(img_new, &small, DOWNSIZE_FACTOR);
//        
//  return 0;
//}



