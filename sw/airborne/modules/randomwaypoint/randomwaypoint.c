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

#include "modules/randomwaypoint/randomwaypoint.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#include <stdio.h>
#include <stdlib.h>

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
    // WP_RNDM is the name defined in the flightplan.xml (which is then compiled to a header file flight_plan.h in ./var/aircraft/AIRCRAFT_NAME/ap/generated)
    //meters=srand()%8;
    //waypoints[WP_RNDM].x = meters * 256; // integer position: unit are meters devided by 2^8 = (1<<POS_FRAC) = 256
    //waypoints[WP_RNDM].y = meters * 256;
    //waypoints[WP_RNDM].z = -2; 
    waypoints[WP_RNDM].x = metersx * 256; // integer position: unit are meters devided by 2^8 = (1<<POS_FRAC) = 256
    waypoints[WP_RNDM].y = metersy * 256;
    //waypoints[WP_RNDM].z = 0; 
    printf("mock_alspaalinmiddenzit %d\n",metersx);
  }
  mysdf=mysdf+1;
  return 0;
}

int alspaalinmiddenzit() 
{
  //The real stuff, still todo
  return 0;
}



