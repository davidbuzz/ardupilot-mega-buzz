// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
static void navigate()
{
	// do not navigate with corrupt data
	// ---------------------------------
	if (g_gps->fix == 0)
	{
		g_gps->new_data = false;
                //TODO - restore this BUZZ, 
		//return;
            // force home to pretend to be the current position/ waypoint when we dont have GPS lock.... for testing, and reasonable power-on behaviour if powered on outside a boundary. 
            current_loc = get_cmd_with_index(0);
	}

        if ( control_mode == TERMINATE  || control_mode == RTL ) { 
              return; // its too late, we can no longer navigate!  
        }
        
        // locate nearest waypoint: wp_nearest, and how far away it is:  wp_nearest_distance
        wp_nearest = 0; 
        wp_nearest_distance = 99999999; // definitely not near to here! 
        for ( int p =1 ; p <= g.command_total ;p++ ) { 
          WP_A =  get_cmd_with_index(p);
          long d = get_distance(  &current_loc, &WP_A ) ;
          
          if ( d < wp_nearest_distance ) { 
            wp_nearest = p;
            wp_nearest_distance = d;
          }
        }
        
        // tell GCS an approximaton ... ie the distance to the nearest point 
        wp_distance = wp_nearest_distance; 

        //  WAYPOINT 'A' is the "nearest one"   :-)      
         WP_A =  get_cmd_with_index(wp_nearest);
 
       // get WP either side of the nearest one! 
       
        // B and C are either side of A.  B to the left &  C to the right ( looking from inside the perimeter/circle ) 
        if (wp_nearest == 1 ) { 
          WP_B =  get_cmd_with_index(g.command_total);  // last 
        } else { 
          WP_B =  get_cmd_with_index(wp_nearest-1); 
        }
         // B and C are either side of A, B to the left &  C to the right ( from inside  ) 
        if (wp_nearest == g.command_total ) { 
          WP_C =  get_cmd_with_index(1);  // last 
        } else { 
          WP_C =  get_cmd_with_index(wp_nearest+1); 
        }
        
         #ifdef HIL_MODE ==  HIL_MODE_ATTITUDE
         fprintf(stdout," WPS: %i %i  |  %i %i |  %i  %i \n", WP_A.lat, WP_A.lng, WP_B.lat,WP_B.lng, WP_C.lat,WP_C.lng); 
        #endif
        
       // see: http://www.topcoder.com/tc?d1=tutorials&d2=geometry1&module=Static
       
       LongPoint A;
       A.x = (unsigned long)WP_A.lat;// /1000000.0;
        A.y = (unsigned long)WP_A.lng;///1000000.0;
 
       LongPoint B;
       B.x = (unsigned long)WP_B.lat;///1000000.0;
        B.y = (unsigned long)WP_B.lng;///1000000.0;
 
       LongPoint C;
       C.x = (unsigned long)WP_C.lat;///1000000.0;
        C.y = (unsigned long)WP_C.lng;///1000000.0;
 
       LongPoint POSN;
       POSN.x = (unsigned long)current_loc.lat;///1000000.0; 
        POSN.y = (unsigned long)current_loc.lng;///1000000.0;
      
      #ifdef HIL_MODE ==  HIL_MODE_ATTITUDE

               fprintf(stdout,"LONGS:  B: %lu %lu | A: %lu %lu | C: %lu %lu \n", B.x,B.y,A.x,A.y,C.x,C.y  ) ;
      #endif    
       
       // LINE A-B, POINT POSN:
       unsigned long xx = linePointDist( A, B, POSN ) ; 
       // LINE A-C, POINT POSN:
       unsigned long yy = linePointDist( A, C, POSN) ; 
       
          #ifdef HIL_MODE ==  HIL_MODE_ATTITUDE
             fprintf(stdout,"%d %d | %lu %lu \n", wp_nearest, wp_nearest_distance, xx, yy  ) ;
          #endif
          
          // update the nav_bearing, so the GCS gets a line pointinghte right way! 
            nav_bearing = get_bearing( &current_loc , &WP_A  ) ; 
            
            // update the target bearing ot the next waypoint away that we were working with.... 
            if ( xx < 400 ) { 
            target_bearing = get_bearing(  &current_loc, &WP_B ) ; 
            } else { 
            target_bearing = get_bearing( &current_loc, &WP_C  ) ;               
            }
     
          //200 = approx 3.5-4 meters in real testing
          if ( xx < 400 | yy < 400 ) {   
          #ifdef HIL_MODE ==  HIL_MODE_ATTITUDE

              fprintf(stdout,"\n\n\n TERMINATED!! ... within set DISTANCE\n\n\n"  ) ; 
                    
            fprintf(stdout,"crossed nearest: %d distance from: %d |  posn: %i %i |  boundary lines: %i %i -> %i %i -> %i %i | %lu %lu \n",wp_nearest, wp_nearest_distance, current_loc.lat, current_loc.lng, WP_B.lat, WP_B.lng, WP_A.lat, WP_A.lng, WP_C.lat,WP_C.lng, xx , yy ); 
          #endif

            
            //control_mode = RTL;  // OR TERMINATE 
            // set_mode(RTL);  - done in terminate() function! 
            
            terminate();
          }

}


void terminate() {   
  
// put us in the mode so that we  stop passing in-chans to out channs 
set_mode(RTL);  // or TERMINATE 
   
 // terminate_servos( );  // in Attitude.pde 
}



static void calc_airspeed_errors()
{
 
}

static void calc_bearing_error()
{

//		bearing_error = nav_bearing - g_gps->ground_course;

//	bearing_error = wrap_180(bearing_error);
}

static void calc_altitude_error()
{

	altitude_error 	= target_altitude - current_loc.alt;
}

static long wrap_360(long error)
{
	if (error > 36000)	error -= 36000;
	if (error < 0)		error += 36000;
	return error;
}

static long wrap_180(long error)
{
	if (error > 18000)	error -= 36000;
	if (error < -18000)	error += 36000;
	return error;
}

static void update_loiter()
{

}

static void update_crosstrack(void)
{

}

static void reset_crosstrack()
{
	//crosstrack_bearing 	= get_bearing(&prev_WP, &next_WP);	// Used for track following
}

static long get_distance(struct Location *loc1, struct Location *loc2)
{
	if(loc1->lat == 0 || loc1->lng == 0)
		return -1;
	if(loc2->lat == 0 || loc2->lng == 0)
		return -1;
	float dlat 		= (float)(loc2->lat - loc1->lat);
	float dlong		= ((float)(loc2->lng - loc1->lng)) * scaleLongDown;
	return sqrt(sq(dlat) + sq(dlong)) * .01113195;
}

static long get_bearing(struct Location *loc1, struct Location *loc2)
{
	long off_x = loc2->lng - loc1->lng;
	long off_y = (loc2->lat - loc1->lat) * scaleLongUp;
	long bearing =	9000 + atan2(-off_y, off_x) * 5729.57795;
	if (bearing < 0) bearing += 36000;
	return bearing;
}
