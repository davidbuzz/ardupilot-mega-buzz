Hi, and thanks for your interest!

First, if you like....  please mail me at:   davidbuzz@gmail.com  I'd love to know youare using this! 

This is what I've done.  the code is still "raw" ( ie an ugly
hack), but it works, and I'll be improving it over the next 6 months
as we go...

I've written a very basic vector interface ( see vector.h/vector.c)
that is based on 'long' values instead of floats ( which are
unnecessarily slow), and I'm doing the internal calculation/s in the
ArduTerminator using the internal  latitude*10^6  and  longitude*10^6
values.  :-)

The Terminator basically has two "Modes" that I re-used/stole from
ArduPlane:  'AUTO'  , and 'RTL'.   ( AUTO means "active-pass-thru, and
ready to terminate", RTL means "Terminated"   ( Really Tough Landing ,
or other backronym.? )

My workflow currently is:
* connect to terminator via USB
* connect a stock copy of 'APM Planner' to Terminator via MavLink.  (
see sysid 2 )
* upload a set of "WAYPOINTS" representing the boundary
* 'home' WP is excluded from the boundary calculation, and is
considered the default/safe virtual starting point of the plane until
the GPS lock is achieved, so I put it inside the rest of the points.
* wait for GPS lock, and look at APM Planner 'bearing' lines ( two of
them are constantly re-pointed to the nearest two waypoints, as found
by the Terminator.)
* test termination mode by clicking 'RTL' button in APM Planner, or by
linking a ground-based RC channel direct to Terminator input 8, and
configuring Switch/s as per usual.   All modes are internally 'AUTO'
except RTL.   :-)
* disconnect, and go to next pre-flight step ( whatever that is)

My planned TODOs:
* allow sharing GPS data between primary navigation system and
termination system, as we'll have 2 GPS units, we might as well
reconcile the best data from them ( based on number of  sats ) .

* fix the boundary system so that the boundary width is "infinite"  (
ie if you run the terminator outside the defined boundary, you should
immediately terminate )   Currently, the boundary is of specific ( and
adjustable ) "fatness" , and the plane only terminates if it passes
"through" this  "fat boundary".

* code cleanups  - I have basically taken ArduPlane, and commented-out
90% of it.   All these commented blocks are still in-line ( as
recerence for me to remind me what was there before), but should
really be removed, and minimised.    Proof of Concepts are like that.:1
:1
