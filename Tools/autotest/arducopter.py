# fly ArduCopter in SIL

import util, pexpect, sys, time, math, shutil, os

# get location of scripts
testdir=os.path.dirname(os.path.realpath(__file__))

sys.path.insert(0, util.reltopdir('../pymavlink'))
import mavutil, mavwp

HOME_LOCATION='-35.362938,149.165085,584,270'

homeloc = None
num_wp = 0

# a list of pexpect objects to read while waiting for
# messages. This keeps the output to stdout flowing
expect_list = []

def message_hook(mav, msg):
    '''called as each mavlink msg is received'''
    global expect_list
    #if msg.get_type() in [ 'NAV_CONTROLLER_OUTPUT', 'GPS_RAW' ]:
    #    print(msg)
    for p in expect_list:
        try:
            p.read_nonblocking(100, timeout=0)
        except pexpect.TIMEOUT:
            pass

def expect_callback(e):
    '''called when waiting for a expect pattern'''
    global expect_list
    for p in expect_list:
        if p == e:
            continue
        try:
            while p.read_nonblocking(100, timeout=0):
                pass
        except pexpect.TIMEOUT:
            pass

class location(object):
    '''represent a GPS coordinate'''
    def __init__(self, lat, lng, alt=0):
        self.lat = lat
        self.lng = lng
        self.alt = alt

def get_distance(loc1, loc2):
    '''get ground distance between two locations'''
    dlat 		= loc2.lat - loc1.lat
    dlong		= loc2.lng - loc1.lng
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_bearing(loc1, loc2):
    '''get bearing from loc1 to loc2'''
    off_x = loc2.lng - loc1.lng
    off_y = loc2.lat - loc1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;

def current_location(mav):
    '''return current location'''
    # ensure we have a position
    mav.recv_match(type='VFR_HUD', blocking=True)
    mav.recv_match(type='GPS_RAW', blocking=True)
    return location(mav.messages['GPS_RAW'].lat,
                    mav.messages['GPS_RAW'].lon,
                    mav.messages['VFR_HUD'].alt)

def wait_altitude(mav, alt_min, alt_max, timeout=60):
    '''wait for a given altitude range'''
    tstart = time.time()
    print("Waiting for altitude between %u and %u" % (alt_min, alt_max))
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        print("Altitude %u" % m.alt)
        if m.alt >= alt_min and m.alt <= alt_max:
            return True
    print("Failed to attain altitude range")
    return False

def wait_waypoint(mav, wpnum_start, wpnum_end, allow_skip=True, timeout=60):
    '''wait for waypoint ranges'''
    tstart = time.time()
    m = mav.recv_match(type='WAYPOINT_CURRENT', blocking=True)

    start_wp = m.seq
    current_wp = start_wp

    print("\n***wait for waypoint ranges***\n\n\n")
    if start_wp != wpnum_start:
        print("Expected start waypoint %u but got %u" % (wpnum_start, start_wp))
        return False

    while time.time() < tstart + timeout:
        m = mav.recv_match(type='WAYPOINT_CURRENT', blocking=True)
        print("WP %u" % m.seq)
        if m.seq == current_wp:
            continue
        if m.seq == current_wp+1 or (m.seq > current_wp+1 and allow_skip):
            print("Starting new waypoint %u" % m.seq)
            tstart = time.time()
            current_wp = m.seq
            if current_wp == wpnum_end:
                print("Reached final waypoint %u" % m.seq)
                return True
        if m.seq > current_wp+1:
            print("Skipped waypoint! Got wp %u expected %u" % (m.seq, current_wp+1))
            return False
    print("Timed out waiting for waypoint %u" % wpnum_end)
    return False

def save_wp(mavproxy, mav):
    mavproxy.send('rc 7 2000\n')
    mav.recv_match(condition='RC_CHANNELS_RAW.chan7_raw==2000', blocking=True)
    mavproxy.send('rc 7 1000\n')
    mav.recv_match(condition='RC_CHANNELS_RAW.chan7_raw==1000', blocking=True)
    mavproxy.send('wp list\n')


def arm_motors(mavproxy):
    '''arm motors'''
    print("Arming motors")
    mavproxy.send('switch 6\n') # stabilize mode
    mavproxy.expect('STABILIZE>')
    mavproxy.send('rc 3 1000\n')
    mavproxy.send('rc 4 2000\n')
    mavproxy.expect('APM: ARMING MOTORS')
    mavproxy.send('rc 4 1500\n')
    print("MOTORS ARMED OK")
    return True

def disarm_motors(mavproxy):
    '''disarm motors'''
    print("Disarming motors")
    mavproxy.send('switch 6\n') # stabilize mode
    mavproxy.send('rc 3 1000\n')
    mavproxy.send('rc 4 1000\n')
    mavproxy.expect('APM: DISARMING MOTORS')
    mavproxy.send('rc 4 1500\n')
    print("MOTORS DISARMED OK")
    return True


def takeoff(mavproxy, mav):
    '''takeoff get to 30m altitude'''
    mavproxy.send('switch 6\n') # stabilize mode
    mavproxy.expect('STABILIZE>')
    mavproxy.send('rc 3 1500\n')
    wait_altitude(mav, 30, 40)
    print("TAKEOFF COMPLETE")
    return True


def loiter(mavproxy, mav, maxaltchange=10, holdtime=10, timeout=60):
    '''hold loiter position'''
    mavproxy.send('switch 2\n') # loiter mode
    mavproxy.expect('LOITER>')
    mavproxy.send('status\n')
    mavproxy.expect('>')
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    start_altitude = m.alt
    tstart = time.time()
    tholdstart = time.time()
    print("Holding loiter at %u meters for %u seconds" % (start_altitude, holdtime))
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        print("Altitude %u" % m.alt)
        if math.fabs(m.alt - start_altitude) > maxaltchange:
            tholdstart = time.time()
        if time.time() - tholdstart > holdtime:
            print("Loiter OK for %u seconds" % holdtime)
            return True
    print("Loiter FAILED")
    return False


def wait_heading(mav, heading, accuracy=5, timeout=30):
    '''wait for a given heading'''
    tstart = time.time()
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        print("Heading %u" % m.heading)
        if math.fabs(m.heading - heading) <= accuracy:
            return True
    print("Failed to attain heading %u" % heading)
    return False


def wait_distance(mav, distance, accuracy=5, timeout=30):
    '''wait for flight of a given distance'''
    tstart = time.time()
    start = current_location(mav)
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='GPS_RAW', blocking=True)
        pos = current_location(mav)
        delta = get_distance(start, pos)
        print("Distance %.2f meters" % delta)
        if math.fabs(delta - distance) <= accuracy:
            return True
    print("Failed to attain distance %u" % distance)
    return False


def wait_location(mav, loc, accuracy=5, timeout=30, target_altitude=None, height_accuracy=-1):
    '''wait for arrival at a location'''
    tstart = time.time()
    if target_altitude is None:
        target_altitude = loc.alt
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='GPS_RAW', blocking=True)
        pos = current_location(mav)
        delta = get_distance(loc, pos)
        print("Distance %.2f meters" % delta)
        if delta <= accuracy:
            if height_accuracy != -1 and math.fabs(pos.alt - target_altitude) > height_accuracy:
                continue
            print("Reached location (%.2f meters)" % delta)
            return True
    print("Failed to attain location")
    return False


def fly_square(mavproxy, mav, side=50, timeout=120):
    '''fly a square, flying N then E'''
    mavproxy.send('switch 6\n')
    mavproxy.expect('STABILIZE>')
    tstart = time.time()
    failed = False

    print("Save WP 1")
    save_wp(mavproxy, mav)

    print("turn")
    mavproxy.send('rc 3 1430\n')
    mavproxy.send('rc 4 1610\n')
    if not wait_heading(mav, 0):
        return False
    mavproxy.send('rc 4 1500\n')

    print("Going north %u meters" % side)
    mavproxy.send('rc 2 1390\n')
    if not wait_distance(mav, side):
        failed = True
    mavproxy.send('rc 2 1500\n')

    print("Save WP 2")
    save_wp(mavproxy, mav)

    print("Going east %u meters" % side)
    mavproxy.send('rc 1 1610\n')
    if not wait_distance(mav, side):
        failed = True
    mavproxy.send('rc 1 1500\n')

    print("Save WP 3")
    save_wp(mavproxy, mav)

    print("Going south %u meters" % side)
    mavproxy.send('rc 2 1610\n')
    if not wait_distance(mav, side):
        failed = True
    mavproxy.send('rc 2 1500\n')
    mav.recv_match(condition='RC_CHANNELS_RAW.chan7_raw==1000', blocking=True)

    print("Save WP 4")
    save_wp(mavproxy, mav)

    print("Going west %u meters" % side)
    mavproxy.send('rc 1 1390\n')
    if not wait_distance(mav, side):
        failed = True
    mavproxy.send('rc 1 1500\n')

    print("Save WP 5")
    save_wp(mavproxy, mav)

    return not failed




def land(mavproxy, mav, timeout=60):
    '''land the quad'''
    print("STARTING LANDING")
    mavproxy.send('switch 6\n')
    mavproxy.expect('STABILIZE>')
    mavproxy.send('status\n')
    mavproxy.expect('>')

    # start by dropping throttle till we have lost 5m
    mavproxy.send('rc 3 1380\n')
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    wait_altitude(mav, 0, m.alt-5)

    # now let it settle gently
    mavproxy.send('rc 3 1400\n')
    tstart = time.time()

    if wait_altitude(mav, -5, 0):
        print("LANDING OK")
        return True
    else:
        print("LANDING FAILED")
        return False

def circle(mavproxy, mav, maxaltchange=10, holdtime=90, timeout=35):
    '''fly circle'''
    print("FLY CIRCLE")
    mavproxy.send('switch 1\n') # CIRCLE mode
    mavproxy.expect('CIRCLE>')
    mavproxy.send('status\n')
    mavproxy.expect('>')
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    start_altitude = m.alt
    tstart = time.time()
    tholdstart = time.time()
    print("Circle at %u meters for %u seconds" % (start_altitude, holdtime))
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        print("heading %u" % m.heading)

    print("CIRCLE OK for %u seconds" % holdtime)
    return True


def fly_mission(mavproxy, mav, height_accuracy=-1, target_altitude=None):
    '''fly a mission from a file'''
    print("Fly a mission")
    global homeloc
    global num_wp
    mavproxy.send('switch 4\n') # auto mode
    mavproxy.expect('AUTO>')

    wait_altitude(mav, 30, 40)
    if wait_waypoint(mav, 1, num_wp):
        print("MISSION COMPLETE")
        return True
    else:
        return False

    #if not wait_distance(mav, 30, timeout=120):
    #    return False
    #if not wait_location(mav, homeloc, timeout=600, target_altitude=target_altitude, height_accuracy=height_accuracy):
    #    return False

def load_mission(mavproxy, mav, filename):
    '''load a mission from a file'''
    global num_wp
    mavproxy.send('wp load %s\n' % filename)
    mavproxy.expect('flight plan received')
    mavproxy.send('wp list\n')
    mavproxy.expect('Requesting [0-9]+ waypoints')

    wploader = mavwp.MAVWPLoader()
    wploader.load(filename)
    num_wp = wploader.count()
    print("loaded mission")
    for i in range(num_wp):
        print (dir(wploader.wp(i)))

def setup_rc(mavproxy):
    '''setup RC override control'''
    for chan in range(1,9):
        mavproxy.send('rc %u 1500\n' % chan)
    # zero throttle
    mavproxy.send('rc 3 1000\n')


def fly_ArduCopter(viewerip=None):
    '''fly ArduCopter in SIL

    you can pass viewerip as an IP address to optionally send fg and
    mavproxy packets too for local viewing of the flight in real time
    '''
    global expect_list, homeloc

    sil = util.start_SIL('ArduCopter', wipe=True)
    mavproxy = util.start_MAVProxy_SIL('ArduCopter')
    mavproxy.expect('Please Run Setup')

    # we need to restart it after eeprom erase
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)
    sil = util.start_SIL('ArduCopter')
    mavproxy = util.start_MAVProxy_SIL('ArduCopter', options='--fgout=127.0.0.1:5502 --fgin=127.0.0.1:5501 --out=127.0.0.1:19550 --quadcopter')
    mavproxy.expect('Received [0-9]+ parameters')

    # setup test parameters
    mavproxy.send("param load %s/ArduCopter.parm\n" % testdir)
    mavproxy.expect('Loaded [0-9]+ parameters')

    # reboot with new parameters
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)
    sil = util.start_SIL('ArduCopter')
    options = '--fgout=127.0.0.1:5502 --fgin=127.0.0.1:5501 --out=127.0.0.1:19550 --quadcopter --streamrate=1'
    if viewerip:
        options += ' --out=%s:14550' % viewerip
    mavproxy = util.start_MAVProxy_SIL('ArduCopter', options=options)
    mavproxy.expect('Logging to (\S+)')
    logfile = mavproxy.match.group(1)
    print("LOGFILE %s" % logfile)

    buildlog = util.reltopdir("../buildlogs/ArduCopter-test.mavlog")
    print("buildlog=%s" % buildlog)
    if os.path.exists(buildlog):
        os.unlink(buildlog)
    os.link(logfile, buildlog)

    mavproxy.expect("Ready to FLY")
    mavproxy.expect('Received [0-9]+ parameters')

    util.expect_setup_callback(mavproxy, expect_callback)

    # start hil_quad.py
    cmd = util.reltopdir('../HILTest/hil_quad.py') + ' --fgrate=200 --home=%s' % HOME_LOCATION
    if viewerip:
        cmd += ' --fgout=192.168.2.15:9123'
    hquad = pexpect.spawn(cmd, logfile=sys.stdout, timeout=10)
    util.pexpect_autoclose(hquad)
    hquad.expect('Starting at')

    expect_list.extend([hquad, sil, mavproxy])

    # get a mavlink connection going
    try:
        mav = mavutil.mavlink_connection('127.0.0.1:19550', robust_parsing=True)
    except Exception, msg:
        print("Failed to start mavlink connection on 127.0.0.1:19550" % msg)
        raise
    mav.message_hooks.append(message_hook)


    failed = False
    try:
        mav.wait_heartbeat()
        mav.recv_match(type='GPS_RAW', blocking=True)
        setup_rc(mavproxy)
        homeloc = current_location(mav)
        if not arm_motors(mavproxy):
            failed = True

        if not takeoff(mavproxy, mav):
            failed = True

        if not fly_square(mavproxy, mav):
            failed = True

        if not loiter(mavproxy, mav):
            failed = True

        #Fly a circle for 60 seconds
        if not circle(mavproxy, mav):
            failed = True

        # fly the stores mission
        if not fly_mission(mavproxy, mav,height_accuracy = 0.5, target_altitude=10):
            failed = True

        #fly_mission(mavproxy, mav, os.path.join(testdir, "mission_ttt.txt"), height_accuracy=0.2)

        if not load_mission(mavproxy, mav, os.path.join(testdir, "mission_ttt.txt")):
            failed = True

        if not fly_mission(mavproxy, mav,height_accuracy = 0.5, target_altitude=10):
            failed = True

        if not land(mavproxy, mav):
            failed = True

        if not disarm_motors(mavproxy):
            failed = True
    except pexpect.TIMEOUT, e:
        failed = True

    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)
    util.pexpect_close(hquad)

    if os.path.exists('ArduCopter-valgrind.log'):
        os.chmod('ArduCopter-valgrind.log', 0644)
        shutil.copy("ArduCopter-valgrind.log", util.reltopdir("../buildlogs/ArduCopter-valgrind.log"))

    if failed:
        print("FAILED: %s" % e)
        return False
    return True
