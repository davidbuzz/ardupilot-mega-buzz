import math
import os, pexpect, sys, time, random
from rotmat import Vector3, Matrix3
from subprocess import call, check_call,Popen, PIPE

def m2ft(x):
    '''meters to feet'''
    return float(x) / 0.3048

def ft2m(x):
    '''feet to meters'''
    return float(x) * 0.3048

def kt2mps(x):
    return x * 0.514444444

def mps2kt(x):
    return x / 0.514444444

def topdir():
    '''return top of git tree where autotest is running from'''
    d = os.path.dirname(os.path.realpath(__file__))
    assert(os.path.basename(d)=='pysim')
    d = os.path.dirname(d)
    assert(os.path.basename(d)=='autotest')
    d = os.path.dirname(d)
    assert(os.path.basename(d)=='Tools')
    d = os.path.dirname(d)
    return d

def reltopdir(path):
    '''return a path relative to topdir()'''
    return os.path.normpath(os.path.join(topdir(), path))


def run_cmd(cmd, dir=".", show=False, output=False, checkfail=True):
    '''run a shell command'''
    if show:
        print("Running: '%s' in '%s'" % (cmd, dir))
    if output:
        return Popen([cmd], shell=True, stdout=PIPE, cwd=dir).communicate()[0]
    elif checkfail:
        return check_call(cmd, shell=True, cwd=dir)
    else:
        return call(cmd, shell=True, cwd=dir)

def rmfile(path):
    '''remove a file if it exists'''
    try:
        os.unlink(path)
    except Exception:
        pass

def deltree(path):
    '''delete a tree of files'''
    run_cmd('rm -rf %s' % path)



def build_SIL(atype, target='sitl'):
    '''build desktop SIL'''
    if os.getenv('MAVLINK10'):
        target += '-mavlink10'
    run_cmd("make clean %s" % target,
            dir=reltopdir(atype),
            checkfail=True)
    return True

def build_AVR(atype, board='mega2560'):
    '''build AVR binaries'''
    config = open(reltopdir('config.mk'), mode='w')
    config.write('''
BOARD=%s
PORT=/dev/null
''' % board)
    config.close()
    run_cmd("make clean", dir=reltopdir(atype),  checkfail=True)
    run_cmd("make", dir=reltopdir(atype),  checkfail=True)
    return True


# list of pexpect children to close on exit
close_list = []

def pexpect_autoclose(p):
    '''mark for autoclosing'''
    global close_list
    close_list.append(p)

def pexpect_close(p):
    '''close a pexpect child'''
    global close_list

    try:
        p.close()
    except Exception:
        pass
    try:
        p.close(force=True)
    except Exception:
        pass
    if p in close_list:
        close_list.remove(p)

def pexpect_close_all():
    '''close all pexpect children'''
    global close_list
    for p in close_list[:]:
        pexpect_close(p)

def pexpect_drain(p):
    '''drain any pending input'''
    try:
        p.read_nonblocking(1000, timeout=0)
    except pexpect.TIMEOUT:
        pass

def start_SIL(atype, valgrind=False, wipe=False, CLI=False, height=None):
    '''launch a SIL instance'''
    cmd=""
    if valgrind and os.path.exists('/usr/bin/valgrind'):
        cmd += 'valgrind -q --log-file=%s-valgrind.log ' % atype
    cmd += reltopdir('tmp/%s.build/%s.elf' % (atype, atype))
    if wipe:
        cmd += ' -w'
    if CLI:
        cmd += ' -s'
    if height is not None:
        cmd += ' -H %u' % height
    ret = pexpect.spawn(cmd, logfile=sys.stdout, timeout=5)
    ret.delaybeforesend = 0
    pexpect_autoclose(ret)
    ret.expect('Waiting for connection')
    return ret

def start_MAVProxy_SIL(atype, aircraft=None, setup=False, master='tcp:127.0.0.1:5760',
                       options=None, logfile=sys.stdout):
    '''launch mavproxy connected to a SIL instance'''
    global close_list
    MAVPROXY = reltopdir('../MAVProxy/mavproxy.py')
    cmd = MAVPROXY + ' --master=%s --out=127.0.0.1:14550' % master
    if setup:
        cmd += ' --setup'
    if aircraft is None:
        aircraft = 'test.%s' % atype
    cmd += ' --aircraft=%s' % aircraft
    if options is not None:
        cmd += ' ' + options
    ret = pexpect.spawn(cmd, logfile=logfile, timeout=60)
    ret.delaybeforesend = 0
    pexpect_autoclose(ret)
    return ret


def expect_setup_callback(e, callback):
    '''setup a callback that is called once a second while waiting for
       patterns'''
    def _expect_callback(pattern, timeout=e.timeout):
        tstart = time.time()
        while time.time() < tstart + timeout:
            try:
                ret = e.expect_saved(pattern, timeout=1)
                return ret
            except pexpect.TIMEOUT:
                e.expect_user_callback(e)
                pass
        print("Timed out looking for %s" % pattern)
        raise pexpect.TIMEOUT(timeout)

    e.expect_user_callback = callback
    e.expect_saved = e.expect
    e.expect = _expect_callback

def mkdir_p(dir):
    '''like mkdir -p'''
    if not dir:
        return
    if dir.endswith("/"):
        mkdir_p(dir[:-1])
        return
    if os.path.isdir(dir):
        return
    mkdir_p(os.path.dirname(dir))
    os.mkdir(dir)

def loadfile(fname):
    '''load a file as a string'''
    f = open(fname, mode='r')
    r = f.read()
    f.close()
    return r

def lock_file(fname):
    '''lock a file'''
    import fcntl
    f = open(fname, mode='w')
    try:
        fcntl.lockf(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except Exception:
        return None
    return f

def check_parent(parent_pid=os.getppid()):
    '''check our parent process is still alive'''
    try:
        os.kill(parent_pid, 0)
    except Exception:
        print("Parent had finished - exiting")
        sys.exit(1)


def EarthRatesToBodyRates(dcm, earth_rates):
    '''convert the angular velocities from earth frame to
    body frame. Thanks to James Goppert for the formula

    all inputs and outputs are in radians

    returns a gyro vector in body frame, in rad/s
    '''
    from math import sin, cos

    (phi, theta, psi) = dcm.to_euler()
    phiDot   = earth_rates.x
    thetaDot = earth_rates.y
    psiDot   = earth_rates.z

    p = phiDot - psiDot*sin(theta)
    q = cos(phi)*thetaDot + sin(phi)*psiDot*cos(theta)
    r = cos(phi)*psiDot*cos(theta) - sin(phi)*thetaDot
    return Vector3(p, q, r)

def BodyRatesToEarthRates(dcm, gyro):
    '''convert the angular velocities from body frame to
    earth frame.

    all inputs and outputs are in radians/s

    returns a earth rate vector
    '''
    from math import sin, cos, tan, fabs

    p      = gyro.x
    q      = gyro.y
    r      = gyro.z

    (phi, theta, psi) = dcm.to_euler()

    phiDot   = p + tan(theta)*(q*sin(phi) + r*cos(phi))
    thetaDot = q*cos(phi) - r*sin(phi)
    if fabs(cos(theta)) < 1.0e-20:
        theta += 1.0e-10
    psiDot   = (q*sin(phi) + r*cos(phi))/cos(theta)
    return Vector3(phiDot, thetaDot, psiDot)


class Wind(object):
    '''a wind generation object'''
    def __init__(self, windstring, cross_section=0.1):
        a = windstring.split(',')
        if len(a) != 3:
            raise RuntimeError("Expected wind in speed,direction,turbulance form, not %s" % windstring)
        self.speed     = float(a[0]) # m/s
        self.direction = float(a[1]) # direction the wind is going in
        self.turbulance= float(a[2]) # turbulance factor (standard deviation)

        # the cross-section of the aircraft to wind. This is multiplied by the
        # difference in the wind and the velocity of the aircraft to give the acceleration
        self.cross_section = cross_section

        # the time constant for the turbulance - the average period of the
        # changes over time
        self.turbulance_time_constant = 5.0

        # wind time record
        self.tlast = time.time()

        # initial turbulance multiplier
        self.turbulance_mul = 1.0

    def current(self, deltat=None):
        '''return current wind speed and direction as a tuple
        speed is in m/s, direction in degrees
        '''
        if deltat is None:
            tnow = time.time()
            deltat = tnow - self.tlast
            self.tlast = tnow

        # update turbulance random walk
        w_delta = math.sqrt(deltat)*(1.0-random.gauss(1.0, self.turbulance))
        w_delta -= (self.turbulance_mul-1.0)*(deltat/self.turbulance_time_constant)
        self.turbulance_mul += w_delta
        speed = self.speed * math.fabs(self.turbulance_mul)
        return (speed, self.direction)

    def accel(self, velocity, deltat=None):
        '''return current wind acceleration in ground frame.  The
           velocity is a Vector3 of the current velocity of the aircraft
           in earth frame, m/s'''
        from math import radians

        (speed, direction) = self.current(deltat=deltat)

        # wind vector
        v = Vector3(speed, 0, 0)
        m = Matrix3()
        m.from_euler(0, 0, radians(direction))
        wind = m.transposed() * v

        # relative wind vector
        relwind = wind - velocity

        # add in cross-section effect
        a = relwind * self.cross_section

        return a


if __name__ == "__main__":
    import doctest
    doctest.testmod()
