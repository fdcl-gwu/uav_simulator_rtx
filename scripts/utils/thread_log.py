from .uav import uav

import datetime
import numpy as np
import rospy


def thread_log():
    print('LOG: thread starting ..')

    freq = 50.0
    t0 = datetime.datetime.now()
    t = datetime.datetime.now()
    t_pre = datetime.datetime.now()
    avg_number = 100

    header_written = False
    file_open = False
    
    # with perspective from the script directory
    file_name = uav.t0.strftime('./data_logs/log_%Y%m%d_%H%M%S.txt')

    rate = rospy.Rate(freq)

    while not rospy.is_shutdown() and uav.on:
        t = datetime.datetime.now()
        dt = (t - t_pre).total_seconds()
        if dt < 1e-6:
            continue

        freq = (freq * (avg_number - 1) + (1 / dt)) / avg_number
        t_pre = t
        uav.freq_log = freq
        
        dt_millis = t - t0
        t_millis = int(dt_millis.seconds * 1e3 + dt_millis.microseconds / 1e3)

        if uav.save_on:
            if not header_written:
                header_written = True
                write_header(file_name)
            else:
                if not file_open:
                    f = open(file_name, 'a')
                    file_open = True
                write_date(f, t_millis)

        rate.sleep()

    if file_open:
        f.close()


    print('LOG: thread closed!')


def write_header(file_name):
    # NOTE: header order and the data order must be of the same order.
    with open(file_name, 'w') as f:
        f.write('time,')
        f.write('t,')
        
        f.write(string_vector('x'))
        f.write(string_vector('v'))
        f.write(string_vector('a'))
        f.write(string_vector('W'))
        f.write(string_3x3('R'))

        f.write(string_vector('xd'))
        f.write(string_vector('xd_dot'))
        f.write(string_vector('b1d'))
        f.write(string_vector('Wd'))
        f.write(string_3x3('Rd'))

        f.write('\n')

    with open('data_logs/last_log.txt', 'w') as f:
        f.write(file_name)


def write_date(f, t_millis):
    # NOTE: header order and the data order must be of the same order.
    write_scalar(f, datetime.datetime.now().strftime('%H%M%S.%f'))
    write_scalar(f, t_millis)

    write_vector(f, uav.x)
    write_vector(f, uav.v)
    write_vector(f, uav.a)
    write_vector(f, uav.W)
    write_3x3(f, uav.R)

    write_vector(f, uav.control.xd)
    write_vector(f, uav.control.xd_dot)
    write_vector(f, uav.control.b1d)
    write_vector(f, uav.control.Wd)
    write_3x3(f, uav.control.Rd)
    
    f.write('\n')


def string_vector(name, length=3):
    out = ''
    for i in range(length):
        out += '{}_{},'.format(name, i)
    return out


def string_3x3(name):
    out = ''
    for i in range(3):
        for j in range(3):
            out += '{}_{}{},'.format(name, i, j)
    return out


def write_scalar(f, data):
    f.write('{},'.format(data))


def write_vector(f, data, length=3):
    line = ''
    for i in range(length):
        line += '{},'.format(data[i])
    f.write(line)


def write_3x3(f, data):
    for i in range(3):
        for j in range(3):
            f.write('{},'.format(data[i, j]))
