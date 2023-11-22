from .uav import uav, reset_uav
from .plot_utils import plot_data

import gi
import numpy as np
import os

gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib, Gdk


class Gui():
    def __init__(self):

        self.builder = Gtk.Builder()
        
        # with perspective from the script directory
        self.builder.add_from_file("utils/gui.glade")

        self.window = self.builder.get_object('window_main')
        self.window.set_title('UAV Simulator')
        self.window.connect('destroy', Gtk.main_quit)
        self.window.connect('key-press-event', self.on_key_press)

        self.btn_close = self.builder.get_object('btn_close')
        self.btn_close.connect('clicked', self.on_btn_close_clicked)
        
        self.btn_reset = self.builder.get_object('btn_reset')
        self.btn_reset.connect('clicked', self.on_btn_reset_clicked)

        self.tgl_motor_on = self.builder.get_object('tgl_motor_on')
        self.tgl_motor_on.connect('toggled', self.on_tgl_motor_on_toggled)

        self.tgl_save = self.builder.get_object('tgl_save')
        self.tgl_save.connect('toggled', self.on_tgl_save_toggled)

        self.lbl_save = self.builder.get_object('lbl_save')
        self.lbl_save.set_text(uav.t0.strftime('log_%Y%m%d_%H%M%S.txt'))

        self.btn_plot = self.builder.get_object('btn_plot')
        self.btn_plot.connect('clicked', self.on_btn_plot_clicked)

        self.lbl_t = self.builder.get_object('lbl_t')
        self.lbl_freq_imu = self.builder.get_object('lbl_freq_imu')
        self.lbl_freq_gps = self.builder.get_object('lbl_freq_gps')
        self.lbl_freq_control = self.builder.get_object('lbl_freq_control')
        self.lbl_freq_log = self.builder.get_object('lbl_freq_log')

        self.modes = ['Idle', 'Warm-up', 'Take-off', 'Land', 'Stay', 'Circle', 'Triangle']
        self.rdo_mode = []
        for i in range(len(self.modes)):
            self.rdo_mode.append(
                self.builder.get_object('rdo_mode_{}'.format(i)))
            self.rdo_mode[i].set_label(self.modes[i])
            self.rdo_mode[i].connect('toggled', self.on_rdo_buton_clicked, i)

        self.x = self.get_vbox('lbl_x')
        self.v = self.get_vbox('lbl_v')
        self.a = self.get_vbox('lbl_a')
        self.W = self.get_vbox('lbl_W')
        self.R = self.get_grid('lbl_R')

        self.xd = self.get_vbox('lbl_xd')
        self.vd = self.get_vbox('lbl_vd')
        self.Wd = self.get_vbox('lbl_Wd')
        self.b1d = self.get_vbox('lbl_b1d')
        self.Rd = self.get_grid('lbl_Rd')

        self.window.connect('destroy', Gtk.main_quit)
        self.window.show_all()


    def on_btn_close_clicked(self, *args):
        print('GUI: close button clicked')
        uav.on = False
        Gtk.main_quit()
        
    def on_btn_reset_clicked(self, *args):
        print('GUI: reset button clicked')
        reset_uav('quadrotor')

    def on_btn_plot_clicked(self, widget):
        print('GUI: generating plots ..')
        plot_data()
        os.system('python3 plot_utils.py')


    def on_tgl_save_toggled(self, widget):
        if self.tgl_save.get_active():
            print('GUI: saving data started ..')
            uav.save_on = True
        else:
            print('GUI: saving data stopped')
            uav.save_on = False


    def on_tgl_motor_on_toggled(self, widget):
        if self.tgl_motor_on.get_active():
            print('GUI: motor on ..')
            uav.motor_on = True
        else:
            print('GUI: motor off')
            uav.motor_on = False


    def on_rdo_buton_clicked(self, widget, num):
        if widget.get_active():
            print('GUI: mode switched to {}'.format(self.modes[num]))
            uav.mode = num
            uav.x_offset = np.zeros(3)
            uav.yaw_offset = 0.0


    def on_key_press(self, widget, event):
        key = event.keyval
        
        x_step = 0.1
        yaw_step = 0.02
        
        if key == Gdk.KEY_M or  key == Gdk.KEY_m:
            print('GUI: turning motors off')
            uav.motor_on = False
            self.tgl_motor_on.set_active(False)
        elif key == Gdk.KEY_0:
            self.rdo_mode[0].set_active(True)
            uav.x_offset = np.zeros(3)
            uav.yaw_offset = 0.0
        elif key == Gdk.KEY_1:
            self.rdo_mode[1].set_active(True)
            uav.x_offset = np.zeros(3)
            uav.yaw_offset = 0.0
        elif key == Gdk.KEY_2:
            self.rdo_mode[2].set_active(True)
            uav.x_offset = np.zeros(3)
            uav.yaw_offset = 0.0
        elif key == Gdk.KEY_3:
            self.rdo_mode[3].set_active(True)
            uav.x_offset = np.zeros(3)
            uav.yaw_offset = 0.0
        elif key == Gdk.KEY_4:
            self.rdo_mode[4].set_active(True)
            uav.x_offset = np.zeros(3)
            uav.yaw_offset = 0.0
        elif key == Gdk.KEY_5:
            self.rdo_mode[5].set_active(True)
            uav.x_offset = np.zeros(3)
            uav.yaw_offset = 0.0
        elif key == Gdk.KEY_6:
            self.rdo_mode[6].set_active(True)
            uav.x_offset = np.zeros(3)
            uav.yaw_offset = 0.0
        elif key == Gdk.KEY_W or key == Gdk.KEY_w:
            uav.x_offset[0] += x_step
        elif key == Gdk.KEY_S or key == Gdk.KEY_s:
            uav.x_offset[0] -= x_step
        elif key == Gdk.KEY_D or key == Gdk.KEY_d:
            uav.x_offset[1] += x_step
        elif key == Gdk.KEY_A or key == Gdk.KEY_a:
            uav.x_offset[1] -= x_step
        elif key == Gdk.KEY_L or key == Gdk.KEY_l:
            uav.x_offset[2] += x_step
        elif key == Gdk.KEY_P or key == Gdk.KEY_p:
            uav.x_offset[2] -= x_step
        elif key == Gdk.KEY_E or key == Gdk.KEY_e:
            uav.yaw_offset += yaw_step
        elif key == Gdk.KEY_Q or key == Gdk.KEY_q:
            uav.yaw_offset -= yaw_step


    def update_gui(self):
        self.lbl_t.set_text('t = {:0.1f} s'.format(uav.t))
        self.lbl_freq_imu.set_text( \
            'IMU = {:3.0f} Hz'.format(uav.freq_imu))
        self.lbl_freq_gps.set_text(
            'GPS = {:3.0f} Hz'.format(uav.freq_gps))
        self.lbl_freq_control.set_text(
            'CTRL = {:3.0f} Hz'.format(uav.freq_control))
        self.lbl_freq_log.set_text(
            'LOG = {:3.0f} Hz'.format(uav.freq_log))

        self.update_vbox(self.x, uav.x)
        self.update_vbox(self.v, uav.v)
        self.update_vbox(self.a, uav.a)
        self.update_vbox(self.W, uav.W)
        self.update_grid(self.R, uav.R)

        self.update_vbox(self.xd, uav.control.xd)
        self.update_vbox(self.vd, uav.control.xd_dot)
        self.update_vbox(self.b1d, uav.control.b1d)
        self.update_vbox(self.Wd, uav.control.Wd)
        self.update_grid(self.Rd, uav.control.Rd)

        return True


    def get_vbox(self, name):
        vbox = []
        for i in range(3):
            vbox.append(self.builder.get_object('{}_{}'.format(name, i + 1)))
        return vbox


    def update_vbox(self, vbox, data):
        for i in range(3):
            vbox[i].set_text('{:8.2f}'.format(data[i]))


    def get_grid(self, name):
        vbox = []
        for i in range(1, 4):
            for j in range(1, 4):
                vbox.append(self.builder.get_object('{}_{}{}'.format(name, \
                    i, j)))
        return vbox


    def update_grid(self, grid, data):
        for i in range(3):
            for j in range(3):
                k = 3 * i + j
                grid[k].set_text('{:8.2f}'.format(data[i, j]))


def thread_gui():
    print('GUI: starting thread ..')

    gui = Gui()
    GLib.idle_add(gui.update_gui)
    Gtk.main()

    uav.on = False
    print('GUI: thread closed!')


if __name__=='__main__':
    thread_gui()