import numpy as np
import matplotlib.pyplot as plt
import math as m

lat = [] # inspvax lat
lng = [] # inspvax lng
yaw = [] # inspvax orinetation.z
pitch = [] # inspvax orientation.x
roll = [] # inspvax orientation.y
gps_v = [] # calculated velocity from gps
gps_enu_x = [] # lat, lng to ENU, east, from kitti paper
gps_enu_y = [] # lat, lng to ENU, north, from kitti paper
gps_enu_x2 = [] # from matrix transfomration
gps_enu_y2 = [] # from matrix transfomration
id = [] # for original data from gps data
vehicle_id = [] # index for vehicle data
index = []
pos_ori = []
pos_ori2 = []
vel = []
n_vel = []  # north velocity
e_vel = []  # east velocity
ori_vel = [] # orientation from INSPVAX velocity
absolute_vel = [] # absolute velocity
vehicle_vel = [] # vehicle velocity from /vehicle/twist
lat0 = 0 #initial value, lat0 denotes the latitude of the first frame's coordinate
lng0 = 0

# for frame transformation
f = 1/298.257722356
a = 6378137.0
e = np.sqrt(f*(2-f))
# the origin coordinate of the local navigation frame
lat_ref = 32.826890 * np.pi/180
lng_ref = -117.230123 * np.pi/180
h_ref = 0
Rn = a/(m.sqrt(1-e**2*(m.sin(lat_ref))**2))
x_ref = (Rn + h_ref) * m.cos(lat_ref) * m.cos(lng_ref)
y_ref = (Rn + h_ref) * m.cos(lat_ref) * m.sin(lng_ref)
z_ref = (Rn * (1-e**2) + h_ref) * m.sin(lat_ref)
A = np.matrix([x_ref, y_ref, z_ref])
R = np.matrix([[-m.sin(lng_ref), m.cos(lng_ref), 0],
               [-m.sin(lat_ref) * m.cos(lng_ref), -m.sin(lat_ref) * m.sin(lng_ref), m.cos(lat_ref)],
               [m.cos(lat_ref) * m.cos(lng_ref), m.cos(lat_ref) * m.sin(lng_ref), m.sin(lat_ref)]])



def read_data():
    file_path = 'gps_orientation_data.txt'
    vehicle_file_path = 'vehicle_data.txt'
    input_file = open(file_path, 'r')
    count = 0
    for line in input_file:
        l = line.split(' ')
        lat.append(float(l[0]))
        lng.append(float(l[1]))
        yaw.append(float(l[2]))
        vel.append(float(l[3]))
        pitch.append(l[4])
        roll.append(l[5])
        id.append(count)
        n_vel.append(float(l[6]))
        e_vel.append(float(l[7]))
        absolute_vel.append(np.sqrt(float(l[6])**2 + float(l[7])**2))
        count += 1
    lat0 = lat[0]
    lng0 = lng[0]
    input_file.close()

    input_file = open(vehicle_file_path, 'r')
    count = 0
    for line in input_file:
        vehicle_vel.append(float(line))
        count += 1
        vehicle_id.append(count)

def orientation_from_velocity():
    for i in range(0, len(e_vel)):
        if np.arctan2(e_vel[i], n_vel[i]) < 0:
            cur_ori = np.arctan2(e_vel[i], n_vel[i]) * 180 / np.pi + 360
        else:
            cur_ori = np.arctan2(e_vel[i], n_vel[i]) * 180 / np.pi

        ori_vel.append(cur_ori)


def calculate_vel():
    toENU2()
    t = 0.02
    gps_enu_x = gps_enu_x2 # if toENU2()
    gps_enu_y = gps_enu_y2 # if toENU2()
    gps_v.append(0)
    for i in range (0, len(gps_enu_x)-1):
        dist = np.sqrt((gps_enu_y[i]-gps_enu_y[i+1])**2 + (gps_enu_x[i]-gps_enu_x[i+1])**2)
        gps_v.append(dist/t)

def toENU():
    length = len(lat)
    for i in range(0, length):
        temp = transform(lat[i], lng[i])
        gps_enu_x.append(temp[0])
        gps_enu_y.append(temp[1])

def toENU2():
    length = len(lat)
    for i in range(0, length):
        temp = transform_gps(lat[i], lng[i], 0)
        gps_enu_x2.append(temp[0])
        gps_enu_y2.append(temp[1])


def transform_gps(t_lat, t_lng, h):
    # t_lat is the transformed latitude
    t_lat = t_lat * np.pi/180
    t_lng = t_lng * np.pi/180

    Rn = a/(m.sqrt(1-(e**2)*(m.sin(t_lat))**2))
    x = (Rn + h) * m.cos(t_lat) * m.cos(t_lng)
    y = (Rn + h) * m.cos(t_lat) * m.sin(t_lng)
    z = (Rn * (1-e**2) + h) * m.sin(t_lat)

    gps_ecef_pos = np.matrix([x, y, z])
    # x, y, z in local navigation frame
    gps_rel = gps_ecef_pos - A
    gps_navi_pos_t = R * np.transpose(gps_rel)
    gps_navi_pos = np.transpose(gps_navi_pos_t)

    E = gps_navi_pos.item(0)
    N = gps_navi_pos.item(1)
    U = gps_navi_pos.item(2)
    l = [gps_rel.item(0), gps_rel.item(1)]
    return l


def transform(lat, lon):
    er = 6378137.0
    s = np.cos(lat0 * np.pi/180)
    tx = s * er * np.pi *lon / 180.0
    ty = s * er * np.log(np.tan((90.0 + lat) * np.pi / 360.0))
    l = [tx, ty]
    return l


def caluclate():
    length = len(x)
    k = 0
    for i in range(1, length):
        y2 = y[i]
        y1 = y[i-1]
        x2 = x[i]
        x1 = x[i-1]
        cur_ori = np.arctan2(x2-x1, y2-y1) * 180 / np.pi
        if cur_ori < 0:
            cur_ori += 360
        pos_ori.append(cur_ori)
        index.append(k)
        k += 1

# from bolun
def method2():
    length = len(lat)
    alpha = .2
    lat_ = 0
    lng_ = 0
    k = 0
    spd_limit_ = 5
    heading_ = 0
    raw_lat_ = lat0
    raw_lng_ = lng0
    for i in range(1, length):
        if raw_lat_ == lat[i] or raw_lng_ == lng[i] or np.absolute(vel[i]) < spd_limit_:
            pos_ori.append(heading_)
        else:
            raw_lat_ = lat[i]
            raw_lng_ = lng[i]
            temp_lat_ = lat_ + alpha * (raw_lat_ - lat_)
            temp_lng_ = lng_ + alpha * (raw_lng_ - lng_)
            dy = temp_lat_ - lat_
            dx = np.cos(np.pi * lat_ / 180) * (temp_lng_ - lng_)
            heading_ = -np.arctan2(dy, dx) * 180 / np.pi + 90

            if heading_ < 0:
                heading_ += 360
            lat_ = temp_lat_
            lng_ = temp_lng_
            pos_ori.append(heading_)
        index.append(k)
        k += 1




def plot():
    plt.ylim([-360,360])
    print len(index)
    print len(yaw)
    plt.plot(index, pos_ori, 'r')
    index.append(len(yaw)+1)
    # index.append(len(yaw)+2)
    plt.plot(index, yaw)
    plt.show()

def plot_gps():
    index.append(100000)
    print len(index), len(lat)
    plt.plot(index, lat)
    plt.show()

def plot_pitch():
    # plt.plot(id, pitch)
    plt.plot(id, yaw)
    plt.show()

def plot_velocity():
    plt.title('velocity')
    plt.plot(id, absolute_vel, 'r')
    plt.plot(vehicle_id, vehicle_vel)

    plt.plot(id, gps_v, 'g')
    plt.show()

def plot_orientation():
    plt.plot(id, yaw)
    plt.plot(id, ori_vel)
    plt.show()

read_data()
calculate_vel()
orientation_from_velocity()
# method 1
# toENU()
# caluclate()

# method 2
method2()
# plot()
# plot_gps()
# plot_pitch()
# plot_velocity()
plot_orientation()
print 'finish'
