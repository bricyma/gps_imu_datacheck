import rosbag
import math as m
bagname = '2017-03-28-11-05-47.bag'
bagname = '../../rosbag/' + bagname
bag = rosbag.Bag(bagname)
gps_file = open('gps_orientation_data.txt', 'w')
vehicle_file = open('vehicle_data.txt', 'w')

sequence = 0
for topic, msg, t in bag.read_messages(topics=['/novatel_data/inspvax']):
    lat = msg.latitude
    lng = msg.longitude
    yaw = msg.azimuth
    roll = msg.roll
    pitch = msg.pitch
    # s =  str(sequence) + ' ' + str(time1) + ' ' +  str(time2)
    north_v = msg.north_velocity
    east_v = msg.east_velocity
    speed = m.sqrt(north_v * north_v + east_v * east_v)
    gps_file.write('%f %f %f %f %f %f %f %f\n' % (lat, lng, yaw, speed, pitch, roll, north_v, east_v))

for topic, msg, t in bag.read_messages(topics=['/vehicle/twist']):
    vel = msg.twist.linear.x
    vehicle_file.write('%f\n' % vel)

vehicle_file.close()
gps_file.close()
bag.close()


