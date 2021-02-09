bagFile = "/home/thomas/Private/JetsonCar/logs/2020-12-15-18-41-39.bag"

#%%
import rosbag
bag = rosbag.Bag(bagFile)
for topic, msg, t in bag.read_messages(topics=['/xsens/imu/acceleration', '/xsens/imu/angular_velocity']):
    print(topic)
    print(msg)
bag.close()

#%%
import bagpy
import pandas as pd
from bagpy import bagreader
b = bagreader("/home/thomas/Private/JetsonCar/logs/2020-12-15-18-41-39.bag")
print(*b.topics, sep='\n')

speed_file = b.message_by_topic('/xsens/imu/angular_velocity')
speed = pd.read_csv(speed_file)

fig, ax = bagpy.create_fig(1)
ax[0].scatter( x = 'Time', y = 'vector.z', data = speed, s = 3, marker = 'o', label = 'Original Speed')
ax[0].legend()
ax[0].set_xlabel('Time')
ax[0].set_ylabel('m/s')
fig.show()

