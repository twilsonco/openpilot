#!/opt/homebrew/bin/python3

import numpy as np

min_accel = 0.16 
max_accel = 3
min_vel = 1
max_vel = 40
mid_vel = 20
num_la_buckets = 12
num_vel_buckets = 14
la_min_points = 6
vel_min_points = 10

la_buckets = np.geomspace(min_accel, max_accel, num_la_buckets//2-1)
la_buckets = np.concatenate((-la_buckets[::-1], [0.0], la_buckets))
vel_buckets = np.linspace(min_vel, max_vel, num_vel_buckets)

la_num_points = lambda la1,la2: (la_min_points - 8*abs(min([la1,la2],key=abs)))
vel_num_points = lambda vel1: vel_min_points - abs(vel1 - mid_vel)
combined_num_points = lambda la1, la2, vel1: int(round(max(0,la_num_points(la1, la2) + vel_num_points(vel1))**1.5))

BUCKETS = {((la1,la2),(vel1,vel2)): combined_num_points(la1,la2,vel1) for la1,la2 in zip(la_buckets[:-1],la_buckets[1:]) for vel1,vel2 in zip(vel_buckets[:-1], vel_buckets[1:])}

# BUCKETS = {}
for la1,la2 in zip(la_buckets[:-1],la_buckets[1:]):
    for vel1,vel2 in zip(vel_buckets[:-1], vel_buckets[1:]):
        # BUCKETS[((la1,la2),(vel1,vel2))] = int(round(max(0,(vel_min_points - abs(vel1 - mid_vel)) + (la_min_points - 8*abs(min([la1,la2],key=abs))))**1.5))
        print(f"(({la1:.2g},{la2:.2g}),({vel1:.2g},{vel2:.2g}))",": ", BUCKETS[((la1,la2),(vel1,vel2))])

# print(la_buckets)
# print(vel_buckets)
# print(BUCKETS)