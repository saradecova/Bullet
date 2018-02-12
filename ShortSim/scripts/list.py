import sys
import math

no_chromosomes = sys.argv[1]
no_segments = sys.argv[2]

radius = 1.06e-7
segments_yz = []
poles_pos = []
for i in range(int(no_chromosomes)):
    y = radius*math.cos(i*2*math.pi/int(no_chromosomes))
    z = radius*math.sin(i*2*math.pi/int(no_chromosomes))
    segments_yz.append([y,z])
    poles_pos.append([(2e-6-radius)/2, y/2, z/2])

x_start = 1e-6 - radius + 6e-8
x_step = 2e-8
for c in range(int(no_chromosomes)):
    for i in range(int(no_segments)):
        x_value  = x_start - i*x_step
        print('k{0}.{1}\tstep\t0.0\t{2:.6e}\t{3:.6e}\t{4:.6e}\t1.0000\t0.0000\t0.0000\t0.0000\t1.0000\t0.0000\t0.0000\t0.0000\t1.0000\t0.0\t0.0\t0.0\t2.000\t2.000\t2.500'.format(c,i,x_value, segments_yz[c][0], segments_yz[c][1]))
    print('MT.{0}\tstep\t0.0\t{1:.6e}\t{2:.6e}\t{3:.6e}\t1.0000\t0.0000\t0.0000\t0.0000\t0.0000\t1.0000\t0.0000\t-1.0000\t0.0000\t0.0\t0.0\t0.0\t2.000\t2.000\t30.000'.format(c, poles_pos[c][0], poles_pos[c][1], poles_pos[c][2]))
