import sys

# arg1 = no of chains
# arg2 = no of segments per chain
for i in range(int(sys.argv[1])):
    for j in range(int(sys.argv[2])):
        value  = 9.75e-7-2.5e-8*i
        print('k1.{0}\tstep\t0.0\t0.00000000e+00\t3.80000000e-07\t{1:.6e}\t1.0000\t0.0000\t0.0000\t0.0000\t1.0000\t0.0000\t0.0000\t0.0000\t1.0000\t0.0\t0.0\t0.0\t2.000\t2.000\t2.500'.format(i,value))
    print 'MT1.\tstep\t0.0\t0.00000000e+00\t2.20000000e-07\t9.00000000e-07\t1.0000\t0.0000\t0.0000\t0.0000\t0.0000\t1.0000\t0.0000\t-1.0000\t0.0000\t0.0\t0.0\t0.0\t2.000\t2.000\t30.000'
