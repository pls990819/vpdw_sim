#import numpy as np
#a = np.loadtxt("traj.txt")
#b = []
#for i in range(len(a)):
#    if(i % 10 == 0):
#        b.append(a[i])
#b = np.array(b)
#print(b)
#np.savetxt("newtraj.txt",b,fmt="%.16f")


import numpy as np
a = np.loadtxt("traj.txt")
b = []
for i in range(len(a)-1):
    c = np.append(a[i],(a[i+1][2]-a[i][2])*10000)
    d = np.append(c,(a[i+1][3]-a[i][3])*10000)
    if(i % 10 == 0):
        b.append(d)
b = np.array(b)
print(b)
np.savetxt("newtraj.txt",b,fmt="%.16f")
    
