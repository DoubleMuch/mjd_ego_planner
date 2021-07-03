import matplotlib.pyplot as plt
filename = 'bezier_data.txt'
T,X,Y,Z,VX,VY,VZ,AX,AY,AZ,YAW,ROLL,PITCH = [],[],[],[],[],[],[],[],[],[],[],[],[]
with open(filename, 'r') as f:#1
    lines = f.readlines()#2
    for line in lines:#3
        value = [float(s) for s in line.split()]#4
        T.append(value[0])
        X.append(value[1])#5
        Y.append(value[2])
        # Z.append(value[3])
        # VX.append(value[4])
        # VY.append(value[5])
        # VZ.append(value[6])
        # AX.append(value[7])
        # AY.append(value[8])
        # AZ.append(value[9])
        # YAW.append(value[10])
        # ROLL.append(value[11])
        # PITCH.append(value[12])


plt.figure(1)
plt.subplot(2, 2, 1) 
plt.plot(T, X,label='X')      #b
# plt.plot(T, Y,label='Y')      #g
# plt.plot(T, Z,label='Z')      #r
font = {'size':10}
plt.legend(loc='upper left',bbox_to_anchor=(0, 1),prop=font)

plt.figure(1)
plt.subplot(2, 2, 2) 
plt.plot(T, Y,label='Y')      #b
# plt.plot(T, Y,label='Y')      #g
# plt.plot(T, Z,label='Z')      #r
font = {'size':10}
plt.legend(loc='upper left',bbox_to_anchor=(0, 1),prop=font)

# plt.figure(1)
# plt.subplot(2, 2, 2) 
# plt.plot(T, VX,label='VX')      #b
# plt.plot(T, VY,label='VY')      #g
# plt.plot(T, VZ,label='VZ')      #r
# font = {'size':10}
# plt.legend(loc='upper left',bbox_to_anchor=(0, 1.2),prop=font)

# plt.figure(1)
# plt.subplot(2, 2, 3) 
# plt.plot(T, AX,label='AX')      #b
# plt.plot(T, AY,label='AY')      #g
# plt.plot(T, AZ,label='AZ')      #r
# font = {'size':10}
# plt.legend(loc='upper left',bbox_to_anchor=(0, 1.1),prop=font)


# plt.figure(1)
# plt.subplot(2, 2, 4) 
# plt.plot(T, YAW,label='yaw')      #b
# plt.plot(T, ROLL,label='roll')      #g
# plt.plot(T, PITCH,label='pitch')      #r
# font = {'size':10}
# plt.legend(loc='upper left',bbox_to_anchor=(0, 1.1),prop=font)
plt.show()
