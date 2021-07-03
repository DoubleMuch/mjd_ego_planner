import matplotlib.pyplot as plt
filename = '/data.txt'
T,X,Y,Z,VX,VT,VZ,AX,AY,AZ = [],[],[],[],[],[],[],[],[],[]
with open(filename, 'r') as f:#1
    lines = f.readlines()#2
    for line in lines:#3
        value = [float(s) for s in line.split()]#4
        T.append(value[0])
        X.append(value[1])#5
        Y.append(value[2])
        Z.append(value[3])
        VX.append(value[4])
        VY.append(value[5])
        VZ.append(value[6])
        AX.append(value[7])
        AY.append(value[8])
        AZ.append(value[9])
 
print(X)
print(Y)
 
plt.plot(T, X)
plt.show()
