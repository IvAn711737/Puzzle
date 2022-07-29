from re import X
import numpy as np

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    print (x)
    print (y)
    return(x, y)

def creating_position(R, Divider):
    segment=6.28/Divider
    poses = np.empty(([Divider, 2]), dtype="float64")
    for i in range(Divider):
        array=pol2cart(R, i*segment)
        poses[i] = array[:]
    return poses



    # poses = np.append(i, poses[0])
    # poses[i,]= creating_position(R)
    # print ("V2")
   

            # publishcreating_position(R)
print ("введите радиус")
R=int(input())
print(R)
print ("введите число точек на окружности ")
Divider=int(input())
print(Divider)
# for i in range(0,7,1):
    # print (creating_position(R))
poses=creating_position(R, Divider)
print (poses)
    