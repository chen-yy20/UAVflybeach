import math
pi = math.pi
a = math.cos(pi/3)
b = math.asin(-0.52)
# print(str(b/pi))



def micro_move(x,y,theta,target_x,target_y):
    target_x = target_x -x
    target_y = target_y -y
    l = math.sqrt(target_x*target_x+target_y*target_y)
    print("斜边和目标角度sin：",l,target_y/l)
    phi = math.asin(target_y/l)
    print("phi_sin:",math.sin(phi))
    if target_x <0:
        phi = pi-phi
    print("phi:",str(phi/pi)+"*pi")
    # 转为弧度制
    theta = theta/180.0*pi
    delta_theta = theta-phi
    print(delta_theta)
    front = l*math.cos(delta_theta)

    command = ""
    if front >3:
        command = "forward"+str(100*int(front))
    elif front <-3:
        command = "back"+str(100*int(-front))
    else:
        print("纵向不需移动，偏差：",str(front))
    print(command)
    # sleep
    command = ""
    heng = l*math.sin(delta_theta)
    if heng >3:
        command = "right"+str(100*int(heng))
    elif front <-3:
        command = "left"+str(100*int(-heng))
    else:
        print("横向不需移动，偏差：",str(heng))
    print(command)
    print(front,heng)



# micro_move(0,0,30.0,1,1.732)
micro_move(0,0,0,1,-1)
# micro_move(0,0,60,-2,0.01)
micro_move(1.2,0.91,90,1.75,1.5)