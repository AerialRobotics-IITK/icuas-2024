def translation(plants_to_look) :
    for i in range(len(plants_to_look)):
        print(f"Plant {i+1} :")
        if(plants_to_look[i]//9 == 0 or plants_to_look == 9):
            for k in range(2):
                for j in range(1,4):
                    if (plants_to_look[i])%9==0:
                        x = 2+(plants_to_look[i]//9-1)*(6)
                        print(f"x: {x}")
                    else:
                        x = 2+(plants_to_look[i]//9)*(6)
                        print(f"x: {x}")
                    if (plants_to_look[i]%3)==0:
                        y = 3+(plants_to_look[i]//3-1)*7.5+j*1.5
                        print(f"y: {y}")
                    else:
                        y = 3+(plants_to_look[i]//3)*7.5+j*1.5
                        print(f"y: {y}")
                    if (plants_to_look[i]%3)==0:
                        z = (2)*2.8+1.1
                        print(f"z: {z}")
                    else:
                        z = ((int(plants_to_look[i]%3)-1)*2.8)+1.1
                        print(f"z: {z}")
                    if k==0:
                        yaw = 1.5707

    
def main():
   point_to_look = [1,2,5,9,11]
   translation(point_to_look)

if __name__ =="__main__":
    main()