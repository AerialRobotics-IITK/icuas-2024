import numpy as np
import filter2
#import aman_code.py
x=0
tolerance =105              #Tune this value
plants_to_look = [] 
plant_looked = 0
filtered_centroid_dict = {}
def filter_centroid(centroid_dict):
    global plant_looked
    filtered_dict = {}
    for key in centroid_dict:
        plant_looked +=1
        plant_number = key
        array = np.array(centroid_dict[key])   
        to_delete = []
        for i in range(len(array)):
            for j in range(len(array)):
                
                if i==j or i in to_delete or j in to_delete:
                    continue
                
                if (array[i][0]-array[j][0])**2+(array[i][1]-array[j][1])**2 < tolerance:
                    to_delete.append(j)

        array = np.delete(array,to_delete,0)
        global x
        for i in range(len(array)):
            x+=len(array[i])
        filtered_dict[key] = array
    return filtered_dict


def main(images,value_plant):
    global filtered_centroid_dict
    centroid_dict = filter2.get_centroid_list(images=images,value_plant=value_plant)
    filtered_centroid_dict = filter_centroid(centroid_dict)
    fruit_count = 0
    for key in filtered_centroid_dict:
        fruit_count += len(filtered_centroid_dict[key])
    # print(fruit_count)
    return fruit_count
            
# if __name__ == '__main__':
#     #aman.code.get_plant_franes()
#     # images={}
#     # images[0] = [0,0]
#     # images[1] = [0,0]
#     # images[0][0] = cv2.imread("1.png",cv2.IMREAD_ANYCOLOR)
#     # images[0][1] = cv2.imread("2.png",cv2.IMREAD_ANYCOLOR)
#     # images[1][0] = cv2.imread("1.png",cv2.IMREAD_ANYCOLOR)
#     # images[1][1] = cv2.imread("2.png",cv2.IMREAD_ANYCOLOR)
#     main(images=images, value_plant="Tomato")
