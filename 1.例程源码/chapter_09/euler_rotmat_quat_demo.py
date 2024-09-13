from scipy.spatial.transform import Rotation as R 

'''
[github]
https://github.com/xiihoo/DIY_A_SLAM_Navigation_Robot
[gitee]
https://gitee.com/xiihoo-robot/DIY_A_SLAM_Navigation_Robot
[website]
www.xiihoo.com
'''

# 欧拉角 ===> 旋转矩阵
def euler2rotmat(seq, angles, degrees_flag):
    if(degrees_flag == True):
        r3 = R.from_euler(seq, angles, degrees = True)
    else:
        r3 = R.from_euler(seq, angles, degrees = False)
    
    rotmat = r3.as_matrix()
    return rotmat

# 旋转矩阵 ===> 欧拉角
def rotmat2euler(rotation_matrix):
    r3 = R.from_matrix(rotation_matrix)

    euler = r3.as_euler()
    return euler

# 旋转矩阵 ===> 四元数 
def rotmat2quaternion(rotation_matrix):
    r3 = R.from_matrix(rotation_matrix)
    qua = r3.as_quat()
    return qua


# 四元数 ===> 旋转矩阵
def quaternion2rotmat(quaternion):
    r = R.from_quat(quaternion) # quaternion = (x, y, z, w)
    rot = r.as_matrix()
    return rot

# 获取欧拉角输入值: x_roll, y_pitch, z_yaw
str = input("please input euler(x_roll, y_pitch, z_yaw):")
str_list = str.split(',')

euler_angles = [float(str_list[0]),float(str_list[1]),float(str_list[2])]

rotmat = euler2rotmat('XYZ', euler_angles, True) # 内旋XYZ，单位°
#rotmat = euler2rotmat('xyz', euler_angles, True) # 外旋xyz，单位°

quat = rotmat2quaternion(rotmat)


print("#########################")
print("euler_angles = ")
print(euler_angles)

print("#########################")
print("rotmat = ")
print(rotmat)

print("#########################")
print("quat = ")
print(quat)
