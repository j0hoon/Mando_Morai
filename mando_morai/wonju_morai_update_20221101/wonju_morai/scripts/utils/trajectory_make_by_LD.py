import numpy as np
import math

def trajectory_make_LD(ego_x,ego_y,desired_angle):
    
    ### Parameter ###

    Lp = 3.5 # Hyper Parameter
    Lp_x = ego_x + Lp * np.cos(desired_angle)
    Lp_y = ego_y + Lp * np.sin(desired_angle)
    
    WP_X, WP_Y = Lp_x, Lp_y

    long1 = ego_x
    lat1 = ego_y
    long2 = WP_X
    lat2 = WP_Y

    dist=np.sqrt((lat2-lat1)**2+(long2-long1)**2)
    target_point_x=long2-long1
    target_point_y=lat2-lat1

    P0=[0,0]
    P3=[target_point_x,target_point_y]

    yaw3=math.atan2(target_point_y,target_point_x)

    yaw0 = desired_angle
    distFactor=0.25

    P1=[P0[0]+dist*distFactor*np.cos(yaw0),P0[1]+dist*distFactor*np.sin(yaw0)]
    P2=[P3[0]-dist*distFactor*np.cos(yaw3),P3[1]-dist*distFactor*np.sin(yaw3)]

    CP1_X=P1[0]
    CP1_Y=P1[1]
    print 'CP1 : ',CP1_X,CP1_Y 
    CP2_X=P2[0]
    CP2_Y=P2[1]

    #Bezier_Point_Control = [P0[0],P0[1],CP1_X[0],CP1_Y[0],CP2_X[0],CP2_Y[0],P3[0][0],P3[1][0]]
    Bezier_Point_Control = [P0[0],P0[1],CP1_X,CP1_Y,CP2_X,CP2_Y,P3[0],P3[1]]
    
    P0_X=Bezier_Point_Control[0]
    P0_Y=Bezier_Point_Control[1]
    CP1_X=Bezier_Point_Control[2]
    CP1_Y=Bezier_Point_Control[3]
    CP2_X=Bezier_Point_Control[4]
    CP2_Y=Bezier_Point_Control[5]
    P3_X=Bezier_Point_Control[6]
    P3_Y=Bezier_Point_Control[7]

    P0 = [P0_X, P0_Y]    # start position
    P1 = [CP1_X, CP1_Y]
    P2 = [CP2_X, CP2_Y]
    P3 = [P3_X, P3_Y]    # end position    

    a_1=np.zeros(2)
    b_1=np.zeros(2)
    c_1=np.zeros(2)

    for i in range(2):   
        c_1[i]=3*(P1[i]-P0[i])
        b_1[i]=3*(P2[i]-P1[i])-c_1[i]
        a_1[i]=P3[i]-P0[i]-b_1[i]-c_1[i]

    t_1=np.linspace(0,1,51)
    Trajectory=np.zeros(len(t_1)*2).reshape(len(t_1),2)

    for i in range(0,51):
        for j in range(0,2):
            Trajectory[i,j]=a_1[j]*(t_1[i]**3)+b_1[j]*(t_1[i]**2)+c_1[j]*t_1[i]+P0[j]   

        Trajectory[:,0]=Trajectory[:,0]+ego_x 
        Trajectory[:,1]=Trajectory[:,1]+ego_y 
        Trajectory_X = Trajectory[1:,0]        
        Trajectory_Y = Trajectory[1:,1] 
        Trajectory_X=Trajectory_X.tolist()
        Trajectory_Y=Trajectory_Y.tolist()
        
        return Trajectory_X, Trajectory_Y
