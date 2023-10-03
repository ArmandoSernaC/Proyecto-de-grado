# PASQUAL JOAN RIBOT LACOSTA 2018
# Tangent Bug algorithm for ROS/MORSE.
#!/usr/bin/env python3

import rospy
from std_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3

import turtle
import pymorse
import numpy
import math
import keyboard
import os
import copy
import time

#DECLARATIONS
DEFAULT_WALL_DISTANCE = 0.5# Distance from the wall
DEFAULT_OI_DISTANCE=0.5 #Distance form Ois in MtG
DEFAULT_MAX_V= 0.5 # Maximum speed of robot m/s
DEFAULT_MAX_W=0.5 # Not rad/s, but from 0 to 1 as a regulator
DEFAULT_RADIUS=0.5 # Robot's radius
DEFAULT_OBSTACLE_JUMP=1 # Jump between obstacle ranges to recognise it as different obstacles
DEFAULT_DRAW_MULTIPLIER=50 # Multiplier to determine draw size.

#FILES FOR RECORDING

f = open("tbuglog.txt",'w') # log for relevant data
pos=open("positions.txt",'w')# Record of all positions
obs=open("obstacles.txt",'w')# Record of obstacles detected
changes=open("changes.txt",'w') #Record of mode change

# pos, obs and changes are used in DrawPath.py

class tbug():
    
    motion = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Cambiado el topic a /cmd_vel
    
    
    #Default declarations
    radius=DEFAULT_RADIUS
    
    #Callback declarations
    obstacle_ranges=[] #Ranges where an obstalce is detected
    ois=[]# List of indexes for the Oi points.
    laser_angles=[]#angle for each laser
    laser_x=[]#global coordinates for laser points
    laser_y=[]
    laser_ranges=[]#distance reading from laser
    laser_incr=0 #angle incremente between each laser
    infl_x=[]#inflated coordinates
    infl_y=[]
    laser_range_max=0 #laser maximum range
    total_lasers=0 #total number of laser
    yaw=0 #yaw rotation of robot
    current_position=Vector3(-0.056736,-0.005891,-0.001008) #robot's position
    current_orientation=Quaternion(0,0,0,0)#robot's rotation
    obs_behind=False
    laser_record=False
    
    #Motion to Goal declarations
    ois_heur=[] # List of Ois with valird heuristic distance
    waypoint_x=math.inf #waypoint result of Move_target()
    waypoint_y=math.inf
    minheur_old=math.inf# Oi with minimum heuristc last cycle
    minheur_indx=-1 #Index for that Oi
    heur_dist=[] # List of all heuristic distances
    cond_count=0 #counter for the MtG to BF condition
    D=0 # distance used in Intersection() with "Between" mode
    minheur_x=math.inf# Coordinates for Oi with minimum heuristic.
    minheur_y=math.inf
    arrived=False #Condition of target reached
    change_x=math.inf #Coordinates where MtG to BF occurs.
    change_y=math.inf
    obs_detected=False # Obstacle detected flag
    motion_to_goal=True #Motion to goal flag
    boundary_following=False#Boundary Following flag
    
    # Boundary Following declarations
    oiBF=-1 #Index for the followed Oi
    oiBF_x=math.inf #Coordinates for OiBF
    oiBF_y=math.inf
    loop_x=math.inf # Coordinates to check if a loop through an obstacle has been completed
    loop_y=math.inf
    leave_x=math.inf # L point coordinates
    leave_y=math.inf
    dleave=math.inf # dleave distance
    min_x=math.inf # M point coordinates
    min_y=math.inf
    dmin=math.inf #dmin distance
    freepathBF=False # check to distinguish when to use L or T
    unreachable=False # unreachalbe condition
    check_unreachable=False # flag to start checking the unreachable condition
    check_loop=False #flag to check for a complete loop when checking the unreachalbe condition
    BFcycles=0

    def __init__(self):
    
        rospy.init_node("Listener")
    
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_laser)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.callback_Pose)
        self.callbackDoneLaser = False
        self.callbackDonePose = False

    def callback_Pose(self,pose):
    
        if self.callbackDonePose == False:
            self.current_position = pose.pose.position
            self.current_orientation = pose.pose.orientation
            self.callbackDonePose = True
    
    def callback_laser(self,laser,jump):
    
        if self.callbackDoneLaser == False and self.callbackDonePose==True:
            
            #pose operations

            siny = 2*(self.current_orientation.w * self.current_orientation.z +  self.current_orientation.x * self.current_orientation.y)
            cosy = 1-2*(self.current_orientation.y * self.current_orientation.y + self.current_orientation.z * self.current_orientation.z)

            self.yaw = math.atan2(siny,cosy)

            pos.write("{0}\n".format(self.current_position.x))
            pos.write("{0}\n".format(self.current_position.y))

            #list clearing

            self.laser_x.clear()
            self.laser_y.clear()
            self.infl_x.clear()
            self.infl_y.clear()
            self.laser_ranges.clear()
            self.laser_angles.clear()
            self.ois.clear()
            self.obstacle_ranges.clear()
            self.ois_heur.clear()
            self.heur_dist.clear()
            self.laser_record=self.laser_record+1
            self.D=math.sin(laser.angle_increment/2)*(laser.range_max*2)*2
            self.laser_range_max=laser.range_max
            self.laser_incr=laser.angle_increment
            self.total_lasers=len(laser.ranges)

            #laser raw data treatment
            ## Cambios procesamiento de los datos
            for i in range(len(laser.ranges)):

                angle = i * laser.angle_increment + laser.angle_min + self.yaw
                self.laser_x.append(math.cos(angle) * laser.ranges[i] + self.current_position.x)
                self.laser_y.append(math.sin(angle) * laser.ranges[i] + self.current_position.y)
                self.laser_angles.append(angle)
                self.laser_ranges.append(laser.ranges[i])

                if -jump < (laser.ranges[i + 1] - laser.ranges[i]) < jump and (laser.ranges[i] < laser.range_max):

                    self.obstacle_ranges.append(i)
                    if self.laser_record >= 5:
                        obs.write("{0}\n".format(self.laser_x[i]))
                        obs.write("{0}\n".format(self.laser_y[i]))

            if self.laser_record>=5:
                self.laser_record=0

            angle=laser.angle_max+self.yaw
            self.laser_x.append(math.cos(angle)*laser.ranges[len(laser.ranges)-1]+self.current_position.x)
            self.laser_y.append(math.sin(angle)*laser.ranges[len(laser.ranges)-1]+self.current_position.y)
            self.laser_angles.append(angle)
            self.laser_ranges.append(laser.ranges[len(laser.ranges)-1])
            
            #infalting obstacles and saving new obstacle ranges
            tb.InflateObstacles()
            self.obstacle_ranges.clear()
            self.laser_ranges.clear()

            for i in range (0,self.total_lasers):
            
                self.laser_ranges.append(tb.distance(self.current_position.x, self.current_position.y,self.infl_x[i],self.infl_y[i]))

            for i in range(0,len(self.laser_ranges)-1):
            
                if -jump<(self.laser_ranges[i+1]-self.laser_ranges[i])<jump and (self.laser_ranges[i]<laser.range_max*0.98):
                
                    self.obstacle_ranges.append(i)

            #changes to Oi points for when there's an obstacle behind the robot.
            if(len(self.obstacle_ranges)!=0):
            
                self.obs_detected=True
                obs_x1=self.infl_x[self.obstacle_ranges[0]]
                obs_y1=self.infl_y[self.obstacle_ranges[0]]
                obs_x2=self.infl_x[self.obstacle_ranges[len(self.obstacle_ranges)-1]]
                obs_y2=self.infl_y[self.obstacle_ranges[len(self.obstacle_ranges)-1]]
                dist=tb.distance(obs_x1,obs_y1,obs_x2,obs_y2)

                if dist>self.D:
                    self.obs_behind=False

                else:
                    self.obs_behind=True

                if self.obs_behind==False:
                    self.ois.append(self.obstacle_ranges[0])
                    j=1

                while j<len(self.obstacle_ranges)-1:
                
                    if(self.obstacle_ranges[j+1]-self.obstacle_ranges[j]!=1):
                        self.ois.append(self.obstacle_ranges[j])
                        self.ois.append(self.obstacle_ranges[j+1])
                        j=j+2
                    else:
                        j=j+1

                if self.obs_behind==False:
                    self.ois.append(self.obstacle_ranges[len(self.obstacle_ranges)-1])

                if self.obs_behind==True and len(self.ois)!=0:
                
                    self.ois_old=copy.copy(self.ois)
                    self.ois[0]=self.ois_old[len(self.ois_old)-1]

                    for i in range (1,len(self.ois)):
                        self.ois[i]=self.ois_old[i-1]

                else:
                    self.obs_behind=False
                    self.obs_detected=False

                self.callbackDoneLaser = True

    def MotiontoGoal(self,goal):

        if self.callbackDoneLaser==True and self.callbackDonePose==True:
        
            #check conditions and calculate distances
            tb.MtG_distances(goal)
            if self.motion_to_goal==True:
                tb.Move_target(self.minheur_indx,goal)
                turtle.clearscreen()
                tb.draw_surroundings(goal,DEFAULT_DRAW_MULTIPLIER)

                # message to publish
                msg = Twist()
                msg.linear.x=DEFAULT_MAX_V
                motion_angle=tb.transform_angle(self.waypoint_x,self.waypoint_y)
                msg.angular.z=motion_angle*DEFAULT_MAX_W

                #check reached condition
                if(goal.x-0.2<=self.current_position.x<=goal.x+0.2 and goal.y-0.2<=self.current_position.y<=goal.y+0.2):
                    msg.angular.z=0
                    msg.linear.x=0
                    for i in range (0,5):
                        self.motion.publish(msg)
                    self.motion_to_goal=False
                    self.arrived=True
                else:
                    self.motion.publish(msg)
                    self.callbackDonePose = False
                    self.callbackDoneLaser = False

    def MtG_distances(self,goal):
    
        minheur=math.inf
        p1x=self.current_position.x
        p1y=self.current_position.y
        p2x=goal.x
        p2y=goal.y
        blocking_obs_MtG=False
        dist_block=math.inf
        oi_blocked=math.inf
        oi_alt=math.inf
        alt_ok=False
        wp_inter=False

        #check for free path
        for i in range (0,len(self.obstacle_ranges)):
        
            blocking_obs_MtG=tb.Intersection(p1x,p1y,p2x,p2y,self.infl_x[self.obstacle_ranges[i]], self.infl_y[self.obstacle_ranges[i]],"Between")

            if blocking_obs_MtG==True:
                break
                
        if blocking_obs_MtG==True:
        
            d_old=math.inf

            #set up blocking Oi in case its need it (to avoid zig-zag)
            if self.minheur_indx>=0:
                for i in range (0,len(self.ois)):
                
                    d_new=tb.distance(self.infl_x[self.ois[i]],self.infl_y[self.ois[i]], self.minheur_x,self.minheur_y)

                    if d_new<d_old:
                        d_old=d_new
                        oi_alt=i

                if oi_alt % 2==0:
                  oi_blocked=oi_alt+1
                else:
                    oi_blocked=oi_alt-1
                    oialt_x=self.infl_x[self.ois[oi_alt]]
                    oialt_y=self.infl_y[self.ois[oi_alt]]
                    dist_block=tb.distance(oialt_x,oialt_y,self.infl_x[self.ois[oi_blocked]],
                    self.infl_y[self.ois[oi_blocked]])

            # Calculating heuristic distances
            for i in range (0,len(self.ois)):
            
                dx_oi=tb.distance(self.current_position.x,self.current_position.y,self.infl_x[self.ois[i]],self.infl_y[self.ois[i]])
                doi_goal=tb.distance(self.infl_x[self.ois[i]],self.infl_y[self.ois[i]],goal.x,goal.y)
                dx_goal=tb.distance(self.current_position.x,self.current_position.y,goal.x,goal.y)
                heur=round(dx_oi+doi_goal,3)
                tb.Move_target(i,goal)

                for j in range(0,len(self.obstacle_ranges)-1):
                
                    wpg_x=self.waypoint_x+self.current_position.x
                    wpg_y=self.waypoint_y+self.current_position.y
                    wp_inter=tb.Intersection(p1x,p1y,wpg_x,wpg_y,self.infl_x[self.obstacle_ranges[j]],self.infl_y[self.obstacle_ranges[j]],"Between")

                    if wp_inter==True:
                        break
                            
            #Oi filtering
            if doi_goal<dx_goal and tb.blocking_obs(i,p2x,p2y)==False and wp_inter==False:
            
                if heur<=minheur:

                    minheur=heur
                    self.heur_dist.append(heur)
                    self.minheur_indx=i
                    self.ois_heur.append(self.ois[i])
                    self.minheur_x=self.infl_x[self.ois[i]]
                    self.minheur_y=self.infl_y[self.ois[i]]

                    if i==oi_alt:
                        alt_ok=True
                else:

                    if i==oi_alt:
                        alt_ok=True

                    self.ois_heur.append(self.ois[i])
                    self.heur_dist.append(heur)

            #Check for Oi blocking
            if self.minheur_indx==oi_blocked and dist_block>0.3:

                for i in range (0,len(self.ois_heur)-1):

                    if self.ois_heur[i]==oi_alt:

                        alt_ok=True
                        break

                if alt_ok==True:

                    self.minheur_indx=oi_alt
                    self.minheur_x=self.infl_x[self.ois[oi_alt]]
                    self.minheur_y=self.infl_y[self.ois[oi_alt]]
                    dx_oi=tb.distance(self.current_position.x,self.current_position.y,self.infl_x[self.ois[oi_alt]],self.infl_y[self.ois[oi_alt]])
                    doi_goal=tb.distance(self.infl_x[self.ois[oi_alt]], self.infl_y[self.ois[oi_alt]],goal.x,goal.y)
                    minheur=round(dx_oi+doi_goal,3)

                else:

                    #Oi to change to when blocking is no valid
                    self.minheur_old=0
                    self.cond_count=6


            # If no Oi goes through filtering, MtG->BF
            if len(self.ois_heur)==0:
            
                self.minheur_old=0
                self.cond_count=6

            # MtG->BF condition with counter
            if minheur>self.minheur_old:

                self.cond_count=self.cond_count+1

                if self.cond_count>5:
                
                    # starting M,L,dmin and dleave
                    self.dmin=math.inf

                    for i in range (0,len(self.obstacle_ranges)):

                        dmin_prov=tb.distance(self.infl_x[self.obstacle_ranges[i]],
                        self.infl_y[self.obstacle_ranges[i]],goal.x,goal.y)

                        if dmin_prov<self.dmin:
                            self.dmin=dmin_prov
                            self.min_x=self.infl_x[self.obstacle_ranges[i]]
                            self.min_y=self.infl_y[self.obstacle_ranges[i]]

                    self.leave_x=self.min_x
                    self.leave_y=self.min_y
                    self.dleave=self.dmin

                    # If a change happens whit no Oi currently
                    #valid, we take the last one that was valid.
                    if self.minheur_indx<0 or len(self.ois_heur)==0:
                    
                        dist=math.inf

                        for i in range(0,len(self.ois)-1):
                        
                            dist_new=tb.distance(self.current_position.x,
                            self.current_position.y,self.infl_x[self.ois[i]],self.infl_x[self.ois[i]])

                            if dist_new<dist:
                                dist=dist_new
                                self.oiBF_x=self.infl_x[self.ois[i]]
                                self.oiBF_y=self.infl_y[self.ois[i]]
                                self.oiBF=i
                    else:
                        self.oiBF_x=self.infl_x[self.ois[self.minheur_indx]]
                        self.oiBF_y=self.infl_y[self.ois[self.minheur_indx]]
                        self.oiBF=self.minheur_indx

                    f.write("minheur'{0}'\r\n".format(minheur))
                    f.write("minheur old'{0}'\r\n".format(self.minheur_old))
                    f.write(" MtG->BF\r\n")

                    changes.write("{0}\n".format(self.current_position.x))
                    changes.write("{0}\n".format(self.current_position.y))
                    
                    self.cond_count=0
                    self.motion_to_goal=False
                    self.callbackDonePose = False
                    self.callbackDoneLaser = False
                    self.change_x=self.current_position.x
                    self.change_y=self.current_position.y
                    self.boundary_following=True

                else:
                    # continue in MtG
                    self.minheur_old=minheur
                    self.cond_count=0

            else:
                #straight to goal
                self.minheur_indx=-1


    def BoundaryFollowing(self,goal):

        if self.callbackDoneLaser==True and self.callbackDonePose==True:
            #check for conditions and get Oi to follow
            tb.BF_distances(goal)
                            
        if self.boundary_following==True:
        
            tb.Move_target(self.oiBF,goal)
            turtle.clearscreen()
            tb.draw_surroundings(goal,DEFAULT_DRAW_MULTIPLIER)
            motion_angle=tb.transform_angle(self.waypoint_x,self.waypoint_y)

            #message to publish
            msg = Twist()
            msg.linear.x=DEFAULT_MAX_V
            msg.angular.z=motion_angle*DEFAULT_MAX_W

            #To check unreachable, first we get to a distance from the change       point and then we get another point.
            #After seperating a distance again from that point, we can start      checking if its unreachalbe.

            dist_change=tb.distance(self.current_position.x,self.current_position.y, self.change_x,self.change_y)

            if dist_change>1 and self.check_loop==False:

                self.check_loop=True
                self.loop_x=self.current_position.x
                self.loop_y=self.current_position.y

            dist_loop=tb.distance(self.current_position.x,self.current_position.y , self.loop_x,self.loop_y)

            if self.check_loop==True and dist_loop>1:
                self.check_unreachable=True

            if self.check_unreachable==True and dist_loop<0.1:
            
                msg.linear.x=0
                msg.angular.z=0

                for i in range (0,5):
                    self.motion.publish(msg)

                self.boundary_following=False
                self.unreachable=True

            else:
                self.motion.publish(msg)
                self.callbackDoneLaser=False
                self.callbackDonePose=False
    

    def BF_distances(self,goal):

        p1x=self.current_position.x
        p1y=self.current_position.y
        p2x=goal.x
        p2y=goal.y
        blocking_obs_BF=False
        self.dleave=math.inf
        d_old=math.inf
        old_oiBF=self.oiBF

        for i in range (0,len(self.ois)):
            #the new OiBF will be the closest to the last one. We block a             change of OiBF when the current and last Oi belong to the same obstacle.

            d_new=tb.distance(self.infl_x[self.ois[i]],self.infl_y[self.ois[i]], self.oiBF_x,self.oiBF_y)

            if d_new<d_old:
                self.oiBF=i
                d_old=d_new

            if old_oiBF % 2==0 and self.oiBF==old_oiBF+1:
                self.oiBF=old_oiBF

            elif old_oiBF % 2 !=0 and self.oiBF==old_oiBF-1:
                self.oiBF=old_oiBF
                self.oiBF_x=self.infl_x[self.ois[self.oiBF]]
                self.oiBF_y=self.infl_y[self.ois[self.oiBF]]

             #Calculatin dleave.
            for i in range (0,len(self.obstacle_ranges)):
            
                blocking_obs_BF=tb.Intersection(p1x,p1y,p2x,p2y,self.infl_x[self.obstacle_ranges[i]], self.infl_y[self.obstacle_ranges[i]],"Between")

                if blocking_obs_BF==True:
                    break
                    
            if blocking_obs_BF==False:
            
                vx=p2x-p1x
                vy=p2y-p1y
                ux=vx/math.sqrt(vx**2+vy**2)
                uy=vy/math.sqrt(vx**2+vy**2)
                self.leave_x=self.current_position.x+(ux*self.laser_range_max)
                self.leave_y=self.current_position.y+(uy*self.laser_range_max)
                self.dleave=tb.distance(self.leave_x,self.leave_y,goal.x,goal.y)
                self.freepathBF=True

            else:

                self.freepathBF=False

                if self.oiBF %2 ==0:
                    closest_oi=self.oiBF
                else:
                    closest_oi=self.oiBF-1

                for i in range (0,len(self.obstacle_ranges)):

                    if closest_oi==0 and self.obs_behind:

                        if self.ois[0]<self.obstacle_ranges[i]<self.total_lasers-1 or 0<self.obstacle_ranges[i]<self.ois[1]:
                        
                            dleave_prov=tb.distance(self.infl_x[self.obstacle_ranges[i]],
                            self.infl_y[self.obstacle_ranges[i]],goal.x,goal.y)

                        if dleave_prov<self.dleave:

                            self.dleave=dleave_prov
                            self.leave_x=self.infl_x[self.obstacle_ranges[i]]
                            self.leave_y=self.infl_y[self.obstacle_ranges[i]]

                    else:
                    
                        if self.ois[closest_oi]<self.obstacle_ranges[i]<self.ois[closest_oi+1]:

                            dleave_prov=tb.distance(self.infl_x[self.obstacle_ranges[i]],
                            self.infl_y[self.obstacle_ranges[i]],goal.x,goal.y)

                            if dleave_prov<self.dleave:
                            
                                self.dleave=dleave_prov
                                self.leave_x=self.infl_x[self.obstacle_ranges[i]]
                                self.leave_y=self.infl_y[self.obstacle_ranges[i]]   
            #BF->MtG condition
            if self.dleave+self.radius<self.dmin:
            
                f.write("dmin'{0}'\r\n".format(self.dmin))
                f.write("dleave'{0}'\r\n".format(self.dleave))
                f.write("BF->MtG \r\n")

                changes.write("{0}\n".format(self.current_position.x))
                changes.write("{0}\n".format(self.current_position.y))

                self.minheur_indx=-1
                self.minheur_x=math.inf
                self.minheur_y=math.inf
                self.minheur_old=math.inf
                self.check_unreachable=False
                self.check_loop=False
                self.boundary_following=False
                self.motion_to_goal=True
                self.callbackDonePose = False
                self.callbackDoneLaser = False

    def transform_angle(self,p1x,p1y):
        #transforming angle so we can know how much the robot has to turn to point to the desired point.

        target_angle=math.atan2(p1y,p1x)

        if(math.radians(90)>target_angle>=math.radians(0) or math.radians(180) >target_angle>=math.radians(90)):

            if(self.yaw>0):

                motion_angle=target_angle-self.yaw

            else:

                if(math.radians(180)-math.fabs(self.yaw)<target_angle):
                    motion_angle=-(math.radians(180)-math.fabs(self.yaw))-(math.radians(180)-target_angle)
                else:
                    motion_angle=target_angle+math.fabs(self.yaw)
        else:

            if(self.yaw>0):

                if(self.yaw>math.radians(180)-math.fabs(target_angle)):
                    motion_angle=(math.radians(180)-self.yaw)+(math.radians(180)-math.fabs(target_angle))
                else:
                    motion_angle=-(self.yaw+math.fabs(target_angle))

            else:

                if(self.yaw<target_angle):
                    motion_angle=math.fabs(self.yaw)-math.fabs(target_angle)
                else:
                    motion_angle=-(math.fabs(target_angle)-math.fabs(self.yaw))

        return motion_angle 

    def Intersection(self,p1x,p1y,p2x,p2y,cx,cy,mode):

        # True if there's an intersection between the line P1P2 and a circle with center at C.
        #y=mx+d
        m=(p2y-p1y)/(p2x-p1x)
        d=p1y-m*p1x

        if m==0:
            m_per=math.inf
        else:
            m_per=-(1/m)

        d_per=cy-m_per*cx
        interx=(d_per-d)/(m-m_per)
        intery=interx*m+d
        dist=tb.distance(cx,cy,interx,intery)

        if mode=="Between":
        
            #Cirle radius is D and also check is the intersection is on the line segement P1P2
            if dist<=self.D and tb.PointisBetween(p1x,p1y,p2x,p2y,interx,intery):
                return True
            else:
                return False


    def InflateObstacles(self):
    
        #Inflating obstacles to the robot's radius to avoid collision.
        p1x=self.current_position.x
        p1y=self.current_position.y

        for i in range (0,self.total_lasers):
        
            mindist=math.inf
            p2x=self.laser_x[i]
            p2y=self.laser_y[i]
            minx=p2x
            miny=p2y

            for j in range (0,len(self.obstacle_ranges)):
            
                cx=self.laser_x[self.obstacle_ranges[j]]
                cy=self.laser_y[self.obstacle_ranges[j]]

                if self.radius>tb.distance(self.current_position.x,self.current_position.y,cx,cy):
                    f.write("ERROR, DENTRO DEL HINCHADO")

                if tb.Intersection(p1x,p1y,p2x,p2y,cx,cy,"Simple"):
                
                    #y=mx+d

                    m=(p2y-p1y)/(p2x-p1x)
                    d=p1y-m*p1x

                    #(x-a)*(x-a)+(y-b)*(y-b)=R*R

                    a=cx
                    b=cy
                    ro=(self.radius**2)*(1+m**2)-(b-m*a-d)**2

                    if ro>=0:
                    
                        xinter1=(a+b*m-d*m+math.sqrt(ro))/(1+m**2)
                        xinter2=(a+b*m-d*m-math.sqrt(ro))/(1+m**2)

                        yinter1=m*xinter1+d
                        yinter2=m*xinter2+d

                        dist1=tb.distance(self.current_position.x,self.current_position.y,xinter1,yinter1)
                        dist2=tb.distance(self.current_position.x,self.current_position.y,xinter2,yinter2)

                        Pmax_x=(math.cos(self.laser_angles[i])*self.laser_range_max)+self.current_position.x
                        Pmax_y=(math.sin(self.laser_angles[i])*self.laser_range_max)+self.current_position.y

                        inter1ok=tb.PointisBetween(self.current_position.x,self.current_position.y,Pmax_x,Pmax_y,xinter1,yinter1)
                        inter2ok= tb.PointisBetween(self.current_position.x,self.current_position.y,Pmax_x,Pmax_y,xinter2,yinter2)
                        
                        if dist1<=dist2:

                            if dist1<=mindist and inter1ok:
                                mindist=dist1
                                minx=xinter1
                                miny=yinter1
                        else:
                            
                            if dist2<=mindist and inter2ok:
                                mindist=dist2
                                minx=xinter2
                                miny=yinter2

        self.infl_x.append(minx)
        self.infl_y.append(miny)
    


    def Move_target(self,i,goal):

        # Move current target to a new waypoint in order to not entering  inflated ranges.

        if i<0:

            self.waypoint_x=goal.x-self.current_position.x
            self.waypoint_y=goal.y-self.current_position.y

        else:

            way1=False
            way2=False

            if self.motion_to_goal:
                ang=DEFAULT_OI_DISTANCE/(self.laser_ranges[self.ois[i]])

            else:
                ang=DEFAULT_WALL_DISTANCE/(self.laser_ranges[self.ois[i]])

            if ang>1:
                ang=1

            elif ang<-1:
                ang=-1

            alpha=2*math.asin(ang)
            lasers=round(alpha/self.laser_incr)

            if self.ois[i]-lasers<=0:

                waypoint1_x=(math.cos(self.laser_angles[0])*self.laser_ranges[self.ois[i]])
                waypoint1_y=(math.sin(self.laser_angles[0])*self.laser_ranges[self.ois[i]])

            else:

                waypoint1_x=(math.cos(self.laser_angles[self.ois[i]-lasers])*self.laser_ranges[self.ois[i]])
                waypoint1_y=(math.sin(self.laser_angles[self.ois[i]-lasers])*self.laser_ranges[self.ois[i]])

            if self.ois[i]+lasers>=len(self.laser_angles)-1:
                waypoint2_x=(math.cos(self.laser_angles[len(self.laser_angles)-1])*self.laser_ranges[self.ois[i]])
                waypoint2_y=(math.sin(self.laser_angles[len(self.laser_angles)-1])*self.laser_ranges[self.ois[i]])

            else:
                waypoint2_x=(math.cos(self.laser_angles[self.ois[i]+lasers])*self.laser_ranges[self.ois[i]])
                waypoint2_y=(math.sin(self.laser_angles[self.ois[i]+lasers])*self.laser_ranges[self.ois[i]])

            p1x=self.current_position.x
            p1y=self.current_position.y
            p2x=waypoint1_x+self.current_position.x
            p2y=waypoint1_y+self.current_position.y
            p3x=waypoint2_x+self.current_position.x
            p3y=waypoint2_y+self.current_position.y

            for j in range (0,len(self.obstacle_ranges)-1):

                inter1=tb.Intersection(p1x,p1y,p2x,p2y,self.infl_x[self.obstacle_ranges[j]],self.infl_y[self.obstacle_ranges[j]],"Between")
                inter2=tb.Intersection(p1x,p1y,p3x,p3y,self.infl_x[self.obstacle_ranges[j]],self.infl_y[self.obstacle_ranges[j]],"Between")

                if inter1==True:

                    self.waypoint_x=waypoint2_x
                    self.waypoint_y=waypoint2_y
                    break

                elif inter2==True:

                    self.waypoint_x=waypoint1_x
                    self.waypoint_y=waypoint1_y
                    break
            
    def blocking_obs (self,i,p2x,p2y):
    
        #check if theres a blocking obstacle between Oi and G, not counting         the Oi's obstacle

       
        p1x=self.infl_x[self.ois[i]]
        p1y=self.infl_y[self.ois[i]]

        if len(self.ois)>2:
        
            if self.obs_behind and (i==0 or i==1):
            
                for j in range (0,len(self.obstacle_ranges)-1):
                
                    if (self.obstacle_ranges[j]<self.ois[0] and  self.obstacle_ranges[j]>self.ois[1]):
                    
                        blocking_obs=tb.Intersection(p1x,p1y,p2x,p2y,self.infl_x[self.obstacle_ranges[j]],self.infl_y[self.obstacle_ranges[j]],"Between")

                        if blocking_obs==True:
                            return True
                        
                return False
            
            else:

                for j in range (0,len(self.obstacle_ranges)-1):
                
                    if i % 2==0 and (self.obstacle_ranges[j]<self.ois[i] or   self.obstacle_ranges[j]>self.ois[i+1]):
                    
                        blocking_obs=tb.Intersection(p1x,p1y,p2x,p2y,self.infl_x[self.obstacle_ranges[j]],self.infl_y[self.obstacle_ranges[j]],"Between")

                        if blocking_obs==True:
                            return True

                    elif i % 2!=0 and (self.obstacle_ranges[j]<self.ois[i-1] or  self.obstacle_ranges[j]>self.ois[i]):
                    
                        blocking_obs=tb.Intersection(p1x,p1y,p2x,p2y, self.infl_x[self.obstacle_ranges[j]], self.infl_y[self.obstacle_ranges[j]],"Between")

                        if blocking_obs==True:
                            return True
                        
                return False
        else:
            return False
        
        

    def PointisBetween(self,ax,ay,bx,by,cx,cy):
        # True if C is between A and B
        crossproduct=(cy-ay)*(bx-ax)-(cx-ax)*(by-ay)

        if abs(round(crossproduct,3))!=0:
            return False


        dotproduct =(cx-ax)*(bx-ax)+(cy-ay)*(by-ay)

        if round(dotproduct,3)<= 0:
            return False

        squaredlengthba = (bx-ax)*(bx-ax)+(by-ay)*(by-ay)

        if dotproduct >= squaredlengthba:
            return False
        return True

    def distance(self,ax,ay,bx,by):
         
        # distance between two points
        dx = bx-ax
        dy = by-ay
        return math.sqrt(dx**2 + dy**2)



if __name__ == '__main__':

    try:
        tb = tbug()
        goal=Vector3(4,4,0.15)

        #Main loop
        while True:
            if tb.motion_to_goal==True:
                tb.MotiontoGoal(goal)
            elif tb.boundary_following==True:
                tb.BoundaryFollowing(goal)
            else:
                if tb.arrived==True:
                    f.write("Arrived")
                    break
                elif tb.unreachable==True:
                    f.write("Goal is unreachable")
                    break
                else:
                    break
        f.close()
        changes.close()
        pos.close()
        obs.close()
        rospy.spin()

    except rospy.ROSInterruptException: pass