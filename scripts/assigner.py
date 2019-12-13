#!/usr/bin/env python

#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import tf
from rrt_exploration.msg import PointArray
from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import robot,informationGain,discount
from numpy.linalg import norm

# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]
global1=OccupancyGrid()
global2=OccupancyGrid()
global3=OccupancyGrid()
globalmaps=[]
def callBack(data):
  global frontiers
  frontiers=[]
  for point in data.points:
    frontiers.append(array([point.x,point.y]))

def mapCallBack(data):
  global mapData
  mapData=data
# Node----------------------------------------------

def node():
  global frontiers,mapData,global1,global2,global3,globalmaps
  rospy.init_node('assigner', anonymous=False)
  
  # fetching all parameters
  map_topic= rospy.get_param('~map_topic','/map')
  info_radius= rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
  info_multiplier=rospy.get_param('~info_multiplier',3.0)		
  hysteresis_radius=rospy.get_param('~hysteresis_radius',3.0)			#at least as much as the laser scanner range
  hysteresis_gain=rospy.get_param('~hysteresis_gain',2.0)				#bigger than 1 (biase robot to continue exploring current region
  frontiers_topic= rospy.get_param('~frontiers_topic','/filtered_points')	
  n_robots = 1
  namespace = rospy.get_param('~namespace','')
  namespace_init_count = rospy.get_param('namespace_init_count',1)
  delay_after_assignement=rospy.get_param('~delay_after_assignement',0.5)
  rateHz = rospy.get_param('~rate',100)
  
  rate = rospy.Rate(rateHz)
#-------------------------------------------
  rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
  rospy.Subscriber(frontiers_topic, PointArray, callBack)
#---------------------------------------------------------------------------------------------------------------

# wait if no frontier is received yet 
  while len(frontiers)<1:
    pass
  centroids=copy(frontiers)	
#wait if map is not received yet
  while (len(mapData.data)<1):
    pass


  robots=[]
  if len(namespace)>0:
    for i in range(0,n_robots):
      robots.append(robot(namespace))
  elif len(namespace)==0:
    robots.append(robot(namespace))
  for i in range(0,n_robots):
    robots[i].sendGoal(robots[i].getPosition(),False)

  listener = tf.TransformListener()
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
  while not rospy.is_shutdown():
    centroids=copy(frontiers)		
#-------------------------------------------------------------------------			
#Get information gain for each frontier point
    infoGain=[]
    for ip in range(0,len(centroids)):
      infoGain.append(informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius))
#-------------------------------------------------------------------------			
#get number of available/busy robots
    na=[] #available robots
    nb=[] #busy robots
    for i in range(0,n_robots):
      if (robots[i].getState()==1):
          nb.append(i)
      else:
          na.append(i)	
    # rospy.loginfo("available robots: "+str(na))	
#------------------------------------------------------------------------- 
#get dicount and update informationGain
    for i in nb+na:
        infoGain=discount(mapData,robots[i].assigned_point,centroids,infoGain,info_radius)
#-------------------------------------------------------------------------            
    revenue_record=[]
    centroid_record=[]
    id_record=[]
    
    for ir in na:
      for ip in range(0,len(centroids)):
        cost=norm(robots[ir].getPosition()-centroids[ip])		
        threshold=1
        information_gain=infoGain[ip]

        if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):
          information_gain*=hysteresis_gain

        revenue=information_gain*info_multiplier-cost
        revenue_record.append(revenue)
        centroid_record.append(centroids[ip])
        id_record.append(ir)
        
    if len(na)<1:
      revenue_record=[]
      centroid_record=[]
      id_record=[]
      for ir in nb:
        for ip in range(0,len(centroids)):
          cost=norm(robots[ir].getPosition()-centroids[ip])		
          threshold=1
          information_gain=infoGain[ip]
          if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):
            information_gain*=hysteresis_gain
            
          if ((norm(centroids[ip]-robots[ir].assigned_point))<hysteresis_radius):
            information_gain=informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius)*hysteresis_gain

          revenue=information_gain*info_multiplier-cost
          revenue_record.append(revenue)
          centroid_record.append(centroids[ip])
          id_record.append(ir)
        
    # if (len(centroid_record) != 0):
    #   rospy.loginfo("revenue record: "+str(revenue_record))	
    #   rospy.loginfo("centroid record: "+str(centroid_record))	
    #   rospy.loginfo("robot IDs record: "+str(id_record))	
        
#-------------------------------------------------------------------------	
    try:
      (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue
      
    # loading the ar_marker tf listener
    trans_ar = [0, 0, 0]
    try:
      (trans_ar, rot_ar) = listener.lookupTransform('odom', 'ar_marker/4000', rospy.Time(0))
      rospy.loginfo('Found Marker')
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue
    
    if (len(id_record)>0):
      winner_id=revenue_record.index(max(revenue_record))
      # if(trans_ar!=[0,0,0] and (robots[id_record[winner_id]].getPosition()[0] - trans_ar[0] > 0.2) and (robots[id_record[winner_id]].getPosition()[1] - trans_ar[1] > 0.2)):
      #   robots[id_record[winner_id]].sendGoal([t - 0.2 for t in trans_ar], False)
      #   rospy.loginfo(namespace +"  assigned to ar_transformation "+str(trans_ar))
      if((centroid_record[winner_id][0] - trans[0]) < 0.2 and (centroid_record[winner_id][1] - trans[1]) < 0.2):
        robots[id_record[winner_id]].sendGoal(centroid_record[winner_id],True)
      else:  
        robots[id_record[winner_id]].sendGoal(centroid_record[winner_id],False)
      rospy.loginfo(namespace +"  assigned to  "+str(centroid_record[winner_id]))
      rospy.sleep(delay_after_assignement)
#------------------------------------------------------------------------- 
    rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
  try:
    node()
  except rospy.ROSInterruptException:
    pass