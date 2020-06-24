#####################################################################################
##Diploma Thesis - Applications of Quaternions in Robot Kinematics by Diana Doctor ##
#####################################################################################
import numpy as np
from numpy.linalg import multi_dot
import math
import pyquaternion
from pyquaternion import Quaternion

import socket
import time
import urx
from urx import urrobot
from urx import RG2Gripper as GripClass

# Set variables
pi = math.pi
cos = math.cos
sin = math.sin
sqrt = math.sqrt
acos = math.acos
asin = math.asin
atan2 = math.atan2

# Initialize Program
print ("Starting AQRKx Program...")
rob = urx.Robot("147.229.132.249")
time.sleep(2)
gripperInstance = GripClass.RG2(rob)
time.sleep(2)

# UR3 Denavit-Harteberg Parameters
d = [0.1519,0,0,0.11235,0.08535,0.0819]
a = [0,0,-0.24365,-0.21325,0,0]
alpha = [0,pi/2,0,0,pi/2,-pi/2]

# Set TCP initial position in space based on the corresponding frame
TCP_endf = np.array([0,0,0.2]) # based on end frame (0,0,1)
TCP_basef = np.array([0,-0.2,0]) # based on base frame (0,-1,0)  

###################### Dual Quaternion Class ############################
class DualQuaternion(object):
   # Initializes a Dual Quaternion
   def __init__(self, q=Quaternion(), p=0):
      self.q = q
      self.p = p

   # Dual Quaternion Multiplication
   def mult(self, dq):
      # For the rotation part
      q1 = self.q
      q2 = dq.q
      
      # For the translation part
      p1 = self.p
      p2 = dq.p
      
      return DualQuaternion(q1*q2, q1*p2*q1.conjugate+p1)

   # Checks equality of Dual Quaternions
   def equals(self,dq):
      # For the rotation part
      q1 = self.q.vector
      q2 = dq.q.vector
      
      # For the translation part
      p1 = self.p.vector
      p2 = dq.p.vector
      
      if((np.allclose(p1,p2,atol=1e-03)
          or np.allclose(p2,p1,atol=1e-03))
         and (np.allclose(q1,q2,atol=1e-03)
              or np.allclose(q2,q1,atol=1e-03))):
         return True
      return False

   # Transforms a point in the space
   def transform(self, pose):
      # Rotated Pose
      rotPose = self.q.rotate(pose)
      print("\nAfter Rotation:",rotPose)

      # Translated Pose
      transPose = self.p.vector + rotPose
      print("After Translation:",transPose)
      
      return transPose

######################### Forward Kinematics ############################
# Records the current pose to a file
def record_pose(fileName = "UR3poses.txt"):
   # Creates or Open File
   filePath = "C:\\Users\\student\\Desktop\\UR3\\"+fileName
   pfile = open(filePath,"a+");
   
   # Gets the current joint angles
   jangles = rob.getj()
   # Gets FK for a pose given the angles by Axis-Angle Representation
   desired_pose = get_forward_kinematics("AA",jangles)

   # Input Confirmation
   rec = input("\nRecord this pose?(Y) ");
   if(rec!="Y" and rec!="y"):
      print("\nCancel recording...")
      return

   grip = input("\nUse Gripper?(Y) ");
   if(grip=="Y" or grip=="y"):
      try:
          gripValue = int(input("\nSet Gripper value (in mm):"))
      except:
          print("Entered value is not an integer. Aborting.");
          return
   else:
      gripValue = ""
       
   # Transforms desired pose elements to array and to string
   arr_desired_pose = [desired_pose.q[0],desired_pose.q[1],desired_pose.q[2],desired_pose.q[3],desired_pose.p[0],desired_pose.p[1],desired_pose.p[2],desired_pose.p[3],gripValue]
   str_desired_pose = " ".join(str(d) for d in arr_desired_pose)+"\n"
   
   # Writes to the file
   pfile.write(str_desired_pose)
   pfile.close()
   print("\nAdded to the list of recorded poses:\n",arr_desired_pose,end='')


# Gets the current pose using Forward Kinematics
def get_forward_kinematics(ans,jangles):
   
   print("\nCurrent Joint Angles:", jangles)

   # Derives the corresponding Dual Quaternions for each joints from their Angle-Axis Representation
   if(ans == "AA"):
      print("\nDeriving Forward Kinematics from Angle-Axis Representation...")
      DualQuatList = derive_dual_quat_from_AA(jangles)
      initial_pose = TCP_basef
   # Derives the corresponding Dual Quaternions for each joints from their Transformation Matrices
   elif (ans == "TM"):
      print("\nDeriving Forward Kinematics from Transformation Matrix...")
      DualQuatList = derive_Dual_Quaternion_from_TM(jangles)
      initial_pose = TCP_endf
   else:
      print("Wrong Input!")
      return
   
   # Computes the Final Dual Quaternion by multiplying all the Dual Quaternions
   FKDualQuat = DualQuaternion()
   for dq in DualQuatList:
      FKDualQuat = FKDualQuat.mult(dq)
         
   print("\nForward Kinematics (by Dual Quaternion):\n([", FKDualQuat.q,"],<", FKDualQuat.p,">)")

   # Transform initial pose by Quaternion-Vector Multiplication
   print("\nInitial TCP Coordinates (at zero pose):", initial_pose)
   curr_pose = FKDualQuat.transform(initial_pose)
   print("\nTransformed TCP Coordinates:\n", np.round(curr_pose,3))

   return FKDualQuat

   
# Derive Dual Quaternion from Axis and Angle of Rotation
def derive_dual_quat_from_AA(jangles):
   # Gets joint angles from current pose
   theta = jangles

   # Set translation for each joints
   trans = ([[0, 0, d[0]],                                     # P1
            [a[2]*cos(theta[1]), 0, a[2]*sin(theta[1])],       # P2
            [a[3]*cos(theta[2]), -d[3], a[3]*sin(theta[2])],   # P3
            [d[4]*sin(theta[3]), 0, -d[4]*cos(theta[3])],      # P4
            [-d[5]*sin(theta[4]), -d[5]*cos(theta[4]), 0],     # P5
            [0,0,0]])                                          # P6

   # Builds the Dual Quaternions
   DQList = []
   alp = 0
   
   for i in range(6):
      # Builds the dual quaternion
      qscalar = cos(theta[i]/2)
      alp += alpha[i]
      qvector = np.multiply(sin(theta[i]/2), [0, -sin(alp), cos(alp)])
      quatRot = np.concatenate(([qscalar],qvector))
      
      DualQuat = DualQuaternion(Quaternion(quatRot),Quaternion(vector=trans[i]))
      print("Dual Quaternion:([",DualQuat.q,"],<",DualQuat.p,">)")
      
      # Store the Dual Quaternion to the list
      DQList.append(DualQuat)
   
   return DQList


# Derive Dual Quaternion from Transformation Matrix
def derive_Dual_Quaternion_from_TM(jangles):
   print("Computing Transformation Matrix by Denavit-Hartenberg Parameters...")
   # Gets joint angles from current pose
   theta = jangles

   # Gets transformation matrices for each 6 frames
   T = np.zeros(shape=(6,4,4))

   # Builds the Dual Quaternions
   DQList = []
   
   for i in range(6):
      T[i] = ([[cos(theta[i]), -sin(theta[i]), 0, a[i]],
               [sin(theta[i])*cos(alpha[i]), cos(theta[i])*cos(alpha[i]), -sin(alpha[i]), -sin(alpha[i])*d[i]],
               [sin(theta[i])*sin(alpha[i]), cos(theta[i])*sin(alpha[i]), cos(alpha[i]), cos(alpha[i])*d[i]],
               [0,0,0,1]])
   
      print("\nTransformation Matrix for Frame",i,"to", i+1,"\n",T[i])

      # Derives the Dual Quaternion from the Transformation Matrix
      DualQuat = DualQuaternion(Quaternion(matrix = T[i]),Quaternion(vector = T[i][0:3,3]))
      print("Dual Quaternion:([",DualQuat.q,"],<",DualQuat.p,">)")
      
      # Store the Dual Quaternion to the list
      DQList.append(DualQuat)
   
   return DQList

####################### Inverse Kinematics ##############################
# Runs All Recorded Poses using the Inverse Kinematics
def run_poses(fileName = "UR3poses.txt"):
   # Checks if a file exists and opens it
   try:
      filePath = "C:\\Users\\student\\Desktop\\UR3\\"+fileName
      pfile = open(filePath,"r");
   except FileNotFoundError:
      print("File for recorded poses is not found!");
      return

   # Reads each lines(poses) from file
   poses = pfile.readlines()
   for pose in poses:
      # Takes the dual quaternion coefficients and create the dual quaternion
      dq_coeff = [float(coeff) for coeff in pose.split()]
      move_robot(dq_coeff)


# Moves UR3 from Given Dual Quaternion Transformation
def move_robot(dq_coeff):
      # Takes the dual quaternion coefficients and create the dual quaternion
      DQ_desired_pose = DualQuaternion(Quaternion(array=dq_coeff[0:4]),Quaternion(array=dq_coeff[4:8]))

      # Applies Inverse Kinematics to derive the joint angles
      joint_angles = get_inverse_kinematics(DQ_desired_pose)

      print("\nThere are ", len(joint_angles) ," possible pose(s)!");

      if len(joint_angles) == 0:
         print("Movement cannot be done!!")
         time.sleep(3)
         return
         
      # Moves the robot given the joint angles
      rob.movej(joint_angles[0], acc=1, vel=1, wait=False)
      time.sleep(7) # Can be set depending on duration of movements

      # Sets the gripper
      try:
         gripValue = dq_coeff[8]
         if dq_coeff[8] is not None:
             gripperInstance.setWidth(gripValue)
             time.sleep(3)
      except:
         print("No gripper value.");

         
# Determines the Joint Angles for Inverse Kinematics
def get_inverse_kinematics(desired_pose):

   print("\nDesired Pose:",desired_pose.q, desired_pose.p);
   
   # Desired Transformation from Frame 1 to 6
   DQ16 = desired_pose
   # Rotation Part
   w6 = DQ16.q[0]
   a6 = DQ16.q[1]
   b6 = DQ16.q[2]
   c6 = DQ16.q[3]
   # Translation Part
   x6 = DQ16.p[1];
   y6 = DQ16.p[2];
   z6 = DQ16.p[3];

   # Remove Translation from Frame 5
   DQ5i = DualQuaternion(p=Quaternion(vector=[0,d[5],0]))
   DQ15p = DQ16.mult(DQ5i)
   x5 = DQ15p.p[1]
   y5 = DQ15p.p[2]
   z5 = DQ15p.p[3]
   
   # Compute possible values for Theta 1
   theta1=[]
   t11 = atan2(-x5,y5)+atan2(sqrt(x5**2+y5**2-d[3]**2),-d[3])
   t12 = atan2(-x5,y5)-atan2(sqrt(x5**2+y5**2-d[3]**2),-d[3])

   # (Base) Theta 1: 2 Values
   theta1.append(t11)
   theta1.append(t12)
   
   # Compute possible values for Theta 5
   theta15 = []
   for t1 in theta1: # For each values of Theta 1
      # Evaluate expression for Theta 5
      exp5 = (x6*sin(t1)-y6*cos(t1)-d[3])/d[5]
      
      # Checks if within the domain for arccos
      if (exp5<=1 and exp5>=-1):
         t5 = acos(exp5)
      elif (round(exp5)<=1 and round(exp5)>=-1):
         t5 = acos(round(exp5))
      else: continue
      
      # (Wrist 2) Theta 5: 2 Values
      theta15.append([t1,t5])
      theta15.append([t1,-t5])
   
   # Compute possible values for Theta 6
   theta156 = []
   for t1t5 in theta15:
      t1 = t1t5[0]
      t5 = t1t5[1]

      # Avoids singularity for Theta 6
      if(t5==0):
         t156 = [t1,t5,0]
         if t156 not in theta156:
            theta156.append(t156)
         continue

      # Evaluate expression for Theta 6
      tm1 = ((w6*cos(t1/2)+c6*sin(t1/2))/cos(t5/2))
      tm2 = ((a6*cos(t1/2)+b6*sin(t1/2))/sin(t5/2))
      tm3 = ((a6*sin(t1/2)-b6*cos(t1/2))/cos(t5/2))
      tm4 = ((w6*sin(t1/2)-c6*cos(t1/2))/sin(t5/2))

      # (Wrist 3) Theta 6: 1 Value
      t6 = 2*atan2(tm3-tm2,tm1+tm4)
      theta156.append([t1,t5,t6])
   
   # Derive transformation from Frame 2 to Frame 4
   DQ24f = []
   for t156 in theta156:
      t1 = t156[0]
      t5 = t156[1]
      t6 = t156[2]
      
      # Joint 1 Dual Quaternion Inverse using Theta 1
      q1 = np.concatenate(([cos(t1/2)], np.multiply(sin(t1/2), [0, 0, -1])))
      DQ1i = DualQuaternion(Quaternion(q1),Quaternion(vector=[0,0,-d[0]]))
      
      q5 = np.concatenate(([cos(t5/2)], np.multiply(sin(t5/2), [0, 0, 1])))
      DQ5i = DualQuaternion(Quaternion(q5),Quaternion(vector=[0,d[5],0]))

      q6 = np.concatenate(([cos(t6/2)], np.multiply(sin(t6/2), [0, 1, 0])))
      DQ6i = DualQuaternion(Quaternion(q6),Quaternion(vector=[0,0,0]))

      # Derives Transformation from Frame 2 to Frame 6
      DQ26 = DQ1i.mult(DQ16)

      # Derives Transformation From Frame 2 to Frame 5
      DQ25 = DQ26.mult(DQ6i)

      # Derives Transformation from Frame 2 to Frame 4
      DQ24 = DQ25.mult(DQ5i)

      # Remove Translation from Frame 4 - Adds to List
      DQ4i = DualQuaternion(p=Quaternion(vector=[0,0,d[4]]))
      DQ24f.append([DQ24.mult(DQ4i),t156])
   
   # Compute possible values for Theta 3 
   theta1356q=[]
   for dq in DQ24f:
      w4 = dq[0].q[0]
      b4 = dq[0].q[2]
      x4 = dq[0].p[1]
      y4 = dq[0].p[2]
      z4 = dq[0].p[3]
      q4 = [w4,b4,x4,y4,z4]

      t1 = dq[1][0]
      t5 = dq[1][1]
      t6 = dq[1][2]
      
      # Evaluate expression for Theta 3
      exp3 = ((x4**2+z4**2)-a[3]**2-a[2]**2)/(2*a[2]*a[3])
      
      # Checks if within the domain for arccos
      if (exp3<=1 and exp3>=-1):
         t3 = acos(exp3)
      elif (round(exp3)<=1 and round(exp3)>=-1):
         t3 = acos(round(exp3))
      else: continue

      # (Elbow) Theta 3: 2 Values
      theta1356q.append([t1,t3,t5,t6,q4])
      theta1356q.append([t1,-t3,t5,t6,q4])
   
   # Compute possible values for Theta 2
   theta12356q=[]
   for t1356q in theta1356q:
      t1 = t1356q[0]
      t3 = t1356q[1]
      t5 = t1356q[2]
      t6 = t1356q[3]
      x4 = t1356q[4][2]
      z4 = t1356q[4][4]

      # Evaluate expression for Theta 2
      t2 = atan2((z4*(a[2]+a[3]*cos(t3))-x4*a[3]*sin(t3)),(x4*(a[2]+a[3]*cos(t3))+z4*a[3]*sin(t3)))

      # (Shoulder) Theta 2: 1 Value
      theta12356q.append([t1,t2,t3,t5,t6,t1356q[4]])
   
   # Compute possible values for Theta 4
   theta123456=[]
   for t12356q in theta12356q:
      t1 = t12356q[0]
      t2 = t12356q[1]
      t3 = t12356q[2]
      t5 = t12356q[3]
      t6 = t12356q[4]
      w4 = t12356q[5][0]
      b4 = t12356q[5][1]

      # Evaluate expression for Theta 4
      t4 = -2*atan2((w4*sin((t2+t3)/2))+(b4*cos((t2+t3)/2)),(w4*cos((t2+t3)/2))-(b4*sin((t2+t3)/2)))
  
      # (Wrist 1) Theta 4: 1 Value
      theta123456.append([t1,t2,t3,t4,t5,t6])

   # Prints all possible joint angles
   print("\nSet of Possible Joint Angles:\n",theta123456)
                         
   # Get the corresponding Final Dual Quaternion based on the combination of angles
   jangles=[]
   for theta in theta123456:
      FinalDQ = get_forward_kinematics("AA",theta)

      print("\nComparing Dual Quaternions: ")
      print("Desired Pose:\n([", DQ16.q,"],<", DQ16.p,">)")
      print("Current Pose:\n([", FinalDQ.q,"],<", FinalDQ.p,">)")

      if(DQ16.equals(FinalDQ)):
         print("PASSED criteria!")
         jangles.append(theta)
      else: print("FAILED criteria!")

   print("\nFinal Joint Angles:",jangles)

   return jangles

######################## Clear File ################################
# Clear the Stored Poses in a File
def clear_poses(fileName = "UR3poses.txt"):
   # Checks if a file exists and opens it
   try:
      filePath = "C:\\Users\\student\\Desktop\\UR3\\"+fileName
      pfile = open(filePath,"w").close();
   except FileNotFoundError:
      print("File for recorded poses is NOT found!");
      return

#####################################################################################
################   End of Program - Diana Doctor - AQRKx.py   #######################
#####################################################################################  
