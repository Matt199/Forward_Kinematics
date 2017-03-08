
#Program on licence of MIT
#original code:
#http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/
#model value copied from: 
#http://www.ohio.edu/people/williar4/html/pdf/DeltaKin.pdf


import math 



# Defining the model value 

Sb = 567.0		# Base equilateral triangle side 

Sp = 76.0			#Platform equilateral triangle side 

L = 524.0			#Upper legs length

l = 1244.0		#Lower legs parallelogram length 

h = 131.0			#lower legs parallelogram width 

Wb = 164.0		#planar distance from {0} to near base side

Ub = 327.0		#planar distance from {0} to a base vertex

Wp = 22.0			#planar distance from {P} to near platform side

Up = 44.0			#planar distance from {P} to a platform vertex



print "Podaj wartosci kata thetha"

t1 = raw_input("thetha 1")
t2 = raw_input("thetha 2")
t3 = raw_input("thetha 3")

t1 = float(t1)
t2 = float(t2)
t3 = float(t3)


def forwardKinematics(thetha1, thetha2, thetha3):
	x0 = 0.0
	y0 = 0.0
	z0 = 0.0
	
	y1 = -Wb - L * math.cos(thetha1) + Up
	z1 = -L * math.sin(thetha1)
	
	x2 = (math.sqrt(3)/2.0) * (Wb + L * math.cos(thetha2)) - Sp/2.0
	y2 = 0.5 * (Wb + L * math.cos(thetha2)) - Wp
	z2 = -L * math.sin(thetha2)
	
	x3 = (math.sqrt(3)/2.0) * (Wb + L * math.cos(thetha3)) + Sp/2.0
	y3 = 0.5 * (Wb + L * math.cos(thetha3)) - Wp
	z3 = -L * math.sin(thetha3)
	
	# Finding the coordinates of center of the sphere
	
	dnm = (y2 - y1)  * x3 - (y3 - y1) * x2
	
	w1 = math.pow(y1,2) + math.pow(z1,2)
	w2 = math.pow(x2,2) + math.pow(y2,2) + math.pow(z2,2)
	w3 = math.pow(x3,2) + math.pow(y3,2) + math.pow(z3,2)
	
	
	a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
	b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) /2.0 
	
	a2 = -(z2-z1)*x3 + (z3-z1)*x2 
	b2 = ( (w2-w1)*x3 - (w3-w1)*x2) / 2.0  
	
	a = a1 * a1 + a2 * a2 + math.pow(dnm,2)
	b = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * math.pow(dnm,2))  
	c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + math.pow(dnm,2) * (z1 * z1 - L * L)
	
	d = math.pow(b,2) - 4.0 * a * c
    
	if(d < 0):
		print "Konfiguracja niemozliwa"

		return -1
	
	x0 = -0.5 * (b + math.sqrt(d)) / a
	y0 = (a1*z0 + b1) / dnm
	z0 = (a2*z0 + b2) / dnm
	
	return[x0,y0,z0]


print forwardKinematics(t1,t2,t3)


	
	
	
	
	
	
	
 	
