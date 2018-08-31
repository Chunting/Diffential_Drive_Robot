# Diffential_Drive_Robot

Vehicle model for differential drive robot with trolleys. While generating the model we have considerd that we know the position(x,y) and  orientation of the vehicle using IMU. And initially all the trolleys have same orientation as vehicle.Position of the trolleys can be calculated using orientation of trolley and its distance with what it is been attached with i.e. AGV or trolley.

X'=d*math.cos(Θ)

Y'=d*math.sin(Θ)
