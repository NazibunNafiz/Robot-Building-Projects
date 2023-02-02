from controller import Robot
from controller import Camera

TIME_STEP = 32
max_speed=2.0

last_error=I=D=P=P_I=error=0
av=sum=error_I=0
pos_left=pos_right=pos_middle=0


ki=0.01
kp=2.0
kd=0.2

robot = Robot()
ds = []
dsNames = ['ds_right','ds_middle', 'ds_left']
for i in range(3):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(TIME_STEP)
    
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
    
cam=Camera("CAM")
cam.enable (TIME_STEP);
cam.recognitionEnable(TIME_STEP)


def PID(error):
    global P,I,D,last_error,error_I           
    P=error
    I=error+I
    D=error-last_error
    balance=((kp*P)+(ki*I)+(kd*D))
    last_error=error
    
    
    return balance
 
def speed(left_Speed,right_Speed):
    
     wheels[0].setVelocity(left_Speed)
     wheels[1].setVelocity(right_Speed)
     wheels[2].setVelocity(left_Speed)
     wheels[3].setVelocity(right_Speed)
     
    
while robot.step(TIME_STEP) != -1:
   
    right_ir_val=ds[0].getValue()
    mid_ir_value=ds[1].getValue()
    left_ir_value=ds[2].getValue()
    
    
    x=505
    
    if left_ir_value < x and right_ir_val < x  and mid_ir_value >= x :
        pos_middle=1
        pos_left=pos_right=0
        sum=1

    if left_ir_value < x  and right_ir_val >= x  and mid_ir_value >= x :
        pos_middle=pos_right=1
        pos_left=0
        sum=2

    if left_ir_value >= x  and right_ir_val < x  and mid_ir_value >= x :
        pos_middle=pos_left=1
        pos_right=0
        sum=2

    if left_ir_value >= x   and right_ir_val < x  and mid_ir_value < x :
        pos_right=1
        pos_left=pos_middle=0
        sum=1

    if left_ir_value < x  and right_ir_val >= x  and mid_ir_value < x :
        pos_left=1
        pos_middle=pos_right=0
        sum=1

    av =(pos_left*(-1))+(pos_right*1)+(pos_middle*0)
    
        
    error=av/sum
      
    
    speed_1=PID(error)
    
    left_Speed=max_speed-speed_1
    
    right_Speed=max_speed+speed_1
    
    
    if left_Speed> max_speed : 
        speed(max_speed,right_Speed) 
        
    if right_Speed> max_speed :
        speed(left_Speed,max_speed)  
        
    if left_Speed < 0:
        speed(0,right_Speed)

    if right_Speed < 0:  
        speed(left_Speed,0)
        