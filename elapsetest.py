
import time
import Adafruit_PCA9685 as ada
pwm= ada.PCA9685(address=0x40, busnum=4)

def elapse(seconds):
    start = time.time()
    time.clock()    
    elapsed = 20
    elapsed = time.time() - start

    while elapsed < seconds:
        elapsed = time.time() - start
        
#        print ("loop cycle time: %f, seconds count: %02d" % (time.clock() , elapsed)) 
        print(elapsed) 
    
    pwm.set_pwm(3, 0, 150)      
    pwm.set_pwm(13, 0, 175)


#elapse(10)
#pwm.set_pwm(3, 0, 150)      
#pwm.set_pwm(13, 0, 175)
