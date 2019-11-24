
import time

def elapse(seconds):
    start = time.time()
    time.clock()    
    elapsed = 20
    elapsed = time.time() - start

    while elapsed < seconds:
        elapsed = time.time() - start
#        print ("loop cycle time: %f, seconds count: %02d" % (time.clock() , elapsed)) 
        #print(elapsed)
          

#elapse(20)
