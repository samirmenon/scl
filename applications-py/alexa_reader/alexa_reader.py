# A script to set up a redis message parser from alexa to scl.
#
# For now, we're focusing on moving the kinova jaco. The script could serve as a
# template for a more general robot-agnostic solution.
# To begin, we'll support front/back left/right commands to move the robot in a
# plane
#
# Created on : July 19th, 2016
import redis
import json
import time
import signal
import sys

# Set up a handler to report ctrl+c and gracefully exit the program.
def signal_handler(signal, frame):
    print('You pressed Ctrl+C! Will now exit...');
    print("SCL-py : We're finished here.");
    sys.exit(0);

# Attach the signal handler to the ctrl+c command
signal.signal(signal.SIGINT, signal_handler)
print('Starting SCL-py reader. Press ctrl+c to exit..');

# Returns a trajectory increment for a given alexa command
def alexa_cmd_to_Dtraj(x):
    dval = 0.06;
    return {
        'front': [0, -dval, 0],
        'back': [0, dval, 0],
        'right': [dval, 0, 0],
        'left': [-dval, 0, 0],
        'up': [0, 0, dval],
        'down': [0, 0, -dval],
        '' : [0, 0, 0]
    }[x]

# Specify the string to be used for sending a traj
str_redis_traj = 'scl::robot::kinovajaco6::traj::xgoal';

# Connect to the redis server
ip_redis_serv = 'localhost';#'192.168.0.101';
r = redis.StrictRedis(host=ip_redis_serv, port=6379, db=0)
print("SCL-py : Connected to the redis server on ip: "+ip_redis_serv);

# Connect to the redis server
r2 = redis.StrictRedis(host='localhost', port=6379, db=0)
print("SCL-py : Connected to the redis server on ip: localhost");

# Enable torque control on the kinova
flag = r.ping();
print("SCL-py : Pinged the redis server. Worked :" + str(flag));

# Starting alexa reader. Flushing alexa queue.
print("Starting alexa reader. Flushing alexa queue.");
alexa_msg = r.set('alexa::from','');

while 1:
    # *********** Get the alexa command ***********
    alexa_msg = r.get('alexa::from');
    if len(alexa_msg)<1:
        #print("SCL-py : Waiting (0.1s) for alexa command...");
        time.sleep(0.1); # Wait for a command if there isn't one..
        continue;
    # Remove the key now that we have it and have found it to be valid.
    r.set('alexa::from','');
    # Expect a json value in return. Something like:
    #  print(json.dumps({commands':'left left left'}))
    print("SCL-py : Received message:");
    print(alexa_msg);

    # *********** Read the scl traj position ***********
    traj_xpos = [];
    tmp_l = r2.get(str_redis_traj).split(" ");
    traj_xpos.append(float(tmp_l[0]));
    traj_xpos.append(float(tmp_l[1]));
    traj_xpos.append(float(tmp_l[2]));
    print("SCL-py : Present scl traj is :");
    print(traj_xpos);

    # *********** Parse the alexa message and update traj ***********
    alexa_cmd_json = json.loads(alexa_msg);

    alexa_cmd_strlist = alexa_cmd_json.get('commands').split(" ")
    print("SCL-py : Number of commands received : "+str(len(alexa_cmd_strlist)))

    # Iterate over the commands
    ii=1;
    for cmd in alexa_cmd_strlist:
        print("SCL-py :Command("+str(ii)+") : "+cmd); ii+=1;
        dtraj = alexa_cmd_to_Dtraj(cmd);
        traj_xpos[0]+=dtraj[0];
        traj_xpos[1]+=dtraj[1];
        traj_xpos[2]+=dtraj[2];
    print("SCL-py : Setting robot goal position to:");
    print(traj_xpos);
    tmp_s = str(traj_xpos[0])+" "+str(traj_xpos[1])+" "+str(traj_xpos[2]);
    r2.set(str_redis_traj,tmp_s);
