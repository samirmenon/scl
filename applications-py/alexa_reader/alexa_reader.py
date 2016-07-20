# A script to set up the kinova redis keys.
import redis

# Connect to the redis server
r = redis.StrictRedis(host='192.168.0.101', port=6379, db=0)

# Enable torque control on the kinova
r.set('bobo', '10');
