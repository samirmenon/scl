This script listens for alexa commands on a redis pipe and translates them into robot commands.
It's a work in progress.

========== MESSAGE FORMAT ==========
The voice server sends a key:
alexa-command <JSON message>
For instance:
redis-cli> set alexa-command '{"commands":"left left left"}'

========== SET UP REDIS SERVER TO LISTEN TO N/W IP ==========

To begin, we need to ensure that the redis server is accessible:
1. Stop it:
$ sudo /etc/init.d/redis-server stop
$ sudo /etc/init.d/redis-server start

Or go to the redis-cli and type 'shutdown'

2. Copy the config file to a local dir.
$ sudo cp /etc/redis/redis.conf .

3. Add your hard network IP to the bind command in the config file (search for it)
Replace :
  bind 127.0.0.1
With (say):
  bind 127.0.0.1 192.168.0.101

4. Re-start the redis server with the new config file..
$ sudo ./redis-server /path/to/redis.conf

5. Connect to the redis server over the network, which should now accept stuff on your ip.
