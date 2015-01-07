This is an example app to test the spatial dynamics integrator
implementation in scl with the puma. The app also connects to
a standard redis database and streams serialized object values
there.

Code file : scl_redis_serv_puma.cpp

1. Compile with the CMake system
$ sh make_dbg.sh
or 
$ sh make_rel.sh

2. Run
$ ./scl_redis_serv_puma

3. (Optionally) Run the scl puma web server to view the same Puma..

Use the mouse to move the camere around:
rotate camera : l-click + drag
zoom camera : r-click + drag
move camera : l-click+ctrl+drag
