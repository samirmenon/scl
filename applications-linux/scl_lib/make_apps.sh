#!/bin/bash

# ****************************
# Compile all the applications
# ****************************
cd ../scl_2rob_task_ctrl
sh make_rel.sh
sh make_dbg.sh

cd ../scl_dynamics
sh make_rel.sh
sh make_dbg.sh

cd ../scl_example_ctrl
sh make_rel.sh
sh make_dbg.sh

cd ../scl_gc_ctrl
sh make_rel.sh
sh make_dbg.sh

cd ../scl_graphics
sh make_rel.sh
sh make_dbg.sh

cd ../scl_haptic_ctrl
sh make_rel.sh
sh make_dbg.sh

cd ../scl_redis_ctrl
sh make_rel.sh
sh make_dbg.sh

cd ../scl_redis_sim
sh make_rel.sh
sh make_dbg.sh

cd ../scl_redis_visualizer
sh make_rel.sh
sh make_dbg.sh

cd ../scl_RParRR_ctrl
sh make_rel.sh
sh make_dbg.sh

cd ../scl_task_ctrl
sh make_rel.sh
sh make_dbg.sh

cd ../scl_test
sh make_rel.sh
sh make_dbg.sh

cd ../scl_lib


