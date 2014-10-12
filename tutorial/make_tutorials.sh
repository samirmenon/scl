#!/bin/bash
cd scl_tutorial0_setup_robot
make release -j
cd ..
cd scl_tutorial1_xml_kinematics_dynamics
make release -j
cd ..
cd scl_tutorial2_physics_integration
make release -j
cd ..
cd scl_tutorial3_graphics_physics_cmake
sh make_dbg.sh
sh make_rel.sh
cd ..
cd scl_tutorial4_control_gc_op
sh make_dbg.sh
sh make_rel.sh
cd ..
cd scl_tutorial5_control_multi_task
sh make_dbg.sh
sh make_rel.sh
cd ..
cd scl_tutorial6_control_humanoid
sh make_dbg.sh
sh make_rel.sh
cd ..
cd scl_advanced_creating_new_tasks
sh make_dbg.sh
sh make_rel.sh
cd ..
