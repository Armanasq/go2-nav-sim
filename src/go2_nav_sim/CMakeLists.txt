cmake_minimum_required(VERSION 3.8)
project(go2_nav_sim)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config urdf worlds maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/lidar_odom.py scripts/gt_odom.py scripts/go_to_goal.py scripts/odom_eval.py
                 scripts/bag_metrics.py scripts/render_dashboard.py DESTINATION lib/${PROJECT_NAME})
ament_package()
