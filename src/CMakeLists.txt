
# to interpolate thrust values
find_package(libInterpolate REQUIRED)

add_executable(tvmc 
    motion-controller/controller.cpp
    reporters/thrust.cpp
    thruster-config/thruster_config.cpp
    thruster-config/json.hpp
    thruster-config/csv.h
    PID-Controller/PID_controller.cpp
)

target_link_libraries(tvmc ${catkin_LIBRARIES})

# target_link_libraries(motion_controller PRIVATE ${catkin_LIBRARIES} libInterpolate::Interpolate)