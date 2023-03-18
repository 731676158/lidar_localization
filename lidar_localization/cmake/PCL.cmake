find_package(PCL 1.7 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${PCL_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES ${PCL_LIBRARIES})