# This helper explicitly finds and sets the include directories and libraries
# for some 3rd part dependencies that are not installed into /usr/local
# For packages with existing rules for find_package, we redefine to unify the
# include directories and libraries variables
#
# For library XXX, the include path should be XXX_INCLUDE_DIRS,
# and the library path should be XXX_LIBRARIES

# Eigen
# [NOTE] Eigen is header only
# include_dir: ${EIGEN_INCLUDE_DIR}
find_package(Eigen REQUIRED)

# YAML-CPP (use custom findYaml.cmake)
# include_dir: ${YAMLCPP_INCLUDE_DIRS}
# lib:         ${YAMLCPP_LIBRARIES}
find_package(Yaml REQUIRED)

# Glog (use custom findGlog.cmake)
# include_dir: ${GLOG_INCLUDE_DIRS}
# lib:         ${GLOG_LIBRARIES}
find_package(Glog REQUIRED)
