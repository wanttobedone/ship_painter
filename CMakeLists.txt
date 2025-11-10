cmake_minimum_required(VERSION 3.10)
project(ship_painter)

# 设置编译选项
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 添加编译选项
add_compile_options(-fPIC -O3 -Wall)

# 禁用 CGAL 警告
add_definitions(-DCGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE)

# 查找catkin包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  mavros_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
)

# 查找PCL
find_package(PCL 1.8 REQUIRED COMPONENTS 
  common 
  io 
  filters 
  features 
  surface
  segmentation
  kdtree
)

# 添加消息依赖
add_message_files(
  FILES
  Bspline.msg
  BsplineLayer.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# 查找CGAL
find_package(CGAL REQUIRED COMPONENTS Core)
if(NOT CGAL_FOUND)
  message(FATAL_ERROR "CGAL library is required but not found")
endif()

# 查找Eigen
find_package(Eigen3 REQUIRED)
if(NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
endif()

# 查找Boost
find_package(Boost REQUIRED COMPONENTS system filesystem)

# === Clipper2 配置 ===
set(CLIPPER2_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Clipper2)
set(CLIPPER2_INCLUDE_DIR ${CLIPPER2_ROOT}/CPP/Clipper2Lib/include)
set(CLIPPER2_SRC_DIR ${CLIPPER2_ROOT}/CPP/Clipper2Lib/src)

# 检查Clipper2是否存在
if(NOT EXISTS ${CLIPPER2_INCLUDE_DIR}/clipper2)
    message(FATAL_ERROR "
    Clipper2 not found or incomplete!
    Expected directory: ${CLIPPER2_INCLUDE_DIR}/clipper2
    
    Please run:
    cd ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/
    rm -rf Clipper2
    git clone https://github.com/AngusJohnson/Clipper2.git
    ")
endif()

# 查找Clipper2源文件
set(CLIPPER2_REQUIRED_SOURCES
    ${CLIPPER2_SRC_DIR}/clipper.engine.cpp
    ${CLIPPER2_SRC_DIR}/clipper.offset.cpp
    ${CLIPPER2_SRC_DIR}/clipper.rectclip.cpp
)

# 检查必需的源文件是否存在
foreach(src_file ${CLIPPER2_REQUIRED_SOURCES})
    if(NOT EXISTS ${src_file})
        message(FATAL_ERROR "Required Clipper2 source file not found: ${src_file}")
    endif()
endforeach()

# 查找所有Clipper2源文件
file(GLOB CLIPPER2_ALL_SOURCES ${CLIPPER2_SRC_DIR}/*.cpp)

# 过滤掉可能的测试或示例文件
set(CLIPPER2_SOURCES)
foreach(src_file ${CLIPPER2_ALL_SOURCES})
    get_filename_component(filename ${src_file} NAME)
    # 排除可能的测试文件
    if(NOT filename MATCHES "(test|example|demo)")
        list(APPEND CLIPPER2_SOURCES ${src_file})
    endif()
endforeach()

message(STATUS "Found Clipper2 include dir: ${CLIPPER2_INCLUDE_DIR}")
message(STATUS "Found Clipper2 sources: ${CLIPPER2_SOURCES}")

# 添加B样条库
add_library(bspline src/bspline.cpp)
target_link_libraries(bspline 
  ${catkin_LIBRARIES} 
  Eigen3::Eigen)
target_include_directories(bspline PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 创建 Clipper2 目标
add_library(Clipper2_static STATIC ${CLIPPER2_SOURCES})

# 设置Clipper2目标属性
target_include_directories(Clipper2_static PUBLIC ${CLIPPER2_INCLUDE_DIR})

set_target_properties(Clipper2_static PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

# 为Clipper2添加编译选项
target_compile_options(Clipper2_static PRIVATE 
    -fPIC 
    -O3 
    -DCLIPPER2_LIB
    -Wno-unused-variable
    -Wno-unused-parameter
)

# === Catkin包配置 ===
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_planner
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    geometry_msgs 
    mavros_msgs 
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    pcl_ros
    pcl_conversions
  DEPENDS 
    PCL
    EIGEN3
    Boost
    CGAL
)

# === 包含目录 ===
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CLIPPER2_INCLUDE_DIR}
)

# 链接目录
link_directories(${PCL_LIBRARY_DIRS})

# 添加定义
add_definitions(${PCL_DEFINITIONS})

# === 构建path_planner库 ===
add_library(path_planner
  src/path_planner.cpp
)

# 设置path_planner属性
set_target_properties(path_planner PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

# 为path_planner添加编译选项
target_compile_options(path_planner PRIVATE -fPIC -O3)

# 链接path_planner
target_link_libraries(path_planner
  PUBLIC
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    CGAL::CGAL
    CGAL::CGAL_Core
    Eigen3::Eigen
    bspline
  PRIVATE
    Clipper2_static
)

# 添加trajectory_server节点
add_executable(trajectory_server_node src/trajectory_server_node.cpp)
target_link_libraries(trajectory_server_node 
    bspline
    ${catkin_LIBRARIES}
)
add_dependencies(trajectory_server_node ${PROJECT_NAME}_generate_messages_cpp)

# === 构建主节点 ===
add_executable(ship_painter_node 
  src/ship_painter_node.cpp
)

# 设置主节点属性
set_target_properties(ship_painter_node PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_compile_options(ship_painter_node PRIVATE -fPIC -O3)

target_link_libraries(ship_painter_node
  path_planner
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

# === 安装规则 ===
install(TARGETS path_planner ship_painter_node Clipper2_static
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  FILES_MATCHING PATTERN "*.world"
)