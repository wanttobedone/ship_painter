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
  cv_bridge
  image_transport
  std_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
  sensor_msgs
  cv_bridge
  image_transport
  tf2
  tf2_geometry_msgs
  visualization_msgs
  tf2_ros
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
  sample_consensus
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

find_package(OpenCV REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
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
    sensor_msgs
    cv_bridge 
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
    OpenCV
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
    ${OpenCV_LIBRARIES} 
)
add_dependencies(trajectory_server_node ${PROJECT_NAME}_generate_messages_cpp)

# =============================================================================
# === Acados MPC Runner Node ===
# =============================================================================
# 说明: 此节点需要先运行 mpc_gen/generate_mpc.py 生成C代码
# 然后将 c_generated_code 复制到 src/mpc/ 目录

# Acados安装路径 (从环境变量或默认路径)
# 注意: CMake不会展开~，所以使用$ENV{HOME}
if(DEFINED ENV{ACADOS_SOURCE_DIR})
    # 如果设置了环境变量，使用它（但要处理~的情况）
    string(REPLACE "~" "$ENV{HOME}" ACADOS_SOURCE_DIR "$ENV{ACADOS_SOURCE_DIR}")
else()
    set(ACADOS_SOURCE_DIR "$ENV{HOME}/acados")
endif()

set(ACADOS_INCLUDE_DIR ${ACADOS_SOURCE_DIR}/include)
set(ACADOS_LIB_DIR ${ACADOS_SOURCE_DIR}/lib)

# MPC生成代码目录
set(MPC_GEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/mpc/c_generated_code)

# 检查Acados是否存在
if(EXISTS ${ACADOS_INCLUDE_DIR}/acados AND EXISTS ${MPC_GEN_DIR})
    message(STATUS "=== Acados MPC Configuration ===")
    message(STATUS "  Acados source: ${ACADOS_SOURCE_DIR}")
    message(STATUS "  MPC generated code: ${MPC_GEN_DIR}")

    # 查找MPC生成的源文件 (包括所有子目录)
    # 注意: 排除 main_*.c 文件，这些是Acados生成的测试文件
    file(GLOB MPC_GENERATED_SOURCES
        ${MPC_GEN_DIR}/acados_solver_*.c
        ${MPC_GEN_DIR}/acados_sim_solver_*.c
        ${MPC_GEN_DIR}/ship_painter_mpc_model/*.c
        ${MPC_GEN_DIR}/ship_painter_mpc_cost/*.c
    )

    if(MPC_GENERATED_SOURCES)
        message(STATUS "  Found MPC sources: ${MPC_GENERATED_SOURCES}")

        # 创建MPC Runner节点
        add_executable(mpc_runner_node
            src/mpc_runner_node.cpp
            ${MPC_GENERATED_SOURCES}
        )

        target_include_directories(mpc_runner_node PRIVATE
            ${ACADOS_INCLUDE_DIR}
            ${ACADOS_INCLUDE_DIR}/acados
            ${ACADOS_INCLUDE_DIR}/blasfeo/include
            ${ACADOS_INCLUDE_DIR}/hpipm/include
            ${MPC_GEN_DIR}
        )

        target_link_libraries(mpc_runner_node
            ${catkin_LIBRARIES}
            bspline
            Eigen3::Eigen
            ${ACADOS_LIB_DIR}/libacados.so
            ${ACADOS_LIB_DIR}/libhpipm.so
            ${ACADOS_LIB_DIR}/libblasfeo.so
            m  # math library
        )

        add_dependencies(mpc_runner_node ${PROJECT_NAME}_generate_messages_cpp)

        set_target_properties(mpc_runner_node PROPERTIES
            CXX_STANDARD 17
            CXX_STANDARD_REQUIRED ON
        )

        message(STATUS "  MPC Runner node configured successfully!")
    else()
        message(WARNING "MPC generated sources not found in ${MPC_GEN_DIR}")
        message(WARNING "Please run: cd mpc_gen && python3 generate_mpc.py")
        message(WARNING "Then copy: cp -r c_generated_code ../src/mpc/")
    endif()
else()
    message(STATUS "=== Acados MPC Not Configured ===")
    if(NOT EXISTS ${ACADOS_INCLUDE_DIR}/acados)
        message(STATUS "  Acados not found at: ${ACADOS_SOURCE_DIR}")
        message(STATUS "  To install Acados, see: https://docs.acados.org/installation/")
    endif()
    if(NOT EXISTS ${MPC_GEN_DIR})
        message(STATUS "  MPC generated code not found at: ${MPC_GEN_DIR}")
        message(STATUS "  Please run: cd mpc_gen && python3 generate_mpc.py")
    endif()
endif()
# =============================================================================

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

#后续删除
add_executable(depth_test_node src/depth_test_node.cpp)
target_link_libraries(depth_test_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

target_link_libraries(depth_test_node 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  )

  # 添加 hover_test_node 可执行文件
add_executable(hover_test_node src/hover_test_node.cpp)

# 链接库
target_link_libraries(hover_test_node
  ${catkin_LIBRARIES}
)

# === VINS to MAVROS Bridge Node ===
add_executable(vins_to_mavros_bridge src/vins_to_mavros_bridge.cpp)

# 链接库
target_link_libraries(vins_to_mavros_bridge
  ${catkin_LIBRARIES}
)

# === 安装Python脚本和Shell脚本 ===
# 安装auto_spawn_model.py等Python工具脚本
catkin_install_python(PROGRAMS
  scripts/auto_spawn_model.py
  scripts/set_px4_vision_params.py
  scripts/set_px4_vision_params.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)