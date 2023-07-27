
add_compile_options(-std=c++11 -Wall)

#设置eigen
# set(EIGEN3_INCLUDE_DIR /usr/local/include/eigen3)
# include_directories(${EIGEN3_INCLUDE_DIR})

# set(OSQPEIGEN_INCLUDE_DIR /usr/local/include/OsqpEigen)
# include_directories(${OSQPEIGEN_INCLUDE_DIR})


find_package(Eigen3 REQUIRED)
#在系统中搜索并配置指定的软件包，如果找不到就会引发错误
#会自动找到头文件路径<NAME>INCLUDE_DIRS
#会自动找到库文件<NAME>LIBRARIES
find_package(OsqpEigen REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${OsqpEigen_INCLUDE_DIR}
)

#添加源文件
# aux_source_directory(. SRC_LIST)

#设置源文件变量
set(SRC_LIST
    ./common.cpp
    ./main.cpp
    ./reference_line/reference_line.cpp)

#添加头文件
include_directories(../include)

#添加reference_line下的头文件
include_directories(../include/reference_line)

add_executable(demo ${SRC_LIST})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# 将OSQpeigen链接到目标demo中
target_link_libraries(demo 
    ${EIGEN3_LIBRARIES}
    OsqpEigen::OsqpEigen)