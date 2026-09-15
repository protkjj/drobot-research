cmake_minimum_required(VERSION 3.8)
project(drobot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install non-code assets into the package share directory
#
# 존재하는 디렉터리만 설치한다. 없는 것을 그냥 나열하면
# ament_cmake_symlink_install_directory() 가 "can't find ..." 로 빌드를 세운다.
#
# 왜 없을 수 있나: models/ 와 worlds/original/ 은 Git LFS 자산인데
# GitHub 에서 실제 데이터가 사라졌다(404). 포인터 파일만 남아 있거나
# 아예 삭제된 머신이 있다. 이들은 hospital 월드용이고 벤치마크 맵
# (worlds/benchmark/)에는 쓰이지 않으므로 없어도 실험이 돌아간다.
set(_asset_dirs urdf meshes scripts config object worlds models)
set(_present_dirs "")
foreach(_d ${_asset_dirs})
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_d}")
    list(APPEND _present_dirs ${_d})
  else()
    message(STATUS "drobot_description: '${_d}' 없음 — 설치에서 건너뜀")
  endif()
endforeach()

install(
  DIRECTORY ${_present_dirs}
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
