# Install script for directory: /home/inphys/bliss_ws/src/bliss

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/inphys/bliss_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bliss/msg" TYPE FILE FILES
    "/home/inphys/bliss_ws/src/bliss/msg/Info.msg"
    "/home/inphys/bliss_ws/src/bliss/msg/KeyPoint.msg"
    "/home/inphys/bliss_ws/src/bliss/msg/GlobalDescriptor.msg"
    "/home/inphys/bliss_ws/src/bliss/msg/ScanDescriptor.msg"
    "/home/inphys/bliss_ws/src/bliss/msg/MapData.msg"
    "/home/inphys/bliss_ws/src/bliss/msg/MapGraph.msg"
    "/home/inphys/bliss_ws/src/bliss/msg/NodeData.msg"
    "/home/inphys/bliss_ws/src/bliss/msg/Link.msg"
    "/home/inphys/bliss_ws/src/bliss/msg/OdomInfo.msg"
    "/home/inphys/bliss_ws/src/bliss/msg/Point2f.msg"
    "/home/inphys/bliss_ws/src/bliss/msg/Point3f.msg"
    "/home/inphys/bliss_ws/src/bliss/msg/Goal.msg"
    "/home/inphys/bliss_ws/src/bliss/msg/RGBDImage.msg"
    "/home/inphys/bliss_ws/src/bliss/msg/RGBDImages.msg"
    "/home/inphys/bliss_ws/src/bliss/msg/UserData.msg"
    "/home/inphys/bliss_ws/src/bliss/msg/GPS.msg"
    "/home/inphys/bliss_ws/src/bliss/msg/Path.msg"
    "/home/inphys/bliss_ws/src/bliss/msg/EnvSensor.msg"
    "/home/inphys/bliss_ws/src/bliss/msg/CameraModel.msg"
    "/home/inphys/bliss_ws/src/bliss/msg/CameraModels.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bliss/cmake" TYPE FILE FILES "/home/inphys/bliss_ws/build/bliss/catkin_generated/installspace/bliss-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/inphys/bliss_ws/devel/include/bliss")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/inphys/bliss_ws/devel/share/roseus/ros/bliss")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/inphys/bliss_ws/devel/share/common-lisp/ros/bliss")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/inphys/bliss_ws/devel/share/gennodejs/ros/bliss")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/inphys/bliss_ws/devel/lib/python3/dist-packages/bliss")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/inphys/bliss_ws/devel/lib/python3/dist-packages/bliss")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/inphys/bliss_ws/build/bliss/catkin_generated/installspace/bliss.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bliss/cmake" TYPE FILE FILES "/home/inphys/bliss_ws/build/bliss/catkin_generated/installspace/bliss-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bliss/cmake" TYPE FILE FILES
    "/home/inphys/bliss_ws/build/bliss/catkin_generated/installspace/blissConfig.cmake"
    "/home/inphys/bliss_ws/build/bliss/catkin_generated/installspace/blissConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bliss" TYPE FILE FILES "/home/inphys/bliss_ws/src/bliss/package.xml")
endif()

