#--------------------------------------------------------
# The CMakeLists.txt for:                           iM1_8
# Author(s):                                  Tyler Errico                             
#--------------------------------------------------------

SET(SRC
  M1_8.cpp
  M1_8_Info.cpp
  Thruster.cpp
  main.cpp
  VehRotController.cpp
)

ADD_EXECUTABLE(iM1_8 ${SRC})

TARGET_LINK_LIBRARIES(iM1_8
  ${MOOS_LIBRARIES}
  ${MOOSGeodesy_LIBRARIES}
  apputil
  sock_util
  mbutil
  m
  pthread)

