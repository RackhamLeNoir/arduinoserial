if(LINUX)
#  add_library(ArduinoSerial STATIC Serial.cpp SerialLinux.cpp)
elseif(APPLE)
  add_library(ArduinoSerial STATIC Serial.cpp SerialMac.cpp)
elseif (WIN32)
  add_library(ArduinoSerial STATIC Serial.cpp SerialWindows.cpp)
endif()

target_include_directories(ArduinoSerial INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
