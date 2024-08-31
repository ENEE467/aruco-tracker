include_directories(${GLAD_INCLUDE_DIR})

add_library(app-framework
  App.h
  App.cpp
  Shader.h
  Native.h
  Native.cpp
  Image.h
  Fonts/Fonts.h
  Fonts/FontAwesome5.cpp
  Fonts/RobotoRegular.cpp
  Fonts/RobotoBold.cpp
  Fonts/RobotoItalic.cpp
  Fonts/RobotoMonoRegular.cpp
  Fonts/RobotoMonoBold.cpp
  Fonts/RobotoMonoItalic.cpp
)

set(APP_FRAMEWORK_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/Fonts
  PARENT_SCOPE
)

target_link_libraries(app-framework glad implot nfd)

target_include_directories(app-framework PUBLIC
  "${APP_FRAMEWORK_INCLUDE_DIRS}"
)

target_compile_features(app-framework PRIVATE cxx_std_17)
