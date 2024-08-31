add_executable(application
  application.cpp
)

target_include_directories(application PUBLIC
  "${APP_FRAMEWORK_INCLUDE_DIRS}"
  "${CORE_INCLUDE_DIRS}"
)

target_link_libraries(application PUBLIC core app-framework)
set_target_properties(application PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
