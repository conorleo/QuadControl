find_package(CasADi REQUIRED)

add_executable(multiple_shooting multipleShooting.cpp)

target_include_directories(multiple_shooting PRIVATE
    ${CMAKE_SOURCE_DIR}/Toolbox
)

target_link_libraries(multiple_shooting PRIVATE CasADi::casadi)

target_compile_features(multiple_shooting PRIVATE cxx_std_17)

if(MSVC)
  target_compile_options(multiple_shooting PRIVATE /W3)
else()
  target_compile_options(multiple_shooting PRIVATE -Wall -Wextra)
endif()
