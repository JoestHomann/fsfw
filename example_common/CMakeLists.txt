add_subdirectory(config)
add_subdirectory(example)
add_subdirectory(mission) # For Reaction Wheel Controller

target_include_directories(${TARGET_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if(TGT_BSP MATCHES "arm/stm32h743zi-nucleo")
  add_subdirectory(stm32h7)
endif()
