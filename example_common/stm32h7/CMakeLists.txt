target_sources(${TARGET_NAME} PRIVATE STM32TestTask.cpp)

option(STM32_ADD_NETWORKING_CODE "Add networking code requiring lwIP" ON)

if(STM32_ADD_NETWORKING_CODE)
  add_subdirectory(networking)
endif()
