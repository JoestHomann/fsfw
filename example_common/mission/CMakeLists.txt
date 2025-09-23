set(MISSION_ACS_SRC
    acs/AcsController.cpp
    acs/Guidance.cpp
    acs/Estimator.cpp
    acs/RwAllocator.cpp
)
target_sources(${TARGET_NAME} PRIVATE ${MISSION_ACS_SRC})
target_include_directories(${TARGET_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})