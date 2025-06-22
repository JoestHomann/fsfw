target_sources(${TARGET_NAME} PRIVATE FsfwReaderTask.cpp FsfwExampleTask.cpp
                                      MutexExample.cpp FsfwTestTask.cpp)

if(FSFW_ADD_FMT_TESTS)
  target_sources(${TARGET_NAME} PRIVATE testFmt.cpp)
endif()
