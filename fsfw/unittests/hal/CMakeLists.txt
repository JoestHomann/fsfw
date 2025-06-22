target_sources(${FSFW_TEST_TGT} PRIVATE testHostFilesystem.cpp testFsMock.cpp)

if(UNIX)
  target_sources(${FSFW_TEST_TGT} PRIVATE testCommandExecutor.cpp)
endif()
