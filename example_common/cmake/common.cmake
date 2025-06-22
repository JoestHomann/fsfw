function(get_common_build_flags TGT_NAME)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set(COMMON_COMPILE_OPTS
        -ffunction-sections
        -fdata-sections
        PARENT_SCOPE)
      set(COMMON_LINK_OPTS
        -Wl,--gc-sections
        -Wl,-Map=${TARGET_NAME}.map
        PARENT_SCOPE)
      set(COMMON_WARNING_FLAGS
        -Wall
        -Wextra
        -Wimplicit-fallthrough=1
        -Wno-unused-parameter
        -Wno-psabi
        -Wduplicated-cond # check for duplicate conditions
        -Wduplicated-branches # check for duplicate branches
        -Wlogical-op # Search for bitwise operations instead of logical
        -Wnull-dereference # Search for NULL dereference
        -Wundef # Warn if undefind marcos are used
        -Wformat=2 # Format string problem detection
        -Wformat-overflow=2 # Formatting issues in printf
        -Wformat-truncation=2 # Formatting issues in printf
        -Wformat-security # Search for dangerous printf operations
        -Wstrict-overflow=3 # Warn if integer overflows might happen
        -Warray-bounds=2 # Some array bounds violations will be found
        -Wshift-overflow=2 # Search for bit left shift overflows (<c++14)
        -Wcast-qual # Warn if the constness is cast away
        -Wstringop-overflow=4
        # -Wstack-protector # Emits a few false positives for low level access
        # -Wconversion # Creates many false positives -Warith-conversion # Use
        # with Wconversion to find more implicit conversions -fanalyzer # Should
        # be used to look through problems
        PARENT_SCOPE)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    add_compile_options(/permissive- /d2SSAOptimizer-)
    # To avoid nameclashes with min and max macro
    add_compile_definitions(NOMINMAX)
  endif()

endfunction()