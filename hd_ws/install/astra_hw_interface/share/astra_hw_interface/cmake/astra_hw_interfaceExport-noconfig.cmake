#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "astra_hw_interface::astra_hw_interface" for configuration ""
set_property(TARGET astra_hw_interface::astra_hw_interface APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(astra_hw_interface::astra_hw_interface PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libastra_hw_interface.so"
  IMPORTED_SONAME_NOCONFIG "libastra_hw_interface.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS astra_hw_interface::astra_hw_interface )
list(APPEND _IMPORT_CHECK_FILES_FOR_astra_hw_interface::astra_hw_interface "${_IMPORT_PREFIX}/lib/libastra_hw_interface.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
