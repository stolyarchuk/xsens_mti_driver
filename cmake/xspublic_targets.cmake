set(XSCONTROLLER_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xscontroller/libxscontroller.a)
add_custom_target(build_xscontroller ALL 
  COMMAND $(MAKE)
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xscontroller
  COMMENT "Original xscontroller makefile target")
add_library(xscontroller STATIC IMPORTED)
set_property(TARGET xscontroller APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(xscontroller PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${XSCONTROLLER_LIBRARY}")

  
set(XSCOMMON_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xscommon/libxscommon.a)
add_custom_target(build_xscommon ALL 
  COMMAND $(MAKE)
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xscommon
  COMMENT "Original xscommon makefile target")
add_library(xscommon STATIC IMPORTED)
set_property(TARGET xscommon APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(xscommon PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${XSCOMMON_LIBRARY}")


set(XSTYPES_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xstypes/libxstypes.a)
add_custom_target(build_xstypes ALL 
  COMMAND $(MAKE)
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xstypes
  COMMENT "Original xstypes makefile target")
add_library(xstypes STATIC IMPORTED)
set_property(TARGET xstypes APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(xstypes PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${XSTYPES_LIBRARY}")
    
add_dependencies(xscontroller build_xscontroller)
add_dependencies(xscommon build_xscommon)
add_dependencies(xstypes build_xstypes)


