# This module attempts to find the libmodbus library.
# It uses pkg-config for discovery.

# Define output variables
# LIBMODBUS_FOUND       - True if libmodbus was found
# LIBMODBUS_INCLUDE_DIRS - Include directories for libmodbus headers
# LIBMODBUS_LIBRARIES   - Libraries to link against libmodbus

find_package(PkgConfig QUIET)
pkg_check_modules(PC_LIBMODBUS QUIET libmodbus)

if (PC_LIBMODBUS_FOUND)
  set(LIBMODBUS_FOUND TRUE)
  set(LIBMODBUS_INCLUDE_DIRS ${PC_LIBMODBUS_INCLUDE_DIRS})
  set(LIBMODBUS_LIBRARIES ${PC_LIBMODBUS_LIBRARIES})

  # Optionally, you can set the version if pkg-config provides it
  set(LIBMODBUS_VERSION ${PC_LIBMODBUS_VERSION})

  # Report status
  if (NOT LIBMODBUS_FIND_QUIETLY)
    message(STATUS "Found libmodbus (using pkg-config):")
    message(STATUS "  Includes: ${LIBMODBUS_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${LIBMODBUS_LIBRARIES}")
    if (LIBMODBUS_VERSION)
      message(STATUS "  Version: ${LIBMODBUS_VERSION}")
    endif()
  endif()
else()
  if (LIBMODBUS_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find libmodbus. Please ensure pkg-config can locate it (e.g., install libmodbus-dev).")
  else()
    if (NOT LIBMODBUS_FIND_QUIETLY)
      message(STATUS "Could not find libmodbus (using pkg-config).")
    endif()
  endif()
endif()

mark_as_advanced(LIBMODBUS_FOUND LIBMODBUS_INCLUDE_DIRS LIBMODBUS_LIBRARIES)