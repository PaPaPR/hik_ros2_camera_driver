cmake_minimum_required(VERSION 3.10)

# Set the path where MVS libraries are located
set(MVS_LIB_DIR "/opt/MVS/lib/64")

# List of MVS libraries to search for
set(MVS_LIBRARIES_NAMES
  MvCameraControl
  MVGigEVisionSDK
  MvUsb3vTL
)

# Loop through the library names and find them
foreach(lib_name ${MVS_LIBRARIES_NAMES})
  find_library(${lib_name}_LIBRARY
    NAMES ${lib_name}
    PATHS ${MVS_LIB_DIR}
    NO_DEFAULT_PATH
  )
  
  # If the library is found, message and set the imported target
  if (${lib_name}_LIBRARY)
    message(STATUS "Found ${lib_name} library: ${${lib_name}_LIBRARY}")
    add_library(${lib_name} SHARED IMPORTED)
    set_target_properties(${lib_name} PROPERTIES
      IMPORTED_LOCATION ${${lib_name}_LIBRARY}
    )
  else()
    message(WARNING "Could not find ${lib_name} library in ${MVS_LIB_DIR}")
  endif()
endforeach()

# Define a macro for linking with these libraries
macro(link_with_mvslibs target)
  foreach(lib ${MVS_LIBRARIES_NAMES})
    if (${lib}_LIBRARY)
      target_link_libraries(${target} ${lib})
    endif()
  endforeach()
endmacro()