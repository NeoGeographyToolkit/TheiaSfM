# Basic programs
add_executable(extract_features extract_features.cc)
target_link_libraries(extract_features theia ${GFLAGS_LIBRARIES})

add_executable(compute_two_view_geometry compute_two_view_geometry.cc)
target_link_libraries(compute_two_view_geometry theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

# Tools for building SfM models
add_executable(build_reconstruction build_reconstruction.cc)
target_link_libraries(build_reconstruction theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})
install(TARGETS build_reconstruction RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
if(APPLE)
  set_target_properties(build_reconstruction PROPERTIES
    INSTALL_RPATH "@loader_path;@loader_path/../lib") # ${MULTIVIEW_DEPS_DIR}/lib is already in 
elseif(UNIX) # Unix which is not Apple
  set_target_properties(build_reconstruction PROPERTIES
    INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib:${MULTIVIEW_DEPS_DIR}/lib")
endif()

add_executable(build_1dsfm_reconstruction build_1dsfm_reconstruction.cc)
target_link_libraries(build_1dsfm_reconstruction theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(calibrate_camera_intrinsics calibrate_camera_intrinsics.cc)
target_link_libraries(calibrate_camera_intrinsics theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(undistort_images undistort_images.cc)
target_link_libraries(undistort_images theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

# Reconstruction visualization.
# This can be tricky to build
set(OpenGL_GL_PREFERENCE GLVND)
#set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL REQUIRED)

if (NOT APPLE)
  find_package(X11 REQUIRED)
endif (NOT APPLE)

if (NOT MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
else (NOT MSVC)
  find_package(GLEW REQUIRED)
  if (GLEW_FOUND)
    message ("-- Found GLEW: ${GLEW_INCLUDE_DIRS}")
  else (GLEW FOUND)
    message (FATAL_ERROR "-- Can't find GLEW. Please set GLEW_INCLUDE_DIR & GLEW_LIBRARY")
  endif (GLEW_FOUND)
  include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLEW_INCLUDE_DIRS} )
  list(APPEND EXTRA_GL_LIBRARIES ${GLEW_LIBRARY} ${GLEW_LIBRARIES})
endif (NOT MSVC)

find_package(GLUT REQUIRED)
if (GLUT_FOUND)
  message ("-- Found GLUT: ${GLUT_INCLUDE_DIR}")
else (GLUT FOUND)
  message (FATAL_ERROR "-- Can't find GLUT. Please set GLUT_INCLUDE_DIR & GLUT_LIBRARY")
endif (GLUT_FOUND)
include_directories(${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIR})
list(APPEND EXTRA_GL_LIBRARIES ${GLUT_LIBRARIES})

# Need this to find libOpenGL and X11 deps on Linux
set(EXTRA_LIBS "")
if (NOT APPLE)
  set(EXTRA_LIBS ${X11_LIBRARIES} -L/usr/lib64 -L/usr/lib/x86_64-linux-gnu -lXxf86vm)
endif()

add_executable(view_reconstruction view_reconstruction.cc)
target_link_libraries(view_reconstruction theia ${EXTRA_LIBS} ${GFLAGS_LIBRARIES}
  ${OPENGL_LIBRARIES} ${EXTRA_GL_LIBRARIES})

if(APPLE)
  set_target_properties(view_reconstruction PROPERTIES
    INSTALL_RPATH "@loader_path;@loader_path/../lib") # ${MULTIVIEW_DEPS_DIR}/lib is already in 
elseif(UNIX) # Unix which is not Apple
  set_target_properties(view_reconstruction PROPERTIES
    INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib:${MULTIVIEW_DEPS_DIR}/lib")
endif()
install(TARGETS view_reconstruction RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Useful tools for analyzing reconstructions.
add_executable(compute_reconstruction_statistics compute_reconstruction_statistics.cc)
target_link_libraries(compute_reconstruction_statistics theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(colorize_reconstruction colorize_reconstruction.cc)
target_link_libraries(colorize_reconstruction theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(compare_reconstructions compare_reconstructions.cc)
target_link_libraries(compare_reconstructions theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(compute_matching_relative_pose_errors compute_matching_relative_pose_errors.cc)
target_link_libraries(compute_matching_relative_pose_errors theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(evaluate_relative_translation_optimization evaluate_relative_translation_optimization.cc)
target_link_libraries(evaluate_relative_translation_optimization theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(verify_1dsfm_input verify_1dsfm_input.cc)
target_link_libraries(verify_1dsfm_input theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

# File conversions and exporters.
add_executable(convert_sift_key_file convert_sift_key_file.cc)
target_link_libraries(convert_sift_key_file theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(convert_bundle_file convert_bundle_file.cc)
target_link_libraries(convert_bundle_file theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(export_colmap_files export_colmap_files.cc)
target_link_libraries(export_colmap_files theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(export_to_nvm_file export_to_nvm_file.cc)
target_link_libraries(export_to_nvm_file theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})
if(APPLE)
  set_target_properties(export_to_nvm_file PROPERTIES
    INSTALL_RPATH "@loader_path;@loader_path/../lib") # ${MULTIVIEW_DEPS_DIR}/lib is already in 
elseif(UNIX) # Unix which is not Apple
  set_target_properties(export_to_nvm_file PROPERTIES
    INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib:${MULTIVIEW_DEPS_DIR}/lib")
endif()
install(TARGETS export_to_nvm_file RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(export_reconstruction_to_vw export_reconstruction_to_vw.cc)
target_link_libraries(export_reconstruction_to_vw theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})
if(APPLE)
  set_target_properties(export_reconstruction_to_vw PROPERTIES
    INSTALL_RPATH "@loader_path;@loader_path/../lib") # ${MULTIVIEW_DEPS_DIR}/lib is already in 
elseif(UNIX) # Unix which is not Apple
  set_target_properties(export_reconstruction_to_vw PROPERTIES
    INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib:${MULTIVIEW_DEPS_DIR}/lib")
endif()
install(TARGETS export_reconstruction_to_vw RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(export_matches_file_to_vw export_matches_file_to_vw.cc)
target_link_libraries(export_matches_file_to_vw theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})
if(APPLE)
  set_target_properties(export_matches_file_to_vw PROPERTIES
    INSTALL_RPATH "@loader_path;@loader_path/../lib") # ${MULTIVIEW_DEPS_DIR}/lib is already in 
elseif(UNIX) # Unix which is not Apple
  set_target_properties(export_matches_file_to_vw PROPERTIES
    INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib:${MULTIVIEW_DEPS_DIR}/lib")
endif()
install(TARGETS export_matches_file_to_vw RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(convert_nvm_file convert_nvm_file.cc)
target_link_libraries(convert_nvm_file theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(convert_theia_reconstruction_to_bundler_file convert_theia_reconstruction_to_bundler_file.cc)
target_link_libraries(convert_theia_reconstruction_to_bundler_file theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(write_reconstruction_ply_file write_reconstruction_ply_file.cc)
target_link_libraries(write_reconstruction_ply_file theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(export_reconstruction_to_pmvs export_reconstruction_to_pmvs.cc)
target_link_libraries(export_reconstruction_to_pmvs theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(create_calibration_file_from_exif create_calibration_file_from_exif.cc)
target_link_libraries(create_calibration_file_from_exif theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(create_reconstruction_from_strecha_dataset create_reconstruction_from_strecha_dataset.cc)
target_link_libraries(create_reconstruction_from_strecha_dataset theia ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})
