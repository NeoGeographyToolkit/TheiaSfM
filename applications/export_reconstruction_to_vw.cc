// Copyright (C) 2015 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <theia/theia.h>
#include <theia/sfm/camera/pinhole_camera_model.h>
#include <theia/sfm/camera/camera.h>
#include <stlplus3/file_system.hpp>

#include <fstream>  // NOLINT

DEFINE_string(reconstruction, "", "Theia Reconstruction file.");
DEFINE_string(images, "",
              "A filepath wildcard specifying all images that were used in the "
              "reconstruction.");
DEFINE_string(output_directory, "", "Output directory.");
DEFINE_int32(num_threads, 1, "Number of threads to use.");

namespace theia {
void CreateDirectoryIfDoesNotExist(const std::string& directory) {
  if (!theia::DirectoryExists(directory)) {
    CHECK(theia::CreateNewDirectory(directory))
        << "Could not create the directory: " << directory;
  }
}

int WriteCamerasForVW(const theia::Reconstruction& reconstruction) {
  std::vector<std::string> image_files;
  CHECK(theia::GetFilepathsFromWildcard(FLAGS_images, &image_files))
      << "Could not find images that matched the filepath: " << FLAGS_images
      << ". NOTE that the ~ filepath is not supported.";
  CHECK_GT(image_files.size(), 0) << "No images found in: " << FLAGS_images;

  // Format for printing eigen matrices.
  const Eigen::IOFormat unaligned(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ");

  int current_image_index = 0;
  for (int i = 0; i < image_files.size(); i++) {

    // Fetch the view ID for this image
    std::string image_name;
    CHECK(theia::GetFilenameFromFilepath(image_files[i], true, &image_name));
    
    const theia::ViewId view_id = reconstruction.ViewIdFromName(image_name);
    if (view_id == theia::kInvalidViewId) {
      continue;
    }

    std::cout << "Exporting parameters for image: " << image_name << "\n";

    // Generate file name for this image
    // Write the camera projection matrix.
    const std::string text_path = stlplus::create_filespec(FLAGS_output_directory, 
                                                           image_name + ".tsai");
    std::cout << "Writing: " << text_path << "\n";
    
    // Get camera information
    const theia::Camera camera = reconstruction.View(view_id)->Camera();
    
    // Write the pinhole camera model to an ASP compatible text file
    std::ofstream ofs(text_path);
    ofs.precision(17);
    
    // Focal length x then y
    ofs << "fu = " << camera.FocalLength() << "\n";
    ofs << "fv = " <<camera.FocalLength() << "\n";
    
    // Center point x then y
    ofs << "cu = " << camera.PrincipalPointX() << "\n";
    ofs << "cv = " << camera.PrincipalPointY() << "\n";
    
    // Some pixel direction vectors that VW expects
    ofs << "u_direction = 1  0  0" << "\n";
    ofs << "v_direction = 0  1  0" << "\n";
    ofs << "w_direction = 0  0  1" << "\n";
    
    // Location C
    ofs << "C = " << camera.GetPosition().format(unaligned) << "\n";
    
    // Rotation matrix R
    ofs << "R = " << camera.GetOrientationAsRotationMatrix().transpose().format(unaligned) << "\n";

    // Distortion parameters
    if (camera.GetCameraIntrinsicsModelType() == CameraIntrinsicsModelType::PINHOLE) {
      ofs << "k1 = "
          << camera.CameraIntrinsics()->GetParameter(PinholeCameraModel::RADIAL_DISTORTION_1)
          << "\n";
      ofs << "k2 = "
          << camera.CameraIntrinsics()->GetParameter(PinholeCameraModel::RADIAL_DISTORTION_2)
          << "\n";
    } else {
      ofs << "k1 = " << 0 << "\n";
      ofs << "k2 = " << 0 << "\n";
    }
    ofs << "p1 = 0" << "\n";
    ofs << "p2 = 0" << "\n";
    
    ofs.close();

    ++current_image_index;
  } // End loop through images

  return current_image_index;
}

} // end namespace Theia

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  THEIA_GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  // Set up output directories.
  theia::CreateDirectoryIfDoesNotExist(FLAGS_output_directory);

  theia::Reconstruction reconstruction;
  CHECK(theia::ReadReconstruction(FLAGS_reconstruction, &reconstruction))
      << "Could not read the reconstruction file.";

  theia::WriteCamerasForVW(reconstruction);

  return 0;
}
