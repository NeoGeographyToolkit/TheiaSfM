// Copyright (C) 2014 The Regents of the University of California (Regents).
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

#include "theia/util/filesystem.h"

#include <glog/logging.h>
#include <stlplus3/file_system.hpp>

#include <fnmatch.h>

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

namespace theia {

bool GetFilepathsFromWildcard(const std::string& filepath_with_wildcard,
                              std::vector<std::string>* filepaths) {
  CHECK_NOTNULL(filepaths)->clear();

  const std::string folder = stlplus::folder_part(filepath_with_wildcard);
  const std::string filename_part =
      stlplus::filename_part(filepath_with_wildcard);

  namespace fs = std::filesystem;
  if (folder.empty() || !fs::exists(folder)) {
    VLOG(2) << "Input folder does not exist:" << folder;
    return false;
  }

  // Keep every directory entry whose name matches the wildcard. Use
  // std::filesystem so symlinked images are included, since is_regular_file
  // follows the link. The older stlplus folder_wildcard skipped symlinks, which
  // broke callers that symlink their inputs (for example theia_sfm).
  std::error_code ec;
  for (const auto& entry : fs::directory_iterator(folder, ec)) {
    if (!entry.is_regular_file(ec))
      continue;
    const std::string name = entry.path().filename().string();
    if (fnmatch(filename_part.c_str(), name.c_str(), 0) == 0)
      filepaths->push_back(entry.path().string());
  }

  // Sort for a deterministic order across platforms.
  std::sort(filepaths->begin(), filepaths->end());

  if (filepaths->empty()) {
    VLOG(2) << "No files matched the input.";
  }

  return true;
}

bool GetFilenameFromFilepath(const std::string& filepath,
                             const bool with_extension,
                             std::string* filename) {
  CHECK_NOTNULL(filename)->clear();

  if (with_extension) {
    *filename = stlplus::filename_part(filepath);
  } else {
    *filename = stlplus::basename_part(filepath);
  }

  return filename->length() > 0;
}

bool GetDirectoryFromFilepath(const std::string& filepath,
                              std::string* directory) {
  CHECK_NOTNULL(directory)->clear();
  *directory = stlplus::folder_part(filepath);
  return directory->length() > 0;
}

bool FileExists(const std::string& filename) {
  return stlplus::file_exists(filename);
}

// Returns true if the directory exists, false otherwise.
bool DirectoryExists(const std::string& directory) {
  return stlplus::folder_exists(directory);
}

// Creates the given directory.
bool CreateNewDirectory(const std::string& directory) {
  return stlplus::folder_create(directory);
}

bool CopyFile(const std::string& filepath_to_copy_from,
              const std::string& filepath_to_copy_to) {
  return stlplus::file_copy(filepath_to_copy_from, filepath_to_copy_to);
}

}  // namespace theia
