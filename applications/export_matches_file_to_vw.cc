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

/* This tool exports a Theia match file in the Vision Workbench binary 
   match file format.
*/

#include <fstream>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <theia/theia.h>

#include <string>

DEFINE_string(theia_match_dir, "",
              "Input directory having Theia matches.");
DEFINE_string(vw_output_prefix, "", "Output files will be written with this folder/prefix.");
DEFINE_int32(min_num_matches, 30,
             "Don't generate files with fewer matches than this.");

namespace theia {
// Return the name minus the extension
std::string strip_ext(const std::string &name) {
  size_t pos = name.rfind('.');
  if (pos == std::string::npos)
    return name;
  return name.substr(0,pos);
}

// Generate a VW style match file name
// Returns true if the order was swapped
bool make_vw_filename(std::string const& out_prefix,
                      std::string const& input_file1,
                      std::string const& input_file2,
                      std::string      & output_name){

  // The names should be used in alphabetical order
  bool swap = false;
  std::string s1 = input_file1;
  std::string s2 = input_file2;
  if (s1 > s2) {
    std::string temp = s1;
    s1 = s2;
    s2 = temp;
    swap = true;
  }
  
  // Copied from VW
  int max_len = 40;
  std::string name1 = strip_ext(s1).substr(0, max_len);
  std::string name2 = strip_ext(s2).substr(0, max_len);

  output_name = out_prefix + "-" + name1 + "__" + name2 + ".match";
  return swap;
}

// Only the location is actually filled in, the rest of the fields
// get dummy values.
inline void write_vw_ip_record(std::ofstream &f, const theia::Feature & ip) {
  float        dummy_f      = 0.0;
  bool         dummy_bool   = false;
  unsigned int dummy_uint32 = 0;
  float x  = ip.x();
  float y  = ip.y();
  int   ix = static_cast<int>(x);
  int   iy = static_cast<int>(y);
  f.write((char*)&x,  sizeof(float));
  f.write((char*)&y,  sizeof(float));
  f.write((char*)&ix, sizeof(int  ));
  f.write((char*)&iy, sizeof(int  ));
  f.write((char*)&(dummy_f), sizeof(float));
  f.write((char*)&(dummy_f), sizeof(float));
  f.write((char*)&(dummy_f), sizeof(float));
  f.write((char*)&(dummy_bool), sizeof(bool));
  f.write((char*)&(dummy_uint32), sizeof(unsigned int));
  f.write((char*)&(dummy_uint32), sizeof(unsigned int));
  unsigned long long size = 1;
  f.write((char*)(&size), sizeof(unsigned long long));
  for (size_t i = 0; i < size; ++i)
    f.write((char*)&(dummy_f), sizeof(float));
}

// Write a single VW compatible binary file
bool write_vw_binary_match_file(const std::string   & matches_file,
                                const theia::ImagePairMatch& matches,
                                const bool swap) {
  // Return false if the file cannot be opened for writing.
  std::ofstream matches_writer(matches_file, std::ios::out | std::ios::binary);
  if (!matches_writer.is_open()) {
    LOG(ERROR) << "Could not open the matches file: " << matches_file
               << " for writing.";
    return false;
  }

  // Write the IP sizes
  const size_t num_matches = matches.correspondences.size();
  std::cout << "Writing " << num_matches << " matches to " << matches_file << std::endl;
  unsigned long long sizeull = num_matches;
  matches_writer.write((char*)&sizeull, sizeof(unsigned long long));
  matches_writer.write((char*)&sizeull, sizeof(unsigned long long));
  
  // Write all IP's for the first image
  for (size_t i=0; i<num_matches; ++i) {
    if (swap)
      write_vw_ip_record(matches_writer, matches.correspondences[i].feature2);
    else
      write_vw_ip_record(matches_writer, matches.correspondences[i].feature1);
  }
  // Write all IP's for the second image
  for (size_t i=0; i<num_matches; ++i) {
    if (swap)
      write_vw_ip_record(matches_writer, matches.correspondences[i].feature1);
    else
      write_vw_ip_record(matches_writer, matches.correspondences[i].feature2);
  }

  matches_writer.close();
  return true;
}

// Convert all the image match information into multiple VW match files.
bool WriteAllVwMatchFiles(
    const std::string& output_prefix,
    const std::vector<std::string>& view_names,
    const std::vector<theia::ImagePairMatch>& matches,
    const int min_num_matches) {

  // Write info for each pair to a seperate match file.
  const size_t num_matches = matches.size();
  for (size_t i=0; i<num_matches; ++i) {
    const size_t num_matches = matches[i].correspondences.size();
    if (num_matches < min_num_matches)
      continue; // Skip pairs with too few IP
    std::string this_file;
    bool swap = make_vw_filename(output_prefix, matches[i].image1, matches[i].image2, this_file);
    if (!write_vw_binary_match_file(this_file, matches[i], swap))
      return false;
  }
  return true;
}

} // end namespace theia

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  THEIA_GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  // Initialize the features and matches database.
  std::unique_ptr<theia::FeaturesAndMatchesDatabase> features_and_matches_database
    (new theia::RocksDbFeaturesAndMatchesDatabase(FLAGS_theia_match_dir));

  // Add the matches
  const auto match_keys = features_and_matches_database->ImageNamesOfMatches();
  std::cout << "Loaded " << match_keys.size() << " pairs of matches.\n";
  for (const auto& match_key: match_keys) {
    const theia::ImagePairMatch& match =
        features_and_matches_database->GetImagePairMatch(match_key.first,
                                                         match_key.second);

    const size_t num_matches = match.correspondences.size();
    if (num_matches < FLAGS_min_num_matches) {
      std::cout << "Too few matches loaded, will not save them.\n";
      continue; // Skip pairs with too few IP
    }
    
    std::string this_file;
    bool swap = theia::make_vw_filename(FLAGS_vw_output_prefix,
                                        match_key.first, match_key.second, this_file);
    if (!write_vw_binary_match_file(this_file, match, swap))
      std::cout << "Failed to write match file: " << this_file << std::endl;
  }

  return 0;
}
