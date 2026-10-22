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

#include "theia/sfm/exif_reader.h"

#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>  // NOLINT
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "theia/sfm/camera_intrinsics_prior.h"
#include "theia/util/map_util.h"

// Get rid of warnings beyond our control
#pragma GCC diagnostic ignored "-Wstringop-overread"
#pragma GCC diagnostic push
// Generated file
#include "camera_sensor_database.h"
#pragma GCC diagnostic pop

namespace theia {
namespace {

void RemoveLeadingTrailingSpaces(std::string* str) {
  size_t p = str->find_first_not_of(" \t");
  str->erase(0, p);

  p = str->find_last_not_of(" \t");
  if (std::string::npos != p)
    str->erase(p + 1);
}

std::string ToLowercase(const std::string& str) {
  std::string str2 = str;
  RemoveLeadingTrailingSpaces(&str2);
  std::transform(str2.begin(), str2.end(), str2.begin(), ::tolower);
  if (!str2.empty() &&
      (str2[str2.size() - 1] == '\r' || str2[str2.size() - 1] == '\0')) {
    str2.erase(str2.size() - 1);
  }
  return str2;
}

bool IsValidFocalLength(const double focal_length) {
  return std::isfinite(focal_length) && focal_length > 0;
}

// The handful of EXIF tags we care about, parsed out of a JPEG or TIFF file.
// OpenCV does not expose EXIF tags, so we parse the TIFF IFD structure here.
struct ExifTags {
  bool has_make = false;
  std::string make;
  bool has_model = false;
  std::string model;
  // Focal length in millimeters (EXIF tag 0x920A).
  bool has_focal_mm = false;
  double focal_mm = 0.0;
  // Focal plane resolution (pixels per resolution unit) and the unit code.
  bool has_fp_x_res = false;
  double fp_x_res = 0.0;
  bool has_fp_y_res = false;
  double fp_y_res = 0.0;
  int fp_res_unit = 2;  // EXIF default is inches
  // Pixel dimensions as recorded in EXIF (may differ from the decoded size).
  bool has_pixel_x = false;
  int pixel_x = 0;
  bool has_pixel_y = false;
  int pixel_y = 0;
  // GPS coordinates are stored as unsigned magnitudes plus a reference tag
  // (N/S, E/W, above/below sea level) that gives the sign. EXIF orders the
  // reference tag before the value, so the sign is applied after parsing rather
  // than inline.
  bool has_latitude = false;
  double latitude = 0.0;
  std::string latitude_ref;
  bool has_longitude = false;
  double longitude = 0.0;
  std::string longitude_ref;
  bool has_altitude = false;
  double altitude = 0.0;
  int altitude_ref = 0;
};

// EXIF/TIFF tag identifiers used below.
constexpr uint16_t kTagMake = 0x010F;
constexpr uint16_t kTagModel = 0x0110;
constexpr uint16_t kTagExifSubIfd = 0x8769;
constexpr uint16_t kTagGpsSubIfd = 0x8825;
constexpr uint16_t kTagFocalLength = 0x920A;
constexpr uint16_t kTagFocalPlaneXRes = 0xA20E;
constexpr uint16_t kTagFocalPlaneYRes = 0xA20F;
constexpr uint16_t kTagFocalPlaneUnit = 0xA210;
constexpr uint16_t kTagPixelXDim = 0xA002;
constexpr uint16_t kTagPixelYDim = 0xA003;
constexpr uint16_t kTagGpsLatRef = 0x0001;
constexpr uint16_t kTagGpsLat = 0x0002;
constexpr uint16_t kTagGpsLonRef = 0x0003;
constexpr uint16_t kTagGpsLon = 0x0004;
constexpr uint16_t kTagGpsAltRef = 0x0005;
constexpr uint16_t kTagGpsAlt = 0x0006;

// Reads one TIFF IFD and the Exif and GPS sub-IFDs it points at. All file
// offsets are relative to the start of the TIFF header. The parser is
// deliberately small and only extracts the tags listed above.
class TiffParser {
 public:
  TiffParser(const std::vector<unsigned char>& buf, size_t tiff_start,
             bool little_endian)
      : buf_(buf), tiff_start_(tiff_start), little_(little_endian) {}

  void ParseRootIfd(size_t ifd_offset, ExifTags* tags) {
    ParseIfd(ifd_offset, tags, false);
  }

 private:
  // Bounds-checked little or big endian reads at a TIFF-relative offset.
  bool InBounds(size_t off, size_t n) const {
    return tiff_start_ + off + n <= buf_.size();
  }

  uint16_t U16(size_t off) const {
    const unsigned char* p = &buf_[tiff_start_ + off];
    if (little_)
      return p[0] | (p[1] << 8);
    return (p[0] << 8) | p[1];
  }

  uint32_t U32(size_t off) const {
    const unsigned char* p = &buf_[tiff_start_ + off];
    if (little_)
      return p[0] | (p[1] << 8) | (p[2] << 16) | (static_cast<uint32_t>(p[3]) << 24);
    return (static_cast<uint32_t>(p[0]) << 24) | (p[1] << 16) | (p[2] << 8) | p[3];
  }

  double Rational(size_t off) const {
    if (!InBounds(off, 8))
      return 0.0;
    const uint32_t num = U32(off);
    const uint32_t den = U32(off + 4);
    if (den == 0)
      return 0.0;
    return static_cast<double>(num) / static_cast<double>(den);
  }

  // Byte size of a TIFF type code, or 0 if unknown.
  static size_t TypeSize(uint16_t type) {
    switch (type) {
      case 1:  // BYTE
      case 2:  // ASCII
      case 7:  // UNDEFINED
        return 1;
      case 3:  // SHORT
        return 2;
      case 4:  // LONG
        return 4;
      case 5:  // RATIONAL
      case 10:  // SRATIONAL
        return 8;
      default:
        return 0;
    }
  }

  // Offset (TIFF-relative) of an entry's value. Values up to 4 bytes sit inline
  // in the entry, otherwise the entry holds an offset to them.
  size_t ValueOffset(size_t entry_off, uint16_t type, uint32_t count) const {
    const size_t bytes = TypeSize(type) * count;
    if (bytes <= 4)
      return entry_off + 8;
    return U32(entry_off + 8);
  }

  std::string ReadAscii(size_t entry_off, uint16_t type, uint32_t count) const {
    const size_t off = ValueOffset(entry_off, type, count);
    std::string out;
    for (uint32_t i = 0; i < count; i++) {
      if (!InBounds(off + i, 1))
        break;
      const char c = static_cast<char>(buf_[tiff_start_ + off + i]);
      if (c == '\0')
        break;
      out.push_back(c);
    }
    return out;
  }

  // A single integer value (SHORT or LONG).
  uint32_t ReadInt(size_t entry_off, uint16_t type, uint32_t count) const {
    const size_t off = ValueOffset(entry_off, type, count);
    if (type == 3)
      return U16(off);
    return U32(off);
  }

  // GPS coordinates are three rationals (degrees, minutes, seconds).
  double ReadGpsCoord(size_t entry_off, uint16_t type, uint32_t count) const {
    if (count < 3)
      return 0.0;
    const size_t off = ValueOffset(entry_off, type, count);
    const double deg = Rational(off);
    const double min = Rational(off + 8);
    const double sec = Rational(off + 16);
    return deg + min / 60.0 + sec / 3600.0;
  }

  void ParseIfd(size_t ifd_offset, ExifTags* tags, bool is_gps) {
    if (!InBounds(ifd_offset, 2))
      return;
    const uint16_t num_entries = U16(ifd_offset);
    for (uint16_t i = 0; i < num_entries; i++) {
      const size_t e = ifd_offset + 2 + i * 12;
      if (!InBounds(e, 12))
        return;
      const uint16_t tag = U16(e);
      const uint16_t type = U16(e + 2);
      const uint32_t count = U32(e + 4);
      if (TypeSize(type) == 0)
        continue;

      if (!is_gps) {
        HandleMainTag(tag, type, count, e, tags);
      } else {
        HandleGpsTag(tag, type, count, e, tags);
      }
    }
  }

  void HandleMainTag(uint16_t tag, uint16_t type, uint32_t count, size_t e,
                     ExifTags* tags) {
    switch (tag) {
      case kTagMake:
        tags->make = ReadAscii(e, type, count);
        tags->has_make = true;
        break;
      case kTagModel:
        tags->model = ReadAscii(e, type, count);
        tags->has_model = true;
        break;
      case kTagFocalLength:
        tags->focal_mm = Rational(ValueOffset(e, type, count));
        tags->has_focal_mm = true;
        break;
      case kTagFocalPlaneXRes:
        tags->fp_x_res = Rational(ValueOffset(e, type, count));
        tags->has_fp_x_res = true;
        break;
      case kTagFocalPlaneYRes:
        tags->fp_y_res = Rational(ValueOffset(e, type, count));
        tags->has_fp_y_res = true;
        break;
      case kTagFocalPlaneUnit:
        tags->fp_res_unit = static_cast<int>(ReadInt(e, type, count));
        break;
      case kTagPixelXDim:
        tags->pixel_x = static_cast<int>(ReadInt(e, type, count));
        tags->has_pixel_x = true;
        break;
      case kTagPixelYDim:
        tags->pixel_y = static_cast<int>(ReadInt(e, type, count));
        tags->has_pixel_y = true;
        break;
      case kTagExifSubIfd:
      case kTagGpsSubIfd: {
        const size_t sub_offset = U32(e + 8);
        ParseIfd(sub_offset, tags, tag == kTagGpsSubIfd);
        break;
      }
      default:
        break;
    }
  }

  void HandleGpsTag(uint16_t tag, uint16_t type, uint32_t count, size_t e,
                    ExifTags* tags) {
    switch (tag) {
      case kTagGpsLat:
        tags->latitude = ReadGpsCoord(e, type, count);
        tags->has_latitude = true;
        break;
      case kTagGpsLatRef:
        tags->latitude_ref = ReadAscii(e, type, count);
        break;
      case kTagGpsLon:
        tags->longitude = ReadGpsCoord(e, type, count);
        tags->has_longitude = true;
        break;
      case kTagGpsLonRef:
        tags->longitude_ref = ReadAscii(e, type, count);
        break;
      case kTagGpsAlt:
        tags->altitude = Rational(ValueOffset(e, type, count));
        tags->has_altitude = true;
        break;
      case kTagGpsAltRef:
        tags->altitude_ref = static_cast<int>(ReadInt(e, type, count));
        break;
      default:
        break;
    }
  }

  const std::vector<unsigned char>& buf_;
  size_t tiff_start_ = 0;
  bool little_ = true;
};

// Finds the TIFF header inside a JPEG (the Exif APP1 segment) or recognizes a
// raw TIFF file. Returns the TIFF-header offset and byte order, or false if the
// file carries no parseable EXIF/TIFF block.
bool FindTiffHeader(const std::vector<unsigned char>& buf, size_t* tiff_start,
                    bool* little_endian) {
  // Raw TIFF: starts with "II" (little) or "MM" (big) then 0x002A.
  if (buf.size() >= 4 &&
      ((buf[0] == 'I' && buf[1] == 'I') || (buf[0] == 'M' && buf[1] == 'M'))) {
    *tiff_start = 0;
    *little_endian = (buf[0] == 'I');
    return true;
  }

  // JPEG: 0xFFD8, then marker segments. The Exif APP1 (0xFFE1) payload begins
  // with "Exif\0\0" immediately before the TIFF header.
  if (buf.size() < 4 || buf[0] != 0xFF || buf[1] != 0xD8)
    return false;

  size_t pos = 2;
  while (pos + 4 <= buf.size()) {
    if (buf[pos] != 0xFF)
      break;
    const unsigned char marker = buf[pos + 1];
    // Start of scan or end of image: no more metadata.
    if (marker == 0xDA || marker == 0xD9)
      break;
    const size_t seg_len = (buf[pos + 2] << 8) | buf[pos + 3];
    const size_t payload = pos + 4;
    if (marker == 0xE1 && payload + 6 <= buf.size() &&
        std::equal(buf.begin() + payload, buf.begin() + payload + 6,
                   reinterpret_cast<const unsigned char*>("Exif\0\0"))) {
      const size_t header = payload + 6;
      if (header + 4 > buf.size())
        return false;
      *tiff_start = header;
      *little_endian = (buf[header] == 'I');
      return true;
    }
    pos += 2 + seg_len;
  }
  return false;
}

// Parses the EXIF tags of an image file. Returns false only if the file could
// not be read at all. A readable file with no EXIF yields an empty ExifTags.
bool ParseExif(const std::string& image_file, ExifTags* tags) {
  std::ifstream ifs(image_file, std::ios::binary);
  if (!ifs.good())
    return false;
  std::vector<unsigned char> buf((std::istreambuf_iterator<char>(ifs)),
                                 std::istreambuf_iterator<char>());
  if (buf.empty())
    return false;

  size_t tiff_start = 0;
  bool little = true;
  if (!FindTiffHeader(buf, &tiff_start, &little))
    return true;  // readable, just no EXIF

  // The TIFF header is: byte order (2), magic 0x002A (2), IFD0 offset (4).
  if (tiff_start + 8 > buf.size())
    return true;
  TiffParser parser(buf, tiff_start, little);
  const unsigned char* p = &buf[tiff_start + 4];
  const uint32_t ifd0 =
      little ? (p[0] | (p[1] << 8) | (p[2] << 16) | (static_cast<uint32_t>(p[3]) << 24))
             : ((static_cast<uint32_t>(p[0]) << 24) | (p[1] << 16) | (p[2] << 8) | p[3]);
  parser.ParseRootIfd(ifd0, tags);

  // Apply GPS reference signs now that both the value and its reference tag
  // have been read (EXIF stores the reference tag first).
  if (tags->latitude_ref == "S")
    tags->latitude = -tags->latitude;
  if (tags->longitude_ref == "W")
    tags->longitude = -tags->longitude;
  if (tags->altitude_ref == 1)
    tags->altitude = -tags->altitude;

  return true;
}

// Sets the focal length (in pixels) from the EXIF focal plane resolution.
// Returns true if a valid focal length is found.
bool SetFocalLengthFromExif(const ExifTags& tags,
                            CameraIntrinsicsPrior* camera_intrinsics_prior) {
  static const double kMinFocalLength = 1e-2;

  if (!tags.has_focal_mm || tags.focal_mm <= kMinFocalLength ||
      !tags.has_fp_x_res || tags.fp_x_res <= 0.0 ||
      !tags.has_fp_y_res || tags.fp_y_res <= 0.0) {
    return false;
  }

  // CCD resolution is the pixels per unit resolution of the CCD. Convert that
  // unit to millimeters.
  double ccd_resolution_units = 1.0;
  switch (tags.fp_res_unit) {
    case 2:
      // Inches to mm.
      ccd_resolution_units = 25.4;
      break;
    case 3:
      // Centimeters to mm.
      ccd_resolution_units = 10.0;
      break;
    case 4:
      // Already in mm.
      break;
    case 5:
      // Micrometers to mm.
      ccd_resolution_units = 1.0 / 1000.0;
      break;
    default:
      return false;
  }

  // EXIF pixel dimensions are preferred for the CCD size, falling back to the
  // decoded image size if absent.
  const int exif_width = tags.has_pixel_x ? tags.pixel_x
                                          : camera_intrinsics_prior->image_width;
  const int exif_height = tags.has_pixel_y ? tags.pixel_y
                                           : camera_intrinsics_prior->image_height;
  const double ccd_width =
      exif_width / (tags.fp_x_res / ccd_resolution_units);
  const double ccd_height =
      exif_height / (tags.fp_y_res / ccd_resolution_units);
  if (ccd_width <= 0.0 || ccd_height <= 0.0)
    return false;

  const double focal_length_x =
      tags.focal_mm * camera_intrinsics_prior->image_width / ccd_width;
  const double focal_length_y =
      tags.focal_mm * camera_intrinsics_prior->image_height / ccd_height;

  const double focal_length = (focal_length_x + focal_length_y) / 2.0;
  camera_intrinsics_prior->focal_length.value[0] = focal_length;
  return IsValidFocalLength(focal_length);
}

// Sets the focal length (in pixels) from a sensor width database lookup keyed
// on the camera make and model. Returns true if a valid focal length is found.
bool SetFocalLengthFromSensorDatabase(
    const ExifTags& tags,
    const std::unordered_map<std::string, double>& sensor_width_database,
    CameraIntrinsicsPrior* camera_intrinsics_prior) {
  if (!tags.has_focal_mm || tags.focal_mm <= 0.0)
    return false;

  const int max_image_dimension = std::max(camera_intrinsics_prior->image_width,
                                            camera_intrinsics_prior->image_height);

  const std::string camera_model = ToLowercase(tags.model);
  const std::string make_model =
      ToLowercase(tags.make) + " " + ToLowercase(tags.model);
  double sensor_width = 0.0;
  if (ContainsKey(sensor_width_database, camera_model)) {
    sensor_width = FindOrDie(sensor_width_database, camera_model);
  } else if (ContainsKey(sensor_width_database, make_model)) {
    sensor_width = FindOrDie(sensor_width_database, make_model);
  }

  if (sensor_width == 0.0)
    return false;

  const double focal_length =
      max_image_dimension * tags.focal_mm / sensor_width;
  camera_intrinsics_prior->focal_length.value[0] = focal_length;
  return IsValidFocalLength(focal_length);
}

}  // namespace

std::vector<std::string> SplitString(const std::string &s, const char delim) {
    std::vector<std::string> tokens;
    std::stringstream ss(s);
    std::string token;
    while (std::getline(ss, token, delim)) {
        tokens.push_back(token);
    }
    return tokens;
}

ExifReader::ExifReader() {
  LoadSensorWidthDatabase();
}

void ExifReader::LoadSensorWidthDatabase() {
  std::stringstream ifs(camera_sensor_database_txt, std::ios::in);

  while (!ifs.eof()) {
    // Read in the filename.
    std::string line;
    std::getline(ifs, line);
    if (line.size() == 0) {
      break;
    }

    const auto& tokens = SplitString(line, ';');
    CHECK_EQ(tokens.size(), 3);

    const std::string make = ToLowercase(tokens[0]);
    const std::string model = ToLowercase(tokens[1]);
    const double camera_sensor_width = stod(tokens[2]);

    // In the database, the model includes the make.
    InsertOrDie(&sensor_width_database_, model, camera_sensor_width);
  }
}

bool ExifReader::ExtractEXIFMetadata(
    const std::string& image_file,
    CameraIntrinsicsPrior* camera_intrinsics_prior) const {
  CHECK_NOTNULL(camera_intrinsics_prior);

  // Image dimensions come from decoding the image. EXIF pixel dimensions are
  // unreliable and often absent, so the decoded size is authoritative.
  const cv::Mat image = cv::imread(image_file, cv::IMREAD_UNCHANGED);
  if (image.empty()) {
    LOG(ERROR) << "Could not open image to read its metadata: " << image_file;
    return false;
  }
  camera_intrinsics_prior->image_width = image.cols;
  camera_intrinsics_prior->image_height = image.rows;

  // Principal point defaults to the image center.
  camera_intrinsics_prior->principal_point.is_set = true;
  camera_intrinsics_prior->principal_point.value[0] =
      camera_intrinsics_prior->image_width / 2.0;
  camera_intrinsics_prior->principal_point.value[1] =
      camera_intrinsics_prior->image_height / 2.0;

  // Parse EXIF. A readable file with no EXIF simply leaves the focal length
  // unset, which the caller handles (default focal, or skip if only calibrated
  // views are wanted).
  ExifTags tags;
  if (!ParseExif(image_file, &tags))
    return false;

  // Attempt to set the focal length from the focal plane resolution, then fall
  // back to the sensor width database.
  if (SetFocalLengthFromExif(tags, camera_intrinsics_prior) ||
      SetFocalLengthFromSensorDatabase(tags, sensor_width_database_,
                                       camera_intrinsics_prior)) {
    camera_intrinsics_prior->focal_length.is_set = true;
  }

  // GPS metadata.
  if (tags.has_latitude) {
    camera_intrinsics_prior->latitude.is_set = true;
    camera_intrinsics_prior->latitude.value[0] = tags.latitude;
  }
  if (tags.has_longitude) {
    camera_intrinsics_prior->longitude.is_set = true;
    camera_intrinsics_prior->longitude.value[0] = tags.longitude;
  }
  if (tags.has_altitude) {
    camera_intrinsics_prior->altitude.is_set = true;
    camera_intrinsics_prior->altitude.value[0] = tags.altitude;
  }

  return true;
}

}  // namespace theia
