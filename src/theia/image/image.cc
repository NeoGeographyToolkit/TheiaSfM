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

#include "theia/image/image.h"

#include <glog/logging.h>
#include <Eigen/Core>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "theia/util/util.h"

namespace theia {

FloatImage::FloatImage(): FloatImage(0, 0, 1) {}

// Read from file.
FloatImage::FloatImage(const std::string& filename) { 
  Read(filename); 
}

FloatImage::FloatImage(const FloatImage& image_to_copy) {
  //CHECK(image_.copy(image_to_copy.image_));
  image_to_copy.m_opencv_image.copyTo(this->m_opencv_image);
}

FloatImage::FloatImage(const int width, const int height, const int channels) {
  cv::Size s;
  s.width = width;
  s.height = height;
  this->m_opencv_image = cv::Mat(s, CV_32FC(channels));
}

FloatImage::FloatImage(const int width, const int height, const int channels,
                       float* buffer) {
  throw std::runtime_error("Image from raw buffer needs porting to OpenCV.");
}

FloatImage& FloatImage::operator=(const FloatImage& input_image) {
  input_image.m_opencv_image.copyTo(this->m_opencv_image);
  return *this;
}

int FloatImage::Rows() const { 
  return Height(); 
}

int FloatImage::Cols() const { 
  return Width(); 
}

int FloatImage::Width() const { 
  return m_opencv_image.cols;
}

int FloatImage::Height() const {
  return m_opencv_image.rows;
}

int FloatImage::Channels() const {
  return m_opencv_image.channels();
}

void FloatImage::SetXY(const int x, const int y, const int c,
                       const float value) {
  throw std::runtime_error("SetXY needs porting to OpenCV.");
  DCHECK_GE(x, 0);
  DCHECK_LT(x, Width());
  DCHECK_GE(y, 0);
  DCHECK_LT(y, Height());
  DCHECK_GE(c, 0);
  DCHECK_LT(c, Channels());

  // Set the ROI to be the precise pixel location in the correct channel.
  // oiio::ImageBuf::Iterator<float> it(image_, x, y, 0);
  //it[c] = value;
}

void FloatImage::SetXY(const int x, const int y, const Eigen::Vector3f& rgb) {
  throw std::runtime_error("SetXY needs porting to OpenCV.");
  DCHECK_GE(x, 0);
  DCHECK_LT(x, Width());
  DCHECK_GE(y, 0);
  DCHECK_LT(y, Height());

  //image_.setpixel(x, y, 0, rgb.data());
}

float FloatImage::GetXY(const int x, const int y, const int c) const {
  throw std::runtime_error("GetXY needs porting to OpenCV.");
  DCHECK_GE(x, 0);
  DCHECK_LT(x, Width());
  DCHECK_GE(y, 0);
  DCHECK_LT(y, Height());
  DCHECK_GE(c, 0);
  DCHECK_LT(c, Channels());
  return -1;
  //return image_.getchannel(x, y, 0, c);
}

Eigen::Vector3f FloatImage::GetXY(const int x, const int y) const {
  throw std::runtime_error("GetXY needs porting to OpenCV.");
  DCHECK_GE(x, 0);
  DCHECK_LT(x, Width());
  DCHECK_GE(y, 0);
  DCHECK_LT(y, Height());
  Eigen::Vector3f rgb;
  //image_.getpixel(x, y, 0, rgb.data());
  return rgb;
}

void FloatImage::SetRowCol(const int row, const int col, const int channel,
                           const float value) {
  throw std::runtime_error("SetRowCol needs porting to OpenCV.");
  SetXY(col, row, channel, value);
}

void FloatImage::SetRowCol(const int row, const int col,
                           const Eigen::Vector3f& rgb) {
  throw std::runtime_error("SetRowCol needs porting to OpenCV.");
  SetXY(col, row, rgb);
}

float FloatImage::GetRowCol(const int row, const int col,
                            const int channel) const {
  throw std::runtime_error("GetRowCol needs porting to OpenCV.");
  return GetXY(col, row, channel);
}

Eigen::Vector3f FloatImage::GetRowCol(const int row, const int col) const {
  throw std::runtime_error("Buffer needs porting to OpenCV.");
  return GetXY(col, row);
}

float FloatImage::BilinearInterpolate(const double x, const double y,
                                      const int c) const {
  throw std::runtime_error("BilinearInterpolate needs porting to OpenCV.");
  DCHECK_GE(c, 0);
  DCHECK_LT(c, Channels());
  // The caller has to ensure that "pixel" has sufficient memory to store all
  // interpolated channels so we have to initialize it as an array.
  std::vector<float> pixel(Channels());
  //image_.interppixel(x, y, pixel.data());
  return pixel[c];
}

Eigen::Vector3f FloatImage::BilinearInterpolate(const double x,
                                                const double y) const {
  throw std::runtime_error("BilinearInterpolate needs porting to OpenCV.");
  Eigen::Vector3f pixel;
  //image_.interppixel(x, y, pixel.data());
  return pixel;
}

void FloatImage::ConvertToGrayscaleImage() {
  if (Channels() == 1) {
    // Image is already a grayscale image. No conversion necessary
    return;
  }

  cv::Mat helper_image;
  cv::cvtColor(m_opencv_image, helper_image, cv::COLOR_BGR2GRAY); // 3 to 1 channels
  helper_image.convertTo(m_opencv_image, CV_32F); // convert int to float

  // // Compute luminance via a weighted sum of R,G,B (assuming Rec709 primaries
  // // and a linear scale)
  // const float luma_weights[3] = {.2126, .7152, .0722};
  // oiio::ImageBuf source = image_;
  // image_.clear();
  // oiio::ImageBufAlgo::channel_sum(image_, source, luma_weights);
}

void FloatImage::ConvertToRGBImage() {
  
  if (Channels() == 3) {
    // Image is already an RGB image. No conversion necessary
    return;
  }

  throw std::runtime_error("ConvertToRGBImage needs porting to OpenCV.");
  
  // if (Channels() == 1) {
  //   // Copy the single grayscale channel into r, g, and b.
  //   const oiio::ImageBuf source(image_);
  //   const oiio::ImageSpec image_spec(Width(), Height(), 3,
  //                                    oiio::TypeDesc::FLOAT);
  //   image_.reset(image_spec);
  //   oiio::ImageBufAlgo::paste(image_, 0, 0, 0, 0, source);
  //   oiio::ImageBufAlgo::paste(image_, 0, 0, 0, 1, source);
  //   oiio::ImageBufAlgo::paste(image_, 0, 0, 0, 2, source);
  // } else if (Channels() > 3) {
  //   // Copy only the r,g,b channels and drop the rest.
  //   const oiio::ImageBuf source(image_);
  //   const oiio::ImageSpec image_spec(Width(), Height(), 3,
  //                                    oiio::TypeDesc::FLOAT);
  //   image_.reset(image_spec);
  //   oiio::ImageBufAlgo::channels(image_, source, 3, NULL);
  // } else {
  //   LOG(FATAL) << "Converting from " << Channels()
  //              << " channels to RGB is unsupported.";
  // }
}

FloatImage FloatImage::AsGrayscaleImage() const {
  if (Channels() == 1) {
    // Image is already a grayscale image. No conversion necessary
    return *this;
  }
  
  FloatImage gray_image(*this);
  gray_image.ConvertToGrayscaleImage();
  return gray_image;
}

FloatImage FloatImage::AsRGBImage() const {
  if (Channels() == 3) {
    // Image is already an RGB image. No conversion necessary
    return *this;
  }

  throw std::runtime_error("AsRGBImage needs porting to OpenCV.");
  FloatImage rgb_image(*this);
  rgb_image.ConvertToRGBImage();
  return rgb_image;
}

void FloatImage::ScalePixels(float scale) {
  throw std::runtime_error("ScalePixels needs porting to OpenCV.");
  //oiio::ImageBufAlgo::mul(image_, image_, scale);
}

void FloatImage::Read(const std::string& filename) {
  
  // Ensure we always read a float image. We count on this later.
  cv::Mat input_image = cv::imread(filename);
  if (input_image.channels() == 1) {
    input_image.convertTo(m_opencv_image, CV_32F);
  } else {
    cv::Mat helper_image;
    cv::cvtColor(input_image, helper_image, cv::COLOR_BGR2GRAY); // 3 to 1 channels
    helper_image.convertTo(m_opencv_image, CV_32F); // convert int to float
  }
}

void FloatImage::Write(const std::string& filename) const {
  cv::imwrite(filename, m_opencv_image);
}

// Get the raw float pixel array
float* FloatImage::Data() {
  return reinterpret_cast<float*>(m_opencv_image.ptr<float>(0));
}
const float* FloatImage::Data() const {
  return reinterpret_cast<const float*>(m_opencv_image.ptr<float>(0));
}

FloatImage FloatImage::ComputeGradientX() const {
  throw std::runtime_error("ComputeGradientX needs porting to OpenCV.");
  float sobel_filter_x[9] = {-.125, 0, .125, -.25, 0, .25, -.125, 0, .125};
  // oiio::ImageSpec spec(3, 3, 1, oiio::TypeDesc::FLOAT);
  // oiio::ImageBuf kernel_x(spec, sobel_filter_x);
  // oiio::ImageBuf gradient_x;
  // oiio::ImageBufAlgo::convolve(gradient_x, image_, kernel_x, false);
  //return FloatImage(gradient_x);
  return FloatImage();
}

FloatImage FloatImage::ComputeGradientY() const {
  throw std::runtime_error("ComputeGradientY needs porting to OpenCV.");
  float sobel_filter_y[9] = {-.125, -.25, -.125, 0, 0, 0, .125, .25, .125};
  // oiio::ImageSpec spec(3, 3, 1, oiio::TypeDesc::FLOAT);
  // oiio::ImageBuf kernel_y(spec, sobel_filter_y);
  // oiio::ImageBuf gradient_y;
  // oiio::ImageBufAlgo::convolve(gradient_y, image_, kernel_y, false);
  //return FloatImage(gradient_y);
  return FloatImage();
}

FloatImage FloatImage::ComputeGradient() const {
  throw std::runtime_error("ComputeGradient needs porting to OpenCV.");
  // Get Dx and Dy.
  float sobel_filter_x[9] = {-.125, 0, .125, -.25, 0, .25, -.125, 0, .125};
  float sobel_filter_y[9] = {-.125, -.25, -.125, 0, 0, 0, .125, .25, .125};

  // oiio::ImageSpec spec(3, 3, 1, oiio::TypeDesc::FLOAT);
  // oiio::ImageBuf kernel_x(spec, sobel_filter_x);
  // oiio::ImageBuf kernel_y(spec, sobel_filter_y);

  // oiio::ImageBuf gradient, gradient_x, gradient_y;
  // oiio::ImageBufAlgo::convolve(gradient_x, image_, kernel_x, false);
  // oiio::ImageBufAlgo::abs(gradient_x, gradient_x);
  // oiio::ImageBufAlgo::convolve(gradient_y, image_, kernel_y, false);
  // oiio::ImageBufAlgo::abs(gradient_y, gradient_y);
  // oiio::ImageBufAlgo::add(gradient, gradient_x, gradient_y);
  //return FloatImage(gradient);
  return FloatImage();
}

void FloatImage::ApproximateGaussianBlur(const int kernel_size) {
  throw std::runtime_error("ApproximateGaussianBlur needs porting to OpenCV.");
  // oiio::ImageBuf kernel;
  // oiio::ImageBufAlgo::make_kernel(kernel, "gaussian",
  //                                 static_cast<float>(kernel_size),
  //                                 static_cast<float>(kernel_size));
  // oiio::ImageBufAlgo::convolve(image_, image_, kernel);
}

void FloatImage::MedianFilter(const int patch_width) {
  throw std::runtime_error("MedianFilter needs porting to OpenCV.");
  // CHECK(oiio::ImageBufAlgo::median_filter(image_, image_, patch_width));
}

void FloatImage::Integrate(FloatImage* integral) const {
  throw std::runtime_error("Integrate needs porting to OpenCV.");
  integral->ResizeRowsCols(Rows() + 1, Cols() + 1);
  for (int i = 0; i < Channels(); i++) {
    // Fill the first row with zeros.
    for (int x = 0; x < Width(); x++) {
      integral->SetXY(x, 0, i, 0);
    }
    for (int y = 1; y <= Height(); y++) {
      // This variable is to correct floating point round off.
      float sum = 0;
      integral->SetXY(0, y, i, 0);
      for (int x = 1; x <= Width(); x++) {
        sum += this->GetXY(x - 1, y - 1, i);
        integral->SetXY(x, y, i, integral->GetXY(x, y - 1, i) + sum);
      }
    }
  }
}

void FloatImage::Resize(int new_width, int new_height, int num_channels) {
  throw std::runtime_error("Resize needs porting to OpenCV.");
  // If the image has not been initialized then initialize it with the image
  // spec. Otherwise resize the image and interpolate pixels accordingly.
  // if (!image_.initialized()) {
  //   oiio::ImageSpec image_spec(new_width, new_height, num_channels,
  //                              oiio::TypeDesc::FLOAT);
  //   image_.reset(image_spec);
  // } else {
  //   CHECK_LE(Channels(), num_channels) << "Decreasing channels is not "
  //                                         "supported. Try ConvertToRGBImage() "
  //                                         "or ConvertToGrayscaleImage().";
  //   // oiio::ROI roi(0, new_width, 0, new_height, 0, 1, 0, num_channels);
  //   // oiio::ImageBuf dst;
  //   // CHECK(oiio::ImageBufAlgo::resize(dst, image_, nullptr, roi))
  //   //     << oiio::geterror();
  //   // image_.copy(dst);
  // }
}

void FloatImage::Resize(int new_width, int new_height) {
  throw std::runtime_error("Resize needs porting to OpenCV.");
  Resize(new_width, new_height, Channels());
}

void FloatImage::ResizeRowsCols(int new_rows, int new_cols) {
  throw std::runtime_error("ResizeRowsCols needs porting to OpenCV.");
  Resize(new_cols, new_rows, Channels());
}

void FloatImage::Resize(double scale) {
  throw std::runtime_error("Resize needs porting to OpenCV.");
  Resize(static_cast<int>(scale * Width()), static_cast<int>(scale * Height()),
         Channels());
}

}  // namespace theia
