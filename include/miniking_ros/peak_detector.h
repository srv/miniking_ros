#ifndef PEAK_DETECTOR_H
#define PEAK_DETECTOR_H

#include <opencv2/opencv.hpp>

class PeakDetector {
 public:
  PeakDetector() {}
  virtual ~PeakDetector() {}
  virtual double detect(const std::vector<unsigned char> v, int pos) = 0;
};

class WeightedMeanPeakDetector : public PeakDetector {
 public:
  explicit WeightedMeanPeakDetector(int window_size) :
                                    window_size_(window_size) {}

  ~WeightedMeanPeakDetector() {}

  double detect(const std::vector<unsigned char> v, int pos) {
    double peak = static_cast<double>(pos);
    double m = 0;
    double mx = 0;
    for (int s = -window_size_/2; s < (window_size_+1)/2; s++) {
      const unsigned char val = v[pos+s];
      mx = val*s;
      m += val;
    }
    mx /= m;
    peak += mx;
    return peak;
  }

 private:
  int window_size_;
};

#endif