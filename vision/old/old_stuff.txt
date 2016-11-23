Mat src, src_threshold;
Mat dst, output_threshold;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 255;
int highThreshold;
int const max_highThreshold = 255;
int ratio = 3;
int kernel_size;
int const max_kernel_size = 21;
int low_hueThreshold;
int high_hueThreshold;
int low_satThreshold;
int high_satThreshold;
int low_valThreshold;
int high_valThreshold;
int const max_colorThreshold = 255;


// *** Color Threshold ***
// Example on syntax for function
void ColorThreshold(int, void*)
{
  inRange(src_threshold, Scalar(low_hueThreshold, low_satThreshold, low_valThreshold), Scalar(high_hueThreshold, high_satThreshold, high_valThreshold), output_threshold);
  dst = Scalar::all(0);

  src_threshold.copyTo( dst, output_threshold);
  imshow( window_name, dst );
}

// *** Canny Threshold ***
// Example on syntax for function
void CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 3x3
  if (kernel_size > 0)
    blur( src_threshold, output_threshold, Size(kernel_size,kernel_size) );
  else
    blur( src_threshold, output_threshold, Size(1,1) );


  /// Canny detector
  Canny( output_threshold, output_threshold, lowThreshold, highThreshold, 3 );

  /// Using Canny's output as a mask, we display our result
  dst = Scalar::all(0);

  src_threshold.copyTo( dst, output_threshold);
  imshow( window_name, dst );
}
