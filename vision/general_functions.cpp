// GENERAL FUNCTIONS
// These functions are used for general purpose opencv
//______________________________________________________________________________

// *** Load data ***
// Loads the image data into a vector of Mat's, and converts to either gray or HSV
void load_data(vector<Mat> &input, String &path, int type)
{
  vector<String> fn;
  glob(path, fn, true); // true = Recursive

  for (size_t k = 0; k < fn.size(); k++)
  {
       Mat im_in = imread(fn[k]);
       Mat im_out;

       if(type == 2){
         cvtColor(im_in, im_out, CV_BGR2GRAY);
       }
       else if(type == 3){
         cvtColor(im_in, im_out, CV_BGR2HSV);
       }
       else{
         im_out = im_in;
       }

       input.push_back(im_out);
  }
}

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
