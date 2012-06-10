void cvMorphology_Operations_Func( 
  	IplImage* src0, 
  	IplImage* dst0,
  	int morph_elem,//"Element:\n 0: Rect - 1: Cross - 2: Ellipse"
	int morph_size,//"Kernel size:\n 2n +1"
	int morph_operator//"Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat"
);

void Morphology_Operations_Func( 
  	Mat& src0, 
  	Mat& dst0,
  	int morph_elem,//"Element:\n 0: Rect - 1: Cross - 2: Ellipse"
	int morph_size,//"Kernel size:\n 2n +1"
	int morph_operator//"Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat"
);
