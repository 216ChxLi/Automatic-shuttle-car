	vector<Vec4d> setWarnings_HSV()
    {
        cv::Scalar scalarL = cv::Scalar( 0, 90, 90 );
        cv::Scalar scalarH = cv::Scalar( 30, 255, 255 );	
	//cv::Scalar scalarL = cv::Scalar( 26, 43, 46 );
        //cv::Scalar scalarH = cv::Scalar( 34, 255, 255 );

		cv::Mat img_hsv;
        cv::cvtColor( image_, img_hsv, COLOR_BGR2HSV );
        cv::inRange( img_hsv, scalarL, scalarH, img_hsv );
        imshow( "HSV", img_hsv );

        vector<Vec4d> warning_lanes;  // will hold all the results of the detection
        //HoughLinesP( img_threshold, warning_lanes, 1, CV_PI / 180, 50, 50, 10 );  // Probabilistic Line Transform

        return warning_lanes;
    }
