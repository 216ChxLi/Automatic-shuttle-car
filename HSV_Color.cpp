	vector<Vec4d> setWarnings_HSV()
    {
        cv::Mat blur_img;  // Remove noise by blurring with a Gaussian filter ( kernel size = 3 )
        GaussianBlur( image_, blur_img, Size( 3, 3 ), 0, 0, BORDER_DEFAULT );

		// set the HSV range
		cv::Scalar scalarL = cv::Scalar( 0, 90, 90 );
        cv::Scalar scalarH = cv::Scalar( 22, 255, 255 );
        // H82 S185 V30			H129 S212 V37		H152 S155 V33

        cv::Mat hsv_img;
        cv::cvtColor( blur_img, hsv_img, COLOR_BGR2HSV );
        cv::inRange( hsv_img, scalarL, scalarH, hsv_img );

        cv::Mat edge_img;  // Edge detection
        Canny( hsv_img, edge_img, 100, 200 );

		// set a polygon mask to only keep thed region of interest
        cv::Mat mask = Mat::zeros( image_.size(), edge_img.type() );
        const int num = 6;
        Point points[1][num] = {Point( 0, rows ),
                                Point( 0, rows * 0.7 ),
                                Point( cols * 0.33, rows * 0.55 ),
                                Point( cols * 0.66, rows * 0.55 ),
                                Point( cols, rows * 0.7 ),
                                Point( cols, rows )};
        const Point* polygon = points[0];
        fillConvexPoly( mask, polygon, num, Scalar( 255 ) );

        cv::Mat roi_img;
        cv::bitwise_and( edge_img, mask, roi_img );
        imshow( "Canny output: Region of Interest", roi_img );

        vector<Vec4d> warning_lanes;  // will hold all the results of the detection
        HoughLinesP( roi_img, warning_lanes, 1, CV_PI / 180, 50, 50, 10 );  // Probabilistic Line Transform

        return warning_lanes;
    }
