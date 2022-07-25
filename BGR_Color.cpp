vector<Vec4d> setWarnings()
    {
        // balance of HSV (optional)
        vector<Mat> hsvSplit;  // vector to keep HSV 3 Channels info
        cv::Mat img_hsv;
        cv::Mat img_bgr;

        cvtColor( image_, img_hsv, COLOR_BGR2HSV );
        split( img_hsv, hsvSplit );                // classify the original image 3 HSV Channels
        equalizeHist( hsvSplit[2], hsvSplit[2] );  // balance the HSV light Channels
        merge( hsvSplit, img_hsv );                // merge all the channels
        cvtColor( img_hsv, img_bgr, COLOR_HSV2BGR );

        // classify the colors: the channels order should be BGR
        Mat img_threshold;
        inRange( img_bgr, Scalar( 0, 128, 128 ), Scalar( 127, 255, 255 ), img_threshold );
        /*
        blue:		(128,0,0) (255,127,127)
        white:		(128,128,128) (255,255,255)
        cyan:		(128,128,0) (255,255,127)
        purple:		(0,128,128) (127,255,255)
        yellow:		(0,128,128) (127,255,255)
        green:		(0,128,0) (127,255,127)
        red:		(0,0,128) (127,127,255)
        black:		(0,0,0,) (127,127,127)
        */
		// R154 G121 B20		R241 G224 B33		R222 G202 B101

        //// remove the noise
        //Mat element = getStructuringElement( MORPH_RECT, Size( 5, 5 ) );
        //morphologyEx( img_threshold, img_threshold, MORPH_OPEN, element );
        //morphologyEx( img_threshold, img_threshold, MORPH_CLOSE, element );

        // set a polygon mask to only keep thed region of interest
        cv::Mat mask = Mat::zeros( img_threshold.size(), img_threshold.type() );
        const int num = 6;
        Point points[1][num] = {Point( 0, rows ),
                                Point( 0, rows * 0.7 ),
                                Point( cols * 0.33, rows * 0.55 ),
                                Point( cols * 0.66, rows * 0.55 ),
                                Point( cols, rows * 0.7 ),
                                Point( cols, rows )};
        const Point* polygon = points[0];
        fillConvexPoly( mask, polygon, num, Scalar( 255 ) );

        //cv::bitwise_and( img_threshold, mask, img_threshold );
        imshow( "color", img_threshold );

        vector<Vec4d> warning_lanes;  // will hold all the results of the detection
        HoughLinesP( img_threshold, warning_lanes, 1, CV_PI / 180, 50, 50,
                     10 );  // Probabilistic Line Transform

        return warning_lanes;
    }
        Point points[1][num] = {Point( 0, rows ),
                                Point( 0, rows * 0.7 ),
                                Point( cols * 0.33, rows * 0.55 ),
                                Point( cols * 0.66, rows * 0.55 ),
                                Point( cols, rows * 0.7 ),
                                Point( cols, rows )};
        const Point* polygon = points[0];
        fillConvexPoly( mask, polygon, num, Scalar( 255 ) );

        cv::bitwise_and( img_threshold, mask, img_threshold );
        imshow( "color", img_threshold );

		vector<Vec4d> warning_lanes;  // will hold all the results of the detection
        HoughLinesP( img_threshold, warning_lanes, 1, CV_PI / 180, 50, 50, 10 );  // Probabilistic Line Transform

        return warning_lanes;
    }
