#include <iostream>
#include <cmath>

//You need OpenCV for this demo
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>

cv::RotatedRect getErrorEllipse(double chisquare_val, cv::Point2f mean, cv::Mat covmat);

int main()
{
		
	//Covariance matrix of our data	
	cv::Mat covmat = (cv::Mat_<double>(2,2) << 500.5886, 400.6111, 400.6111, 500.7801);	
	
	//The mean of our data
	cv::Point2f mean(160,120);

	//Calculate the error ellipse for a 95% confidence intervanl
	cv::RotatedRect ellipse = getErrorEllipse(2.4477, mean, covmat);

	//Show the result
	cv::Mat visualizeimage(240, 320, CV_8UC1, cv::Scalar::all(0));	
	cv::ellipse(visualizeimage, ellipse, cv::Scalar::all(255), 2);
	cv::imshow("EllipseDemo", visualizeimage);
	cv::waitKey();

}

cv::RotatedRect getErrorEllipse(double chisquare_val, cv::Point2f mean, cv::Mat covmat){
	
	//Get the eigenvalues and eigenvectors
	cv::Mat eigenvalues, eigenvectors;
	cv::eigen(covmat, true, eigenvalues, eigenvectors);

	//Calculate the angle between the largest eigenvector and the x-axis
	double angle = atan2(eigenvectors.at<double>(0,1), eigenvectors.at<double>(0,0));

	//Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
	if(angle < 0)
		angle += 6.28318530718;

	//Conver to degrees instead of radians
	angle = 180*angle/3.14159265359;

	//Calculate the size of the minor and major axes
	double halfmajoraxissize=chisquare_val*sqrt(eigenvalues.at<double>(0));
	double halfminoraxissize=chisquare_val*sqrt(eigenvalues.at<double>(1));

	//Return the oriented ellipse
	//The -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
	return cv::RotatedRect(mean, cv::Size2f(halfmajoraxissize, halfminoraxissize), -angle);

}
