#include "label_detector/c_locator.h"

using std::string;
using std::shared_ptr;
using std::vector;

namespace polymechanon_vision {

Locator::Locator()
{
	ROS_WARN("New [Locator] created!");

	_calibration_loaded = false;
	
	string calibration_path(PACKAGE_PATH);
	calibration_path.append("/config/camera_calibration.yml");
	ROS_INFO("%s ", calibration_path.c_str());
	loadCalibration(calibration_path);


}

Locator::~Locator()
{
	ROS_ERROR("New [Locator] deleted!");
}

void Locator::setTestImage(const shared_ptr<cv::Mat> input_image)
{	
	if ( input_image == nullptr )  throw std::runtime_error("[Scanner]-setImageToScan() : image's pointer is nullptr. ");
	_test_image = (*input_image).clone();
}


vector< Point3D > Locator::calculate3DPosition(const vector<Point2D>& object2Dpoints, const vector<Point3D>& real3Dpoints)
{
	if( !_calibration_loaded )
		throw std::runtime_error("[Locator]-calculate3DPosition() : there is no loaded calibration.");


	vector<Point3D> object3Dpoints;
	// switch(_localizing_method)
	// {
	// 	case 0:
	// 		translateCoordinates(object2Dpoints, real3Dpoints, object3Dpoints);

	// 		break;
	// 	default: 
	// 		ROS_ERROR("[Locator]-translateCoordinates() : unknown method selected.");
	// 		break;
	// }

	if( _localizing_method == 0) 
	{
		translateCoordinates(object2Dpoints, real3Dpoints, object3Dpoints);
	}


	return object3Dpoints;

}

bool Locator::setParameters(bool debugging, int loc_method)
{	
	_debugging = debugging;
	_localizing_method = loc_method;
	return true;
}

void Locator::translateCoordinates(const vector<Point2D>& object2Dpoints, const vector<Point3D>& real3Dpoints, vector<Point3D>& object3Dpoints)
{

	cv::Mat rvec(1,3,cv::DataType<double>::type);
	cv::Mat tvec(1,3,cv::DataType<double>::type);
	cv::Mat rotation_matrix(3,3,cv::DataType<double>::type);

	//FIND EXTRINSIC PARAMETERS
	cv::solvePnP(real3Dpoints, object2Dpoints, _camera_matrix, _distortion_matrix, rvec, tvec);
	cv::Rodrigues(rvec, rotation_matrix);
	// TranslateCoordinates(object2Dpoints, real3Dpoints, rotation_matrix, _camera_matrix, tvec);


	cv::Mat PointOnImage;
	cv::Mat RealPoint;

	double s; //scale
	// s = cv_distance(object2Dpoints[0],object2Dpoints[2])/findDiagonal(qrwidth,qrheight);  
	s = calculateDistance(object2Dpoints[0],object2Dpoints[2])/calculateDistance(real3Dpoints[0],real3Dpoints[2]);  
	// s = findDiagonal(qrwidth,qrheight)/cv_distance(object2Dpoints[0],object2Dpoints[2]);


	PointOnImage = cv::Mat::zeros(3,1,cv::DataType<double>::type);
	RealPoint = cv::Mat::ones(3,1,cv::DataType<double>::type);

	cv::Mat homography(3,3,cv::DataType<double>::type);
	homography.at<double>(0,0) = rotation_matrix.at<double>(0,0);
	homography.at<double>(1,0) = rotation_matrix.at<double>(1,0);
	homography.at<double>(2,0) = rotation_matrix.at<double>(2,0);

	homography.at<double>(0,1) = rotation_matrix.at<double>(0,1);
	homography.at<double>(1,1) = rotation_matrix.at<double>(1,1);
	homography.at<double>(2,1) = rotation_matrix.at<double>(2,1);

	homography.at<double>(0,2) = tvec.at<double>(0,0);
	homography.at<double>(1,2) = tvec.at<double>(1,0);
	homography.at<double>(2,2) = tvec.at<double>(2,0);

	// homography.at<double>(0,2)=tvec.at<double>(0,0)/tvec.at<double>(2,0);
	// homography.at<double>(1,2)=tvec.at<double>(1,0)/tvec.at<double>(2,0);
	// homography.at<double>(2,2)=1.;

	// homography = homography/tvec.at<double>(2,0);

	// homography = homography/s;

	for (int i=0; i<4; i++) 
	{


	PointOnImage.at<double>(0,0) = object2Dpoints[i].x/s;
	PointOnImage.at<double>(1,0) = object2Dpoints[i].y/s;
	PointOnImage.at<double>(2,0) = 1.;



	RealPoint=homography*PointOnImage;
	// RealPoint= RealPoint / RealPoint.at<double>(2,0);



	// // Solving the transform below,
	// // we can find the coordinations in the physical world (relatively to camera):
	// //
	// // s->scale , A->camera matrix , R-> rotation matrix , t->translate vector , m->2d coordinates , M->3d coordinates
	// // 
	// //  s*m'= A*[R|t]*M' =>
	// //      |u| |fx 0  cx| |r11 r12 r13 t1| |X|
	// // => s*|v|=|0  fy cy|*|r21 r22 r23 t2|*|Y| => (M '= [R|t].inv * A.inv * s * m' )  =>
	// //      |1| |0  0   1| |r31 r32 r33 t3| |Z|
	// //                                      |1|
	// //
	// // => M' = R.inv * A.inv * s * m' + t  
	// //
	// //So: M'=RealPoint , R=RotationMatrix , A=CameraMatrix , s=s , m'=PointOnImage , t=tvec


	// RealPoint=RotationMatrix.inv()*CameraMatrix.inv()*s*PointOnImage - RotationMatrix.inv()*tvec;    //*PointOnImage;





	object3Dpoints.push_back(cv::Point3f());
	object3Dpoints[i].x = RealPoint.at<double>(0,0)*(-1);
	object3Dpoints[i].y = RealPoint.at<double>(1,0)*(-1);
	object3Dpoints[i].z = RealPoint.at<double>(2,0)*(-1);
	}

	// draw3Daxes(cv::Mat& image, vector<Point3D>& corners, vector<Point3D>& corners_axis)

}

double Locator::findDiagonal(double width,double height)
{
	return sqrt(pow(width,2)+pow(height,2));
}

float Locator::cv_distance(cv::Point2f P, cv::Point2f Q)
{
  return sqrt(pow(abs(P.x - Q.x),2) + pow(abs(P.y - Q.y),2)) ; 
}

void Locator::set2DPoints(std::vector< Point2D >& points)
{

}


std::vector< Point2D > Locator::get2DPoints() const
{
	std::vector< Point2D > some_vector;
	return some_vector;
}



void Locator::set3DPoints(std::vector< Point3D >& points)
{
	
}



std::vector< Point3D > Locator::get3DPoints() const
{
	std::vector< Point3D > some_vector;
	return some_vector;	
}



void Locator::loadCalibration(const std::string& filename)
{
	try	{
		cv::FileStorage fs(filename, cv::FileStorage::READ);
		fs["camera_matrix"] >> _camera_matrix;
		fs["distortion_matrix"] >> _distortion_matrix;
		_calibration_loaded = true;
		ROS_ERROR("A?");
	}
	catch (const cv::Exception &e)	{
		ROS_ERROR("%s", e.what() );
		// std::cout << e.what() << std::endl;
	}
}
















void Locator::calibrateImage(cv::Mat& image) 
{	
	if( _calibration_loaded )
		throw std::runtime_error("[Locator]-calibrateImage() : there is no loaded calibration.");

	cv::Mat map1,map2;
	cv::initUndistortRectifyMap(_camera_matrix,  // computed camera matrix
								_distortion_matrix,    // computed distortion matrix
								cv::Mat(),     // optional rectification (none)
								cv::Mat(),     // camera matrix to generate undistorted
								image.size(),  // size of undistorted
								CV_32FC1,      // type of output map
								map1, map2);   // the x and y mapping functions

  cv::remap(image, image, map1, map2,cv::INTER_LINEAR); // interpolation type
}




template<typename Ta, typename Tb>
double Locator::calculateDistance(const Ta& pointA, const Tb& pointB)
{
	return sqrt(pow(abs(pointA.x - pointB.x),2) + pow(abs(pointA.y - pointB.y),2)) ; 
}
template<typename T>
double Locator::calculateSlope(const T& pointA, const T& pointB)
{
	if (pointA.x - pointB.x == .0)    throw std::runtime_error( "[Locator]-calculateSlope() : division by zero encountered." );	
	return (pointA.y - pointB.y) / (pointA.x - pointB.x);
	// return ((double)pointA.y - (double)pointB.y) / ((double)pointA.x - (double)pointB.x);
}


void Locator::draw3Daxes(cv::Mat& image, vector<Point3D>& corners, vector<Point3D>& corners_axis)
{
	// cv::line(image, corners[0], corners_axis[0], cv::Scalar(255,0,0), 5);
	// cv::line(image, corners[1], corners_axis[1], cv::Scalar(255,0,0), 5);
	// cv::line(image, corners[2], corners_axis[2], cv::Scalar(255,0,0), 5);
}


} // "namespace polymechanon_vision"

