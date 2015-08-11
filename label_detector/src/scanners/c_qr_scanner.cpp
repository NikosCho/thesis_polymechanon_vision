#include "label_detector/scanners/c_qr_scanner.h"

namespace polymechanon_vision {

QrScanner::QrScanner(shared_ptr<cv::Mat> input_image /* = nullptr */)
{	
	// if ( input_image != nullptr)    setImageToScan(input_image);
	if ( input_image != nullptr)    Scanner::setImageToScan(input_image);
	ROS_WARN("QR-Scanner created!");
}

QrScanner::~QrScanner() { 
	ROS_ERROR("QR-Scanner deleted!"); 
}

LabelType QrScanner::getType() const
{
	return _type;
}

bool QrScanner::scan()
{	
	if (_image_to_scan == nullptr ) throw std::runtime_error("[QrScanner]-scan() : image's pointer is nullptr (use 'setImageToScan()'). ");

	cv::Mat caminput_gray(_image_to_scan->size(), CV_MAKETYPE(_image_to_scan->depth(), 1));    // To hold Grayscale Image
	cv::Mat caminput_edges(caminput_gray.size(), CV_MAKETYPE(caminput_gray.depth(), 1));    // To hold Grayscale Image

	cvtColor(*_image_to_scan, caminput_gray, CV_RGB2GRAY);    // Convert Image captured from Image Input to GrayScale 
	Canny(caminput_gray, caminput_edges, 100 , 200, 3);    // Apply Canny edge detection on the img_gray image
	
	vector<vector<ContourPoint> > alignment_markers = findAlignmentMarkers(caminput_edges);
	
	// In case we found less than 3 possible alignment markers, stop scanning
	if ( alignment_markers.size() < 3 ) return false;

	vector<Point2D> mass_centers = findContoursMassCenters(alignment_markers);
	vector<vector<int> > marker_groups = clusterMarkers(mass_centers);

	// In case clusterMarkers() returned no group, stop scanning
	if ( marker_groups.size() < 1 ) {
		ROS_WARN("[QrScanner]-scan() : no clusters returned, even if the markers were 3.");
		return false;	
	}

	// bool groups_size_is_ok = all_of(begin(marker_groups), end(marker_groups), [&](const <vector<int> > group){
	// 							 return group.size() == 3;
	// 						 });

	// if ( !groups_size_is_ok ) {
	// 	ROS_WARN("QrScanner::scan() : at least one of clusters has wrong size ( not 3).");
	// 	return false;	
	// }	

	_detected_labels.clear();    // remove previous labels detected

	for ( auto marker_group : marker_groups ) {
		vector<QrMarker> qr_markers = getMarkers(marker_group, alignment_markers, mass_centers);

		sortMarkers(qr_markers);    // 0-Bottom_left , 1-Top_left , 2 - Top_right
		sortMarkersVertices(qr_markers);

		vector<Point2D> qr_code_vertices = findSquare(qr_markers);

		if ( qr_code_vertices.size() == 4 && isContourConvex(qr_code_vertices) ) {
			std::string message = translate(caminput_gray, qr_code_vertices);
			cout << message << endl;
			_detected_labels.push_back(qr_code_vertices);
		}
	}

	if( _detected_labels.size() == 0 )    return false;




	return true;
}

vector<vector<Point2D> > QrScanner::getDetectedLabels()
{
	return _detected_labels;
}

vector<vector<ContourPoint> > QrScanner::findAlignmentMarkers(const cv::Mat& canny_image)
{
	vector<vector<ContourPoint> > contours;
	vector<cv::Vec4i> hierarchy;   
	findContours( canny_image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);    // Find contours with full hierarchy

	vector<vector<ContourPoint> > qr_alignment_markers;    // To hold contours that meet the conditions of an alignment marker
	vector<ContourPoint> approximated_contour;    // Used to save the approximated sides of each contour

	// Start processing the contour data
	// Find Three repeatedly enclosed contours A,B,C
	// Contour enclosing other contours is assumed to be the Alignment markers of the QR code.
	int contour_counter = 0;
	for_each(begin(contours), end(contours), [&](const vector<ContourPoint>& contour) {
		vector<ContourPoint> approximated_contour;
		// Find the approximated polygon of the contour we are examining
		cv::approxPolyDP(contour, approximated_contour, cv::arcLength(contour, true)*0.05, true);  
		if ( approximated_contour.size() == 4 ) {
			int k = contour_counter;
			int c = 0;
			// Count repeatedly nested contours. 
			while(hierarchy[k][2] != -1) {
				k = hierarchy[k][2] ;
				c++;
			}
			// if (c == 4)	
			if (c == 5 && isContourConvex(approximated_contour)) 
				qr_alignment_markers.push_back(approximated_contour);    // Contours that meet the conditions of an Alignment marker are saved seperate
		}
		contour_counter++;
	});
	return qr_alignment_markers;
}

vector<Point2D> QrScanner::findContoursMassCenters(const vector<vector<ContourPoint> >& contours_vector)
{	
	vector<Point2D> mass_centers;
	mass_centers.reserve(contours_vector.size());

	for_each(begin(contours_vector), end(contours_vector), [&mass_centers](const vector<ContourPoint>& contour) {
		cv::Moments contour_moments = cv::moments(contour, false);
		mass_centers.push_back( Point2D( contour_moments.m10/contour_moments.m00 , contour_moments.m01/contour_moments.m00 ));	
	});
	return mass_centers;
}

// Clustering our contours into groups of 3.
// Method:
// For each alignment marker we found, we find the 2 nearest markers and make a group
// When 2 groups contain the same markers, one of them will be deleted.
// The 'groupsmemory' data structure will contain marker's contour id.
// Each row of 'groupsmemory' is a different group(of 3) and
// each column contains the marker's id. 
vector< vector<int> > QrScanner::clusterMarkers(const vector<Point2D>& contours_mass_centers)
{	
	if ( contours_mass_centers.size() < 3 )    throw std::runtime_error("[QrScanner]-clusterMarkers() : input's size is less than 3. ");

	// Initializing groups.
	// Groups is a 2D vector:
	// - row represents a mass center
	// - each column holds the distance (and id) between a mass center (row) and another (id)
	vector< vector< pair<float, int> > > groups; 
	groups  = vector< vector< pair<float, int> > >(contours_mass_centers.size(), vector< pair<float, int> >(contours_mass_centers.size()));

	// Calculate the distance between each pair of points
	// and save it to groups.
	for( int m = 0; m < contours_mass_centers.size(); m++)
	{	
		groups[m][m] = pair<float, int>{0, m};    // saving itself
		for( int om = m + 1; om < contours_mass_centers.size(); om++)
		{
			float temp_dist = calculateDistance(contours_mass_centers[m],contours_mass_centers[om]);
			groups[m][om] = pair<float, int>{temp_dist, om};
			groups[om][m] = pair<float, int>{temp_dist, m};
		}	
	}

	// Sort each row of groups according to distance.
	for( int om = 0; om < groups.size(); om++)
		sort(begin(groups[om]), end(groups[om]));

	vector< vector<int> > groups_memory;
	// groups_memory.reserve(contours_mass_centers.size());

	// Iterate through groups and save only the unique
	for( int om = 0; om < groups.size(); om++) {
		vector<int> temp_group = {om, groups[om][1].second, groups[om][2].second};
		sort(begin(temp_group), end(temp_group));

		// Find if there is another group of the same elements
		bool group_is_unique = true;
		for_each(begin(groups_memory), end(groups_memory), [&](const vector<int>& vec){
			if ( temp_group == vec)
				group_is_unique = false;
		});

		if (group_is_unique)
			groups_memory.push_back(temp_group);
	}

	return groups_memory;
}

vector<Point2D> QrScanner::getMassCenters(const vector<int>& contours_by_id, const vector<Point2D>& mass_centers)
{	
	vector<Point2D> mass_centers_to_return;
	mass_centers_to_return.reserve(contours_by_id.size());

	try {
		for_each(begin(contours_by_id), end(contours_by_id), [&](const int id){
			mass_centers_to_return.push_back(mass_centers[id]);
		});
	} catch (const std::out_of_range& oor) {
		ROS_ERROR("[QrScanner]-getMassCenters() : While accesing vector(mass_centers) by id \"out_of_range\" encountered");
	}

	return mass_centers_to_return;
}

vector<vector<ContourPoint> > QrScanner::getContours(const vector<int>& contours_by_id, const vector<vector<ContourPoint> >& contours)
{	
	vector<vector<ContourPoint> > contours_to_return;
	contours_to_return.reserve(contours_by_id.size());

	try {
		for_each(begin(contours_by_id), end(contours_by_id), [&](const int id){
			contours_to_return.push_back(contours[id]);
		});
	} catch (const std::out_of_range& oor) {
		ROS_ERROR("[QrScanner]-getContours() : While accesing vector(contours) by id \"out_of_range\" encountered");
	}

	return contours_to_return;
}

vector<QrMarker> QrScanner::getMarkers(const vector<int>& contours_by_id, const vector<vector<ContourPoint> >& contours, const vector<Point2D>& mass_centers)
{
	vector<QrMarker> group_to_return;
	group_to_return.reserve(contours_by_id.size());

	try {
		for_each(begin(contours_by_id), end(contours_by_id), [&](const int id){
			group_to_return.push_back(QrMarker(contours[id], mass_centers[id]));
		});
	} catch (const std::out_of_range& oor) {
		ROS_ERROR("[QrScanner]-getMarkers() : While accesing vector(contours or mass_centers) by id \"out_of_range\" encountered");
	}

	return group_to_return;
}

void QrScanner::sortMarkers(vector<QrMarker>& markers)
{
	int top,bottom,right;
	QrOrientation orientation;

	float dist1,dist2,dist3;
	dist1 = calculateDistance(markers[0].mass_center,markers[1].mass_center);
	dist2 = calculateDistance(markers[1].mass_center,markers[2].mass_center);
	dist3 = calculateDistance(markers[2].mass_center,markers[0].mass_center);

	if ( dist1 > dist2 && dist1 > dist3)	{		// If the longest distance is 'dist1'
		top=2;		// Set the top marker
		bottom=0;	right=1;		// Set the other 2 vertices randomly
	}
	else if ( dist2 > dist1 && dist2 > dist3)	{		// If the longest distance is 'dist2'
		top=0;		// Set the top marker
		bottom=1;	right=2;		// Set the other 2 vertices randomly
	}	
	else if ( dist3 > dist1 && dist3 > dist2)	{		// If the longest distance is 'dist3'
		top=1;		// Set the top marker
		bottom=0;	right=2;		// Set the other 2 vertices randomly
	}

	// Now we are ready to check which one of the other 2 markers is the 'bottom'
	// and which is the 'right' one
	if ( markers[bottom].mass_center.x != markers[right].mass_center.x )	{		//To avoid divide by 0
		float diagonal_slope;
		diagonal_slope = calculateSlope(markers[bottom].mass_center, markers[right].mass_center);
		// Point2D proj_point=findProjection(markers[top].mass_center, markers[right].mass_center, markers[bottom].mass_center);
		Point2D proj_point=findMiddlePoint(markers[right].mass_center, markers[bottom].mass_center);

		if ( diagonal_slope > 0 && proj_point.y > markers[top].mass_center.y)	{
			orientation = QrOrientation::NORTH;	//NORTH
		}
		else if ( diagonal_slope < 0 && proj_point.y > markers[top].mass_center.y)	{
			orientation = QrOrientation::EAST;	//EAST
		}
		else if ( diagonal_slope > 0 && proj_point.y < markers[top].mass_center.y)	{
			orientation = QrOrientation::SOUTH;	//SOUTH
		}
		else if ( diagonal_slope < 0 && proj_point.y < markers[top].mass_center.y)	{
			orientation = QrOrientation::WEST;	//WEST
		}

		int temp_vert;
		switch(orientation)	{
			case QrOrientation::NORTH:	//NORTH
				if (markers[bottom].mass_center.x > markers[right].mass_center.x)	{
					temp_vert = bottom;
					bottom = right;
					right = temp_vert;
					// swap(bottom,right);
				}
				break;
			case QrOrientation::EAST:	//EAST
				if (markers[bottom].mass_center.x > markers[right].mass_center.x)	{
					temp_vert = bottom;
					bottom = right;
					right = temp_vert;
					// swap(bottom,right);

				}
				break;
			case QrOrientation::SOUTH:	//SOUTH
				if (markers[bottom].mass_center.x < markers[right].mass_center.x)	{
					temp_vert = bottom;
					bottom = right;
					right = temp_vert;
					// swap(bottom,right);

				}
				break;
			case QrOrientation::WEST:	//WEST
				if (markers[bottom].mass_center.x < markers[right].mass_center.x)	{
					temp_vert = bottom;
					bottom = right;
					right = temp_vert;
					// swap(bottom,right);

				}
				break;
			default:
				ROS_ERROR("qr_identifier: Orientation error..");
		}//END of switch  
	}
	// In case we cant find slope (when the 2 markers have the same x coordinate ) 
	// we will use the height difference (y coordinate) of them and the position of the top marker
	// according to them   
	else if (markers[bottom].mass_center.x == markers[right].mass_center.x)	{
		int temp_vert;
		if ( markers[bottom].mass_center.y > markers[right].mass_center.y && markers[top].mass_center.x < markers[bottom].mass_center.x )	{		
			temp_vert = bottom;
			bottom = right;
			right = temp_vert;
			orientation = QrOrientation::NORTH; //NORTH (or WEST)
		}
		else if ( markers[bottom].mass_center.y < markers[right].mass_center.y && markers[top].mass_center.x < markers[bottom].mass_center.x )	{
			// Ιncidentally the marker were set in the right way
			orientation = QrOrientation::WEST; //WEST (or NORTH)
		}
		else if ( markers[bottom].mass_center.y > markers[right].mass_center.y && markers[top].mass_center.x > markers[bottom].mass_center.x )	{
			// Ιncidentally the marker were set in the right way
			orientation = QrOrientation::EAST; //EAST (or SOUTH)
		}
		else if ( markers[bottom].mass_center.y < markers[right].mass_center.y && markers[top].mass_center.x > markers[bottom].mass_center.x )	{
			temp_vert = bottom;
			bottom = right;
			right = temp_vert;
			orientation = QrOrientation::SOUTH; //SOUTH (or EAST)
		}
	}

	// vector<QrMarker> temp{markers[bottom], markers[top], markers[right]};
	// markers = temp;
	markers = vector<QrMarker>{markers[top], markers[right], markers[bottom]};

}

// void QrScanner::sortMarkers(vector<QrMarker>& markers)
// {	
// 	if ( markers.size() != 3 )    throw std::runtime_error("QrScanner::sortMarkers() : input's size is not 3. ");

// 	int top_left, bottom_left, top_right;    // To hold the id of each marker that we located

// 	// Calculate the distance between all points
// 	float dist1,dist2,dist3;
// 	dist1 = calculateDistance(markers[0].mass_center,markers[1].mass_center);
// 	dist2 = calculateDistance(markers[1].mass_center,markers[2].mass_center);
// 	dist3 = calculateDistance(markers[2].mass_center,markers[0].mass_center);

// 	// Marker not involved in the longest distance, among them, is the 'top_left' 
// 	// Set the other 2 markers (bottom_left and top_right) at RANDOM. 
// 	if ( dist1 > dist2 && dist1 > dist3 )	{
// 		top_left = 2;		
// 		bottom_left = 0;	top_right = 1;		
// 	}
// 	else if ( dist2 > dist1 && dist2 > dist3 )	{
// 		top_left = 0;		
// 		bottom_left = 1;	top_right = 2;		
// 	}	
// 	else if ( dist3 > dist1 && dist3 > dist2 )	{
// 		top_left = 1;		
// 		bottom_left = 0;	top_right = 2;		
// 	}

// 	// Find the projection of top_left marker on the line formed by the other 2 markers
// 	Point2D proj_point = findProjection(markers[top_left].mass_center, markers[bottom_left].mass_center, markers[top_right].mass_center);
// 	Point2D top_marker = markers[top_left].mass_center;    // Used to hold the top left marker's id (just to keep the following lines cleaner)

// 	// Now we will find the orientation of top marker according to its projection
// 	int	orientation;
// 	// Orientation ///////////// (cv::Mat's axes)
// 	//           0   x --------------- x++
// 	//           y           |
// 	//           |      0    |     1
// 	//           |           |
// 	//           |  ---------P-----------
// 	//           |           |
// 	//           |      3    |    2
// 	//          y++          |
// 	////////////////////////////////////////
// 	if ( top_marker.x <= proj_point.x ) {
// 		if ( top_marker.y <= proj_point.y )    orientation = 0;
// 		else if ( top_marker.y > proj_point.y )    orientation = 3;
// 	}
// 	else if ( top_marker.x > proj_point.x ) {
// 		if ( top_marker.y <= proj_point.y )    orientation = 1;
// 		else if ( top_marker.y > proj_point.y )    orientation = 2;
// 	}

// 	if ( orientation == 0 || orientation == 3 ) {
// 		if ( markers[bottom_left].mass_center.x > markers[top_right].mass_center.x )    std::swap(bottom_left, top_right);
// 	} else if ( orientation == 1 || orientation == 2 ) {
// 		if ( markers[bottom_left].mass_center.x <= markers[top_right].mass_center.x )    std::swap(bottom_left, top_right);
// 	}

// 	markers = vector<QrMarker>{markers[bottom_left], markers[top_left], markers[top_right]};

// }

// void QrScanner::sortMarkers(vector<QrMarker>& markers)
// {	
// 	if ( markers.size() != 3 )    throw std::runtime_error("QrScanner::sortMarkers() : input's size is not 3. ");

// 	findTopLeftMarker(markers);
// 	// findTopLeftMarker2(markers);

// 	// int top_left, bottom_left, top_right;    // To hold the id of each marker that we located

// 	Point2D proj_point0 = findProjection(markers[0].mass_center, markers[1].mass_center, markers[2].mass_center);
// 	// cv::line(_some_image, markers[0].mass_center, proj_point0, cv::Scalar(0,0,255), 2);
// 	Point2D mid_point0 = findMid(markers[1].mass_center, markers[2].mass_center);
// 	float dist0 = calculateDistance(proj_point0, mid_point0);

// 	Point2D proj_point1 = findProjection(markers[1].mass_center, markers[0].mass_center, markers[2].mass_center);
// 	// cv::line(_some_image, markers[1].mass_center, proj_point1, cv::Scalar(0,255,0), 2);
// 	Point2D mid_point1 = findMiddlePoint(markers[0].mass_center, markers[2].mass_center);
// 	float dist1 = calculateDistance(proj_point1, mid_point1);

// 	Point2D proj_point2 = findProjection(markers[2].mass_center, markers[0].mass_center, markers[1].mass_center);
// 	// cv::line(_some_image, markers[2].mass_center, proj_point2, cv::Scalar(255,0,0), 2);
// 	Point2D mid_point2 = findMiddlePoint(markers[0].mass_center, markers[1].mass_center);
// 	float dist2 = calculateDistance(proj_point2, mid_point2);


// 	if ( dist0 < dist1 && dist0 < dist2 ) top_left = 0;
// 	else if ( dist1 < dist0 && dist1 < dist2 ) 	top_left = 1;

// 	else if ( dist2 < dist0 && dist2 < dist1 ) 	top_left = 2;

// }


// Iterate through markers and shift their points so that the first point ( '[0]' / 'begin()' ) is
// the outer point of qrcode.
// (One of the 4 vertices forming the qrcode's square)
void QrScanner::sortMarkersVertices(vector<QrMarker>& markers)
{	
	if ( markers.size() != 3 )    throw std::runtime_error("[QrScanner]-findVertices() : input's size is not 3. ");

	// Find the center of qr codes contour, using bottom_left and top_right markers  
	Point2D mid_point = findMiddlePoint( markers[1].mass_center, markers[2].mass_center);

    // SPECIAL COMMENT ///////////////
    // The code line below initializes a new object, not an iterator for its elements:
	// [DO NOT USE] --- " for ( auto marker : markers ) " ---
	for ( auto marker = begin(markers) ; marker != end(markers); marker++ ) {
		vector<float> distance_memory;
		distance_memory.reserve(4);

		// Create a vector to hold the distance between each point (of our marker) and mid_point
		for_each(begin(marker->contour), end(marker->contour), [&](const ContourPoint& point) {
			distance_memory.push_back(calculateDistance(point,mid_point));
		});

		if ( distance_memory.size() != marker->contour.size() )    throw std::runtime_error("[QrScanner]-findVertices() : vector's sizes are not equal. ");

		auto it = max_element(begin(distance_memory), end(distance_memory));
		int dist = std::distance( begin(distance_memory), it );
		// Shift all the elements of marker, so that the outer one lands on first position of vector ([0])
		std::rotate(marker->contour.begin(), marker->contour.begin() + dist, marker->contour.end());
	}
}

vector<Point2D> QrScanner::findSquare(const vector<QrMarker>& markers)
{	
	if ( markers.size() != 3 )    throw std::runtime_error("[QrScanner]-findSquare() : input's size is not 3. ");

	vector<Point2D> right_side_line = { markers[1].contour[0], markers[1].contour[3] };
	vector<Point2D> bottom_side_line = { markers[2].contour[0], markers[2].contour[1] };

	vector<Point2D> vertices_to_return = {  markers[0].contour[0],
											markers[1].contour[0],
											findIntersection(bottom_side_line, right_side_line),
											markers[2].contour[0]};
	return vertices_to_return; 
}

std::string QrScanner::translate(const cv::Mat &gray_input_image, const vector<Point2D>& qr_vertices)
{
		cv::Mat qr = cv::Mat::zeros(100, 100, CV_8UC3 );
		cv::Mat qr_raw = cv::Mat::zeros(100, 100, CV_8UC3 );
		cv::Mat qr_gray = cv::Mat::zeros(100, 100, CV_8UC1);
		cv::Mat qr_thres = cv::Mat::zeros(100, 100, CV_8UC1);

		std::vector<cv::Point2f> dst;
		cv::Mat warp_matrix;

		dst.push_back(cv::Point2f(0,0));
		dst.push_back(cv::Point2f(qr.cols,0));
		dst.push_back(cv::Point2f(qr.cols, qr.rows));
		dst.push_back(cv::Point2f(0, qr.rows));


		warp_matrix = getPerspectiveTransform(qr_vertices, dst);
		warpPerspective(gray_input_image, qr_raw, warp_matrix, cv::Size(qr.cols, qr.rows));
		copyMakeBorder( qr_raw, qr, 10, 10, 10, 10,cv::BORDER_CONSTANT, cv::Scalar(255,255,255) );
		threshold(qr, qr_thres, 0, 255, CV_THRESH_OTSU);

		std::string qr_translation_;
		std::stringstream qr_message_stream;

		uchar *raw = (uchar *)qr.data;  
		zbar::Image image_to_scan(qr.cols, qr.rows, "Y800", raw, qr.cols * qr.rows);        
		int nsymbols = scanner_.scan(image_to_scan);  

		for(   zbar::Image::SymbolIterator symbol = image_to_scan.symbol_begin();  
											symbol != image_to_scan.symbol_end();  
																		++symbol) {  
			qr_message_stream << symbol->get_data();
			qr_translation_ = qr_message_stream.str();
		} 

		return qr_translation_;
}

template<typename Ta, typename Tb>
double QrScanner::calculateDistance(const Ta& pointA, const Tb& pointB)
{
	return sqrt(pow(abs(pointA.x - pointB.x),2) + pow(abs(pointA.y - pointB.y),2)) ; 
}

template<typename Ta, typename Tb>
Ta QrScanner::findMiddlePoint(const Ta& pointA, const Tb& pointB)
{
	return Ta((pointA.x+pointB.x)/2, (pointA.y+pointB.y)/2);
}

template<typename T>
double QrScanner::calculateSlope(const T& pointA, const T& pointB)
{
	if (pointA.x - pointB.x == .0)    throw std::runtime_error( "[QrScanner]-calculateSlope() : division by zero encountered." );	
	return (pointA.y - pointB.y) / (pointA.x - pointB.x);
}

template<typename T>
double QrScanner::calculateAngle(const T& pointA, const T& center, const T& pointB)
{	
	double a = calculateDistance(pointA, pointB);
	double b = calculateDistance(pointA, center);
	double c = calculateDistance(pointB, center);
	return acos((pow(b,2) + pow(c,2) - pow(a,2)) / (2*a*b));
	// return angle * 180 / M_PI;
}

// Line equation: ax + by + c = 0  ///////////////////////////////////////////////////////////////////////////
template<typename T>
T QrScanner::findIntersection(const vector<T>& lineA, const vector<T>& lineB)
{		
	if ( lineA.size() != 2 && lineB.size() !=2 )    throw std::runtime_error("[QrScanner]-findIntersection() : wrong input vectors('lines') sizes. ");
	
	// Equation of each line : a*x + b*y + c = 0
	// Compute the coefficients of the first line
	double a1, b1, c1;
	calculateCoefficients(lineA[0], lineA[1], a1, b1, c1);
	// Compute the coefficients of the second line
	double a2, b2, c2;
	calculateCoefficients(lineB[0], lineB[1], a2, b2, c2);

	if ( b1 == .0 && b2 == .0 )    throw std::runtime_error("[QrScanner]-findIntersection() : parallel lines as input. ");
	if ( b1 != .0 && b2 != .0 )
		if ( (-a1/b1) == (-a2/b2) )    throw std::runtime_error("[QrScanner]-findIntersection() : parallel lines as input. ");
	
	// // Calculate the intersection point
	T cross_point;	
	if ( b1 == .0 ) {      // In case first line is vertical
		cross_point.x = lineA[0].x; 
		cross_point.y = - c2 - a2*cross_point.x;
	}
	else if ( b2 == .0 ) {      // In case second line is vertical
		cross_point.x = lineB[1].x; 
		cross_point.y = - c1 - a1*cross_point.x;
	}
	else  {
		cross_point.x = -(c1 - c2) / (a1 - a2);
		cross_point.y = - c2 - a2*cross_point.x;
	}

	return cross_point;	
}

template<typename T, typename Tc>
void QrScanner::calculateCoefficients(const T& pointA, const T& pointB, Tc &a, Tc &b, Tc &c)
{	
	if ( pointA.x - pointB.x == .0) {    // In case line is vertical (to avoid division by zero error)
		a = 1.0;
		b = .0;
		c = -pointA.x;
	} else {
		a = -(pointA.y - pointB.y) / (pointA.x - pointB.x);		
		b = 1.0;
		c = - a*pointA.x - pointA.y;
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Line equation: y = ax + b  ////////////////////////////////////////////////////////////////////////////////
// template<typename T>
// T QrScanner::findIntersection(const vector<T>& lineA, const vector<T>& lineB)
// {		
// 	if ( lineA.size() != 2 && lineB.size() !=2 )    throw std::runtime_error("QrScanner::findIntersection() : wrong input vectors('lines') sizes. ");

// 	// Compute the coefficients of the first line
// 	if ( lineA[0].x-lineA[1].x == 0.0 )    throw std::runtime_error("QrScanner::findIntersection() : division by zero encountered. ");
// 	// if ( lineA[0].x-lineA[1].x == 0.0 )     return std::numeric_limits<double>::max();    // To assign a calue approximately
// 	double a1 = (lineA[0].y - lineA[1].y) / (lineA[0].x - lineA[1].x);
// 	double b1 = lineA[1].y - a1*lineA[1].x;

// 	// Compute the coefficients of the second line
// 	if ( lineB[0].x-lineB[1].x == 0.0 )    throw std::runtime_error("QrScanner::findIntersection() : Division by zero on SECOND line. ");
// 	// if ( lineB[0].x-lineB[1].x == 0.0 )     return std::numeric_limits<double>::max();    // To assign a calue approximately
// 	double a2 = (lineB[0].y - lineB[1].y) / (lineB[0].x - lineB[1].x);
// 	double b2 = lineB[1].y - a2*lineB[1].x;

// 	if ( a1-a2 == 0 )    throw std::runtime_error("QrScanner::findIntersection() : parallel lines as input.. ");
// 	// Find the point that the 2 lines intersect each other
// 	T cross_point;		// To hold the intersection point
// 	cross_point.x = (b2 - b1) / (a1 - a2);
// 	cross_point.y = (a2*cross_point.x) + b2;

// 	return cross_point;	
// }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Line equation: ax + by + c = 0  ///////////////////////////////////////////////////////////////////////////

// template<typename T>
// T QrScanner::findProjection(const T& point, const T& point_of_lineA, const T& point_of_lineB)
// {	
// 	// Equation of each line : a*x + b*y + c = 0
// 	// Compute the coefficients of the first line
// 	double a, b, c;
// 	calculateCoefficients(point_of_lineA, point_of_lineB, a, b, c);
// 	// Compute the coefficients of the second line

// 	double a_p, b_p, c_p;
// 	if ( a == 0.0 ) {
// 		a_p = 1.0;
// 		b_p = 0.0;
// 	}
// 	if ( b == 0.0 ) {
// 		a_p = 0.0;
// 		b_p = 1.0;
// 	} else {
// 		a_p = 1.0;
// 		b_p = 1.0;	
// 	}
// 	c_p = - c/a;

// 	// Calculate the projection
// 	T projection_point;		
// 	if ( b == 0 ) projection_point.x = point_of_lineA.x;    // In case input line is vertical
// 	else if ( b_p == 0 ) projection_point.x = point.x;    // In case othogonal line is vertical
// 	else projection_point.x = -(c - c_p) / (a - a_p);
// 	projection_point.y = - c - a*projection_point.x;

// 	return projection_point;	
// }	
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Line equation: y = ax + b  ////////////////////////////////////////////////////////////////////////////////
// template<typename T>
// T QrScanner::findProjection(const T& point, const T& point_of_lineA, const T& point_of_lineB)
// {	

// 	if (point_of_lineA.x - point_of_lineB.x == 0) {
// 		projection_point.y = point.y;
// 		projection_point.x = point_of_lineA.x;

// 	} else if ( point_of_lineA.y - point_of_lineB.y == 0) {
// 		projection_point.x = point.x;
// 		projection_point.y = point_of_lineA.y;		

// 	} else {

// 		// Line equation :  y = ax + b
// 		double a, b;
// 		a = calculateSlope(point_of_lineA, point_of_lineB);
// 		b = point_of_lineA.y - a*point_of_lineA.x;

// 		// Line orthogonal to previous line  equation : a_p*x + b_p*y + c_p = 0
// 		double a_p, b_p;
// 		a_p = -1.0/a;
// 		b_p = point.y - a_p*point.x;

// 		// Find their intersection
// 		projection_point.x = -((b - b_p) / (a - a_p));
// 		projection_point.y = a*projection_point.x + b;
// 	}

// 	return projection_point;
// }	
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


template<typename T>
T QrScanner::findProjection(const T& point, const T& point_of_lineA, const T& point_of_lineB)
{	
	T projection_point;		// To hold the projection of the 'point' on the straight line

	if (point_of_lineA.x - point_of_lineB.x == 0) {
		projection_point.y = point.y;
		projection_point.x = point_of_lineA.x;

	} else if ( point_of_lineA.y - point_of_lineB.y == 0) {
		projection_point.x = point.x;
		projection_point.y = point_of_lineA.y;		

	} else {

		// Line equation :  y = ax + b
		double a, b;
		a = calculateSlope(point_of_lineA, point_of_lineB);
		b = point_of_lineA.y - a*point_of_lineA.x;

		// Line orthogonal to previous line  equation : a_p*x + b_p*y + c_p = 0
		double a_p, b_p;
		a_p = -1.0/a;
		b_p = point.y - a_p*point.x;

		// Find their intersection
		projection_point.x = -((b - b_p) / (a - a_p));
		projection_point.y = a*projection_point.x + b;
	}

	return projection_point;
}	




cv::Scalar QrScanner::randomColor()
{
	static cv::RNG rng(12345);
	return cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
}

// Draw on input image the contours that this QrIdentifier is examining
void QrScanner::drawContours(cv::Mat &inputimage, const vector<vector<ContourPoint> >& contours )
{	
	// cv::Scalar color = randomColor();
	cv::Scalar color = cv::Scalar(0,0,255);

	for( size_t i = 0; i < contours.size(); ++i )	{	// For each (of the 3) markers of our identifier
		cv::drawContours(inputimage, contours, i , color, 2, 8);		// Draw its contour (red)
		for( size_t j = 0; j < contours[i].size(); ++j )
		cv::circle(inputimage, contours[i][j], 2,  cv::Scalar(255,255,0), 2, 8, 0 );
	}
}

// Draw on input image the contours that this QrIdentifier is examining
void QrScanner::drawMassCenters(cv::Mat &inputimage, const vector<Point2D>& mass_centers )
{	
	for( int i = 0; i < mass_centers.size(); ++i )	{	// For each (of the 3) markers of our identifier
		std::ostringstream text;
		text << i;
		cv::circle(inputimage, mass_centers[i], 2,  cv::Scalar(0,255,255), 2, 8, 0 );
		cv::putText(inputimage, text.str(), mass_centers[i], cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,255,255), 2 );
	}

	// cv::circle( inputimage, mycenter, 2,  cv::Scalar(255,255,0), 2, 8, 0 );
}

// Draw on input image the contours that this QrIdentifier is examining
void QrScanner::drawVertices(cv::Mat &inputimage, const vector<Point2D>& vertices )
{	
	if ( vertices.size() != 4 )    throw std::runtime_error("[QrScanner]-drawVertices() : input's size is not 4. ");

	cv::circle(inputimage, vertices[0], 2,  cv::Scalar(255,0,0), 2, 8, 0 );
	cv::circle(inputimage, vertices[1], 2,  cv::Scalar(0,0,255), 2, 8, 0 );
	cv::circle(inputimage, vertices[2], 2,  cv::Scalar(255,0,0), 2, 8, 0 );
	cv::circle(inputimage, vertices[3], 2,  cv::Scalar(0,255,255), 5, 8, 0 );
}
// Draw on input image the contours that this QrIdentifier is examining
void QrScanner::drawMarkerVertices(cv::Mat &inputimage, const vector<QrMarker>& markers )
{	
	if ( markers.size() != 3 )    throw std::runtime_error("[QrScanner]-drawVertices() : input's size is not 4. ");

	cv::line(inputimage, markers[0].contour[0], markers[0].mass_center, cv::Scalar(255,140,140), 2);
	cv::line(inputimage, markers[1].contour[0], markers[1].mass_center, cv::Scalar(255,140,140), 2);
	cv::line(inputimage, markers[2].contour[0], markers[2].mass_center, cv::Scalar(255,140,140), 2);

	for ( auto point : markers[0].contour) 
		cv::circle(inputimage, point, 2,  cv::Scalar(0,0,255), 2, 8, 0 );

	for ( auto point : markers[1].contour)
		cv::circle(inputimage, point, 2,  cv::Scalar(0,0,255), 2, 8, 0 );

	for ( auto point : markers[2].contour)
		cv::circle(inputimage, point, 2,  cv::Scalar(0,0,255), 2, 8, 0 );

	cv::circle(inputimage,  markers[0].contour[1], 2,  cv::Scalar(255,0,0), 2, 8, 0 );
	cv::circle(inputimage,  markers[1].contour[1], 2,  cv::Scalar(255,0,0), 2, 8, 0 );
	cv::circle(inputimage,  markers[2].contour[1], 2,  cv::Scalar(255,0,0), 2, 8, 0 );


}

void QrScanner::drawSquare(cv::Mat &inputimage, const vector<Point2D>& vertices, const vector<ContourPoint>& top_marker)
{	
	if ( vertices.size() != 4 )    throw std::runtime_error("[QrScanner]-drawVertices() : input's size is not 4. ");

	cv::line(inputimage, vertices[0], vertices[1], cv::Scalar(125,255,125), 2);
	cv::line(inputimage, vertices[1], vertices[2], cv::Scalar(125,255,125), 2);
	cv::line(inputimage, vertices[2], vertices[3], cv::Scalar(125,255,125), 2);
	cv::line(inputimage, vertices[3], vertices[0], cv::Scalar(125,255,125), 2);

	// "Mark" top marker
	// cv::line(inputimage, top_marker[0], top_marker[1], cv::Scalar(0,0,255), 5);
	// cv::line(inputimage, top_marker[0], top_marker[3], cv::Scalar(0,0,255), 5);
	cv::line(inputimage, top_marker[0], top_marker[1], cv::Scalar(125,125,255), 4);
	cv::line(inputimage, top_marker[0], top_marker[3], cv::Scalar(125,125,255), 4);

}
void QrScanner::drawLines(cv::Mat &inputimage, const vector<QrMarker>& markers )
{	
	if ( markers.size() != 3 )    throw std::runtime_error("[QrScanner]-drawLines() : input's size is not 4. ");

	cv::line(inputimage, markers[0].contour[0], markers[0].contour[1], cv::Scalar(0,255,255), 2);
	cv::line(inputimage, markers[2].contour[0], markers[2].contour[3], cv::Scalar(0,255,255), 2);
		
}

////////////////////////////////////////// Debugging Functions ////////////////////////////////////////////////////

bool QrScanner::drawDetectedLabels(shared_ptr<cv::Mat> inputimage)
{	
	if ( _detected_labels.size() == 0)    return false;

	for_each(begin(_detected_labels), end(_detected_labels), [&](const vector<Point2D>& label) {
		cv::line(*inputimage, label[0], label[1], cv::Scalar(145,255,145), 2);
		cv::line(*inputimage, label[1], label[2], cv::Scalar(145,255,145), 2);
		cv::line(*inputimage, label[2], label[3], cv::Scalar(145,255,145), 2);
		cv::line(*inputimage, label[3], label[0], cv::Scalar(145,255,145), 2);

		// cv::line(*inputimage, label[1], findMiddlePoint(label[1], label[0]), cv::Scalar(125,125,255), 4);
		// cv::line(*inputimage, label[1], findMiddlePoint(label[1], label[2]), cv::Scalar(125,125,255), 4);
		cv::line(*inputimage, label[0],findMiddlePoint(label[0], findMiddlePoint(label[0], label[1])), cv::Scalar(125,125,255), 4);
		cv::line(*inputimage, label[0],findMiddlePoint(label[0], findMiddlePoint(label[0], label[3])), cv::Scalar(125,125,255), 4);
	});
	return true;
}

bool QrScanner::drawDetectedLabels(cv::Mat &inputimage)
{
	if ( _detected_labels.size() == 0)    return false;

	for_each(begin(_detected_labels), end(_detected_labels), [&](const vector<Point2D>& label) {
		cv::line(inputimage, label[0], label[1], cv::Scalar(145,255,145), 2);
		cv::line(inputimage, label[1], label[2], cv::Scalar(145,255,145), 2);
		cv::line(inputimage, label[2], label[3], cv::Scalar(145,255,145), 2);
		cv::line(inputimage, label[3], label[0], cv::Scalar(145,255,145), 2);

		// cv::line(inputimage, label[1], findMiddlePoint(label[1], label[0]), cv::Scalar(125,125,255), 4);
		// cv::line(inputimage, label[1], findMiddlePoint(label[1], label[2]), cv::Scalar(125,125,255), 4);
		cv::line(inputimage, label[0],findMiddlePoint(label[0], findMiddlePoint(label[0], label[1])), cv::Scalar(125,125,255), 4);
		cv::line(inputimage, label[0],findMiddlePoint(label[0], findMiddlePoint(label[0], label[3])), cv::Scalar(125,125,255), 4);

	});
	return true;
}

cv::Mat QrScanner::getImageOfDetectedLabels(const cv::Mat &inputimage)
{	
	cv::Mat image_to_draw = inputimage.clone();
	if ( _detected_labels.size() == 0 )    ROS_WARN("[QrScanner]-drawDetectedLabels(): there are no detected labels.");
    else {
		for_each(begin(_detected_labels), end(_detected_labels), [&](const vector<Point2D>& label) {
			cv::line(image_to_draw, label[0], label[1], cv::Scalar(145,255,145), 2);
			cv::line(image_to_draw, label[1], label[2], cv::Scalar(145,255,145), 2);
			cv::line(image_to_draw, label[2], label[3], cv::Scalar(145,255,145), 2);
			cv::line(image_to_draw, label[3], label[0], cv::Scalar(145,255,145), 2);

			// cv::line(image_to_draw, label[1], findMiddlePoint(label[1], label[0]), cv::Scalar(125,125,255), 4);
			// cv::line(image_to_draw, label[1], findMiddlePoint(label[1], label[2]), cv::Scalar(125,125,255), 4);
			cv::line(image_to_draw, label[0],findMiddlePoint(label[0], findMiddlePoint(label[0], label[1])), cv::Scalar(125,125,255), 4);
			cv::line(image_to_draw, label[0],findMiddlePoint(label[0], findMiddlePoint(label[0], label[3])), cv::Scalar(125,125,255), 4);

		});
	}
	return image_to_draw;
}



} // "namespace polymechanon_vision"