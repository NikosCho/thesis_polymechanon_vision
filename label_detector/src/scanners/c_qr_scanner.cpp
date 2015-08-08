#include "label_detector/scanners/c_qr_scanner.h"

// TODO List ////////////////////////////////////////////////////////////////////////
// -> findAlignmentMarkers():
//		- Na kanw pio katanohth thn ierarxia. ( sxolia)
// -> clusterMarkers():
//		-  Na to ftiaksw kalytera.
// -> sortMarkers():
//		-  Na ftiaksw ena sortMarkers pou na mhn eksartatai apo to mhkos ths diagwniou metaksy top_right kai bottom_left.
//
/////////////////////////////////////////////////////////////////////////////////////

// #include <string>

namespace polymechanon_vision {

QrScanner::QrScanner() //: Scanner()
{
	ROS_WARN("QR-Scanner created!");
}

QrScanner::~QrScanner(){ ROS_ERROR("QR-Scanner deleted!"); }

LabelType QrScanner::getType() const
{
	return _type;
}

bool QrScanner::scan()
{	
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
		ROS_WARN("QrScanner::scan() : no clusters returned, even if the markers were 3.");
		return false;	
	}

	// bool groups_size_is_ok = all_of(begin(marker_groups), end(marker_groups), [&](const <vector<int> > group){
	// 							 return group.size() == 3;
	// 						 });

	// if ( !groups_size_is_ok ) {
	// 	ROS_WARN("QrScanner::scan() : at least one of clusters has wrong size ( not 3).");
	// 	return false;	
	// }	

	cv::Mat some_image = (*_image_to_scan).clone();

	for ( auto marker_group : marker_groups ) {
		vector<QrMarker> qr_markers = getMarkers(marker_group, alignment_markers, mass_centers);
		sortMarkers(qr_markers);    // 0-Bottom_left , 1-Top_left , 2 - Top_right

		vector<Point2D> qr_code_vertices = findVertices(qr_markers);


		cv::putText(some_image, "BL" , qr_markers[0].mass_center, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(51, 153, 255), 2, 8);
		cv::putText(some_image, "TL" , qr_markers[1].mass_center, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(153, 51, 255), 2, 8);
		cv::putText(some_image, "TR" , qr_markers[2].mass_center, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(51, 255, 153), 2, 8);


		// drawVertices(some_image,qr_markers);

		Point2D mid_point = findMid( qr_markers[0].mass_center, qr_markers[2].mass_center);
		cv::circle(some_image, mid_point, 2,  cv::Scalar(0,255,255), 3, 8, 0 );


		cv::circle(some_image, qr_markers[0].contour[0], 2,  cv::Scalar(0,0,255), 2, 8, 0 );
		cv::circle(some_image, qr_markers[0].contour[1], 2,  cv::Scalar(255,0,0), 2, 8, 0 );
		cv::circle(some_image, qr_markers[0].contour[2], 2,  cv::Scalar(255,0,0), 2, 8, 0 );
		cv::circle(some_image, qr_markers[0].contour[3], 2,  cv::Scalar(255,0,0), 2, 8, 0 );
		cv::circle(some_image, qr_markers[1].contour[0], 2,  cv::Scalar(0,0,255), 2, 8, 0 );
		cv::circle(some_image, qr_markers[1].contour[1], 2,  cv::Scalar(255,0,0), 2, 8, 0 );
		cv::circle(some_image, qr_markers[1].contour[2], 2,  cv::Scalar(255,0,0), 2, 8, 0 );
		cv::circle(some_image, qr_markers[1].contour[3], 2,  cv::Scalar(255,0,0), 2, 8, 0 );
		cv::circle(some_image, qr_markers[2].contour[0], 2,  cv::Scalar(0,0,255), 2, 8, 0 );
		cv::circle(some_image, qr_markers[2].contour[1], 2,  cv::Scalar(255,0,0), 2, 8, 0 );
		cv::circle(some_image, qr_markers[2].contour[2], 2,  cv::Scalar(255,0,0), 2, 8, 0 );
		cv::circle(some_image, qr_markers[2].contour[3], 2,  cv::Scalar(255,0,0), 2, 8, 0 );
		

	}







	// TEST ////////////////////
	// cv::Mat some_image = (*_image_to_scan).clone();
	// drawContours(some_image, alignment_markers);
	// drawMassCenters(some_image,mass_centers);
	std::ostringstream text;
	text << "Groups found: " << marker_groups.size();	
	// cv::putText(some_image, "geia", cv::Point(0, some_image.rows), cv::FONT_HERSHEY_PLAIN, 5, cv::Scalar(0,0,255), 2);
	cv::rectangle(some_image, cv::Point(0, some_image.rows), cv::Point(some_image.cols, some_image.rows-25), cv::Scalar(0,0,0), CV_FILLED );
	// cv::putText(some_image, text.str(), cv::Point(0, some_image.rows-5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,0), 4);
	cv::putText(some_image, text.str(), cv::Point(0, some_image.rows-7), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255), 1);

	// DRAW XYZ - - - http://docs.opencv.org/master/d7/d53/tutorial_py_pose.html#gsc.tab=0

	cv::imshow("Debugging-label_detector", some_image);
	////////////////////////////



	return true;
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
	if ( contours_mass_centers.size() < 3 )    throw std::runtime_error("QrScanner::clusterMarkers() : input's size is less than 3. ");

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
		ROS_ERROR("QrScanner::getMassCenters() : While accesing vector(mass_centers) by id \"out_of_range\" encountered");
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
		ROS_ERROR("QrScanner::getContours() : While accesing vector(contours) by id \"out_of_range\" encountered");
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
		ROS_ERROR("QrScanner::getMarkers() : While accesing vector(contours or mass_centers) by id \"out_of_range\" encountered");
	}

	return group_to_return;
}

// void QrScanner::sortMarkers(vector<QrMarker>& markers)
// {
// 	int top,bottom,right;
// 	QrOrientation orientation;

// 	float dist1,dist2,dist3;
// 	dist1 = calculateDistance(markers[0].mass_center,markers[1].mass_center);
// 	dist2 = calculateDistance(markers[1].mass_center,markers[2].mass_center);
// 	dist3 = calculateDistance(markers[2].mass_center,markers[0].mass_center);

// 	if ( dist1 > dist2 && dist1 > dist3)	{		// If the longest distance is 'dist1'
// 		top=2;		// Set the top marker
// 		bottom=0;	right=1;		// Set the other 2 vertices randomly
// 	}
// 	else if ( dist2 > dist1 && dist2 > dist3)	{		// If the longest distance is 'dist2'
// 		top=0;		// Set the top marker
// 		bottom=1;	right=2;		// Set the other 2 vertices randomly
// 	}	
// 	else if ( dist3 > dist1 && dist3 > dist2)	{		// If the longest distance is 'dist3'
// 		top=1;		// Set the top marker
// 		bottom=0;	right=2;		// Set the other 2 vertices randomly
// 	}

// 	// Now we are ready to check which one of the other 2 markers is the 'bottom'
// 	// and which is the 'right' one
// 	if ( markers[bottom].mass_center.x != markers[right].mass_center.x )	{		//To avoid divide by 0
// 		float diagonal_slope;
// 		diagonal_slope = calculateSlope(markers[bottom].mass_center, markers[right].mass_center);
// 		Point2D proj_point=findProjection(markers[top].mass_center, markers[right].mass_center, markers[bottom].mass_center);

// 		if ( diagonal_slope > 0 && proj_point.y > markers[top].mass_center.y)	{
// 			orientation = QrOrientation::NORTH;	//NORTH
// 		}
// 		else if ( diagonal_slope < 0 && proj_point.y > markers[top].mass_center.y)	{
// 			orientation = QrOrientation::EAST;	//EAST
// 		}
// 		else if ( diagonal_slope > 0 && proj_point.y < markers[top].mass_center.y)	{
// 			orientation = QrOrientation::SOUTH;	//SOUTH
// 		}
// 		else if ( diagonal_slope < 0 && proj_point.y < markers[top].mass_center.y)	{
// 			orientation = QrOrientation::WEST;	//WEST
// 		}

// 		int temp_vert;
// 		switch(orientation)	{
// 			case QrOrientation::NORTH:	//NORTH
// 				if (markers[bottom].mass_center.x > markers[right].mass_center.x)	{
// 					temp_vert = bottom;
// 					bottom = right;
// 					right = temp_vert;
// 					// swap(bottom,right);
// 				}
// 				break;
// 			case QrOrientation::EAST:	//EAST
// 				if (markers[bottom].mass_center.x > markers[right].mass_center.x)	{
// 					temp_vert = bottom;
// 					bottom = right;
// 					right = temp_vert;
// 					// swap(bottom,right);

// 				}
// 				break;
// 			case QrOrientation::SOUTH:	//SOUTH
// 				if (markers[bottom].mass_center.x < markers[right].mass_center.x)	{
// 					temp_vert = bottom;
// 					bottom = right;
// 					right = temp_vert;
// 					// swap(bottom,right);

// 				}
// 				break;
// 			case QrOrientation::WEST:	//WEST
// 				if (markers[bottom].mass_center.x < markers[right].mass_center.x)	{
// 					temp_vert = bottom;
// 					bottom = right;
// 					right = temp_vert;
// 					// swap(bottom,right);

// 				}
// 				break;
// 			default:
// 				ROS_ERROR("qr_identifier: Orientation error..");
// 		}//END of switch  
// 	}
// 	// In case we cant find slope (when the 2 markers have the same x coordinate ) 
// 	// we will use the height difference (y coordinate) of them and the position of the top marker
// 	// according to them   
// 	else if (markers[bottom].mass_center.x == markers[right].mass_center.x)	{
// 		int temp_vert;
// 		if ( markers[bottom].mass_center.y > markers[right].mass_center.y && markers[top].mass_center.x < markers[bottom].mass_center.x )	{		
// 			temp_vert = bottom;
// 			bottom = right;
// 			right = temp_vert;
// 			orientation = QrOrientation::NORTH; //NORTH (or WEST)
// 		}
// 		else if ( markers[bottom].mass_center.y < markers[right].mass_center.y && markers[top].mass_center.x < markers[bottom].mass_center.x )	{
// 			// Ιncidentally the marker were set in the right way
// 			orientation = QrOrientation::WEST; //WEST (or NORTH)
// 		}
// 		else if ( markers[bottom].mass_center.y > markers[right].mass_center.y && markers[top].mass_center.x > markers[bottom].mass_center.x )	{
// 			// Ιncidentally the marker were set in the right way
// 			orientation = QrOrientation::EAST; //EAST (or SOUTH)
// 		}
// 		else if ( markers[bottom].mass_center.y < markers[right].mass_center.y && markers[top].mass_center.x > markers[bottom].mass_center.x )	{
// 			temp_vert = bottom;
// 			bottom = right;
// 			right = temp_vert;
// 			orientation = QrOrientation::SOUTH; //SOUTH (or EAST)
// 		}
// 	}

// 	// vector<QrMarker> temp{markers[bottom], markers[top], markers[right]};
// 	// markers = temp;
// 	markers = vector<QrMarker>{markers[bottom], markers[top], markers[right]};

// }

void QrScanner::sortMarkers(vector<QrMarker>& markers)
{	
	if ( markers.size() != 3 )    throw std::runtime_error("QrScanner::sortMarkers() : input's size is not 3. ");

	int top_left, bottom_left, top_right;    // To hold the id of each marker that we located

	// Calculate the distance between all points
	float dist1,dist2,dist3;
	dist1 = calculateDistance(markers[0].mass_center,markers[1].mass_center);
	dist2 = calculateDistance(markers[1].mass_center,markers[2].mass_center);
	dist3 = calculateDistance(markers[2].mass_center,markers[0].mass_center);

	// Marker not involved in the longest distance, among them, is the 'top_left'
	// if ( dist1 > dist2 && dist1 > dist3)    top_left = 2;
	// else if ( dist2 > dist1 && dist2 > dist3)    top_left = 0;	
	// else if ( dist3 > dist1 && dist3 > dist2)    top_left = 1;	
	if ( dist1 > dist2 && dist1 > dist3 )	{
		top_left = 2;		
		bottom_left = 0;	top_right = 1;		// Set the other 2 vertices randomly
	}
	else if ( dist2 > dist1 && dist2 > dist3 )	{
		top_left = 0;		
		bottom_left = 1;	top_right = 2;		// Set the other 2 vertices randomly
	}	
	else if ( dist3 > dist1 && dist3 > dist2 )	{
		top_left = 1;		
		bottom_left = 0;	top_right = 2;		// Set the other 2 vertices randomly
	}

	// Find the projection of top_left marker on the line formed by the other 2 markers
	Point2D proj_point = findProjection(markers[top_left].mass_center, markers[bottom_left].mass_center, markers[top_right].mass_center);
	
	Point2D top_marker = markers[top_left].mass_center;    // Used to hold the top left marker's id (just to keep follwing lines cleaner)
	
	// cout << top_marker.x << " , " << top_marker.y << endl;

	// Now we will find the orientation of top marker according to its projection
	int	orientation;
	// Orientation //////////
	//           0   x --------------- x++
	//           y           |
	//           |      0    |     1
	//           |           |
	//           |  ---------P-----------
	//           |           |
	//          y++     3    |    2
	//                       |
	////////////////////////////////////////
	if ( top_marker.x <= proj_point.x ) {
		if ( top_marker.y <= proj_point.y )    orientation = 0;
		else if ( top_marker.y > proj_point.y )    orientation = 3;
	}
	else if ( top_marker.x > proj_point.x ) {
		if ( top_marker.y <= proj_point.y )    orientation = 1;
		else if ( top_marker.y > proj_point.y )    orientation = 2;
	}

	if ( orientation == 0 || orientation == 3 ) {
		if ( markers[bottom_left].mass_center.x > markers[top_right].mass_center.x )    std::swap(bottom_left, top_right);
	} else if ( orientation == 1 || orientation == 2 ) {
		if ( markers[bottom_left].mass_center.x <= markers[top_right].mass_center.x )    std::swap(bottom_left, top_right);
	}

	markers = vector<QrMarker>{markers[bottom_left], markers[top_left], markers[top_right]};

}

vector<Point2D> QrScanner::findVertices(vector<QrMarker>& markers)
{	
	if ( markers.size() != 3 )    throw std::runtime_error("QrScanner::sortMarkers() : input's size is not 3. ");

	// Find the center of qr codes contour, using bottom_left and top_right markers  
	Point2D mid_point = findMid( markers[0].mass_center, markers[2].mass_center);

	float maxdistance;		// To hold the maximum distance calculated (for each pair of center and contour)
	float tempdistance;		// To temporarily save the distance to the contour we are examining

	vector<float> distance_memory;
	distance_memory.reserve(3);

	// Iterate through markers, calculate the distance between 
    
	// for ( auto marker : markers ) {
	for ( auto marker = begin(markers) ; marker != end(markers); marker++ ) {

		for_each(begin(marker->contour), end(marker->contour), [&](const ContourPoint& point) {
			distance_memory.push_back(calculateDistance(point,mid_point));
		});

		auto mark = marker->contour;

		cout << "===================================" << endl;
		cout << "Distances - [";
		for_each(begin(distance_memory), end(distance_memory),[](int num){  cout << num << " ,";   });
		cout << " ] " << endl;

		auto it = max_element(begin(distance_memory), end(distance_memory));

		cout << "MAX - ["<< *it << " ] " << endl;

		int dist = distance( begin(distance_memory), it );

		cout << "Dist - ["<< dist << " ] " << endl;


		cout << "Before - [";
		for_each(marker->contour.begin(),marker->contour.end(),[](ContourPoint num){  cout << " (" << num.x << " ," << num.y << ") ";   });
		cout << " ] " << endl;

		// if ( distance_memory.size() == marker.contour.size()){
			// std::rotate(begin(marker.contour), begin(marker.contour) + dist, end(marker.contour));
			std::rotate(marker->contour.begin(), marker->contour.begin() + dist, marker->contour.end());
			// cout << "yieeeeeeeee"<< endl;
		// }



		cout << "Rotated - [";
		for_each(marker->contour.begin(),marker->contour.end(),[](ContourPoint num){  cout << " (" << num.x << " ," << num.y << ") ";   });
		cout << " ] " << endl;

		distance_memory.clear();
	}

	// WTF ????????????????????????????????????????????????????????????????????????

	// std::vector<int>::iterator marker;
	for ( auto marker = begin(markers) ; marker != end(markers); marker++ ) {

	// for ( auto marker : markers ) {

	// for_each( begin(markers), end(markers), 



		cout << "AGAIN - [";
		for_each(marker->contour.begin(),marker->contour.end(),[](ContourPoint num){  cout << " (" << num.x << " ," << num.y << ") ";   });
		cout << " ] " << endl;

		// distance_memory.clear();

		cout << " =TYPE --- " << typeid(marker).name() << endl;

	// });
	}



		// vector<int> distance_memory = { 1,2,3,4,5,20,6,7,8,9,10,};
		// cout << "BEFORE - [";
		// for_each(begin(distance_memory), end(distance_memory),[](int num){  cout << num << " ,";   });
		// cout << " ] " << endl;


		// auto it = max_element(begin(distance_memory), end(distance_memory));
		// std::rotate(begin(distance_memory), it, end(distance_memory));
		// cout << "AFTER - [";
		// for_each(begin(distance_memory), end(distance_memory),[](int num){  cout << num << " ,";   });
		// cout << " ] " << endl;



	vector<Point2D> vertices_to_return;

	return vertices_to_return; 
}

// vector<Point2D> QrScanner::findVertices(vector<QrMarker>& markers)
// {	
// 	if ( markers.size() != 3 )    throw std::runtime_error("QrScanner::sortMarkers() : input's size is not 3. ");

// 	// Find the center of qr codes contour, using bottom_left and top_right markers  
// 	Point2D mid_point = findMid( markers[0].mass_center, markers[2].mass_center);

// 	float maxdistance;		// To hold the maximum distance calculated (for each pair of center and contour)
// 	float tempdistance;		// To temporarily save the distance to the contour we are examining

// 	vector<float> distance_memory;
// 	distance_memory.reserve(3);

// 	// Iterate through markers, calculate the distance between 

// 	for ( auto marker : markers ) {
// 		for_each(begin(marker.contour), end(marker.contour), [&](const ContourPoint& point) {
// 			distance_memory.push_back(calculateDistance(point,mid_point));
// 		});

// 		auto mark = marker.contour;

// 		cout << "===================================" << endl;
// 		cout << "Distances - [";
// 		for_each(begin(distance_memory), end(distance_memory),[](int num){  cout << num << " ,";   });
// 		cout << " ] " << endl;

// 		auto it = max_element(begin(distance_memory), end(distance_memory));

// 		cout << "MAX - ["<< *it << " ] " << endl;

// 		int dist = distance( begin(distance_memory), it );

// 		cout << "Dist - ["<< dist << " ] " << endl;


// 		cout << "Before - [";
// 		for_each(marker.contour.begin(),marker.contour.end(),[](ContourPoint num){  cout << " (" << num.x << " ," << num.y << ") ";   });
// 		cout << " ] " << endl;

// 		// if ( distance_memory.size() == marker.contour.size()){
// 			// std::rotate(begin(marker.contour), begin(marker.contour) + dist, end(marker.contour));
// 			std::rotate(marker.contour.begin(), marker.contour.begin() + dist, marker.contour.end());
// 			// cout << "yieeeeeeeee"<< endl;
// 		// }



// 		cout << "Rotated - [";
// 		for_each(marker.contour.begin(),marker.contour.end(),[](ContourPoint num){  cout << " (" << num.x << " ," << num.y << ") ";   });
// 		cout << " ] " << endl;

// 		distance_memory.clear();
// 	}

// 	// WTF ????????????????????????????????????????????????????????????????????????

// 	// std::vector<int>::iterator marker;
// 	for ( auto marker = begin(markers) ; marker != end(markers); marker++ ) {

// 	// for ( auto marker : markers ) {

// 	// for_each( begin(markers), end(markers), 



// 		cout << "AGAIN - [";
// 		for_each(marker->contour.begin(),marker->contour.end(),[](ContourPoint num){  cout << " (" << num.x << " ," << num.y << ") ";   });
// 		cout << " ] " << endl;

// 		// distance_memory.clear();

// 		cout << " =TYPE --- " << typeid(marker).name() << endl;

// 	// });
// 	}



// 		// vector<int> distance_memory = { 1,2,3,4,5,20,6,7,8,9,10,};
// 		// cout << "BEFORE - [";
// 		// for_each(begin(distance_memory), end(distance_memory),[](int num){  cout << num << " ,";   });
// 		// cout << " ] " << endl;


// 		// auto it = max_element(begin(distance_memory), end(distance_memory));
// 		// std::rotate(begin(distance_memory), it, end(distance_memory));
// 		// cout << "AFTER - [";
// 		// for_each(begin(distance_memory), end(distance_memory),[](int num){  cout << num << " ,";   });
// 		// cout << " ] " << endl;



// 	vector<Point2D> vertices_to_return;

// 	return vertices_to_return; 
// }


template<typename Ta, typename Tb>
double QrScanner::calculateDistance(const Ta& pointA, const Tb& pointB)
{
	return sqrt(pow(abs(pointA.x - pointB.x),2) + pow(abs(pointA.y - pointB.y),2)) ; 
}

template<typename T>
double QrScanner::calculateSlope(const T& pointA, const T& pointB)
{
	float dx,dy;
	dx = pointA.x - pointB.x;
	dy = pointA.y - pointB.y;

	if (dx == 0)
		throw std::runtime_error( "QrScanner::calculateSlope() : division by zero encountered." );

	return -(dy / dx);
}
	
// ERROR HANDLING ////////////////////////

// try{
// 	calculateSlope(point1,point2);
// } catch (const std::exception& e) {
// 	ROS_WARN("%s", e.what());
// }

//////////////////////////////////////////

//Find the projection (another Point) of a Point on the straight line formed by 2 other points 
template<typename T>
T QrScanner::findProjection(const T& point, const T& point_of_lineA, const T& point_of_lineB)
{	
	T projection_point;		// To hold the projection of the 'point' on the straight line
	// ax+by+c=0 , where b=1.0
	float a,b,c;
	// Find the coefficients of the line
	a = -(point_of_lineA.y-point_of_lineB.y)/(point_of_lineA.x-point_of_lineB.x);		// a=-dy/dx
	b = 1.0;
	c = -a*point_of_lineA.x-point_of_lineA.y;		// c=-a*x-y
	// Find the coefficients of the vertical line formed by 'point'
	float a2,b2,c2;
	a2 = -1/a;		// The 2 lines are vertical
	b2 = 1.0;
	c2 = -a2*point.x-point.y;
	// Find the point that the 2 lines intersect each other
	projection_point.x = -(c - c2)/(a-a2);
	projection_point.y = -c-a*projection_point.x;

	return projection_point;
}	

// Function used to find the midpoint between two points
template<typename Ta, typename Tb>
Ta QrScanner::findMid(const Ta& pointA, const Tb& pointB)
{
	Ta mid_point;	// To hold the midpoint
	mid_point.x = (pointA.x+pointB.x)/2;
	mid_point.y = (pointA.y+pointB.y)/2;
	return mid_point;	
}

// template<typename T>
// T findIntersection(const T& point1A, const T& point1B, const T& point2A, const T& point2B)
// {
// 	cv::Point2f crossPoint;		// To hold the intersection point
// 	// First we compute the equation of the first line
// 	float a1,b1,c1;		// The 3 coefficients of our line
// 	if(Line1.size() == 2 )	{
// 		// Find the coefficients of the equation
// 		a1 = -(Line1[0].y-Line1[1].y)/(Line1[0].x-Line1[1].x);		// a=-dy/dx
// 		b1 = 1.0;
// 		c1 = -a1*Line1[0].x-Line1[0].y;		// c=-a*x-y
// 	}	else 	ROS_ERROR("qr_identifier: Wrond input items for intersection point's calculation");
// 	// Now we compute equation of the second line
// 	float a2,b2,c2;		// The 3 coefficients of our line
// 	if(Line2.size() == 2 )	{
// 		// Find the coefficients of the line RIGHT
// 		a2 = -(Line2[0].y-Line2[1].y)/(Line2[0].x-Line2[1].x);		// a=-dy/dx
// 		b2 = 1.0;
// 		c2 = -a2*Line2[0].x-Line2[0].y;		// c=-a*x-y
// 	}	else 	ROS_ERROR("qr_identifier: Wrond input items for intersection point's calculation");
// 	// Find the point that the 2 lines intersect each other
// 	crossPoint.x = -(c1-c2)/(a1-a2);
// 	crossPoint.y = -c1-a1*crossPoint.x;
	
// 	return crossPoint;	
// }











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
// void QrScanner::drawVertices(cv::Mat &inputimage, const vector<vector<ContourPoint> >& contours )
// {	
// 	for( size_t i = 0; i < contours.size(); ++i )	{	// For each (of the 3) markers of our identifier
// 		cv::circle(inputimage, contours[i][0], 2,  cv::Scalar(0,0,255), 2, 8, 0 );
// 		cv::circle(inputimage, contours[i][1], 2,  cv::Scalar(255,0,0), 2, 8, 0 );
// 		cv::circle(inputimage, contours[i][2], 2,  cv::Scalar(255,0,0), 2, 8, 0 );
// 		cv::circle(inputimage, contours[i][3], 2,  cv::Scalar(255,0,0), 2, 8, 0 );
// 	}
// }




} // "namespace polymechanon_vision"