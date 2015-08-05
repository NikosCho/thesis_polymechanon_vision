#include "label_detector/scanners/c_qr_scanner.h"

// TODO List ////////////////////////////////////////////////////////////////////////
// -> findAlignmentMarkers():
//		- Na kanw pio katanohth thn ierarxia. ( sxolia)
// -> clusterMarkers():
//		-  Na to ftiaksw kalytera.
//
/////////////////////////////////////////////////////////////////////////////////////

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

	// cv::imshow("piou piou", *_image_to_scan);	

	cv::Mat caminput_gray(_image_to_scan->size(), CV_MAKETYPE(_image_to_scan->depth(), 1));    // To hold Grayscale Image
	cv::Mat caminput_edges(caminput_gray.size(), CV_MAKETYPE(caminput_gray.depth(), 1));    // To hold Grayscale Image

	cvtColor(*_image_to_scan,caminput_gray,CV_RGB2GRAY);    // Convert Image captured from Image Input to GrayScale 
	Canny(caminput_gray, caminput_edges, 100 , 200, 3);    // Apply Canny edge detection on the img_gray image
	
	vector<vector<ContourPoints> > alignment_markers = findAlignmentMarkers(caminput_edges);
	
	// In case we found less than 2 possible alignment markers, stop scanning
	if ( alignment_markers.size() < 2 ) return false;

	vector<Point2D> mass_centers = findContoursMassCenters(alignment_markers);
	vector<vector<int> > marker_groups = clusterMarkers(mass_centers);
	


	vector<vector<vector<ContourPoints> > > marker_contours_groups = getContours(marker_groups, alignment_markers);
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// Na to allaksw etsi wste na mh xreiazetai na desmeuw olh auth th mnhmh
	// (kalytera akrivws prin ta steilw gia ksekatharisma)





	// TEST ////////////////////
	cv::Mat some_image = (*_image_to_scan).clone();
	drawContours(some_image, alignment_markers);
	drawMassCenters(some_image,mass_centers);
	std::ostringstream text;
	text << "Groups found: " << marker_contours_groups.size();	
	// cv::putText(some_image, "geia", cv::Point(0, some_image.rows), cv::FONT_HERSHEY_PLAIN, 5, cv::Scalar(0,0,255), 2);
	cv::rectangle(some_image, cv::Point(0, some_image.rows), cv::Point(some_image.cols, some_image.rows-25), cv::Scalar(0,0,0), CV_FILLED );
	// cv::putText(some_image, text.str(), cv::Point(0, some_image.rows-5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,0), 4);
	cv::putText(some_image, text.str(), cv::Point(0, some_image.rows-7), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255,255,255), 2);
	cv::imshow("Debugging-label_detector", some_image);
	////////////////////////////



	return true;
}

vector<vector<ContourPoints> > QrScanner::findAlignmentMarkers(const cv::Mat& canny_image)
{
	vector<vector<ContourPoints> > contours;
	vector<cv::Vec4i> hierarchy;   
	findContours( canny_image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);    // Find contours with full hierarchy

	vector<vector<ContourPoints> > qr_alignment_markers;    // To hold contours that meet the conditions of an alignment marker
	vector<ContourPoints> approximated_contour;    // Used to save the approximated sides of each contour

	// Start processing the contour data
	// Find Three repeatedly enclosed contours A,B,C
	// Contour enclosing other contours is assumed to be the Alignment markers of the QR code.
	int contour_counter = 0;
	for_each(begin(contours), end(contours), [&](const vector<ContourPoints>& contour) {
		vector<ContourPoints> approximated_contour;
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
			if (c == 5)	// if (c == 4)
				qr_alignment_markers.push_back(approximated_contour);    // Contours that meet the conditions of an Alignment marker are saved seperate
		}
		contour_counter++;
	});
	return qr_alignment_markers;
}


vector<Point2D> QrScanner::findContoursMassCenters(const vector<vector<ContourPoints> >& contours_vector)
{	
	vector<Point2D> mass_centers;
	mass_centers.reserve(contours_vector.size());

	for_each(begin(contours_vector), end(contours_vector), [&mass_centers](const vector<ContourPoints>& contour) {
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
		vector<int> something = {om, groups[om][1].second, groups[om][2].second};
		sort(begin(something), end(something));

		// Find if there is another group of the same elements
		bool group_is_unique = true;
		for_each(begin(groups_memory), end(groups_memory), [&](const vector<int>& vec){
			if ( something == vec)
				group_is_unique = false;
		});

		if (group_is_unique)
			groups_memory.push_back(something);
	}

	return groups_memory;
}

vector<vector<vector<ContourPoints> > > QrScanner::getContours(const vector<vector<int> >& contours_by_id, const vector<vector<ContourPoints> >& contours)
{	
	vector<vector<vector<ContourPoints> > > contours_to_return;
	contours_to_return.reserve(contours_by_id.size());

	for ( int i = 0; i < contours_by_id.size(); i++) {
		vector<vector<ContourPoints> > temp_group = {
			contours[contours_by_id[i][0]],
			contours[contours_by_id[i][1]],
			contours[contours_by_id[i][2]],
		};
		contours_to_return.push_back(temp_group);
	} 


	return contours_to_return;
}






template<typename T>
double QrScanner::calculateDistance(const T& pointA, const T& pointB)
{
	return sqrt(pow(abs(pointA.x - pointB.x),2) + pow(abs(pointA.y - pointB.y),2)) ; 
}





















// Draw on input image the contours that this QrIdentifier is examining
void QrScanner::drawContours(cv::Mat &inputimage, const vector<vector<ContourPoints> >& contours )
{	
	for( size_t i = 0; i < contours.size(); ++i )	{	// For each (of the 3) markers of our identifier
		cv::drawContours(inputimage, contours, i , cv::Scalar(0,0,255), 2, 8);		// Draw its contour (red)
		for( size_t j = 0; j < contours[i].size(); ++j )
		cv::circle(inputimage, contours[i][j], 2,  cv::Scalar(255,255,0), 2, 8, 0 );
	}

	// cv::circle( inputimage, mycenter, 2,  cv::Scalar(255,255,0), 2, 8, 0 );
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


} // "namespace polymechanon_vision"