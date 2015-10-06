#include "label_detector/c_label.h"

using std::vector;
using std::string;

namespace polymechanon_vision {

Label::Label()
{
	_is_distinctive = false;
}

Label::Label(const LabelType& type, string& text, vector<Point2D>& points2D)
{
	setType(type);
	setText(text);
	set2DPoints(points2D);

	_is_distinctive = false;
}

Label::~Label(){}

int Label::_labels_counter = 0;

int Label::setID()
{	
	_labels_counter++;

	static int qr_code_counter;
	if ( this->getType() == LabelType::QRCODE )	{
		_id = qr_code_counter++;
		_is_distinctive = true;
	}

	static int hzl_counter;
	if ( this->getType() == LabelType::HZL ){
		_id = hzl_counter++;
		_is_distinctive = true;
	}

	return _id;
}

int Label::getID() const
{	
	if ( _is_distinctive ) 
		return this->_id;
	else    
		throw std::runtime_error("[Label]-getID() : label has no id. Use 'setID()' first.");
}

void Label::setType(const LabelType& type)
{	
	switch( type )
	{
		case LabelType::QRCODE: break;
		case LabelType::HZL: break;
		//////////////////////////////////////
		//// ANY NEW TYPE MUST BE PLACED HERE
		//////////////////////////////////////
		default:
			throw std::runtime_error("[Label]-setType() : unknown type inserted.");
	}
	_type = type;
}

LabelType Label::getType() const
{
	return this->_type;
}

void Label::setText(std::string& text)
{
	_label_text = text;
}

std::string Label::getText() const
{
	return this->_label_text;
}

void Label::set2DPoints(vector< Point2D >& points)
{
	switch( this->_type )
	{
		case LabelType::QRCODE: 
			if ( points.size() != 4 )
				throw std::runtime_error("[Label]-setPoints() : invalid size of QR vector.");
			_2D_points = points;
			break;
		case LabelType::HZL: 
			if ( points.size() != 4 )
				throw std::runtime_error("[Label]-setPoints() : invalid size of QR vector.");
			_2D_points = points;
			break;
		//////////////////////////////////////
		//// ANY NEW TYPE MUST BE PLACED HERE
		//////////////////////////////////////
		default:
			throw std::runtime_error("[Label]-setPoints() : unknown label's type inserted.");
	}
}

vector< Point2D > Label::get2DPoints() const
{
	return _2D_points;
}

void Label::set3DPoints(vector< Point3D >& points)
{
	switch( this->_type )
	{
		case LabelType::QRCODE: 
			if ( points.size() != 4 )
				throw std::runtime_error("[Label]-setPoints() : invalid size of QR vector.");
			_3D_points = points;
			break;
		case LabelType::HZL: 
			if ( points.size() != 4 )
				throw std::runtime_error("[Label]-setPoints() : invalid size of HZL vector.");
			_3D_points = points;
			break;
		//////////////////////////////////////
		//// ANY NEW TYPE MUST BE PLACED HERE
		//////////////////////////////////////
		default:
			throw std::runtime_error("[Label]-setPoints() : unknown label's type inserted.");
	}
}

vector< Point3D > Label::get3DPoints() const
{
	return _3D_points;
}

void Label::set3DCenter(Point3D& point)
{
	_center = point;
}

Point3D Label::get3DCenter() const
{
	return _center;
}

int Label::getLabelsCounter()
{
	return _labels_counter;
}


}	// "namespace polymechanon_vision"
