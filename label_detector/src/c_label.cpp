#include "label_detector/c_label.h"

namespace polymechanon_vision {

Label::Label(){}

Label::~Label(){}

int Label::_labels_counter = 0;

int Label::setID()
{	
	_labels_counter++;

	static int qr_code_counter;
	if ( this->getType() == LabelType::QRCODE )	
		_id = qr_code_counter++;

	static int hzl_counter;
	if ( this->getType() == LabelType::HZL )	
		_id = hzl_counter++;

	return _id;
}

int Label::getID() const
{
	return this->_id;
}

void Label::setType(LabelType& type)
{
	_type = type;
}

LabelType Label::getType() const
{
	return this->_type;
}

void Label::setLabel(std::string& label)
{
	_label = label;
}

std::string Label::getLabel() const
{
	return this->_label;
}

int Label::getLabelsCounter()
{
	return _labels_counter;
}


}	// "namespace polymechanon_vision"
