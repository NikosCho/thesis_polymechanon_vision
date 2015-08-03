#include "label_detector/c_label.h"

namespace polymechanon_vision {

Label::Label()
{

}

Label::~Label()
{

}

int Label::setID()
{
	static int labes_counter;
	return _id = ++labes_counter;
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

}	// "namespace polymechanon_vision"
