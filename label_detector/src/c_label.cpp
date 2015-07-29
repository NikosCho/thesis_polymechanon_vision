#include "label_detector/c_label.h"

namespace polymechanonvision {

Label::Label()
{

}

Label::~Label()
{
	
}

int Label::setID()
{	
	static int labes_counter;
	return id_ = ++labes_counter;
}

int Label::getID() const
{
	return this->id_;
}

void Label::setType(LabelType& type)
{
	type_ = type;
}

LabelType Label::getType() const
{
	return this->type_;
}

void Label::setLabel(std::string& label)
{
	label_ = label;
}

std::string Label::getLabel() const
{
	return this->label_;
}

}	// "namespace polymechanonvision"