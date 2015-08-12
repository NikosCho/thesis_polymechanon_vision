
#ifndef LABEL_H
#define LABEL_H

#include <iostream>
#include <string>

namespace polymechanon_vision {

enum class LabelType
{
	QRCODE, 
	HZL 
};


class Label
{
public:

	Label();
	~Label();

	int setID();
	int getID() const;

	void setType(LabelType& type);
	LabelType getType() const;

	void setLabel(std::string& label);
	std::string getLabel() const;

	static int getLabelsCounter();

private:
	static int _labels_counter;

	int _id;

	LabelType _type;

	std::string _label;
};

} // "namespace polymechanon_vision"
#endif // LABEL_H