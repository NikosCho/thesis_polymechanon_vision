#ifndef LABEL_H
#define LABEL_H

#include <iostream>
#include <string>

namespace polymechanonvision {

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

private:
	int id_;
	LabelType type_;
	std::string label_;

};

} // "namespace polymechanonvision"
#endif // LABEL_H