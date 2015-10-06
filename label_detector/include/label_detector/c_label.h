
#ifndef LABEL_H
#define LABEL_H

#include <iostream>
#include <string>
#include <stdexcept> // std::runtime_error

#include <opencv2/opencv.hpp> // cv::Point_


typedef cv::Point2f Point2D;
typedef cv::Point3f Point3D;

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
	Label(const LabelType& type, std::string& text, std::vector<Point2D>& points2D);
	~Label();

	int setID();
	int getID() const;

	void setType(const LabelType& type);
	LabelType getType() const;

	void setText(std::string& label);
	std::string getText() const;

	void set2DPoints(std::vector< Point2D >& points);
	std::vector< Point2D > get2DPoints() const;
	void set3DPoints(std::vector< Point3D >& points);
	std::vector< Point3D > get3DPoints() const;
	void set3DCenter(Point3D& point);
	Point3D get3DCenter() const;

	static int getLabelsCounter();

private:
	static int _labels_counter;

	int _id;
	bool _is_distinctive;
	LabelType _type;
	std::string _label_text;

	std::vector < Point2D > _2D_points;
	std::vector < Point3D > _3D_points;
	Point3D _center;

};

} // "namespace polymechanon_vision"
#endif // LABEL_H