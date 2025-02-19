#ifndef QUICKHULL_H
#define QUICKHULL_H
#include <vcg/complex/complex.h>
#include"mymesh.h"
class QuickHull
{
public:
	QuickHull(std::vector<CMeshO> mesh);
	~QuickHull();

public:
	void apply();
	CMeshO getMesh();
	float getArea();
	float getVolume();
private:
	class PImpl;
	std::shared_ptr<PImpl> impl_;
};
#endif