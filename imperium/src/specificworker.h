#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H
#define THRESHOLD 280

#include <iostream>
#include <fstream>
#include <algorithm>
#include <genericworker.h>
#include <innermodel/innermodel.h>
#include "grid.h"
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsEllipseItem>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void RCISMousePicker_setPick(const Pick &myPick);


	/// Grid cell definition
		struct TCell
		{
			bool free;
			bool visited;
			QGraphicsRectItem* rect;
			
			// method to save the value
			void save(std::ostream &os) const {	os << free << " " << visited; };
			void read(std::istream &is) {	is >> free >> visited ;};
		};

public slots:
	void compute();
	void initialize(int period);

private:
	std::shared_ptr<InnerModel> innerModel;
	using TDim = Grid<TCell>::Dimensions;
	Grid<TCell> grid;
	QGraphicsScene scene;
	QGraphicsView view;
	void draw();
	QGraphicsRectItem *robot;
	QGraphicsEllipseItem *noserobot;
	
	int tilesize = 70;
	int xmin, xmax, ymin, ymax;

	RoboCompGenericBase::TBaseState bState;
	RoboCompLaser::TLaserData ldata;
	
	void readRobotState();
	enum class State {IDLE,GOTO,OBSTACULO};
	Pick pick;

	float f1(float d);
	float f2(float r,float h, float Vx);

	void RCISMousePicker_setPick(Pick myPick);

	/* nuevas declaraciones */
  
};

#endif