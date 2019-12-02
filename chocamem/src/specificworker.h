/*
 *    Copyright (C)2019 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H
#define pi_medios 1.5707963267948965579989817342720925807952880859375
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
	RoboCompLaser::TLaserData front;

	void updateVisitedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata);
	void updateOccupiedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata);
	void readRobotState();

	/* nuevas declaraciones */

	void giro(bool dir = true);
	void parar();
	void avance(int speed = 700, int secs = 0); //250000
	bool esquinaEncontrada();
	enum class MacroEstado {BUSCARESQUINA, BARRIDOH, BARRIDOV };
	enum class MicroEstado {IDLE, GO, IZQ, DER};
	MacroEstado macro;
	MicroEstado micro;
	float firstpos;
	bool hemis_esquina;
	bool hemis_bh;
	bool hemis_bv;
	bool where;
	void esquinas();
	void barrido_horizontal();
	void barrido_vertical();

	bool fase; // true: visitados no se tratan como obstaculos - false: visitados se tratan como obstaculos  
};

#endif
