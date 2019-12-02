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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	qDebug() << "set params!!";
	macro = MacroEstado::BUSCARESQUINA;
	micro = MicroEstado::IDLE;
	where = true;
	RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
	innerModel = std::make_shared<InnerModel>(par.value);
	xmin = std::stoi(params.at("xmin").value);
	xmax = std::stoi(params.at("xmax").value);
	ymin = std::stoi(params.at("ymin").value);
	ymax = std::stoi(params.at("ymax").value);
	tilesize = std::stoi(params.at("tilesize").value);

	// Scene
 	scene.setSceneRect(xmin, ymin, fabs(xmin)+fabs(xmax), fabs(ymin)+fabs(ymax));
 	view.setScene(&scene);
 	view.scale(1, -1);
 	view.setParent(scrollArea);
 	view.fitInView(scene.sceneRect(), Qt::KeepAspectRatio );


	qDebug() << "Grid initialize adonkias";

	grid.initialize( TDim{ tilesize, xmin, xmax, ymin, ymax}, TCell{true, false, nullptr} );

	qDebug() << "Grid initialize ok";

	for(auto &[key, value] : grid)
 	{
	 	value.rect = scene.addRect(-tilesize/2,-tilesize/2, 100,100, QPen(Qt::NoPen));			
		value.rect->setPos(key.x,key.z);
	}

 	robot = scene.addRect(QRectF(-200, -200, 400, 400), QPen(), QBrush(Qt::blue));
 	noserobot = new QGraphicsEllipseItem(-50,100, 100,100, robot);
 	noserobot->setBrush(Qt::magenta);

	view.show();

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	this->Period = period;
	timer.start(Period);
	qDebug() << "End initialize";

}

void SpecificWorker::avance(int speed, int secs)
{
	differentialrobot_proxy->setSpeedBase(speed, 0); 
	usleep(secs);
}

void SpecificWorker::parar()
{
	differentialrobot_proxy->setSpeedBase(0,0); 
	return;
}

void SpecificWorker::giro(bool dir)
/*	Mediante el parámetro se decide si se girará 90 grados para la derecha (parámetro por defecto = no hace falta establecer el parámetro)
*	o para la izquierda (parámetro igual a false o a 0)
*/
{
	
	double giro = pi_medios/2.0;

	differentialrobot_proxy->setSpeedBase(0, (dir)?giro:-giro); 
	usleep(2*1000000);
	parar();
	return; 
}


bool SpecificWorker::esquinaEncontrada(){

	if((this->bState.x <= xmin+1.75*THRESHOLD && this->bState.z <= ymin+1.75*THRESHOLD)|| /* esquina izquierda*/
	   (this->bState.x <= xmin+1.75*THRESHOLD && this->bState.z >= ymax-1.75*THRESHOLD) || 
	   (this->bState.x >= xmax-1.75*THRESHOLD && this->bState.z >= ymax-1.75*THRESHOLD) ||
	   (this->bState.x >= xmax-1.75*THRESHOLD && this->bState.z <= ymin+1.75*THRESHOLD)) return true;
	else return false;
}

void SpecificWorker::esquinas()
{
	auto frente = this->ldata.at(this->ldata.size()/2).dist;
	switch(micro)
	{
		case MicroEstado::IDLE:

			if(frente < THRESHOLD)
				if(esquinaEncontrada()){
				macro = MacroEstado::BARRIDOH;
				giro();
				}else micro = MicroEstado::DER;
			else micro = MicroEstado::GO;
		break;
		
		case MicroEstado::DER:
			giro();
			micro = MicroEstado::IDLE;
		break;

		case MicroEstado::IZQ:
		break;
		
		case MicroEstado::GO:
			avance();
			micro = MicroEstado::IDLE;
		break;
	}
}

void SpecificWorker::barrido_horizontal()
{
	
	front.clear();
	std::copy(ldata.begin()+32, ldata.end()-32, std::back_inserter(front));
	std::sort(front.begin(), front.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
	double frente = this->front.front().dist;
	
	switch(micro)
	{
		case MicroEstado::IDLE:

			if(frente < THRESHOLD) 
			{
				micro = (where) ? MicroEstado::DER : MicroEstado::IZQ; /* true=derecha : false=izquierda*/
				where=!where;
			} else micro=MicroEstado::GO; 
			
		break;
		
		case MicroEstado::DER:		
			giro();
			ldata = laser_proxy->getLaserData();
			front.clear();
			std::copy(ldata.begin()+32, ldata.end()-32, std::back_inserter(front));
			std::sort(front.begin(), front.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
			if(this->front.front().dist < THRESHOLD) 
			{
				macro = MacroEstado::BARRIDOV;
				micro = MicroEstado::IDLE;
				giro();
				giro();
			}
			else
			{
				avance(1000, 220000); /* ajustar el pequeño avance*/
				giro();
				micro=MicroEstado::IDLE;
			}
		break;
		
		case MicroEstado::IZQ:
			giro(0);
			ldata = laser_proxy->getLaserData();
			front.clear();
			std::copy(ldata.begin()+32, ldata.end()-32, std::back_inserter(front));
			std::sort(front.begin(), front.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
			frente = this->front.front().dist;
			if(this->front.front().dist < THRESHOLD) 
			{
				macro = MacroEstado::BARRIDOV;
				micro = MicroEstado::IDLE;
				giro(0);
				giro(0);
			}
			else
			{
				avance(1000, 220000); /* ajustar el pequeño avance*/
				giro(0);
				micro=MicroEstado::IDLE;
			}
		break;
		
		case MicroEstado::GO:
			avance();
			micro=MicroEstado::IDLE;
		break;
	}
}
void SpecificWorker::barrido_vertical()
{
	front.clear();
	std::copy(ldata.begin()+32, ldata.end()-32, std::back_inserter(front));
	std::sort(front.begin(), front.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
	double frente = this->front.front().dist;
	qDebug() << "Acabo de entrar en el barrido vertical";
	qDebug() << frente;
	switch(micro)
	{
		case MicroEstado::IDLE:
			
			if(frente < THRESHOLD) 
			{
				micro = (where) ? MicroEstado::DER : MicroEstado::IZQ; /* true=derecha : false=izquierda*/
				where=!where;
			} else micro=MicroEstado::GO; 
		break;
		
		case MicroEstado::DER:
			giro();
			avance(1000, 220000); /* ajustar el pequeño avance*/
			giro();
			micro=MicroEstado::IDLE;
		break;
		
		case MicroEstado::IZQ:
			giro(0);
			avance(1000, 220000); /* ajustar el pequeño avance*/
			giro(0);
			micro=MicroEstado::IDLE;
		break;
		
		case MicroEstado::GO:
			avance();
			micro=MicroEstado::IDLE;
		break;
	}
}

void SpecificWorker::compute()
{
	readRobotState();

	switch(this->macro){

		case MacroEstado::BUSCARESQUINA:
			esquinas();
		break;

		case MacroEstado::BARRIDOH:
			barrido_horizontal();
		break;

		case MacroEstado::BARRIDOV:
			barrido_vertical();
		break;

	}

	return;
}
	



void SpecificWorker::readRobotState()
{
	try
	{
		differentialrobot_proxy->getBaseState(bState);
		innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
		ldata = laser_proxy->getLaserData();
		
		//draw robot
		robot->setPos(bState.x, bState.z);
		robot->setRotation(-180.*bState.alpha/M_PI);

		//update  occupied cells
		updateOccupiedCells(bState, ldata);
		updateVisitedCells(bState, ldata);
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from Laser" << e << std::endl;
	}
	//Resize world widget if necessary, and render the world
	if (view.size() != scrollArea->size())
	 		view.setFixedSize(scrollArea->width(), scrollArea->height());
	
}

void SpecificWorker::updateVisitedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata)
{

	// we set the cell corresponding to r as occupied 
	auto [valid, cell] = this->grid.getCell(bState.x, bState.z); 
	if(valid)
	{
		cell.visited = true;
		cell.rect->setBrush(Qt::green);
	}

}
	


void SpecificWorker::updateOccupiedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata)
{
	InnerModelLaser *n = innerModel->getNode<InnerModelLaser>(QString("laser"));
	for(auto l: ldata)
	{
		auto r = n->laserTo(QString("world"), l.dist, l.angle);	// r is in world reference system
		// we set the cell corresponding to r as occupied 
		auto [valid, cell] = grid.getCell(r.x(), r.z()); 
		if(valid)
		{
			cell.free = false;
			cell.rect->setBrush(Qt::darkRed);
		}
	}
}


///////////////////////////////////////////////////////////////////77
////  SUBSCRIPTION
/////////////////////////////////////////////////////////////////////

void SpecificWorker::RCISMousePicker_setPick(const Pick &myPick)
{
//subscribesToCODE

}



