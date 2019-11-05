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
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }



	


	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	timer.start(Period);

}

void SpecificWorker::compute()
{
	switch (this->estado)
	{
	case Estados::IDLE:
		break;
	
	case Estados::GO:
		differentialrobot_proxy->setSpeedBase(800, 0); 
		break;
	
	case Estados::ALINEACION:

		break;
	
	case Estados::OBSTACULO:

		break;
	}
}


void SpecificWorker::AlinearRobot()
{
	QVec tr = innerModel->transform("base", this->objetivo.punto, "world");
	float angulo = atan2(tr.x(),tr.z());
	return;
}

void SpecificWorker::RCISMousePicker_setPick(Pick myPick)
{
	this->objetivo.copy(myPick.x, myPick.z);
	differentialrobot_proxy->setSpeedBase(0, 0);
	estado = Estados::ALINEACION;	
	qDebug() << "[!] OBJETIVO SELECCIONADO - COORDENADAS {"<< myPick.x << "," << myPick.z << "}" << endl;
	return;
}


