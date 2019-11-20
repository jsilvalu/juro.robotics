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
	try
	{
		estado=Estados::IDLE;
		distancia=0.0;
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innerModel = std::make_shared<InnerModel>(innermodel_path);

	}
	catch(std::exception e) { qFatal("Error reading config params"); }

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
	differentialrobot_proxy->getBaseState(bState);
	innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
	ldata = laser_proxy->getLaserData();
	front.clear();
	std::copy(ldata.begin()+32, ldata.end()-32, std::back_inserter(front));
	std::sort(front.begin(), front.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
	frente = this->front.front().dist;

	switch (this->estado)
	{
	
	case Estados::IDLE:
		
		break;
		
	case Estados::CONDITIONS:
		if(region_objetivo_DIOS(this->objetivo.punto[0], this->objetivo.punto[2]))
		{
			differentialrobot_proxy->setSpeedBase(0, 0);
			qDebug() << " --------------------";
			qDebug() << "| OBJETIVO ALCANZADO |";
			qDebug() << " --------------------";
			estado = Estados::IDLE;

		}else if(frente <= 300)
		{
			differentialrobot_proxy->setSpeedBase(0, 0);
			estado = Estados::OBSTACULO;
			/* si estamos en el antiguo corral y la bandera esta a 1 */
			if(flag!=0) flag = 2;
			/* guardamos corral */
		} 
		break;
	
	case Estados::GO:
		differentialrobot_proxy->setSpeedBase(800, 0);
		estado = Estados::CONDITIONS; 
		break;
	
	case Estados::ALINEACION:
		alinear_robot();		
		break;
	
	case Estados::OBSTACULO:
		mano_derecha();
		break;

	case Estados::GIRO:
		differentialrobot_proxy->setSpeedBase(0, 0.6);
		break;
	}
}

void SpecificWorker::mano_derecha()
{
	QVec tr = innerModel->transform("base", this->objetivo.punto, "world");
	distancia = tr.norm2(); /* distancia euclidea */


	if(pinline()) /* punto en linea*/
	{
		
		/* alineamos el robot y vemos si la distancia euclidea es menor que la distancia al objetivo*/
		if(flag == 1) /* alineamos y pasamos a estado GO*/
		{
			alinear_robot(); //establecer una bandera al inicio para comprobar si es la primera vez que nos tyopamos con un obstaculo o no.
			estado=Estados::GO;
			return;
		
		}else if (flag == 2)
		{
			/* seguimos haciendo mano derecha */
			flag = 0
		}else flag=1;
	}

	/* CODIGO DE MANO DERECHA */

}

void SpecificWorker::alinear_robot()
{
	QVec tr = innerModel->transform("base", this->objetivo.punto, "world");
	robot_angle = atan2(tr.x(),tr.z());

	if(fabs(robot_angle)<0.05)
	{
		differentialrobot_proxy->setSpeedBase(0, 0); /* nos hemos alineado*/
		estado= Estados::GO;
		qDebug() << "[!] alineao";
		return;
	}
	differentialrobot_proxy->setSpeedBase(0, robot_angle);
	qDebug() << "valor del angulo: " << robot_angle;
	qDebug() << "x: " << this->objetivo.punto[0];
	qDebug() << "z: " << this->objetivo.punto[2];
	
	return;
}

void SpecificWorker::RCISMousePicker_setPick(Pick myPick)
{
	differentialrobot_proxy->setSpeedBase(0, 0);
	
	/* establecimiento del pto de origen*/
	differentialrobot_proxy->getBaseState(bState);
	pto_inicial.clear();
	pto_inicial.setItem(0,bState.x);
	pto_inicial.setItem(1,0);
	pto_inicial.setItem(2,bState.z);

	/* establecimiento del pto objetivo */
	this->objetivo.copy(myPick.x, myPick.z);
	QVec tr = innerModel->transform("base", this->objetivo.punto, "world");
	objective_angle = atan2(tr.x(),tr.z());
	estado = Estados::ALINEACION;
			

	qDebug() << "[!] OBJETIVO SELECCIONADO - COORDENADAS {"<< myPick.x << "," << myPick.z << "}";
	return;
}


bool SpecificWorker::pinline()
{

	//Calculo para compobar si un punto pertenece a una recta. Debe cumplirse que el resultado de la operacion:
	//d = (Y2-Y1)*X0 + (X1-X2)*Y0 + (X2*Y1-Y2*X1)  debe retornar 0. En tal caso, el punto pertenece a una recta.
	//P1 (X1,Y1)  -  Punto de origen
	//P2 (X2,Y2)  -  Punto objetivo
	//P0 (X0,Y0)  -  Punto actual
	auto d = (this->objetivo.punto[2]-pto_inicial[2])*bState.x + (pto_inicial[0]-this->objetivo.punto[0])*bState.z +(objetivo.punto[0]*pto_inicial[2]-this->objetivo.punto[2]*pto_inicial[0]);
	return (d >= -100 || d <= 100);
}
	

bool SpecificWorker::region_objetivo_DIOS(float x, float z){

	if((this->bState.x > x-200 && this->bState.z > z-200) && 
	   (this->bState.x < x+200 && this->bState.z < z+200))return true;
	else return false;
}

