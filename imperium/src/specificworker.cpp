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
		manoderecha=EstadosMD::INICIAL;
		distancia=0.0;
		pto_inicial = QVec::zeros(3);
		pto_normalX = QVec::zeros(3);
		pto_normalY = QVec::zeros(3);
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

void SpecificWorker::update_laser()
{
	innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
	ldata = laser_proxy->getLaserData();
	front.clear();
	std::copy(ldata.begin()+32, ldata.end()-32, std::back_inserter(front));
	std::sort(front.begin(), front.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
	frente = this->front.front().dist;
}

void SpecificWorker::compute()
{
	differentialrobot_proxy->getBaseState(bState);
	update_laser();

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
			qDebug() << "[!] Me he parado!";
			estado = Estados::OBSTACULO;
			manoderecha = EstadosMD::INICIAL;
			QVec tr = innerModel->transform("base", this->objetivo.punto, "world");
			dist_eu = tr.norm2();
			qDebug() << "[!] Distancia con el objetivo: " << dist_eu;
		} 
		break;
	
	case Estados::GO:
		differentialrobot_proxy->setSpeedBase(800, 0);
		estado = Estados::CONDITIONS; 
		break;
	
	case Estados::ALINEACION:
		if(flag == 0){if(alinear_robot(this->objetivo.punto)) estado = Estados::GO;} 	
		else if(flag == 1)
		{
			if(alinear_robot(this->pto_normalX))
			{
				update_laser();
				if(frente <= 300)
				{
					flag = 2;
					izq = !izq;
					up=!up;
				}
				else
				{
					estado = Estados::OBSTACULO;
					manoderecha = EstadosMD::IDLE;
					differentialrobot_proxy->setSpeedBase(400, 0);
				} 
			}
		}else if(alinear_robot(this->pto_normalY)) 
		{
			estado = Estados::OBSTACULO;
			manoderecha = EstadosMD::IDLE;
			differentialrobot_proxy->setSpeedBase(400, 0);
		}
		
		break;
	
	case Estados::OBSTACULO:
		mano_derecha();
		break;

	case Estados::GIRO:
		differentialrobot_proxy->setSpeedBase(0, 0.6);
		break;
	}
}

void SpecificWorker::mano_derecha()  //modo = false, mano derecha normal  modo = true, mano derecha sin alinear
{
	QVec tr = innerModel->transform("base", this->objetivo.punto, "world");

	qDebug() << "[!] Distancia actual con el objetivo: " << tr.norm2();

	qDebug() << "[!] Distancia calculada para la comparación: " << dist_eu-100;

	if(pinline() && tr.norm2() < dist_eu-100) /* punto en linea*/
	{
		qDebug() << "[!] Punto en linea y distancia euclidea menor!";
		flag = 0;
		estado = Estados::ALINEACION;
		return;
	}

	/* CODIGO DE MANO DERECHA */
	switch (manoderecha)
	{
		case EstadosMD::INICIAL:
			qDebug() << "[!] MANO DERECHA SUBESTADO INICIAL!";
			flag = 1;
			padondegiro();
		break;
	
		case EstadosMD::IDLE:
			update_laser();
			if(ldata.begin() > 500){
				differentialrobot_proxy->setSpeedBase(0, 0); /* hay que crear las variables booleanas up y izq */
				if(flag==1)
				{
					flag=2;	
					if(up) /* up = true ya he girado hacia arriba */
					{
						pto_normalY.setItem(0,bState.x);
						pto_normalY.setItem(1,0);
						pto_normalY.setItem(2,bState.z-1000);
						estado = Estados::ALINEACION;
						up=!up;
					}else
					{
						pto_normalY.setItem(0,bState.x);
						pto_normalY.setItem(1,0);
						pto_normalY.setItem(2,bState.z+1000);
						estado = Estados::ALINEACION;
						up=!up;						
					}
					
				}else if(flag==2) /* alternamos giros */
				{
					flag=1; /* me alineo con x*/
					
					if(izq) /*contador para saber hacia donde giro: true = he girado ya a la izquierda*/
					{
						pto_normalX.setItem(0,1000+bState.x);
						pto_normalX.setItem(1,0);
						pto_normalX.setItem(2,bState.z);
						estado = Estados::ALINEACION;
						izq=!izq;
					}else
					{
						pto_normalX.setItem(0,bState.x-1000);
						pto_normalX.setItem(1,0);
						pto_normalX.setItem(2,bState.z);
						estado = Estados::ALINEACION;
						izq=!izq;
					}	
					
				}	
			}
		break;
	
		default:
		break;
	}
	
}

bool SpecificWorker::alinear_robot(QVec point)
{
	QVec tr = innerModel->transform("base", point, "world");
	robot_angle = atan2(tr.x(),tr.z());

	if(fabs(robot_angle)<0.005)
	{
		differentialrobot_proxy->setSpeedBase(0, 0); /* nos hemos alineado*/
		
		return true;
	}
	differentialrobot_proxy->setSpeedBase(0, robot_angle);
	
	return false;
}

void SpecificWorker::padondegiro()
{
	if(bState.x > pto_inicial[0] && bState.z < pto_inicial[2]) /* izq superior: alineamos con +x y -z*/
	{
		pto_normalX.setItem(0,1000+bState.x);
		pto_normalX.setItem(1,0);
		pto_normalX.setItem(2,bState.z);
		pto_normalY.setItem(0,bState.x);
		pto_normalY.setItem(1,0);
		pto_normalY.setItem(2,bState.z-1000);
		up = true;
		izq = false;
		estado = Estados::ALINEACION;
	}
	else if(bState.x > pto_inicial[0] && bState.z > pto_inicial[2])  /* izq abajo: alineamos con +x y +z */ 
	{
		pto_normalX.setItem(0,1000+bState.x);
		pto_normalX.setItem(1,0);
		pto_normalX.setItem(2,bState.z);
		pto_normalY.setItem(0,bState.x);
		pto_normalY.setItem(1,0);
		pto_normalY.setItem(2,1000+bState.z);
		up = false;
		izq = false;
		estado = Estados::ALINEACION;
	}
	else if(bState.x < pto_inicial[0] && bState.z < pto_inicial[2])  /* der superior: alineamos con -x y -z*/ 
	{
		pto_normalX.setItem(0,bState.x-1000);
		pto_normalX.setItem(1,0);
		pto_normalX.setItem(2,bState.z);
		pto_normalY.setItem(0,bState.x);
		pto_normalY.setItem(1,0);
		pto_normalY.setItem(2,bState.z-1000);
		estado = Estados::ALINEACION;	
		up = true;
		izq = true;
	}
	else if(bState.x < pto_inicial[0] && bState.z > pto_inicial[2])  /* der abajo: alineamos con -x y +z*/ 
	{
		pto_normalX.setItem(0,bState.x-1000);
		pto_normalX.setItem(1,0);
		pto_normalX.setItem(2,bState.z);
		pto_normalY.setItem(0,bState.x);
		pto_normalY.setItem(1,0);
		pto_normalY.setItem(2,1000+bState.z);
		up = false;
		izq = true;
		estado = Estados::ALINEACION;	
	}
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
	flag = 0;
	estado = Estados::ALINEACION;
			

	qDebug() << "[!] OBJETIVO SELECCIONADO - COORDENADAS {"<< myPick.x << "," << myPick.z << "}";
	return;
}


bool SpecificWorker::pinline()
/*
	Calculo para compobar si un punto pertenece a una recta. Debe cumplirse que el resultado de la operacion:
	d = (Y2-Y1)*X0 + (X1-X2)*Y0 + (X2*Y1-Y2*X1)  debe retornar 0. En tal caso, el punto pertenece a una recta.
	P1 (X1,Y1)  -  Punto de origen
	P2 (X2,Y2)  -  Punto objetivo
	P0 (X0,Y0)  -  Punto actual 
*/
{
	auto d = (this->objetivo.punto[2]-pto_inicial[2])*bState.x + (pto_inicial[0]-this->objetivo.punto[0])*bState.z +(objetivo.punto[0]*pto_inicial[2]-this->objetivo.punto[2]*pto_inicial[0]);
	return (d >= -100 || d <= 100);
}
	

bool SpecificWorker::region_objetivo_DIOS(float x, float z){

	if((this->bState.x > x-200 && this->bState.z > z-200) && 
	   (this->bState.x < x+200 && this->bState.z < z+200))return true;
	else return false;
}

