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
	derecho.clear();
	izquierdo.clear();
	
	std::copy(ldata.begin()+32, ldata.end()-32, std::back_inserter(front));
	std::copy(ldata.begin()+1, ldata.end()-60, std::back_inserter(derecho));
	std::copy(ldata.begin()+60, ldata.end()-1, std::back_inserter(izquierdo));
	
	std::sort(front.begin(), front.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
	std::sort(derecho.begin(), derecho.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
	std::sort(izquierdo.begin(), izquierdo.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
	
	frente = this->front.front().dist;
	derecha = this->derecho.front().dist;
	izquierda = this->izquierdo.front().dist;

	return;
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
			estado = Estados::IDLE;

		}else if(frente <= 300)
		{
			differentialrobot_proxy->setSpeedBase(0, 0);
			estado = Estados::OBSTACULO;
			manoderecha = EstadosMD::INICIAL;
			QVec tr = innerModel->transform("base", this->objetivo.punto, "world");
			dist_eu = tr.norm2();
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
					dir = !dir;
				}
				else
				{
					estado = Estados::OBSTACULO;
					manoderecha = EstadosMD::IDLE;
					tanto = (dir) ? derecha : izquierda; 
					differentialrobot_proxy->setSpeedBase(1000, 0);
				} 
			}
		}else if(alinear_robot(this->pto_normalY)) 
		{
			estado = Estados::OBSTACULO;
			manoderecha = EstadosMD::IDLE;
			update_laser();
			tanto = (dir) ? derecha : izquierda; 
			if(tanto > 600)
			{
				differentialrobot_proxy->setSpeedBase(1000, 0); /* hay que crear las variables booleanas up y izq */
				usleep(350000);

				/* Giro */
				double giro = pi_medios/2.0;
				differentialrobot_proxy->setSpeedBase(0, (dir)?giro:-giro); 
				usleep(2*1000000);
				differentialrobot_proxy->setSpeedBase(0, 0);
				tanto = (dir) ? derecha : izquierda; 
				differentialrobot_proxy->setSpeedBase(1000, 0);

			}else differentialrobot_proxy->setSpeedBase(1000, 0);
			
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


	if(pinline() && tr.norm2() < dist_eu-250) /* punto en linea*/
	{
		flag = 0;
		estado = Estados::ALINEACION;
		QVec aux = QVec::vec3(bState.x, 0, bState.z);
		recta = QLine2D( aux, objetivo.punto);
		return;
	}

	float mano = (dir) ? derecha : izquierda; 

	/* CODIGO DE MANO DERECHA */
	switch (manoderecha)
	{
		case EstadosMD::INICIAL:
			flag = 1;
			padondegiro();
		break;
	
		case EstadosMD::IDLE:
			update_laser();
			if(std::abs(mano) > 100 + std::abs(tanto)){
				
				/* Aceleración*/
				differentialrobot_proxy->setSpeedBase(1000, 0); /* hay que crear las variables booleanas up y izq */
				usleep(350000);
				
				/* Giro */
				double giro = pi_medios/2.0;
				differentialrobot_proxy->setSpeedBase(0, (dir)?giro:-giro); 
				usleep(2*1000000);
				differentialrobot_proxy->setSpeedBase(0, 0);
				
				/* Avance */
				manoderecha = EstadosMD::GO;
			}
		break;

		case EstadosMD::GO:
			differentialrobot_proxy->setSpeedBase(600, 0); 
			manoderecha = EstadosMD::IDLE;
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
		pto_normalX.setItem(0,100+bState.x);
		pto_normalX.setItem(1,0);
		pto_normalX.setItem(2,bState.z);
		pto_normalY.setItem(0,bState.x);
		pto_normalY.setItem(1,0);
		pto_normalY.setItem(2,bState.z-100);
		dir = true;
		estado = Estados::ALINEACION;
	}
	else if(bState.x > pto_inicial[0] && bState.z > pto_inicial[2])  /* izq abajo: alineamos con +x y +z */ 
	{
		pto_normalX.setItem(0,100+bState.x);
		pto_normalX.setItem(1,0);
		pto_normalX.setItem(2,bState.z);
		pto_normalY.setItem(0,bState.x);
		pto_normalY.setItem(1,0);
		pto_normalY.setItem(2,100+bState.z);
		dir = false;
		estado = Estados::ALINEACION;
	}
	else if(bState.x < pto_inicial[0] && bState.z < pto_inicial[2])  /* der superior: alineamos con -x y -z*/ 
	{
		pto_normalX.setItem(0,bState.x-100);
		pto_normalX.setItem(1,0);
		pto_normalX.setItem(2,bState.z);
		pto_normalY.setItem(0,bState.x);
		pto_normalY.setItem(1,0);
		pto_normalY.setItem(2,bState.z-100);
		dir = false;
		estado = Estados::ALINEACION;	
	}
	else if(bState.x < pto_inicial[0] && bState.z > pto_inicial[2])  /* der abajo: alineamos con -x y +z*/ 
	{
		pto_normalX.setItem(0,bState.x-100);
		pto_normalX.setItem(1,0);
		pto_normalX.setItem(2,bState.z);
		pto_normalY.setItem(0,bState.x);
		pto_normalY.setItem(1,0);
		pto_normalY.setItem(2,100+bState.z);
		dir = true;
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
			
	/* estblecimiento de la linea */
	QVec aux = QVec::vec3(bState.x, 0, bState.z);
	recta = QLine2D( aux, objetivo.punto);
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
	QVec posicion = QVec::vec3(bState.x, 0., bState.z);
	float actual = fabs(recta.perpendicularDistanceToPoint(posicion));
	float diferencia = actual - anterior;
	anterior = actual;
  	return (anterior < 100 && diferencia <= 0);	
}
	



bool SpecificWorker::region_objetivo_DIOS(float x, float z){

	if((this->bState.x > x-200 && this->bState.z > z-200) && 
	   (this->bState.x < x+200 && this->bState.z < z+200))return true;
	else return false;
}

