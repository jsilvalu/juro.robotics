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
#include <qt4/QtCore/qdebug.h>

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
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innerModel = new InnerModel(innermodel_path);
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
	try{

		RoboCompLaser::TLaserData laserData = laser_proxy->getLaserData();
		RoboCompDifferentialRobot::TBaseState bState;
		differentialrobot_proxy->getBaseState(bState);
		innerModel->updateTransformValues ("base",bState.x, 0, bState.z, 0, bState.alpha, 0 ); 
		//Vector auxiliar 3D para para obtener la posicion del robot y pasarselas a linea junto con el destino
		QVec auxLinea;
		
		
			switch(state)
			{
			
				case State::IDLE:
				if (pick.isActive()){
					auxLinea = QVec::vec3(bState.x, 0, bState.z);
					linea = QLine2D( auxLinea, pick.getAux() );
					state = State::GOTO;
				}
				break;


				case State::GOTO:

				break;


				case State::OBSTACULO:

				break;
				}


	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from Camera" << e << std::endl;
	}

}



void SpecificWorker::RCISMousePicker_setPick(Pick myPick)
{
    qDebug() << "Nuevo objetivo seleccionado: " << myPick.x << myPick.z;
    pick.copy(myPick.x, myPick.z);
    pick.setActive(true);

}






//Devuelve la diferencia entre la distanciaAnterior a la linea y la distanciaActual a la linea
//si la distanciaAnterior<100 y la diferencia es < 0 estamos cruzando la linea, es decir
//si la distanciaActual a la linea es menor que 100 y la distanciaAnterior>distanciaActual
//hemos llegado a la linea y seguimos hacia el objetivo.
// float SpecificWorker::distanceToLine(const TBaseState& bState){
float SpecificWorker::distanceToLine(const TBaseState &bState){
  
  QVec posicion = QVec::vec3(bState.x, 0., bState.z);
  float distanciaActual = fabs(linea.perpendicularDistanceToPoint(posicion));
  float diferencia = distanciaActual - distanciaAnterior;
  distanciaAnterior = distanciaActual;
  
  return diferencia;
  
}





void SpecificWorker::gotoTarget(const TLaserData &tLaser){
  
  if( obstacle(tLaser) == true)   // si hay un obstaculo delante va a BUG
  {
    qDebug() << "Obstaculo detectado, de GOTO a BUG";
    state = State::BUG;
    return;
  }
  
  
  QVec tr = innerModel->transform ("base",pick.getAux(),"world" );
  const float MAXADV = 400;
  const float MAXROT=0.5;
  float angulo;
  float distanciaObjetivo;
  distanciaObjetivo = tr.norm2();
  angulo = atan2 ( tr.x(),tr.z());
  
  if ( distanciaObjetivo < 50 ){
    
    state = State::IDLE;
    pick.setActive (false);
    differentialrobot_proxy->setSpeedBase(0,0);
    qDebug() << "Fin: de GOTO a IDLE";
    return;
    
  }else{
    
    float vAdv = distanciaObjetivo;
    float vrot = angulo;
    if(vrot > MAXROT)
      vrot=MAXROT;
    if(vrot< -MAXROT)
      vrot=-MAXROT;
    vAdv = MAXADV*f1(vAdv)*f2(vrot,0.9,0.1);
    differentialrobot_proxy->setSpeedBase(vAdv,vrot);
    
    }
}





float SpecificWorker::f1(float d)
{
  return (1/(1+exp(-d)-0.5));
}




float SpecificWorker::f2(float r,float h, float Vx)
{
  float y;
  
  y=(-pow(Vx,2))/log(h);
  return exp((-pow(r,2))/y);
  
}