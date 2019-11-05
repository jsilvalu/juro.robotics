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

}

void SpecificWorker::compute()
{
    
    /* actualización de info */
		RoboCompLaser::TLaserData laserData = laser_proxy->getLaserData();
		RoboCompDifferentialRobot::TBaseState bState;
		differentialrobot_proxy->getBaseState(bState);
		innerModel->updateTransformValues ("base",bState.x, 0, bState.z, 0, bState.alpha, 0 ); 
		QVec vector_linea; /* vector que guardará la linea de alineamiento */
		
		
    switch(state)
    {
    
      case State::IDLE:
        if (pick.isActive())
        {
          vector_linea = QVec::vec3(bState.x, 0, bState.z);
          linea = QLine2D(auxLinea, pick.getAux() );
          state = State::GOTO;
        }
      break;

      case State::GOTO:
        differentialrobot_proxy->setSpeedBase(0,0);
      break;


      case State::OBSTACULO:

      break;
    }


	return;

}



void SpecificWorker::RCISMousePicker_setPick(Pick myPick)
/* guarda en la clase Pick la posición seleccionada por el ratón */
{
  pick.copy(myPick.x, myPick.z);
  qDebug() << "Coordenada pinchada: " << myPick.x << myPick.z;
  pick.setActive(1);
  return;
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





void SpecificWorker::gotoTarget(const TLaserData &tLaser)
{
  
    if(obstacle(tLaser) == true)   // si hay un obstaculo delante va a BUG
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
        
        if(vrot > MAXROT) vrot=MAXROT;
        if(vrot < -MAXROT) vrot=-MAXROT;
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