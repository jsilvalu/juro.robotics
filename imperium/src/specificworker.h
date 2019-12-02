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

#include <genericworker.h>
#include <innermodel/innermodel.h>
#define pi_medios 1.5707963267948965579989817342720925807952880859375

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void RCISMousePicker_setPick(Pick myPick);

public slots:
	void compute();
	void initialize(int period);


private:
	std::shared_ptr<InnerModel> innerModel;
	bool region_objetivo_DIOS(float x, float z);
	bool alinear_robot(QVec point);
	void mano_derecha();
	bool pinline();
	void padondegiro();  
	void update_laser();
	
	struct Objetivo /* estructura del punto objetivo */
	{
		QVec punto = QVec::zeros(3);
		void copy(float _x, float _z)
		{
			punto.setItem(0,_x);
			punto.setItem(1,0);
			punto.setItem(2,_z);
			return;
		}

		QVec getAux(){ return punto;}
	};

	RoboCompGenericBase::TBaseState bState;
	RoboCompLaser::TLaserData ldata;
	RoboCompLaser::TLaserData front;
	RoboCompLaser::TLaserData derecho;
	RoboCompLaser::TLaserData izquierdo;

	Objetivo objetivo;
	enum class Estados {IDLE, CONDITIONS, GO, ALINEACION, OBSTACULO, GIRO};
	Estados estado;
	enum class EstadosMD {IDLE, GO, INICIAL};
	EstadosMD manoderecha;
	float objective_angle;
	float robot_angle;
	float distancia;
	float anterior;
	double frente;
	double izquierda;
	double derecha;
	int flag; /* control de angulo*/
	QVec pto_inicial;
	QVec pto_normalX;
	QVec pto_normalY;
	float dist_eu;
	bool izq;
	bool up;
	double tanto;
	bool dir;
	QLine2D recta; 
};

#endif
