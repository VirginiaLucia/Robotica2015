/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{   
  std::cout << "compute" << std::endl;
    switch( estado )
    {
      case State::INIT:
	std::cout << "INIT" << std::endl;
	estado = State::SEARCH;
	break;
      case State::SEARCH:
	std::cout << "SEARCH" << std::endl;
	searchMark(initMark);
	break;
      case State::NAVEGATE:
	std::cout << "NAVEGATE" << std::endl;
	  navegate();
	break;
      case State::WAIT:
	std::cout << "WAIT" << std::endl;
	  wait();
	break;
      case State::FINISH:
	std::cout << "FINISH" << std::endl;
	break;
    } 
}
void SpecificWorker::searchMark(int initMark)
{
  static bool firstTime=true;
  if(listaMarcas.exists(initMark))
  {
    std::cout << "Existe la marca" << std::endl;
    try
    {
      //parar el robot
      differentialrobot_proxy->setSpeedBase(0,0);
    }
    catch(const Ice::Exception e){
      std::cout << e << std::endl;
    }
    //cambiar al estado navegate
    estado = State::NAVEGATE;
    firstTime=true;
    return;
  }
  
 if(firstTime)
  {
    std::cout << "firstTime" << std::endl;
    try
    {
      //girar el robot
      differentialrobot_proxy->setSpeedBase(0,0.5);
    }
    catch(const Ice::Exception e){
      std::cout << e << std::endl;
    }
    firstTime=false;
  }

}

void SpecificWorker::wait()
{
  static bool primeraVez=true;
  static QTime reloj;
  if(primeraVez){
    reloj = QTime::currentTime();
    primeraVez=false;
  }
  if(reloj.elapsed() > 10000){
    estado = State::SEARCH;
    primeraVez=true;
    return;
  } 
}
void SpecificWorker::navegate()
{
    float rot = 0.9;  //rads per second
    const int offset = 20;
    int v;
    static float B=-(M_PI/4*M_PI/4)/log(0.3);
    bool giro=false;
  
    //mirar que la distancia sea menor a 300, si es menos buscamos de nuevo.
    if(listaMarcas.exists(initMark))
    {
      std::cout << "Nav existe la marca::" << initMark << std::endl;
      if(listaMarcas.distance(initMark)<0.8)
      {	//parar robot
	differentialrobot_proxy->setSpeedBase(0,0);
	//ESPERAR UN TIEMPO
	std::cout << "Parado en la marca" << std::endl;
	//sleep(3);
	estado = State::WAIT;
	initMark = (initMark + 1) % 4;
	return;
      }
      std::cout << "Dentro if" << std::endl;
    }
    else
    {
      estado = State::SEARCH;
      return;
    }
    

    try
    {
        std::cout << "Navega2" << std::endl;
	
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
        std::sort( ldata.begin()+offset, ldata.end()-offset, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
	
	float angle=(ldata.data()+offset)->angle;
	float dist=(ldata.data()+offset)->dist/1000.f;
	
	if(angle>0) 
	  giro=false;
	else 
	  giro=true;
	
	v=0.5*(ldata.data()+offset)->dist;
	if(v>500)  v=500;
	
	rot=exp(-(angle*angle)/B)/dist;
	
	if(giro) 
	  differentialrobot_proxy->setSpeedBase(v, rot);
	else 
	  differentialrobot_proxy->setSpeedBase(v, -rot);
	usleep(rand()%(1500000-100000 + 1) + 100000);   
	
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}



////////////////////////////77
/// ICE
/////////////////////////////7

void SpecificWorker::newAprilTag(const tagsList& tags)
{
  
   for( auto t: tags)
   {
     listaMarcas.add(t);
     qDebug() << t.id;
    } 
}
 