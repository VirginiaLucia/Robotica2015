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
 inner = new InnerModel("/home/salabeta/robocomp/files/innermodel/simpleworld.xml");
 listaMarcas= new ListaMarcas(inner);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	timer.start(500);

	return true;
}

void SpecificWorker::compute()
{   
  TBaseState bState;

  differentialrobot_proxy->getBaseState(bState);
  inner->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);//actualiza los valores del robot en el arbol de memoria
  
  
  ldata = laser_proxy->getLaserData();  //read laser data 

    switch( estado )
    {
      case State::INIT:
	std::cout << "INIT" << std::endl;
	estado = State::SEARCH;
	break;
      case State::SEARCH:
	std::cout << "SEARCH" << std::endl;
	searchMark(listaMarcas->initMark);
	break;
      case State::NAVEGATE:
	std::cout << "NAVEGATE" << std::endl;
	  navegate();
	break;
      case State::WALL:
	std::cout << "WALL" << std::endl;
	  wall();
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
  if(listaMarcas->exists(initMark))
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
    listaMarcas->inMemory=false;
  }
  if(reloj.elapsed() > 5000){
    estado = State::SEARCH;
    primeraVez=true;
    return;
  } 
}

void SpecificWorker::navegate()
{
    //float rot = 0.9;  //rads per second
    const int offset = 20;
    bool giro=false;
    
    float distance= listaMarcas->distance(listaMarcas->initMark);
    
    //mirar que la distancia sea menor a 300, si es menos buscamos de nuevo.
    if(listaMarcas->exists(listaMarcas->initMark))
    {
      std::cout << "Nav existe la marca::" << listaMarcas->initMark << std::endl;
      if(distance<400)
      {	//parar robot
	qDebug()<<"distancia nav: " << distance << " id: "<< listaMarcas->initMark;
	differentialrobot_proxy->setSpeedBase(0,0);

	//ESPERAR UN TIEMPO
	listaMarcas->initMark = (listaMarcas->initMark + 1) % 4;
	estado = State::WAIT;
	return;
      }
    }
    else
    {
      estado = State::SEARCH;
      return;
    }
    

    try
    {		
        std::sort( ldata.begin()+offset, ldata.end()-offset, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
	
	float angle=(ldata.data()+offset)->angle;
	float dist=(ldata.data()+offset)->dist;
	
	//si encuentra un obstaculo
	if(dist<450)
	{
	    estado = State::WALL;
	    return;
	}
	    
	else{
	  float tx= listaMarcas->get(listaMarcas->initMark).tx;
	  float tz= listaMarcas->get(listaMarcas->initMark).tz;
	  float r= atan2(tx, tz);
	  differentialrobot_proxy->setSpeedBase(150, 0.4*r);
	}
	
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}


void SpecificWorker::wall()
{
  std::cout << "WALL" << std::endl;
    RoboCompLaser::TLaserData ldataCopy = ldata;
    int l = ldataCopy.size();
    float rot;
    const int offset = 30;
   
    std::sort( ldataCopy.begin()+offset, ldataCopy.end()-offset, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
    //std::sort( ldataCopy.begin()+l/2, ldataCopy.end()-5, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
    
    if((ldataCopy.data() + offset )->dist > 400)
    {
      estado = State::NAVEGATE;
      return;
    }

    /*if((ldataCopy.data() + l/2 )->dist < 200)
    {
      differentialrobot_proxy->setSpeedBase(300, 0.5); //derecha
    }
    else
    {
      differentialrobot_proxy->setSpeedBase(300, -0.5); //izquierda
    }*/
//      if ((ldataCopy.data()+l/2)->angle > 0)
// 	  rot = -0.4; 
//     
//       else if ((ldataCopy.data()+l/2)->angle < 0)
// 	  rot = 0.4;
    
    
      differentialrobot_proxy->setSpeedBase(50, -0.3);                  
      usleep(1000000);


    
    
    
    
}


////////////////////////////77
/// ICEPeriod
/////////////////////////////7

void SpecificWorker::newAprilTag(const tagsList& tags)
{
  
   for( auto t: tags)
   {
     listaMarcas->add(t);
    
   }
    

}
 