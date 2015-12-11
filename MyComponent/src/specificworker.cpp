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
 inner = new InnerModel("/home/salabeta/Robotica2015/RoCKIn@home/world/apartment.xml");
 listaMarcas= new ListaMarcas(inner);
 
  map= new ListDigraph::NodeMap<QVec>(grafo);
  try
  {
    differentialrobot_proxy->getBaseState(bState);
     ninit = grafo.addNode();
      map->set(ninit,QVec::vec3(bState.x,0,bState.z));
  }
    catch(const Ice::Exception e)
    {
      std::cout << e << std::endl;
    }

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
   

    try
    {
      differentialrobot_proxy->getBaseState(bState);
      inner->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);	//actualiza los valores del robot en el arbol de memoria
      ldata = laser_proxy->getLaserData();  //read laser data 
     
      switch( estado )
      {
	case State::INIT:
	  std::cout << "INIT" << std::endl;
	  crearGrafo();
	  break;
	case State::CONTROLLER:
	  std::cout << "CONTROLLER" << std::endl;
	  controller();
          break;
      }
    }
    
    catch(const Ice::Exception e)
    {
      std::cout << e << std::endl;
    }
  


}

void SpecificWorker::crearGrafo()
{
    const int offset = 20;
    int maxDist = 0;
    int i, j;

    RoboCompLaser::TLaserData copiaLaser = ldata;
    std::sort( copiaLaser.begin()+offset, copiaLaser.end()-offset, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist > b.dist; }) ;  
   
    nodo = inner->laserTo("world", "laser", copiaLaser[offset].dist - 1000, copiaLaser[offset].angle);
    estado = State::CONTROLLER;
}

void SpecificWorker::controller()
{
  try
  {
    RoboCompTrajectoryRobot2D::NavState state=trajectoryrobot2d_proxy->getState();
 
    qDebug() << QString::fromStdString(state.state);
    if(state.state == "IDLE")
    {
      TargetPose t;
        t.x=nodo.x();
	t.y=nodo.y();
	t.z=nodo.z();
      //TargetPose t={2500, 0, -5000};
      qDebug()<<"nodo: "<<nodo;
      
      trajectoryrobot2d_proxy->go(t);
      state=trajectoryrobot2d_proxy->getState();
    }
    else if(state.state == "FINISH")
    {
      //crearGrafo();
      //estado = State::WAIT;
      ListDigraph::Node n = grafo.addNode();
      map->set(n,nodo);
      grafo.addArc(ninit, n);
      grafo.addArc(n, ninit);
      ninit= n;
      crearGrafo();
      return;
    }
  }
  catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}




void SpecificWorker::searchMark(int initMark)
{
    static bool firstTime=true;
    
    if(listaMarcas->exists(initMark))
    {
      try
      {
	differentialrobot_proxy->setSpeedBase(0,0);
      }
      catch(const Ice::Exception e)
      {
	std::cout << e << std::endl;
      }
      estado = State::CONTROLLER;
      firstTime=true;
      return;
    }
    
    if(firstTime)
    {
      try
      {
	differentialrobot_proxy->setSpeedBase(0,0.5);
      }
      catch(const Ice::Exception e)
      {
	std::cout << e << std::endl;
      }
      firstTime=false;
    }

}

void SpecificWorker::wait()
{  inner = new InnerModel("/home/salabeta/robocomp/files/innermodel/simpleworld.xml");

    static bool primeraVez=true;
    static QTime reloj;
    int initMark=listaMarcas->getInitMark();
    
    if(primeraVez)
    {
      reloj = QTime::currentTime();
      primeraVez=false;
      int newState = (initMark + 1) % 4;
      listaMarcas->setInitMark(newState);
      listaMarcas->setInMemory(false);
    }
    
    if(reloj.elapsed() > 3000)
    {
      estado = State::SEARCH;
      primeraVez=true;
      return;
    } 
}

void SpecificWorker::navegate()
{
    const int offset = 20;
    int initMark=listaMarcas->getInitMark();
    float distance= listaMarcas->distance(initMark);
    
    if(listaMarcas->exists(initMark))
    {
      std::cout << "Existe la marca::" << initMark << std::endl;
      if(distance<400)
      {	
	differentialrobot_proxy->setSpeedBase(0,0);

	estado = State::WAIT;
	return;
      }  inner = new InnerModel("/home/salabeta/robocomp/files/innermodel/simpleworld.xml");

    }
    else
    {
      estado = State::SEARCH;
      return;
    }
    

    try
    {		
        std::sort( ldata.begin()+offset, ldata.end()-offset, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
	
	float dist=(ldata.data()+offset)->dist;
	
	//Si encuentra un obstaculo
	if(dist <= 450)
	{
	    estado = State::WALL;
	    return;
	}  
	else
	{
	  float tx= listaMarcas->get(initMark).tx;
	  float tz= listaMarcas->get(initMark).tz;
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
    RoboCompLaser::TLaserData ldataCopy = ldata;
    const int offset = 30;
    float rot=-0.3;
    
    std::sort( ldataCopy.begin()+offset, ldataCopy.end()-offset, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
    
    //Si no hay obstaculo
    if((ldataCopy.data() + offset )->dist > 450)
    {
      estado = State::NAVEGATE;
      return;
    }
    
    //gira a izq o der para no traspasar el obstaculo o la pared
    if((ldataCopy.data() + offset )->angle < 0)
      rot=0.3;
    else
    {
      if((ldataCopy.data() + offset )->angle > 0)
	rot=-0.3;
    }
    
     differentialrobot_proxy->setSpeedBase(40, rot);
     usleep(1000000);
}

/*void SpecificWorker::controller()
{
  try
  {
    NavState state=controller_proxy->getState();
    //qDe
    if(state.state == "IDLE")
    {
      ListaMarcas::Marca m=listaMarcas->get(listaMarcas->getInitMark());
      QVec w = inner -> transform("world",QVec::vec3(m.tx,m.ty,m.tz),"rgbd");
      TargetPose t={w.x(), w.y(), w.z()};
      controller_proxy->go(t);
    }
    else if(state.state == "FINISH")
    {
      
      estado = State::WAIT;
      return;
    }
  }
  catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}*/


////////////////////////////
/// ICEPeriod
////////////////////////////

void SpecificWorker::newAprilTag(const tagsList& tags)
{
   for( auto t: tags)
   {
     listaMarcas->add(t);
    
   }
}
 
