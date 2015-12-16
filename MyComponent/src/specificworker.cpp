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

      
      trajectoryrobot2d_proxy->go(t);
      state=trajectoryrobot2d_proxy->getState();
    }
    else if(state.state == "FINISH")
    {

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





void SpecificWorker::newAprilTag(const tagsList& tags)
{
   for( auto t: tags)
   {
     listaMarcas->add(t);
    
   }
}
 

