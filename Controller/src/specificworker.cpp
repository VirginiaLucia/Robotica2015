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
  state.state="IDLE";

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
     ldata = laser_proxy->getLaserData();
     inner->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
     
     if( state.state == "WORKING")
     {
	if( heLlegado() )
	{ 
	  qDebug()<<"he llegado";
	  differentialrobot_proxy->setSpeedBase(0,0);
	  state.state = "FINISH";
	  sleep(2);
	   state.state = "IDLE";
	  return;
	}
      
       else if(hayCamino())
       {
	   goToTarget(); 
       }
       else if(cTarget.activeSub == true)
       {
	  goToSubTarget(); 
       }
       else
       {
	 createSubTarget();
       }
    }
  }
  catch(const Ice::Exception &e)
  {
    std::cout << "Error reading from Camera" << e << std::endl;
  }
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
}

bool SpecificWorker::heLlegado()
{
  QVec t = inner->transform("rgbd", cTarget.target, "world");
  //qDebug()<< cTarget.target;
  float d = t.norm2();
  //qDebug()<< "distancia: "<<d;
  if ( d < 400 ) 
    return true;
  else return false;
}

bool SpecificWorker::hayCamino()
{
  
  QVec t = inner->transform("rgbd", cTarget.target, "world");
  float d = t.norm2();
  float alpha =atan2(t.x(), t.z() );
  
  for(uint i = 5; i<ldata.size()-5; i++)
  {
      if(ldata[i].angle < alpha)
      {
	if( ldata[i].dist < d)
	{
	  return false;
	}
	else
	{
	  cTarget.activeSub=false;
	  qDebug()<<"hay camino";
	  return true;
	}
      }
  }
  return false;
}

void SpecificWorker::goToTarget()
{
   qDebug()<<"andar";

    QVec t = inner->transform("rgbd", cTarget.target, "world");
    float alpha =atan2(t.x(), t.z());
    float r= 0.3*alpha;
    float d = 0.3*t.norm2();
    if( fabs(r) > 0.2) d = 0;
    if(d>300)d=300;
    differentialrobot_proxy->setSpeedBase(d,r);
}

void SpecificWorker::goToSubTarget()
{
    qDebug()<<  __FUNCTION__<<"ir a subTarget";  
    QVec t = inner->transform("laser", cTarget.subTarget, "world");
    float alpha =atan2(t.x(), t.z());
    float r= 0.4*alpha;
    float d = t.norm2();
    
    if(d<100)
    {
        cTarget.activeSub=false;
        differentialrobot_proxy->setSpeedBase(0,0);
	sleep(1);
      
    }else
    {
      if( fabs(r) > 0.2) d = 0;
      if(d>300)d=300;
      differentialrobot_proxy->setSpeedBase(d,r);
    }   
}

void SpecificWorker::createSubTarget()
{
  
  qDebug() <<  __FUNCTION__ << "creando subTarget";
  /*uint i;
  
  for(i = 5; i<ldata.size()-5; i++)
  {
    if(ldata[i-1].dist - ldata[i].dist	> 400)	//izquierda
    {
      qDebug()<<  __FUNCTION__<< "Crea subtarget por la izquierda";
      cTarget.subTarget=inner->transform("world", QVec::vec3(ldata[i].dist *sin(ldata[i].angle),0, ldata[i].dist *cos(ldata[i].angle)), "laser");
      cTarget.activeSub=true;
      break;
    }
    
    if(ldata[i+1].dist - ldata[i].dist > 400)	//derecha
    {
      qDebug()<<  __FUNCTION__<< "Crea subtarget por la derecha";
      cTarget.subTarget=inner->transform("world", QVec::vec3(ldata[i].dist *sin(ldata[i].angle),0, ldata[i].dist *cos(ldata[i].angle)), "laser");
      cTarget.activeSub=true;
      break;
    }
  }*/
  
  float dt;
  QVec t = inner->transform("rgbd", cTarget.target, "world");
  float d = t.norm2();
  float alpha =atan2(t.x(), t.z() );
  uint i,j;
	const int R =400;
	for(i=ldata.size()/2; i>5; i--)
	{
		if( (ldata[i].dist - ldata[i-1].dist) < -R )
		{
			if(i<=7) 
			{ 
				i=0; 
				break;
			}
			uint k=i-2;
			while( (k >= 0) and (fabs( ldata[k].dist*sin(ldata[k].angle - ldata[i-1].angle)) < R ))
			{ k--; }
			i=k;
			break;
		}
	}
	for(j=ldata.size()/2-5; j<ldata.size()-1; j++)
	{
		if( (ldata[j].dist - ldata[j+1].dist) < -R )
		{
			if(j>ldata.size()-3)
			{
				j=ldata.size()-1;
				break;
			}
			uint k=j+2;
			while( (k < ldata.size()) and (fabs( ldata[k].dist*sin(ldata[k].angle - ldata[j+1].angle)) < R ))
			{ k++; }
			j=k;
			break;
		}
	}
	
	QVec sI = inner->transform("world", QVec::vec3(ldata[j].dist *sin(ldata[j].angle),0, ldata[j].dist *cos(ldata[j].angle)), "laser");
	QVec sD = inner->transform("world", QVec::vec3(ldata[i].dist *sin(ldata[i].angle),0, ldata[i].dist *cos(ldata[i].angle)), "laser");
	
	if( (sI-cTarget.target).norm2() > (sD-cTarget.target).norm2() ) 
		cTarget.subTarget=sD;
	else
		cTarget.subTarget=sI;
		
	cTarget.activeSub=true;
  qDebug() << "Subtarget: " << cTarget.subTarget;

  
}



float SpecificWorker::go(const TargetPose &target)
{
 qDebug()<<"GO";
 //primeraVez=true;
 cTarget.target = QVec::vec3(target.x, target.y, target.z);
 cTarget.activeT = true;
 state.state = "WORKING";
 return 0.0;
}

NavState SpecificWorker::getState()
{
  return state;
}



void SpecificWorker::stop()
{

}