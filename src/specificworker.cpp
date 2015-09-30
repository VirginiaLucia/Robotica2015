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
  const float threshold = 450; //millimeters
  float rot = 0.9;  //rads per second
  const int offset = 5;
  int v;
  static float B=-(M_PI/4*M_PI/4)/log(0.3);
  static float C=1/log(0.5);
    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
        std::sort( ldata.begin()+offset, ldata.end()-offset, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
	
	float angle=(ldata.data()+offset)->angle;
	float dist=(ldata.data()+offset)->dist/1000.f;
	
	v=0.5*(ldata.data()+offset)->dist;
	if(v>500)  v=500;
	
	rot=exp(-(angle*angle)/B)/dist;
	differentialrobot_proxy->setSpeedBase(v, rot);
	qDebug()<<v<<rot;
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}
// void SpecificWorker::compute()
// {
//  const float threshold = 450; //millimeters
//     float rot = 0.9;  //rads per second
//  const int offset = 20;
//   int v;
// 
//     try
//     {
//         RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
//         std::sort( ldata.begin()+offset, ldata.end()-offset, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
// 
// 	if( (ldata.data()+offset)->dist < threshold)
// 	{
// 	  
// 	  if((ldata.data()+offset)->angle < 0)
// 	    {
// 	      differentialrobot_proxy->setSpeedBase(5, rot);
// 	      usleep(rand()%(1500000-100000 + 1) + 100000);  //random wait between 1.5s and 0.1sec
// 	    }
// 	    else
// 	    {
// 	      differentialrobot_proxy->setSpeedBase(5, -rot);
// 	      usleep(rand()%(1500000-100000 + 1) + 100000);
// 	    }
// 	}
// 	else
// 	{
// 	  v=0.5*(ldata.data()+offset)->dist;
// 	  differentialrobot_proxy->setSpeedBase(v, 0);   
// 	}
//     }
//     catch(const Ice::Exception &ex)
//     {
//         std::cout << ex << std::endl;
//     }
// }







