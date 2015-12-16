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
SpecificWorker::SpecificWorker ( MapPrx& mprx ) : GenericWorker ( mprx )
{
    inner = new InnerModel ( "/home/salabeta/Robotica2015/RoCKIn@home/world/apartment.xml" );
    //Set odometry for initial robot TargetPose
    try {
        differentialrobot_proxy->getBaseState ( bState );
        qDebug() << __FUNCTION__<< bState.x << bState.z << bState.alpha;
        try {
            inner->transform ( "world",QVec::zeros ( 6 ),"initialRobotPose" );
            if ( bState.x == 0 and bState.z == 0 ) {	//RCIS just initiated. We change robot odometry to the initialRobotPose
                QVec rpos = inner->transform ( "world", QVec::zeros ( 6 ),"robot" );
                RoboCompDifferentialRobot::TBaseState bs;
                bs.x=rpos.x();
                bs.z=rpos.z();
                bs.alpha=rpos.ry();
                differentialrobot_proxy->setOdometer ( bs );
                qDebug() << "Robot odometry set to" << rpos;
            } else {
                inner->updateTransformValues ( "initialRobotPose", 0,0,0,0,0,0 );
            }
        } catch ( std::exception &ex ) {
            std::cout<<ex.what() <<std::endl;
        };
    } catch ( Ice::Exception &ex ) {
        std::cout<<ex.what() <<std::endl;
    };
    qDebug() << __FUNCTION__<< bState.x << bState.z << bState.alpha;

    graphicsView->setScene ( &scene );
    graphicsView->show();
    graphicsView->scale ( 3,3 );

    //Innermodelviewer
    osgView = new OsgView ( this );
    osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
    osg::Vec3d eye ( osg::Vec3 ( 4000.,4000.,-1000. ) );
    osg::Vec3d center ( osg::Vec3 ( 0.,0.,-0. ) );
    osg::Vec3d up ( osg::Vec3 ( 0.,1.,0. ) );
    tb->setHomePosition ( eye, center, up, true );
    tb->setByMatrix ( osg::Matrixf::lookAt ( eye,center,up ) );
    osgView->setCameraManipulator ( tb );
    innerViewer = new InnerModelViewer ( inner, "root", osgView->getRootGroup(), true );
    show();
}


/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams ( RoboCompCommonBehavior::ParameterList params )
{
    timer.start ( 300 );

    return true;
}

void SpecificWorker::compute()
{

    try {
        differentialrobot_proxy->getBaseState ( bState );
        ldata = laser_proxy->getLaserData();
        inner->updateTransformValues ( "robot", bState.x, 0, bState.z, 0, bState.alpha, 0 );

        float alpha;
        QVec t;

        switch ( state ) {
        case State::INIT:
	    qDebug()<<"ESTADO::INIT";
            state = State::IDLE;
            break;

        case State::IDLE:
	  qDebug()<<"ESTADO::IDLE";
            break;
	    
	case State::WORKING:
	  qDebug()<<"ESTADO::WORKING";
	  heLlegado();
	  break;
	 
	case State::FREEWAY:
	  qDebug()<<"ESTADO::FREEWAY";
	  hayCamino2();
	  break;
	  
	case State::GOTARGET:
	  qDebug()<<"ESTADO::GOTARGET";
	  goToTarget();
	  state = State::WORKING;
	  break;
	
	case State::GOSUBTARGET:
	  qDebug()<<"ESTADO::GOSUBTARGET";
	  if(cTarget.activeSub == true)
	    {
	      goToSubTarget();
	      state = State::WORKING;
	    }
	    else
	    {
	      createSubTarget();
	    }
	  break;
	  
        case State::TURN:
	  qDebug()<<"ESTADO::TURN";
	    turn();
            break;

        case State::FINISH:
	  qDebug()<<"ESTADO::FINISH";
            state = State::IDLE;
            break;
        }
    } catch ( const Ice::Exception &e ) {
        std::cout << "Error reading from Camera" << e << std::endl;
    }

    innerViewer->update();
    osgView->autoResize();
    osgView->frame();
}

void SpecificWorker::heLlegado()
{
    QVec t = inner->transform ( "robot", cTarget.target, "world" );
    float d = t.norm2()-t.y();
    if ( d < 100 ) 
    {
        qDebug() << __FUNCTION__<< "He llegado";
	stopRobot();
	if(ldata.front().dist < 3000){
	   if(ldata.front().angle < 0)
	      differentialrobot_proxy->setSpeedBase(0, 0.4);
	   else
	   {
	   if(ldata.front().angle > 0)
		differentialrobot_proxy->setSpeedBase(0, -0.4);    }
	  }
     
	state = State::FINISH;
    } 
    else 
    {
       state = State::FREEWAY;       
    }
}

void SpecificWorker::hayCamino2()
{

    QVec t = inner->transform ( "robot", cTarget.target, "world" );
    float d = t.norm2();
    float alpha =atan2 ( t.x(), t.z() );

    for ( uint i = 0; i<ldata.size(); i++ ) 
    {
        if ( ldata[i].angle <= alpha ) 
	{
            if (ldata[i].dist < 4000 and ldata[i].dist < d ) 
	    {
	        qDebug() <<"NO hay camino";
		state = State::GOSUBTARGET;
		return;
            }
            else if(caja(t))
	    {
                cTarget.activeSub=false;
                qDebug() <<"hay camino";
		state = State::GOTARGET;
		return;
            }
        }
    }
    
    qDebug() <<"NO ve la marca";
    state = State::TURN;
}

bool SpecificWorker::caja(const QVec &t)
{
  int n = (int)t.norm2()/200;
  float landaD = t.norm2()/n;
  QVec tn = cTarget.target.normalize();
  int R = 200;

  
  for(float landa=landaD; landa<t.norm2();landa+=landaD)
  {
    QVec p = tn*landa;
    float z = R*cos(M_PI/4);
    float x = R*sin(M_PI/4);
    
    QVec e1 = p + QVec::vec3(x,0,z);
    if(!dentroLaser(e1)) return false;
    QVec e2 = p + QVec::vec3(-x,0,z);
    if(!dentroLaser(e2)) return false;
    QVec e3 = p + QVec::vec3(-x,0,-z);
    if(!dentroLaser(e3)) return false;
    QVec e4 = p + QVec::vec3(x,0,-z);
    if(!dentroLaser(e4)) return false;
    
    return true;
  }
}
bool SpecificWorker::dentroLaser(const QVec &e)
{
  float d = e.norm2();
  float alpha =atan2 ( e.x(), e.z() );
  
  for ( uint i = 0; i<ldata.size(); i++ ) 
  {
     if ( ldata[i].dist > d) 
     {
	return true;
     }
  }
  return false;
}

void SpecificWorker::goToTarget()
{
    QVec t = inner->transform ( "robot", cTarget.target, "world" );

    float alpha =atan2 ( t.x(), t.z() );
    float r= 0.3*alpha;
    float d = 0.3*t.norm2();
    
    if ( fabs ( r ) > 0.2 )
    {
        d = 0;
    }
    
    if ( d > 300 )
    {
        d=300;
    }
    
    try 
    {
        differentialrobot_proxy->setSpeedBase ( d,r );
    } 
    catch ( const Ice::Exception &ex )
    {
        std::cout << ex << std::endl;
    }
}

void SpecificWorker::goToSubTarget()
{
    qDebug() <<  __FUNCTION__<<"IR a subTarget";
    
    QVec t = inner->transform ( "robot", cTarget.subTarget, "world" );
    float alpha =atan2 ( t.x(), t.z() );
    float r= 0.4*alpha;
    float d = t.norm2();

    if ( d < 100 ) 
    {
        cTarget.activeSub=false;
	try
	{
            differentialrobot_proxy->setSpeedBase ( 0, 0);
	    
        } 
        catch ( const Ice::Exception &ex ) 
	{
            std::cout << ex << std::endl;
        }
    } 
    else
    {
        if ( fabs ( r ) > 0.2 )
	{
            d = 0;
        }
        
        if ( d>300 )
	{
            d=300;
        }
        
        try 
        {
            differentialrobot_proxy->setSpeedBase ( d,r );
        } 
        catch ( const Ice::Exception &ex ) 
	{
            std::cout << ex << std::endl;
        }
    }
}

void SpecificWorker::createSubTarget()
{

    qDebug() <<  __FUNCTION__ << "CREANDO subTarget";

    float dt;
    QVec t = inner->transform ( "robot", cTarget.target, "world" );
    float d = t.norm2();
    float alpha =atan2 ( t.x(), t.z() );
    uint i,j;

    for ( i = 5; i<ldata.size()-5; i++ )
    {
        if ( ldata[i].angle < alpha )
	{
            if ( d > ldata[i].dist )
	    {
                dt=ldata[i].dist;
                break;
            }
        }
    }

    for ( j = i; j<ldata.size()-5; j++ ) 
    {
        if ( ldata[j].dist > ( dt+ ( dt*0.2 ) ) and ldata[j].angle < 0 ) 
	{
            cTarget.subTarget=inner->transform ( "world", QVec::vec3 ( ldata[j].dist *sin ( ldata[j].angle )-2000,0, ldata[j].dist *cos ( ldata[j].angle ) ), "laser" );
            cTarget.activeSub = true;
            break;
        }
    }
}

void SpecificWorker::turn()
{
  float alpha;
  QVec t;
  qDebug()<<"GIRANDO";
  t = inner->transform("robot", cTarget.target, "world");
  alpha =atan2(t.x(), t.z() );
	
  if( alpha <= ldata.front().angle and alpha >= ldata. back().angle)
  {
    stopRobot();
    state = State::WORKING;
  }
  else
  {
    if( alpha > ldata.front().angle )  // turn right
    {
      try{ differentialrobot_proxy->setSpeedBase(0, 0.4);}
    
      catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;}
    }
    else	// turn left
    {
      try{ differentialrobot_proxy->setSpeedBase(0, -0.4);}
      catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};
    }
  }
}


void SpecificWorker::stopRobot()
{
    try
    {	qDebug()<<"STOP";
	differentialrobot_proxy->setSpeedBase ( 0,0 );
	cTarget.activeT = false;
	cTarget.activeSub = false;
    } 
    catch ( Ice::Exception &ex ) 
    {
        std::cout<<ex.what() <<std::endl;
    };
}

void SpecificWorker::drawTarget ( const QVec &target , const QString &nombre)
{
    InnerModelDraw::addPlane_ignoreExisting ( innerViewer, nombre, "world", QVec::vec3 ( target ( 0 ), 100, target ( 2 ) ), QVec::vec3 ( 1,0,0 ), "#009900", QVec::vec3 ( 100,100,100 ) );
}

void SpecificWorker::undrawTarget ( const QString& name )
{
    InnerModelDraw::removeNode ( innerViewer, name );
}

float SpecificWorker::go ( const TargetPose &target )
{
    static int cont=0;
    qDebug() <<"GO";
    cTarget.target = QVec::vec3 ( target.x, target.y, target.z );
    cTarget.activeT = true;
    state = State::WORKING;
    drawTarget ( cTarget.target, "target_" + QString::number(cont++) );
    qDebug()<<cTarget.target;
    return ( inner->transform ( "world","robot" ) - cTarget.target ).norm2();
}

NavState SpecificWorker::getState()
{
 
		qDebug()<<"GETSTATE";
	
    			switch( state )
			{
				case State::INIT:
					nState.state = "INIT";
					break;
				case State::WORKING:
					nState.state = "WORKING";
					break;
				case State::IDLE:
					nState.state = "IDLE";
					break;
				case State::FINISH:
					nState.state = "FINISH";
					break;
				case State::TURN:
					nState.state = "TURN";
					break;
			}
  return  nState;
}

float SpecificWorker::goBackwards ( const TargetPose &target )
{
    return 0;
}

float SpecificWorker::goReferenced ( const TargetPose &target, const float xRef, const float zRef, const float threshold )
{
    return 0;
}

float SpecificWorker::changeTarget ( const TargetPose &target )
{
    return 0;
}

void SpecificWorker::mapBasedTarget ( const NavigationParameterMap &parameters )
{

}

void SpecificWorker::stop()
{
    stopRobot();
}
