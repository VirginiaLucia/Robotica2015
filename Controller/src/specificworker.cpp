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
            state = State::IDLE;
            break;

        case State::IDLE:
            break;

        case State::WORKING:
            if ( heLlegado() ) {
                qDebug() << __FUNCTION__<< "Arrived to target" << cTarget.target;
                stopRobot();
                state = State::FINISH;
            } else if ( hayCamino() ) {
                goToTarget();
            }

            break;

        case State::TURN:
	    qDebug() << "Buscando punto" << cTarget.target;
            t = inner->transform ( "robot", cTarget.target, "world" );
            alpha =atan2 ( t.x(), t.z() );
            if ( alpha <= ldata.front().angle and alpha >= ldata. back().angle ) {
                stopRobot();
                state = State::WORKING;
            } else
                try {
                    differentialrobot_proxy->setSpeedBase ( 0, 0.4 );
                } catch ( Ice::Exception &ex ) {
                    std::cout<<ex.what() <<std::endl;
                };
            break;

        case State::FINISH:
            sleep ( 2 );
            undrawTarget ( "target" );
            state = State::IDLE;
            break;
        }
    } catch ( const Ice::Exception &e ) {
        std::cout << "Error reading from Camera" << e << std::endl;
    }

    //histogram();
    innerViewer->update();
    osgView->autoResize();
    osgView->frame();
}

bool SpecificWorker::heLlegado()
{
    QVec t = inner->transform ( "robot", cTarget.target, "world" );
    qDebug() << __FUNCTION__<< cTarget.target;
    qDebug() << __FUNCTION__<< t;

    float d = t.norm2();
    qDebug() << __FUNCTION__<< d;

    if ( d < 100 ) {
        qDebug() << __FUNCTION__<< d;
        return true;
    } else {
        return false;
    }
}

bool SpecificWorker::hayCamino()
{

    QVec t = inner->transform ( "robot", cTarget.target, "world" );
    qDebug() << __FUNCTION__<< cTarget.target;
    float d = t.norm2();
    float alpha =atan2 ( t.x(), t.z() );

    for ( uint i = 0; i<ldata.size(); i++ ) {
        if ( ldata[i].angle <= alpha ) {
            if ( ldata[i].dist < d ) {
	        qDebug() <<"NO hay camino";
                return false;
            } else {
                cTarget.activeSub=false;
                qDebug() <<"hay camino";
                return true;
            }
        }
    }
    
    qDebug() <<"NO ve la marca";
    state = State::TURN;
    return false;
}

void SpecificWorker::goToTarget()
{
    QVec t = inner->transform ( "robot", cTarget.target, "world" );
    qDebug() << __FUNCTION__<< cTarget.target;

    qDebug() <<  __FUNCTION__<< t;
    float alpha =atan2 ( t.x(), t.z() );
    float r= 0.3*alpha;
    float d = 0.3*t.norm2();
    qDebug() << "velocidad " << d;
    if ( fabs ( r ) > 0.2 ) {
        d = 0;
    }
    if ( d>300 ) {
        d=300;
    }
    try {
        differentialrobot_proxy->setSpeedBase ( d,r );
    } catch ( const Ice::Exception &ex ) {
        std::cout << ex << std::endl;
    }
}

void SpecificWorker::goToSubTarget()
{
    qDebug() <<  __FUNCTION__<<"ir a subTarget";
    QVec t = inner->transform ( "robot", cTarget.subTarget, "world" );
    float alpha =atan2 ( t.x(), t.z() );
    float r= 0.4*alpha;
    float d = t.norm2();

    if ( d<100 ) {
        cTarget.activeSub=false;
        differentialrobot_proxy->setSpeedBase ( 0,0 );
        sleep ( 1 );

    } else {
        if ( fabs ( r ) > 0.2 ) {
            d = 0;
        }
        if ( d>300 ) {
            d=300;
        }
        try {
            differentialrobot_proxy->setSpeedBase ( d,r );
        } catch ( const Ice::Exception &ex ) {
            std::cout << ex << std::endl;
        }
    }
}

void SpecificWorker::createSubTarget()
{

    qDebug() <<  __FUNCTION__ << "creando subTarget";

    float dt;
    QVec t = inner->transform ( "rgbd", cTarget.target, "world" );
    float d = t.norm2();
    float alpha =atan2 ( t.x(), t.z() );
    uint i,j;
    //const int R =400;

    for ( i = 5; i<ldata.size()-5; i++ ) {
        if ( ldata[i].angle < alpha ) {
            if ( d>ldata[i].dist ) {
                dt=ldata[i].dist;
                break;
            }
        }
    }
    qDebug() <<  __FUNCTION__<<i;
    qDebug() <<  __FUNCTION__<<ldata[i].dist<<ldata[i].angle;

    for ( j = i; j<ldata.size()-5; j++ ) {
        qDebug() <<  __FUNCTION__<<dt<< dt+ ( dt*0.2 ) <<ldata[j].dist << ldata[j].angle;

        if ( ldata[j].dist> ( dt+ ( dt*0.2 ) ) and ldata[j].angle < 0 ) {
            cTarget.subTarget=inner->transform ( "world", QVec::vec3 ( ldata[j].dist *sin ( ldata[j].angle )-2000,0, ldata[j].dist *cos ( ldata[j].angle ) ), "laser" );
            cTarget.activeSub = true;
            break;
        }
    }
    qDebug() <<  __FUNCTION__<< "Subtargeet" << QVec::vec3 ( ldata[j].dist *sin ( ldata[j].angle ),0, ldata[j].dist *cos ( ldata[j].angle ) );

}


void SpecificWorker::stopRobot()
{
    try {
        differentialrobot_proxy->setSpeedBase ( 0,0 );
    } catch ( Ice::Exception &ex ) {
        std::cout<<ex.what() <<std::endl;
    };
}

void SpecificWorker::drawTarget ( const QVec &target )
{
    InnerModelDraw::addPlane_ignoreExisting ( innerViewer, "target", "world", QVec::vec3 ( target ( 0 ), 100, target ( 2 ) ), QVec::vec3 ( 1,0,0 ), "#009900", QVec::vec3 ( 100,100,100 ) );
}
void SpecificWorker::undrawTarget ( const QString& name )
{
    InnerModelDraw::removeNode ( innerViewer, name );
}

float SpecificWorker::go ( const TargetPose &target )
{
    qDebug() <<"GO";
//primeraVez=true;
    cTarget.target = QVec::vec3 ( target.x, target.y, target.z );
    cTarget.activeT = true;
    state = State::WORKING;
    drawTarget ( cTarget.target );
    return ( inner->transform ( "world","robot" ) - cTarget.target ).norm2();
}

NavState SpecificWorker::getState()
{
    
  return nState;
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
    cTarget.activeT = true;
    stopRobot();
}
