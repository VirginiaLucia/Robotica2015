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

/**
       \brief
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <math.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void newAprilTag(const tagsList &tags);


public slots:
	void compute();
	void navegate();
	void searchMark(int initMark);
	void wait();
	void wall();

private:
  


  struct ListaMarcas
  {
      ListaMarcas(InnerModel *inner_) : inner(inner_){
	inMemory=false;
	initMark=0;
      };
      
      typedef struct
      {
	int id;
	float tx;
	float ty;
	float tz;
	float rx;
	float ry;
	float rz;
	QTime reloj;
      } Marca;
      
      QMap<int,Marca> lista;
      QMutex mutex;
      QVec memory;
      InnerModel *inner;
      bool inMemory;
      int initMark;
      
      void add(const RoboCompAprilTags::tag &t)
      {
	QMutexLocker ml(&mutex);
	
	Marca marca;
	marca.id = t.id;
	marca.rx = t.rx;
	marca.ry = t.ry;
	marca.rz = t.rz;
	marca.tx = t.tx * 1000;
	marca.ty = t.ty * 1000;
	marca.tz = t.tz * 1000;
	marca.reloj=QTime::currentTime();
	
	lista.insert(marca.id, marca);
	
	if(initMark==marca.id)
	{
	  memory = inner->transform("world", QVec::vec3(marca.tx, 0, marca.tz),"rgbd");
	  inMemory=true;
	}
      };
      
      Marca get(int id)
      {
	QMutexLocker ml(&mutex);
	if(lista.contains(id))
	{
	  return lista.value(id);
	}
	else
	{
	  //RECUPERAR memoria
	  QVec reality = inner->transform("rgbd", memory ,"world");
	  Marca m;
	  m.id=id;
	  m.tx=reality.x();
	  m.ty=reality.y();
	  m.tz=reality.z();
	  return m;
	}
      };
      
      bool exists(int id)
      {
	QMutexLocker ml(&mutex);
	borraMarca(id);

	return lista.contains(id) or inMemory;
      };
      
      float distance(int initMark)
      {
	Marca m = get(initMark);
	QMutexLocker ml(&mutex);
	borraMarca(initMark);
	float d = sqrt(pow(m.tx,2) + pow(m.tz,2));
	return d;
      };
      
      void borraMarca(int id)
      {
	if(lista.value(id).reloj.elapsed()>300)
	  lista.remove(id);
      };
  };
	
  ListaMarcas* listaMarcas;
  
  enum class State  { INIT, SEARCH, NAVEGATE, WAIT, WALL, FINISH};
  State estado = State::INIT;
  TLaserData ldata;
  
  InnerModel* inner;

};

#endif

