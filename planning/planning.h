/************************************************************************
*************************************************************************
 Reminder: None of the functions are set in stone, these are just in place
 to provide guideance on what we will need to implement.  Make whatever 
 changes you think are necessary.
 ************************************************************************
 ***********************************************************************/

#ifndef PLANNING_H
#define PLANNING_H

#include "StateInformation.h"

class MapperPathPlanner
{
 public:
  MapperPathPlanner();
  ~MapperPathPlanner();
  void* getNextCommand();
 protected:
};

#endif
