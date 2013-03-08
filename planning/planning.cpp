/************************************************************************
*************************************************************************
 Reminder: None of the functions are set in stone, these are just in place
 to provide guideance on what we will need to implement.  Make whatever 
 changes you think are necessary.
 ************************************************************************
 ***********************************************************************/
#include "planning.h"
#include "kinect_interface.h"
#include "string.h"
#include "fovis.hpp"
#include "frame.hpp"
#include "pyramid_level.hpp"
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

using namespace fovis;

MapperPathPlanner::MapperPathPlanner(int width, int height)
{
        width_ = width;
	height_ = height;
	minDepth = 1.0;		// min depth to object in meters
	depthSumThreshold = (width_*height_)/2;

	

	NODATA = -1;
	STOP = 0;
	FORWARD = 1;
	LEFT=2;
	RIGHT=-2;
	current_direction=STOP;
	BLOCKED=6;
	srand(time(NULL));
	RandTurn=false;

}

MapperPathPlanner::~MapperPathPlanner()
{
}

bool MapperPathPlanner::canMove(FrameDataPtr currFrame){
  if (!currFrame)
    return false;
	bool canMove = true;
	// determine whether there is something ahead
	float minDepthSum = 0;
	for(int i = 0; i < width_*height_; i++){
	  if(currFrame->depth_data[i] <= minDepth || isnan(currFrame->depth_data[i]))
			minDepthSum += 1;
		// TODO: handle unknown values (only vals b/n .8 and 4m record normally)
		if(minDepthSum > depthSumThreshold){
			canMove = false;
			break;
		}
	}
	return canMove;
}

char MapperPathPlanner::getNextCommand(FrameDataPtr currFrame)
{
  if (currFrame == NULL)
    return NODATA;
  if(current_direction==BLOCKED)
    {
    current_direction= turn();
    return current_direction;
    }
  if (!canMove(currFrame))
    { //if I am still blocked keep going the same way and schedule a random turn  
      if (current_direction==LEFT)
	{
	  RandTurn = true;
	  return LEFT;
	}
      else if (current_direction==RIGHT)
	{
	  RandTurn = true;
	  return RIGHT;
	}
      current_direction=BLOCKED;
      return STOP;
    }
  if (RandTurn)
    {
      if (rand() % 100) //change if called more (or less) then expected
	{
	  RandTurn =false;
	  current_direction =turn();
	  return current_direction;
	}
    }

  //TODO: more interesting planning
  current_direction=FORWARD;
  return FORWARD;
}

int MapperPathPlanner::turn(){
  if (rand() %2)
    {return LEFT;}
  return RIGHT;
}
