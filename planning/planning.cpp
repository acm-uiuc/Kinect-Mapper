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

using namespace fovis;

MapperPathPlanner::MapperPathPlanner()
{
        width_ = 640;
	height_ = 480;
	minDepth = 1.0;		// min depth to object in meters
	depthSumThreshold = (width_*height_)/2;

	

	NODATA = -1;
	STOP = 0;
	FORWARD = 1;
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
  if (!canMove(currFrame))
  	return STOP;

  //TODO: more interesting planning

  return FORWARD;
}
