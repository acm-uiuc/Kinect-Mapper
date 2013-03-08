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

MapperPathPlanner::MapperPathPlanner()
{
	minDepth = 1.0;		// min depth to object in meters
	depthSumThreshold = 20;

	width_ = 640;
	height_ = 480;

	NODATA = -1;
	STOP = 0;
	FORWARD = 1;
	LEFT=2;
	RIGHT=-2;
	current_direction=STOP;
	srand(time(NULL));

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
	CameraIntrinsicsParameters rgb_params;
	rgb_params.width = width_;
	rgb_params.height = height_;
	rgb_params.fx = 528.49404721; 
	rgb_params.fy = rgb_params.fx;
	rgb_params.cx = width_ / 2.0;
	rgb_params.cy = height_ / 2.0;
	Rectification rect(rgb_params);
	OdometryFrame frame(&rect,VisualOdometry::getDefaultOptions());
	if(currFrame->depth_image != NULL)
		(currFrame->depth_image->getXyz(&frame));
	else
		return false;
	PyramidLevel* image_data;
	if(frame.getNumLevels() > 0)
		image_data = frame.getLevel(0);
	else
		return false;
	
	for(int i = 0; i < image_data->getNumKeypoints(); i++){
		if(image_data->getKeypointData(i)->xyz.z() <= minDepth)
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
  if(current_direction==STOP)
    turn();
  if (!canMove(currFrame))
  	return STOP;

  //TODO: more interesting planning

  return FORWARD;
}

int MapperPathPlanner::turn(){
  if (rand() %2)
    {return LEFT;}
  return RIGHT;
}
