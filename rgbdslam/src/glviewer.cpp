/* This file is part of RGBDSLAM.
 * 
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "ros/ros.h"
#include <QtGui>
#include <QtOpenGL>
#include <QThread>
#include <GL/gl.h>
#include <cmath>
#include <gl2ps.h>
#include "glviewer.h"
#include "GL/glut.h"
#include "boost/foreach.hpp"
#include "pcl/point_types.h"

template <typename PointType>
inline bool hasValidXYZ(const PointType& p){
      return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

const double PI= 3.14159265358979323846;

GLViewer::GLViewer(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers|QGL::StereoBuffers), parent),
      xRot(180*16.0),
      yRot(0),
      zRot(0),
      xTra(0),
      yTra(0),
      zTra(-50),//go back 50 (pixels?)
      polygon_mode(GL_FILL),
      cloud_list_indices(),
      edge_list_(NULL),
      cloud_matrices(new QList<QMatrix4x4>()),
      show_poses_(ParameterServer::instance()->get<bool>("show_axis")),
      show_ids_(false), //more or less for debuging
      show_edges_(ParameterServer::instance()->get<bool>("show_axis")),
      show_clouds_(true),
      follow_mode_(true),
      stereo_(false),
      width_(0),
      height_(0),
      stereo_shift_(0.1),
      fov_(100.0/180.0*PI),
      rotation_stepping_(1.0)
{
    bg_col_[0] = bg_col_[1] = bg_col_[2] = bg_col_[3] = 0.0;//black background
    ROS_DEBUG_COND(!this->format().stereo(), "Stereo not supported");
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding); //can make good use of more space
    viewpoint_tf_.setToIdentity();
}

GLViewer::~GLViewer() { }

QSize GLViewer::minimumSizeHint() const {
    return QSize(400, 400);
}

QSize GLViewer::sizeHint() const {
    return QSize(640, 480);
}

static void qNormalizeAngle(int &angle) {
    while (angle < 0)
        angle += 360 * 16;
    while (angle > 360 * 16)
        angle -= 360 * 16;
}

void GLViewer::setXRotation(int angle) { 
    qNormalizeAngle(angle);
    if (angle != xRot) {
        xRot = angle;
        updateGL();
    }
}


void GLViewer::setYRotation(int angle) {
    qNormalizeAngle(angle);
    if (angle != yRot) {
        yRot = angle;
        updateGL();
    }
}

void GLViewer::setRotationGrid(double rot_step_in_degree) {
  rotation_stepping_ = rot_step_in_degree;
}

void GLViewer::setStereoShift(double shift) {
  stereo_shift_ = shift;
  updateGL();
}

void GLViewer::setZRotation(int angle) {
    qNormalizeAngle(angle);
    if (angle != zRot) {
        zRot = angle;
        updateGL();
    }
}

void GLViewer::initializeGL() {
    glClearColor(bg_col_[0],bg_col_[1],bg_col_[2],bg_col_[3]); 
    glEnable (GL_BLEND); 
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_LINE_SMOOTH);
    //glShadeModel(GL_SMOOTH);
    //glEnable(GL_LIGHTING);
    //glEnable(GL_LIGHT0);
    //glEnable(GL_MULTISAMPLE);
    //gluPerspective(fov_, 1.00, 0.01, 1e9); //1.38 = tan(57/2°)/tan(43/2°)
    ////gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);
    //static GLfloat lightPosition[4] = { 0.5, 5.0, 7.0, 1.0 };
    //glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
}
//assumes gl mode for triangles
inline
void GLViewer::drawTriangle(const point_type& p1, const point_type& p2, const point_type& p3){
    unsigned char b,g,r;
    b = *(  (unsigned char*)(&p1.rgb));
    g = *(1+(unsigned char*)(&p1.rgb));
    r = *(2+(unsigned char*)(&p1.rgb));
    glColor3ub(r,g,b); //glColor3f(1.0,1.0,1.0);
    glColor3ub(255,0,0); 
    glVertex3f(p1.x, p1.y, p1.z);

    b = *(  (unsigned char*)(&p2.rgb));
    g = *(1+(unsigned char*)(&p2.rgb));
    r = *(2+(unsigned char*)(&p2.rgb));
    glColor3ub(r,g,b); //glColor3f(1.0,1.0,1.0);
    glColor3ub(0,255,0); 
    glVertex3f(p2.x, p2.y, p2.z);

    b = *(  (unsigned char*)(&p3.rgb));
    g = *(1+(unsigned char*)(&p3.rgb));
    r = *(2+(unsigned char*)(&p3.rgb));
    //glColor3ub(r,g,b); //glColor3f(1.0,1.0,1.0);
    glColor3ub(0,0,255); 
    glVertex3f(p3.x, p3.y, p3.z);
}

void GLViewer::drawAxis(float scale){
    glBegin(GL_LINES);
    glLineWidth(4);
    glColor4f (0.9, 0, 0, 1.0);
    glVertex3f(0, 0, 0);
    glColor4f (0.9, 0, 0, 0.0);
    glVertex3f(scale, 0, 0);
    glColor4f (0, 0.9, 0, 1.0);
    glVertex3f(0, 0, 0);
    glColor4f (0, 0.9, 0, 0.0);
    glVertex3f(0, scale, 0);
    glColor4f (0, 0, 0.9, 1.0);
    glVertex3f(0, 0, 0);
    glColor4f (0, 0, 0.9, 0.0);
    glVertex3f(0, 0, scale);
    glEnd();
}

void GLViewer::paintGL() {
    if(!this->isVisible()) return;
    //ROS_INFO("This is paint-thread %d", (unsigned int)QThread::currentThreadId());
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if(stereo_){
        float ratio = (float)(width_) / (float) height_;
        glViewport(0, 0, width_/2, height_);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(fov_, ratio, 0.1, 1e4); 
        glMatrixMode(GL_MODELVIEW);
        drawClouds(stereo_shift_);

        glViewport(width_/2, 0, width_/2, height_);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(fov_, ratio, 0.1, 1e4); 
        glMatrixMode(GL_MODELVIEW);
    }
    drawClouds(0.0);
}

void GLViewer::drawClouds(float xshift) {
    if(follow_mode_){
        int id = cloud_matrices->size()-1;
        if(id >= 0)setViewPoint((*cloud_matrices)[id]);
    }
    glLoadIdentity();
    //Camera transformation
    glTranslatef(xTra+xshift, yTra, zTra);
    int x_steps = (xRot / 16.0)/rotation_stepping_;
    int y_steps = (yRot / 16.0)/rotation_stepping_;
    int z_steps = (zRot / 16.0)/rotation_stepping_;
    glRotatef(x_steps*rotation_stepping_, 1.0, 0.0, 0.0);
    glRotatef(y_steps*rotation_stepping_, 0.0, 1.0, 0.0);
    glRotatef(z_steps*rotation_stepping_, 0.0, 0.0, 1.0);
    glMultMatrixd(static_cast<GLdouble*>( viewpoint_tf_.data() ));//works as long as qreal and GLdouble are typedefs to double (might depend on hardware)
    ROS_DEBUG("Drawing %i PointClouds", cloud_list_indices.size());
    for(int i = 0; i<cloud_list_indices.size() && i<cloud_matrices->size(); i++){
        //GLdouble* entry = static_cast<GLdouble*>( cloud_matrices[i].data() );
        //for(int j = 0; j < 16; j++, entry++){
        //    ROS_INFO("Matrix[%d]: %f", j, *entry);
        //}
        glPushMatrix();
        glMultMatrixd(static_cast<GLdouble*>( (*cloud_matrices)[i].data() ));//works as long as qreal and GLdouble are typedefs to double (might depend on hardware)
        if(show_clouds_) glCallList(cloud_list_indices[i]);
        if(show_poses_) drawAxis(0.075);
        if(show_ids_) {
          glColor4f(1-bg_col_[0],1-bg_col_[1],1-bg_col_[2],1.0); //inverse of bg color, but transp
          this->renderText(0.,0.,0.,QString::number(i));
        }
        glPopMatrix();
    }
    if(show_poses_) drawAxis(0.2);//Show origin as big axis
    if(show_edges_) drawEdges();
}

void GLViewer::resizeGL(int width, int height)
{
    width_ = width;
    height_ = height;
    //int side = qMin(width, height);
    glViewport(0, 0, width, height);
    //glViewport((width - side) / 2, (height - side) / 2, side, side);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
//#ifdef QT_OPENGL_ES_1
//    glOrthof(-0.5, +0.5, -0.5, +0.5, 1.0, 15.0);
//#else
//    glOrtho(-0.5, +0.5, -0.5, +0.5, 1.0, 15.0);
//#endif
    //gluPerspective(fov_, 1.38, 0.01, 1e9); //1.38 = tan(57/2°)/tan(43/2°) as kinect has viewing angles 57 and 43
    float ratio = (float)width / (float) height;
    gluPerspective(fov_, ratio, 0.1, 1e4); 
    glMatrixMode(GL_MODELVIEW);
}

void GLViewer::mouseDoubleClickEvent(QMouseEvent *event) {
    xRot=180*16.0;
    yRot=0;
    zRot=0;
    xTra=0;
    yTra=0;
    if(cloud_matrices->size()>0){
      if(!setClickedPosition(event->x(), event->y())){
        if (event->buttons() & Qt::LeftButton) {
          int id = cloud_matrices->size()-1;
          setViewPoint((*cloud_matrices)[id]);
        } else if (event->buttons() & Qt::RightButton) {
          int id = 0;
          setViewPoint((*cloud_matrices)[id]);
        } else if (event->buttons() & Qt::MidButton) { 
          viewpoint_tf_.setToIdentity();
          zTra=-50;
        }
      }
    }
    updateGL();
}
void GLViewer::toggleStereo(bool flag){
  stereo_ = flag;
  resizeGL(width_, height_);
  updateGL();
}
void GLViewer::toggleBackgroundColor(bool flag){
  if(flag){
    bg_col_[0] = bg_col_[1] = bg_col_[2] = bg_col_[3] = 0.0;//black background
  }
  else{
    bg_col_[0] = bg_col_[1] = bg_col_[2] = 1.0;//white background
  }
  glClearColor(bg_col_[0],bg_col_[1],bg_col_[2],bg_col_[3]); 
  updateGL();
}
void GLViewer::toggleShowClouds(bool flag){
  show_clouds_ = flag;
  updateGL();
}
void GLViewer::toggleShowIDs(bool flag){
  show_ids_ = flag;
  updateGL();
}
void GLViewer::toggleShowEdges(bool flag){
  show_edges_ = flag;
  updateGL();
}
void GLViewer::toggleShowPoses(bool flag){
  show_poses_ = flag;
  updateGL();
}
void GLViewer::toggleFollowMode(bool flag){
  follow_mode_ = flag;
}
void GLViewer::mousePressEvent(QMouseEvent *event) {
    lastPos = event->pos();
}

void GLViewer::wheelEvent(QWheelEvent *event) {
    zTra += ((float)event->delta())/25.0; 
    updateGL();
}
void GLViewer::mouseMoveEvent(QMouseEvent *event) {//TODO: consolidate setRotation methods
    int dx = event->x() - lastPos.x();
    int dy = event->y() - lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        setXRotation(xRot - 8 * dy);
        setYRotation(yRot + 8 * dx);
    } else if (event->buttons() & Qt::RightButton) {
        setXRotation(xRot - 8 * dy);
        setZRotation(zRot + 8 * dx);
    } else if (event->buttons() & Qt::MidButton) {
        xTra += dx/200.0;
        yTra -= dy/200.0;
        updateGL();
    }
    lastPos = event->pos();
}

void GLViewer::updateTransforms(QList<QMatrix4x4>* transforms){
    ROS_WARN_COND(transforms->size() < cloud_matrices->size(), "Got less transforms than before!");
    // This doesn't deep copy, but should work, as qlist maintains a reference count 
    delete cloud_matrices;
    cloud_matrices = transforms; 
    ROS_DEBUG("New Cloud matrices size: %d", cloud_matrices->size());
    updateGL();
}

void GLViewer::addPointCloud(pointcloud_type * pc, QMatrix4x4 transform){
    ROS_DEBUG("pc pointer in addPointCloud: %p (this is %p in thread %d)", pc, this, (unsigned int)QThread::currentThreadId());
    pointCloud2GLStrip(pc);
    cloud_matrices->push_back(transform); //keep for later
    updateGL();
}

void GLViewer::pointCloud2GLStrip(pointcloud_type * pc){
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    ROS_DEBUG("Making GL list from point-cloud pointer %p in thread %d", pc, (unsigned int)QThread::currentThreadId());
    GLuint cloud_list_index = glGenLists(1);
    if(!cloud_list_index) {
        ROS_ERROR("No display list could be created");
        return;
    }
    float mesh_thresh = ParameterServer::instance()->get<double>("squared_meshing_threshold");
    glNewList(cloud_list_index, GL_COMPILE);
    cloud_list_indices.push_back(cloud_list_index);
    //ROS_INFO_COND(!pc->is_dense, "Expected dense cloud for opengl drawing");
    point_type origin;
    origin.x = 0;
    origin.y = 0;
    origin.z = 0;

    float depth;
    bool strip_on = false, flip = false; //if flip is true, first the lower then the upper is inserted
    const int w=pc->width, h=pc->height;
    unsigned char b,g,r;
    const int step = ParameterServer::instance()->get<int>("visualization_skip_step");
    for( int y = 0; y < h-step; y+=step){ //go through every point and make two triangles 
        for( int x = 0; x < w-step; x+=step){//for it and its neighbours right and/or down
            using namespace pcl;
            if(!strip_on){ //Generate vertices for new triangle
                const point_type* ll = &pc->points[(x)+(y+step)*w]; //one down (lower left corner)
                if(!hasValidXYZ(*ll)) continue; // both new triangles in this step would use this point
                const point_type* ur = &pc->points[(x+step)+y*w]; //one right (upper right corner)
                if(!hasValidXYZ(*ur)) continue; // both new triangles in this step would use this point
          
                const point_type* ul = &pc->points[x+y*w]; //current point (upper right)
                if(hasValidXYZ(*ul)){ //ul, ur, ll all valid
                  depth = squaredEuclideanDistance(*ul,origin);
                  if (squaredEuclideanDistance(*ul,*ll)/depth <= mesh_thresh  and 
                      squaredEuclideanDistance(*ul,*ll)/depth <= mesh_thresh  and
                      squaredEuclideanDistance(*ur,*ll)/depth <= mesh_thresh){
                    glBegin(GL_TRIANGLE_STRIP);
                    strip_on = true;
                    flip = false; //correct order, upper first
                    //Prepare the first two vertices of a triangle strip
                    //drawTriangle(*ul, *ll, *ur);
                    b = *(  (unsigned char*)(&ul->rgb));
                    g = *(1+(unsigned char*)(&ul->rgb));
                    r = *(2+(unsigned char*)(&ul->rgb));
                    glColor3ub(r,g,b);

                    //glColor3ub(255,0,0);
                    glVertex3f(ul->x, ul->y, ul->z);
                  }
                } 
                if(!strip_on) { //can't use the point on the upper left, should I still init a triangle?
                  const point_type* lr = &pc->points[(x+step)+(y+step)*w]; //one right-down (lower right)
                  if(!hasValidXYZ(*lr)) {
                    //if this is not valid, there is no way to make a new triangle in the next step
                    //and one could have been drawn starting in this step, only if ul had been valid
                    x++;
                    continue;
                  } else { //at least one can be started at the lower left
                    depth = squaredEuclideanDistance(*ur,origin);
                    if (squaredEuclideanDistance(*ur,*ll)/depth <= mesh_thresh  and 
                        squaredEuclideanDistance(*lr,*ll)/depth <= mesh_thresh  and
                        squaredEuclideanDistance(*ur,*lr)/depth <= mesh_thresh){
                      glBegin(GL_TRIANGLE_STRIP);
                      strip_on = true;
                      flip = true; //but the lower has to be inserted first, for correct order
                    }
                  }
                }
                if(strip_on) { //Be this the second or the first vertex, insert it
                  b = *(  (unsigned char*)(&ll->rgb));
                  g = *(1+(unsigned char*)(&ll->rgb));
                  r = *(2+(unsigned char*)(&ll->rgb));
                  glColor3ub(r,g,b);

                  //glColor3ub(0,255,0);
                  glVertex3f(ll->x, ll->y, ll->z);
                }
                continue; //not relevant but demonstrate that nothing else is done in this iteration
            } // end strip was off
            else 
            {//neighbours to the left and left down are already set
              const point_type* ul;
              if(flip){ ul = &pc->points[(x)+(y+step)*w]; } //one down (lower left corner) 
              else { ul = &pc->points[x+y*w]; } //current point (upper right)
              if(hasValidXYZ(*ul)){ //Neighbours to the left are prepared
                depth = squaredEuclideanDistance(*ul,origin);
                if (squaredEuclideanDistance(*ul,*(ul-step))/depth > mesh_thresh){
                  glEnd();
                  strip_on = false;
                  continue;
                }
                //Complete the triangle with both leftern neighbors
                //drawTriangle(*ul, *ll, *ur);
                b = *(  (unsigned char*)(&ul->rgb));
                g = *(1+(unsigned char*)(&ul->rgb));
                r = *(2+(unsigned char*)(&ul->rgb));
                glColor3ub(r,g,b);

                //glColor3ub(255,0,0);
                glVertex3f(ul->x, ul->y, ul->z);
              } else {
                glEnd();
                strip_on = false;
                continue; //TODO: Could restart with next point instead
              }
              //The following point connects one to the left with the other on this horizontal level
              const point_type* ll;
              if(flip){ ll = &pc->points[x+y*w]; } //current point (upper right)
              else { ll = &pc->points[(x)+(y+step)*w]; } //one down (lower left corner) 
              if(hasValidXYZ(*ll)){ 
                depth = squaredEuclideanDistance(*ll,origin);
                if (squaredEuclideanDistance(*ul,*ll)/depth > mesh_thresh or
                    squaredEuclideanDistance(*ul,*(ul-step))/depth > mesh_thresh or
                    squaredEuclideanDistance(*ll,*(ll-step))/depth > mesh_thresh){
                  glEnd();
                  strip_on = false;
                  continue;
                }
                b = *(  (unsigned char*)(&ll->rgb));
                g = *(1+(unsigned char*)(&ll->rgb));
                r = *(2+(unsigned char*)(&ll->rgb));
                glColor3ub(r,g,b);

                glVertex3f(ll->x, ll->y, ll->z);
              } else {
                glEnd();
                strip_on = false;
                continue;
              }
            }//completed triangles if strip is running
        }
        if(strip_on) glEnd();
        strip_on = false;
    }
    ROS_DEBUG("Compiled pointcloud into list %i",  cloud_list_index);
    glEndList();
    //pointcloud_type pc_empty;
    //pc_empty.points.swap(pc->points);
    //pc->width = 0;
    //pc->height = 0;
    Q_EMIT cloudRendered(pc);
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

void GLViewer::deleteLastNode(){
  if(cloud_list_indices.size() <= 1){
    this->reset();
    return;
  }
	GLuint nodeId = cloud_list_indices.back();
	cloud_list_indices.pop_back();
	glDeleteLists(nodeId,1);
}

void GLViewer::pointCloud2GLList(pointcloud_type const * pc){
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    ROS_DEBUG("Making GL list from point-cloud pointer %p in thread %d", pc, (unsigned int)QThread::currentThreadId());
    GLuint cloud_list_index = glGenLists(1);
    if(!cloud_list_index) {
        ROS_ERROR("No display list could be created");
        return;
    }
    float mesh_thresh = ParameterServer::instance()->get<double>("squared_meshing_threshold");
    cloud_list_indices.push_back(cloud_list_index);
    glNewList(cloud_list_index, GL_COMPILE);
    glBegin(GL_TRIANGLES);
    //ROS_INFO_COND(!pc->is_dense, "Expected dense cloud for opengl drawing");
    point_type origin;
    origin.x = 0;
    origin.y = 0;
    origin.z = 0;

    float depth;
    unsigned int w=pc->width, h=pc->height;
    for(unsigned int x = 0; x < w-1; x++){
        for(unsigned int y = 0; y < h-1; y++){
            using namespace pcl;

            const point_type* pi = &pc->points[x+y*w]; //current point

            if(!(hasValidXYZ(*pi))) continue;
            depth = squaredEuclideanDistance(*pi,origin);

            const point_type* pl = &pc->points[(x+1)+(y+1)*w]; //one right-down
            if(!(hasValidXYZ(*pl)) or squaredEuclideanDistance(*pi,*pl)/depth > mesh_thresh)  
              continue;

            const point_type* pj = &pc->points[(x+1)+y*w]; //one right
            if(hasValidXYZ(*pj)
               and squaredEuclideanDistance(*pi,*pj)/depth <= mesh_thresh  
               and squaredEuclideanDistance(*pj,*pl)/depth <= mesh_thresh){
              drawTriangle(*pi, *pj, *pl);
            }
            const point_type* pk = &pc->points[(x)+(y+1)*w]; //one down
            
            if(hasValidXYZ(*pk)
               and squaredEuclideanDistance(*pi,*pk)/depth <= mesh_thresh  
               and squaredEuclideanDistance(*pk,*pl)/depth <= mesh_thresh){
              drawTriangle(*pi, *pk, *pl);
            }
        }
    }
    glEnd();
    ROS_DEBUG("Compiled pointcloud into list %i",  cloud_list_index);
    glEndList();
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

void GLViewer::reset(){
    glDeleteLists(1,cloud_list_indices.back());
    cloud_list_indices.clear();
    cloud_matrices->clear();
    if(edge_list_) {
      delete edge_list_;
      edge_list_ = NULL;
    }
    updateGL();
}
QImage GLViewer::renderList(QMatrix4x4 transform, int list_id){
    return QImage();
}

void GLViewer::setEdges(QList<QPair<int, int> >* edge_list){
  if(edge_list_) delete edge_list_;
  edge_list_ = edge_list;
}

void GLViewer::drawEdges(){
  if(edge_list_ == NULL) return;
  glBegin(GL_LINES);
  glLineWidth(12);
  for(int i = 0; i < edge_list_->size(); i++){
    int id1 = (*edge_list_)[i].first;
    int id2 = (*edge_list_)[i].second;
    float x,y,z;
    if(cloud_matrices->size() > id1 && cloud_matrices->size() > id2){//only happens in weird circumstances
      if(abs(id1 - id2) == 1){//consecutive
        glColor4f(bg_col_[0],1-bg_col_[1],1-bg_col_[2],1.0); //cyan on black, red on white
      } else if(abs(id1 - id2) > 20){//consider a loop closure
        glColor4f(1-bg_col_[0],1-bg_col_[1],bg_col_[2],0.4); //orange on black, blue on black, transp
      } else { //near predecessor
        glColor4f(1-bg_col_[0],1-bg_col_[1],1-bg_col_[2],0.4); //inverse of bg color, but transp
      }
      x = (*cloud_matrices)[id1](0,3);
      y = (*cloud_matrices)[id1](1,3);
      z = (*cloud_matrices)[id1](2,3);
      glVertex3f(x,y,z);
      x = (*cloud_matrices)[id2](0,3);
      y = (*cloud_matrices)[id2](1,3);
      z = (*cloud_matrices)[id2](2,3);
      glVertex3f(x,y,z);
    }
    else ROS_ERROR("Not enough cloud matrices (%d) for vertex ids (%d and %d)", cloud_matrices->size(), id1, id2);
  }
  glEnd();
}


void GLViewer::setViewPoint(QMatrix4x4 new_vp){
    ///Moving the camera is inverse to moving the points to draw
    viewpoint_tf_ = new_vp.inverted();
}

bool GLViewer::setClickedPosition(int x, int y) {
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posX, posY, posZ;

    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );

    winX = (float)x;
    winY = (float)viewport[3] - (float)y;
    glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
    if(winZ != 1){ //default value, where nothing was rendered
      gluUnProject( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
      ROS_INFO_STREAM((float)winZ << ", " << posX << "," << posY << "," << posZ);
      viewpoint_tf_(0,3) = -posX;
      viewpoint_tf_(1,3) = -posY;
      viewpoint_tf_(2,3) = -posZ;
      return true;
    } else {
      return false;
    }
}

void GLViewer::toggleTriangulation() {
    ROS_INFO("Toggling Triangulation");
    if(polygon_mode == GL_FILL){ // Turn on Pointcloud mode
        polygon_mode = GL_POINT;
    } else if(polygon_mode == GL_POINT){ // Turn on Wireframe mode
        polygon_mode = GL_LINE;
    } else { // Turn on Surface mode
        polygon_mode = GL_FILL;
    }
    glPolygonMode(GL_FRONT_AND_BACK, polygon_mode);
    updateGL();
}

void GLViewer::drawToPS(QString filname){
  FILE *fp = fopen("glviewer.pdf", "wb");
  GLint buffsize = 0, state = GL2PS_OVERFLOW;
  GLint viewport[4];
  char *oldlocale = setlocale(LC_NUMERIC, "C");


  glGetIntegerv(GL_VIEWPORT, viewport);

  while( state == GL2PS_OVERFLOW ){
    buffsize += 1024*1024;
    gl2psBeginPage ( "GL View", "RGBD-SLAM", viewport,
                     GL2PS_PDF, GL2PS_BSP_SORT, GL2PS_SILENT |
                     GL2PS_SIMPLE_LINE_OFFSET | GL2PS_NO_BLENDING |
                     GL2PS_OCCLUSION_CULL | GL2PS_BEST_ROOT,
                     GL_RGBA, 0, NULL, 0, 0, 0, buffsize,
                     fp, "LatexFile" );
    drawClouds(0.0);
    state = gl2psEndPage();
  }
  setlocale(LC_NUMERIC, oldlocale);
  fclose(fp);
}

