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


/* This is the main widget of the application.
* It sets up some not yet useful menus and
* three qlabels in the layout of the central widget
* that can be used to show qimages via the slots
* setDepthImage and setVisualImage.
*
* there is an alternative: RosUI for headless use of the rgbdslam
*/
#ifndef QTCV_H
#define QTCV_H

#include <QMainWindow>
#include <QGridLayout>
#include "parameter_server.h"
#include <QMatrix4x4>

class QAction;
class QActionGroup;
class QLabel;
class QMenu;
class GLViewer;

//TODO:
//Choice between Binary and ASCII outputfiles
//Buttons for start/stop
//GUI/Commandline options for switching on/off the individual visualizations

//!Constructs a QT GUI for easy control of RGBD-SLAM
/** Small GUI Class to visualize and control rgbdslam
* See Help->About for a short description */
class Graphical_UI: public QMainWindow
{
    Q_OBJECT

public:
    Graphical_UI();
    GLViewer* getGLViewer();
Q_SIGNALS:
    ///User selected to reset the graph
    void reset(); 
    ///User selected to start or resume processing
    void togglePause();
    ///User selected to start or resume bag recording
    void toggleBagRecording();
    ///User wants the next frame to be processed
    void getOneFrame();
    ///User wants the last node to be removed from the graph
    void deleteLastFrame();
    void sendAllClouds(); ///< Signifies the sending of the whole model
    ///User wants the current world model to be saved to a pcd-file or ply file
    void saveAllClouds(QString filename);
    void saveAllFeatures(QString filename);
    void saveTrajectory(QString filename);
    ///User wants the current world model to be saved to one pcd-file per node
    void saveIndividualClouds(QString file_basename);
    void setMaxDepth(float max_depth);
    void evaluation();
    void optimizeGraph();
    void printEdgeErrors(QString);
    void pruneEdgesWithErrorAbove(float);
    void toggleMapping(bool);
     
public Q_SLOTS:
    void setVisualImage(QImage);
    void setFeatureFlowImage(QImage);
    void setDepthImage(QImage);
    void sendFinished(); ///< Call to display, that sending finished

private Q_SLOTS:
    void saveVectorGraphic();
    void resetCmd();
    void sendAll();
    void setMax();
    void setStereoShift();
    void setRotationGrid();
    void saveAll();
    void saveIndividual();
    void quickSaveAll();
    void saveFeatures();
    void pause(bool);
    void bagRecording(bool);
    void about();
    void help();
    void setInfo(QString);
    void setInfo2(QString);
    void setStatus(QString);
    void getOneFrameCmd();
    void deleteLastFrameCmd();
//    void set3DDisplay(bool is_on);
    void set2DStream(bool is_on);
    void saveTrajectoryDialog();
    void optimizeGraphTrig();
    void showEdgeErrors();
    void pruneEdgesWithHighError();
    void toggleFullscreen(bool);
    void toggleCloudStorage(bool);
    void toggleMappingPriv(bool);
private:
    //!Menus and Menu elements are defined here
    void createMenus();

    //QString *menuHelpText;
    QString *mouseHelpText;
    QString *infoText;
    QString *licenseText;
    QLabel *infoLabel;
    QLabel *infoLabel2;
    QLabel *tmpLabel;
    QLabel *visual_image_label;
    QLabel *feature_flow_image_label;
    QLabel *depth_image_label;
    QLabel *stats_image_label;
    //QLabel *transform_label;
    QGridLayout* gridlayout;
    QString filename;
    GLViewer* glviewer;
    bool pause_on;
};

#endif
