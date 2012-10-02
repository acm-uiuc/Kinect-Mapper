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
 */
#include <QtGui>
#include <QDir>
#include <QPixmap>
#include <QFont>
#include <QList>
#include <QIcon>
#include <QKeySequence>
#include "qt_gui.h"
#include <limits>
#include "glviewer.h"

///Constructs a QT GUI for easy control of RGBDSLAM
Graphical_UI::Graphical_UI() : filename("quicksave.pcd"), glviewer(NULL)
{
    QWidget *widget = new QWidget;
    setCentralWidget(widget);

    //QWidget *topFiller = new QWidget;
    //topFiller->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    infoText = new QString(tr(
                "<p><b>RGBDSLAM</b> uses visual features to identify corresponding 3D locations "
                "in RGBD data. The correspondences are used to reconstruct the camera motion. "
                "The SLAM-backend g2o is used to integrate the transformations between"
                "the RGBD-images and compute a globally consistent 6D trajectory.</p>"
                "<p></p>"));
    licenseText = new QString(tr(
                 "<p>RGBDSLAM is free software: you can redistribute it and/or modify"
                 "it under the terms of the GNU General Public License as published by"
                 "the Free Software Foundation, either version 3 of the License, or"
                 "(at your option) any later version.</p>"
                 "<p>RGBDSLAM is distributed in the hope that it will be useful,"
                 "but WITHOUT ANY WARRANTY; without even the implied warranty of"
                 "MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the"
                 "GNU General Public License for more details.</p>"
                 "<p>You should have received a copy of the GNU General Public License"
                 "along with RGBDSLAM.  If not, refer to <a href=\"http://www.gnu.org/licenses/\">http://www.gnu.org/licenses</a>.</p>"));
    mouseHelpText = new QString(tr(
                "<p><b>3D Viewer Mouse Commands:</b>"
                "<ul><li><i>Left/Right button:</i> rotation.</li>"
                "    <li><i>Middle button:</i> shift.</li>"
                "    <li><i>Wheel:</i> zoom.</li>"
                "    <li><i>Middle double click:</i> reset camera position.</li>"
                "    <li><i>Double click on object:</i> set pivot to clicked point.</li>"
                "    <li><i>Double click on background:</i> reset view to camera pose.</li><ul></p>")); feature_flow_image_label = new QLabel(*mouseHelpText);
    feature_flow_image_label->setWordWrap(true);
    feature_flow_image_label->setMargin(10);
    feature_flow_image_label->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    visual_image_label = new QLabel("<i>Waiting for monochrome image...</i>");
    visual_image_label->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    visual_image_label->setAlignment(Qt::AlignCenter);
    //visual_image_label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    depth_image_label = new QLabel(tr("<i>Waiting for depth image...</i>"));
    depth_image_label->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    depth_image_label->setAlignment(Qt::AlignCenter);
    //depth_image_label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    //transform_label = new QLabel(tr("<i>Waiting for transformation matrix...</i>"));
    //transform_label->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    //transform_label->setAlignment(Qt::AlignCenter);
    if(ParameterServer::instance()->get<bool>("use_glwidget")) glviewer = new GLViewer(this);//displays the cloud in 3d

    //QFont typewriter_font;
    //typewriter_font.setStyleHint(QFont::TypeWriter);
    //transform_label->setFont(typewriter_font);

    //QWidget *bottomFiller = new QWidget;
    //bottomFiller->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    gridlayout = new QGridLayout;
    gridlayout->setMargin(5);
    //gridlayout->addWidget(infoLabel, 0,0);
    //gridlayout->addWidget(transform_label, 0,0,1,0); //with rowspan
    //gridlayout->addWidget(transform_label, 0,0);
    if(ParameterServer::instance()->get<bool>("use_glwidget")) gridlayout->addWidget(glviewer, 0,0,1,0);
    gridlayout->addWidget(visual_image_label, 1,0);
    gridlayout->addWidget(depth_image_label, 1,1);
    gridlayout->addWidget(feature_flow_image_label, 1,2);
    widget->setLayout(gridlayout);

    createMenus();

    tmpLabel = new QLabel();
    statusBar()->insertWidget(0,tmpLabel, 0);
    QString message = tr("Ready for RGBDSLAM");
    statusBar()->showMessage(message);

    infoLabel2 = new QLabel(tr("Waiting for motion information..."));
    infoLabel2->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    infoLabel2->setAlignment(Qt::AlignRight);
    statusBar()->addPermanentWidget(infoLabel2, 0);

    infoLabel = new QLabel(tr("<i>Press Enter or Space to Start</i>"));
    infoLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    infoLabel->setAlignment(Qt::AlignRight);
    statusBar()->addPermanentWidget(infoLabel, 0);

    setWindowTitle(tr("RGBDSLAM"));
    setMinimumSize(790, 590);
    resize(1000, 700);
}

void Graphical_UI::setFeatureFlowImage(QImage qimage){
  feature_flow_image_label->setAlignment(Qt::AlignCenter);
  feature_flow_image_label->setPixmap(QPixmap::fromImage(qimage));
}
void Graphical_UI::setVisualImage(QImage qimage){
  if(visual_image_label->isVisible()){
      visual_image_label->setPixmap(QPixmap::fromImage(qimage));
      visual_image_label->repaint();
  }
}

void Graphical_UI::setDepthImage(QImage qimage){
  if(depth_image_label->isVisible()){
      depth_image_label->setPixmap(QPixmap::fromImage(qimage));
      depth_image_label->repaint();
  }
}

void Graphical_UI::resetCmd() {
    Q_EMIT reset();
    if(ParameterServer::instance()->get<bool>("use_glwidget")) glviewer->reset();
    QString message = tr("Graph Reset");
    statusBar()->showMessage(message);
    infoLabel->setText("A fresh new graph is waiting");
}

void Graphical_UI::setStatus(QString message){
    statusBar()->showMessage(message);
}
void Graphical_UI::setInfo2(QString message){
    infoLabel2->setText(message);
    infoLabel2->repaint();
}
void Graphical_UI::setInfo(QString message){
    infoLabel->setText(message);
    infoLabel->repaint();
}

void Graphical_UI::quickSaveAll() {
    Q_EMIT saveAllClouds(filename);
    QString message = tr("Saving Whole Model to ");
    message.append(QDir::currentPath());
    message.append(filename);
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}

void Graphical_UI::saveFeatures() {
    filename = QFileDialog::getSaveFileName(this, "Save Features to File", filename, tr("YAML (*.yml);;XML (*.xml)"));
    Q_EMIT saveAllFeatures(filename);
    QString message = tr("Saving Features");
    statusBar()->showMessage(message);
}

void Graphical_UI::saveAll() {
    filename = QFileDialog::getSaveFileName(this, "Save Point CLoud to File", filename, tr("PCD (*.pcd);;PLY (*ply)"));
    Q_EMIT saveAllClouds(filename);
    QString message = tr("Saving Whole Model");
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}
void Graphical_UI::showEdgeErrors() {
    QString myfilename = QFileDialog::getSaveFileName(this, "Save Current Trajectory Estimate", "trajectory", tr("All Files (*.*)"));
    Q_EMIT printEdgeErrors(myfilename);
    QString message = tr("Triggering Edge Printing");
    statusBar()->showMessage(message);
}
void Graphical_UI::optimizeGraphTrig() {
    Q_EMIT optimizeGraph();
    QString message = tr("Triggering Optimizer");
    statusBar()->showMessage(message);
}
void Graphical_UI::saveVectorGraphic() {
    QMessageBox::warning(this, tr("Don't render to pdf while point clouds are shown"), tr("This is meant for rendereing the pose graph. Rendering clouds to pdf will generate huge files!"));
    QString myfilename = QFileDialog::getSaveFileName(this, "Save Current 3D Display to Vector Graphic", "pose_graph.pdf", tr("All Files (*.*)"));
    glviewer->drawToPS(myfilename);
    QString message = tr("Drawing current Display");
    statusBar()->showMessage(message);
}
void Graphical_UI::saveTrajectoryDialog() {
    QString myfilename = QFileDialog::getSaveFileName(this, "Save Current Trajectory Estimate", "trajectory", tr("All Files (*.*)"));
    Q_EMIT saveTrajectory(myfilename);
    QString message = tr("Saving current trajectory estimate (and possibly ground truth).");
    statusBar()->showMessage(message);
}
void Graphical_UI::saveIndividual() {
    QString tmpfilename(filename);
    tmpfilename.remove(".pcd", Qt::CaseInsensitive);
    tmpfilename.remove(".ply", Qt::CaseInsensitive);
    filename = QFileDialog::getSaveFileName(this, "Save point cloud to one file per node", tmpfilename, tr("PCD (*.pcd)"));
    Q_EMIT saveIndividualClouds(filename);
    QString message = tr("Saving Model Node-Wise");
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}

void Graphical_UI::sendAll() {
    Q_EMIT sendAllClouds();
    QString message = tr("Sending Whole Model");
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}

void Graphical_UI::setRotationGrid() {
    bool ok;
    double value = QInputDialog::getDouble(this, tr("Set Rotation Stepping in Degree"),
                                           tr("Enter Stepping:"), 1.00, -10000000, 10000000, 2, &ok);
    if(ok){ glviewer->setRotationGrid(value); }
}
void Graphical_UI::setStereoShift() {
    bool ok;
    double value = QInputDialog::getDouble(this, tr("Set Stereo Camera shift"),
                                           tr("Enter Shift:"), 0.10, -10000000, 10000000, 2, &ok);
    if(ok){ glviewer->setStereoShift(value); }
}

void Graphical_UI::setMax() {
    bool ok;
    double value = QInputDialog::getDouble(this, tr("Set Max Depth"),
                                            tr("Enter Max Depth [in cm]\n (negativ if no Filtering is required):"), -100.00, -10000000, 10000000, 2, &ok);
    if(ok){
    	Q_EMIT setMaxDepth(value/100.0);
    }
}
void Graphical_UI::toggleFullscreen(bool mode){
    this->menuBar()->setVisible(!mode);
    this->gridlayout->setMargin(mode?0:5);
    this->statusBar()->setVisible(!mode);
    if(mode){
      this->showFullScreen();
    } else {
      this->showNormal();
    }
}
void Graphical_UI::pruneEdgesWithHighError(){
    bool ok;
    float value = QInputDialog::getDouble(this, tr("Set Max Edge Error"),
                                          tr("No Text"), 1.00, -10000000, 10000000, 2, &ok);
    if(ok){
    	Q_EMIT pruneEdgesWithErrorAbove(value);
    }
}
void Graphical_UI::sendFinished() {
    
    QString message = tr("Finished Sending");
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}

void Graphical_UI::getOneFrameCmd() {
    Q_EMIT getOneFrame();
    QString message = tr("Getting a single frame");
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}
void Graphical_UI::deleteLastFrameCmd() {
    Q_EMIT deleteLastFrame();
    QString message = tr("Deleting the last node from the graph");
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}

void Graphical_UI::toggleMappingPriv(bool mapping_on) {
    Q_EMIT toggleMapping(mapping_on);
}

void Graphical_UI::bagRecording(bool pause_on) {
    Q_EMIT toggleBagRecording();
    if(pause_on) {
        QString message = tr("Recording Bagfile.");
        statusBar()->showMessage(message);
        //infoLabel->setText(message);
    } else {
        QString message = tr("Stopped Recording.");
        statusBar()->showMessage(message);
        //infoLabel->setText(message);
    }
}
void Graphical_UI::toggleCloudStorage(bool storage) {
  ParameterServer::instance()->set("store_pointclouds", storage);
}

void Graphical_UI::pause(bool pause_on) {
    Q_EMIT togglePause();
    if(pause_on) {
        QString message = tr("Processing.");
        statusBar()->showMessage(message);
        //infoLabel->setText(message);
    } else {
        QString message = tr("Stopped processing.");
        statusBar()->showMessage(message);
        //infoLabel->setText(message);
    }
}

void Graphical_UI::help() {
    QMessageBox::about(this, tr("Help Menu"), /**menuHelpText +*/ *mouseHelpText );
}
void Graphical_UI::about() {
    QMessageBox::about(this, tr("About RGBDSLAM"), *infoText + *licenseText);
}

void Graphical_UI::set2DStream(bool is_on) {
    if(is_on){ 
        visual_image_label->show();
        depth_image_label->show(); 
        feature_flow_image_label->show(); 
    } else { 
        visual_image_label->hide(); 
        depth_image_label->hide(); 
        feature_flow_image_label->hide(); 
    } 
}

/*
void Graphical_UI::set3DDisplay(bool is_on) {
    if(!ParameterServer::instance()->get<bool>("use_glwidget")) return;
    if(is_on){ glviewer->show(); } 
    else { glviewer->hide(); } 
}
*/


void Graphical_UI::createMenus() {
    //these are the menus created here
    QMenu *graphMenu;
    QMenu *actionMenu;
    QMenu *viewMenu;
    QMenu *helpMenu;

    //Graph Menu
    graphMenu = menuBar()->addMenu(tr("&Graph"));

    QAction *quickSaveAct = new QAction(tr("&Save"), this);
    quickSaveAct->setShortcuts(QKeySequence::Save);
    quickSaveAct->setStatusTip(tr("Save all stored point clouds with common coordinate frame to a pcd file"));
    quickSaveAct->setIcon(QIcon::fromTheme("document-save"));//doesn't work for gnome
    connect(quickSaveAct, SIGNAL(triggered()), this, SLOT(quickSaveAll()));
    graphMenu->addAction(quickSaveAct);
    this->addAction(quickSaveAct);

    QAction *saveFeaturesAct = new QAction(tr("Save &Features"), this);
    saveFeaturesAct->setShortcut(QString("Ctrl+F"));
    saveFeaturesAct->setStatusTip(tr("Save all feature positions and descriptions in a common coordinate frame to a yaml or xml file"));
    saveFeaturesAct->setIcon(QIcon::fromTheme("document-save"));//doesn't work for gnome
    connect(saveFeaturesAct, SIGNAL(triggered()), this, SLOT(saveFeatures()));
    graphMenu->addAction(saveFeaturesAct);
    this->addAction(saveFeaturesAct);

    QAction *saveAct = new QAction(tr("&Save as..."), this);
    saveAct->setShortcuts(QKeySequence::SaveAs);
    saveAct->setStatusTip(tr("Save all stored point clouds with common coordinate frame"));
    saveAct->setIcon(QIcon::fromTheme("document-save-as"));//doesn't work for gnome
    connect(saveAct, SIGNAL(triggered()), this, SLOT(saveAll()));
    graphMenu->addAction(saveAct);
    this->addAction(saveAct);

    QAction *saveIndiAct = new QAction(tr("&Save Node-Wise..."), this);
    saveIndiAct->setShortcut(QString("Ctrl+N"));
    saveIndiAct->setStatusTip(tr("Save stored point clouds in individual files"));
    saveAct->setIcon(QIcon::fromTheme("document-save-all"));//doesn't work for gnome
    connect(saveIndiAct, SIGNAL(triggered()), this, SLOT(saveIndividual()));
    graphMenu->addAction(saveIndiAct);
    this->addAction(saveIndiAct);

    QAction *sendAct = new QAction(tr("&Send Model"), this);
    sendAct->setShortcut(QString("Ctrl+M"));
    sendAct->setStatusTip(tr("Send out all stored point clouds with corrected transform"));
    sendAct->setIcon(QIcon::fromTheme("document-send"));//doesn't work for gnome
    connect(sendAct, SIGNAL(triggered()), this, SLOT(sendAll()));
    graphMenu->addAction(sendAct);
    this->addAction(sendAct);

    graphMenu->addSeparator();

    QAction *toggleMappingAct = new QAction(tr("Toggle &Mapping"), this);
    toggleMappingAct->setShortcut(QString("M"));
    toggleMappingAct->setCheckable(true);
    toggleMappingAct->setChecked(true);
    toggleMappingAct->setStatusTip(tr("Toggle between SLAM and Localization"));
    toggleMappingAct->setIcon(QIcon::fromTheme("media-playback-start"));//doesn't work for gnome
    connect(toggleMappingAct, SIGNAL(toggled(bool)), this, SLOT(toggleMappingPriv(bool)));
    graphMenu->addAction(toggleMappingAct);
    this->addAction(toggleMappingAct);

    graphMenu->addSeparator();

    QAction *optimizeAct = new QAction(tr("Optimize Trajectory &Estimate"), this);
    optimizeAct->setShortcut(QString("O"));
    optimizeAct->setStatusTip(tr("Compute optimized pose graph with g2o"));
    connect(optimizeAct, SIGNAL(triggered()), this, SLOT(optimizeGraphTrig()));
    graphMenu->addAction(optimizeAct);
    this->addAction(optimizeAct);

    graphMenu->addSeparator();

    QAction *exitAct = new QAction(tr("E&xit"), this);
    exitAct->setShortcuts(QKeySequence::Quit);
    exitAct->setStatusTip(tr("Exit the application"));
    exitAct->setIcon(QIcon::fromTheme("application-exit"));//doesn't work for gnome
    connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));
    graphMenu->addAction(exitAct);
    this->addAction(exitAct);


    //Processing Menu
    actionMenu = menuBar()->addMenu(tr("&Processing"));

    QAction *newAct;
    newAct = new QAction(tr("&Reset"), this);
    newAct->setShortcut(QString("Ctrl+R"));
    newAct->setStatusTip(tr("Reset the graph, clear all data collected"));
    newAct->setIcon(QIcon::fromTheme("edit-delete"));//doesn't work (for gnome
    connect(newAct, SIGNAL(triggered()), this, SLOT(resetCmd()));
    actionMenu->addAction(newAct);
    this->addAction(newAct);

    QAction *pauseAct = new QAction(tr("&Process"), this);
    pauseAct->setShortcut(QString(" "));
    pauseAct->setCheckable(true);
    pauseAct->setChecked(!ParameterServer::instance()->get<bool>("start_paused"));
    pauseAct->setStatusTip(tr("Start/stop processing of frames"));
    pauseAct->setIcon(QIcon::fromTheme("media-playback-start"));//doesn't work for gnome
    connect(pauseAct, SIGNAL(toggled(bool)), this, SLOT(pause(bool)));
    actionMenu->addAction(pauseAct);
    this->addAction(pauseAct);

    QAction *oneFrameAct = new QAction(tr("Capture One& Frame"), this);
    oneFrameAct->setShortcuts(QKeySequence::InsertParagraphSeparator);
    oneFrameAct->setStatusTip(tr("Process one frame only"));
    connect(oneFrameAct, SIGNAL(triggered()), this, SLOT(getOneFrameCmd()));
    actionMenu->addAction(oneFrameAct);
    this->addAction(oneFrameAct);

    QAction *delFrameAct = new QAction(tr("&Delete Last Node"), this);
    delFrameAct->setShortcut(QString("Backspace"));
    delFrameAct->setStatusTip(tr("Remove last node from graph"));
    delFrameAct->setIcon(QIcon::fromTheme("edit-undo"));//doesn't work for gnome
    connect(delFrameAct, SIGNAL(triggered()), this, SLOT(deleteLastFrameCmd()));
    actionMenu->addAction(delFrameAct);
    this->addAction(delFrameAct);

    QAction *bagRecordingAct = new QAction(tr("&Bagfile Recording"), this);
    bagRecordingAct->setShortcut(QString("R"));
    bagRecordingAct->setCheckable(true);
    bagRecordingAct->setChecked(false);
    bagRecordingAct->setStatusTip(tr("Start/stop recording of frames to bagfile"));
    bagRecordingAct->setIcon(QIcon::fromTheme("media-record"));//doesn't work for gnome
    connect(bagRecordingAct, SIGNAL(toggled(bool)), this, SLOT(bagRecording(bool)));
    actionMenu->addAction(bagRecordingAct);
    this->addAction(bagRecordingAct);

    QAction *compareAct = new QAction(tr("Save Trajectory &Estimate"), this);
    compareAct->setShortcut(QString("Ctrl+E"));
    compareAct->setStatusTip(tr("Save trajectory estimate (and ground truth trajectory if available) for external evaluation."));
    connect(compareAct, SIGNAL(triggered()), this, SLOT(saveTrajectoryDialog()));
    actionMenu->addAction(compareAct);
    this->addAction(compareAct);


    /*
    QAction *showErrorAct = new QAction(tr("Show Edge Errors"), this);
    showErrorAct->setShortcut(QString("Ctrl+Shift+E"));
    showErrorAct->setStatusTip(tr(""));
    connect(showErrorAct, SIGNAL(triggered()), this, SLOT(showEdgeErrors()));
    actionMenu->addAction(showErrorAct);
    this->addAction(showErrorAct);
    */

    QAction *maxAct = new QAction(tr("Set Maximum &Depth"), this);
    maxAct->setShortcut(QString("Ctrl+D"));
    maxAct->setStatusTip(tr("Set the Maximum Depth a Point can have (negativ if no Filtering is required)"));
    connect(maxAct, SIGNAL(triggered()), this, SLOT(setMax()));
    actionMenu->addAction(maxAct);
    this->addAction(maxAct);

    QAction *pruneAct = new QAction(tr("Set Ma&ximum Edge Error"), this);
    pruneAct->setShortcut(QString("Ctrl+X"));
    pruneAct->setStatusTip(tr("Set the Maximum Allowed for Edges"));
    connect(pruneAct, SIGNAL(triggered()), this, SLOT(pruneEdgesWithHighError()));
    actionMenu->addAction(pruneAct);
    this->addAction(pruneAct);

    QAction *psOutputAct = new QAction(tr("&Write PDF File"), this);
    psOutputAct->setShortcut(QString("W"));
    psOutputAct->setStatusTip(tr("Write 3D Scene to a PDF File. Warning: Meant for Pose Graphs not for the clouds"));
    psOutputAct->setIcon(QIcon::fromTheme("application-pdf"));//doesn't work for gnome
    connect(psOutputAct, SIGNAL(triggered()), this, SLOT(saveVectorGraphic()));
    actionMenu->addAction(psOutputAct);
    this->addAction(psOutputAct);

    QAction *toggleCloudStorageAct = new QAction(tr("&Store Point Clouds"), this);
    QList<QKeySequence> tcs_shortcuts;
    tcs_shortcuts.append(QString("Ctrl+P"));
    toggleCloudStorageAct->setShortcuts(tcs_shortcuts);
    toggleCloudStorageAct->setCheckable(true);
    toggleCloudStorageAct->setChecked(ParameterServer::instance()->get<bool>("store_pointclouds"));
    toggleCloudStorageAct->setStatusTip(tr("Toggle storing of point clouds (for later sending, map creation)"));
    toggleCloudStorageAct->setIcon(QIcon::fromTheme("server-database"));//doesn't work for gnome
    connect(toggleCloudStorageAct, SIGNAL(toggled(bool)), this, SLOT(toggleCloudStorage(bool)));
    actionMenu->addAction(toggleCloudStorageAct);
    this->addAction(toggleCloudStorageAct);


    //View Menu ###############################################################
    viewMenu = menuBar()->addMenu(tr("&View"));


    QAction *toggleFullscreenAct = new QAction(tr("&Fullscreen"), this);
    QList<QKeySequence> shortcuts;
    shortcuts.append(QString("F"));
    toggleFullscreenAct->setShortcuts(shortcuts);
    toggleFullscreenAct->setCheckable(true);
    toggleFullscreenAct->setChecked(false);
    toggleFullscreenAct->setStatusTip(tr("Toggle Fullscreen"));
    toggleFullscreenAct->setIcon(QIcon::fromTheme("view-fullscreen"));//doesn't work for gnome
    connect(toggleFullscreenAct, SIGNAL(toggled(bool)), this, SLOT(toggleFullscreen(bool)));
    viewMenu->addAction(toggleFullscreenAct);
    this->addAction(toggleFullscreenAct);

    QAction *toggleStreamAct = new QAction(tr("Toggle &2D Stream"), this);
    toggleStreamAct->setShortcut(QString("2"));
    toggleStreamAct->setCheckable(true);
    toggleStreamAct->setChecked(true);
    toggleStreamAct->setStatusTip(tr("Turn off the Image Stream"));
    connect(toggleStreamAct, SIGNAL(toggled(bool)), this, SLOT(set2DStream(bool)));
    viewMenu->addAction(toggleStreamAct);
    this->addAction(toggleStreamAct);

    if(ParameterServer::instance()->get<bool>("use_glwidget"))
    {
      QAction *toggleGLViewerAct = new QAction(tr("Toggle &3D Display"), this);
      toggleGLViewerAct->setShortcut(QString("3"));
      toggleGLViewerAct->setCheckable(true);
      toggleGLViewerAct->setChecked(true);
      toggleGLViewerAct->setStatusTip(tr("Turn off the OpenGL Display"));
      connect(toggleGLViewerAct, SIGNAL(toggled(bool)), glviewer, SLOT(setVisible(bool)));
      viewMenu->addAction(toggleGLViewerAct);
      this->addAction(toggleGLViewerAct);

      QAction *toggleTriangulationAct = new QAction(tr("&Toggle Triangulation"), this);
      toggleTriangulationAct->setShortcut(QString("T"));
      toggleTriangulationAct->setStatusTip(tr("Switch between surface, wireframe and point cloud"));
      connect(toggleTriangulationAct, SIGNAL(triggered(bool)), glviewer, SLOT(toggleTriangulation()));
      viewMenu->addAction(toggleTriangulationAct);
      this->addAction(toggleTriangulationAct);

      QAction *toggleFollowAct = new QAction(tr("Follow &Camera"), this);
      toggleFollowAct->setShortcut(QString("Shift+F"));
      toggleFollowAct->setCheckable(true);
      toggleFollowAct->setChecked(true);
      toggleFollowAct->setStatusTip(tr("Always use viewpoint of last frame (except zoom)"));
      connect(toggleFollowAct, SIGNAL(toggled(bool)), glviewer, SLOT(toggleFollowMode(bool)));
      viewMenu->addAction(toggleFollowAct);
      this->addAction(toggleFollowAct);

      QAction *toggleShowIDsAct = new QAction(tr("Show Pose IDs"), this);
      toggleShowIDsAct->setShortcut(QString("I"));
      toggleShowIDsAct->setCheckable(true);
      toggleShowIDsAct->setChecked(false);
      toggleShowIDsAct->setStatusTip(tr("Display pose ids at axes"));
      connect(toggleShowIDsAct, SIGNAL(toggled(bool)), glviewer, SLOT(toggleShowIDs(bool)));
      viewMenu->addAction(toggleShowIDsAct);
      this->addAction(toggleShowIDsAct);

      QAction *toggleShowPosesAct = new QAction(tr("Show &Poses of Graph"), this);
      toggleShowPosesAct->setShortcut(QString("P"));
      toggleShowPosesAct->setCheckable(true);
      toggleShowPosesAct->setChecked(ParameterServer::instance()->get<bool>("show_axis"));
      toggleShowPosesAct->setStatusTip(tr("Display poses as axes"));
      connect(toggleShowPosesAct, SIGNAL(toggled(bool)), glviewer, SLOT(toggleShowPoses(bool)));
      viewMenu->addAction(toggleShowPosesAct);
      this->addAction(toggleShowPosesAct);

      QAction *toggleShowEdgesAct = new QAction(tr("Show &Edges of Graph"), this);
      toggleShowEdgesAct->setShortcut(QString("E"));
      toggleShowEdgesAct->setCheckable(true);
      toggleShowEdgesAct->setChecked(ParameterServer::instance()->get<bool>("show_axis"));
      toggleShowEdgesAct->setStatusTip(tr("Display edges of pose graph as lines"));
      connect(toggleShowEdgesAct, SIGNAL(toggled(bool)), glviewer, SLOT(toggleShowEdges(bool)));
      viewMenu->addAction(toggleShowEdgesAct);
      this->addAction(toggleShowEdgesAct);

      QAction *toggleStereoAct = new QAction(tr("Stere&o View"), this);
      toggleStereoAct->setShortcut(QString("Ctrl+Shift+O"));
      toggleStereoAct->setCheckable(true);
      toggleStereoAct->setChecked(false);
      toggleStereoAct->setStatusTip(tr("Split screen view with slightly shifted Camera"));
      connect(toggleStereoAct, SIGNAL(toggled(bool)), glviewer, SLOT(toggleStereo(bool)));
      viewMenu->addAction(toggleStereoAct);
      this->addAction(toggleStereoAct);

      QAction *toggleCloudDisplay = new QAction(tr("Show &Clouds"), this);
      toggleCloudDisplay->setShortcut(QString("C"));
      toggleCloudDisplay->setCheckable(true);
      toggleCloudDisplay->setChecked(true);
      toggleCloudDisplay->setStatusTip(tr("Toggle whether point clouds should be rendered"));
      connect(toggleCloudDisplay, SIGNAL(toggled(bool)), glviewer, SLOT(toggleShowClouds(bool)));
      viewMenu->addAction(toggleCloudDisplay);
      this->addAction(toggleCloudDisplay);

      QAction *toggleBGColor = new QAction(tr("Toggle Background"), this);
      toggleBGColor->setShortcut(QString("B"));
      toggleBGColor->setCheckable(true);
      toggleBGColor->setChecked(true);
      toggleBGColor->setStatusTip(tr("Toggle whether background should be white or black"));
      connect(toggleBGColor, SIGNAL(toggled(bool)), glviewer, SLOT(toggleBackgroundColor(bool)));
      viewMenu->addAction(toggleBGColor);
      this->addAction(toggleBGColor);

      QAction *setRotationGridAct = new QAction(tr("Set Rotation &Grid"), this);
      setRotationGridAct->setShortcut(QString("G"));
      setRotationGridAct->setStatusTip(tr("Discretize Rotation in Viewer"));
      connect(setRotationGridAct, SIGNAL(triggered()), this, SLOT(setRotationGrid()));
      viewMenu->addAction(setRotationGridAct);
      this->addAction(setRotationGridAct);

      QAction *setShiftAct = new QAction(tr("Set Stereo Offset"), this);
      setShiftAct->setShortcut(QString("<"));
      setShiftAct->setStatusTip(tr("Set the distance between the virtual cameras for stereo view"));
      connect(setShiftAct, SIGNAL(triggered()), this, SLOT(setStereoShift()));
      viewMenu->addAction(setShiftAct);
      this->addAction(setShiftAct);

    }


    //Help Menu
    helpMenu = menuBar()->addMenu(tr("&Help"));

    QAction *helpAct = new QAction(tr("&Usage Help"), this);
    helpAct->setShortcuts(QKeySequence::HelpContents);
    helpAct->setStatusTip(tr("Show usage information"));
    helpAct->setIcon(QIcon::fromTheme("help-contents"));//doesn't work for gnome
    connect(helpAct, SIGNAL(triggered()), this, SLOT(help()));
    helpMenu->addAction(helpAct);
    this->addAction(helpAct);

    QAction *aboutAct = new QAction(tr("&About RGBDSLAM"), this);
    aboutAct->setShortcut(QString("Ctrl+A"));
    aboutAct->setStatusTip(tr("Show information about RGBDSLAM"));
    aboutAct->setIcon(QIcon::fromTheme("help-about"));//doesn't work for gnome
    connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));
    helpMenu->addAction(aboutAct);
    this->addAction(aboutAct);

}

GLViewer* Graphical_UI::getGLViewer() { 
  return glviewer; 
}
