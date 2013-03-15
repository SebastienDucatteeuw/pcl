/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 * 
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: $
 *
 * @author: Koen Buys - KU Leuven
 */

#include <pcl/apps/pcd_video_player.h>

//QT4
#include <QApplication>
#include <QMutexLocker>
#include <QEvent>
#include <QObject>
#include <QFileDialog>

// VTK
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>

using namespace pcl;
using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
PCDVideoPlayer::PCDVideoPlayer ()
{
  //Create a timer
  vis_timer_ = new QTimer (this);
  vis_timer_->start (5);//5ms

  connect (vis_timer_, SIGNAL (timeout ()), this, SLOT (timeoutSlot()));

  ui_ = new Ui::MainWindow;
  ui_->setupUi (this);
  
  this->setWindowTitle ("PCL PCD Video Player");

  // Create the list of motions
  motions_ << "In_between_motions" << "Jumping_jacks" << "Wave_left" << "Wave_right" << "Forbid_left";

  /*

  // Set up the qvtk window
  vis_.reset (new pcl::visualization::PCLVisualizer ("", false));
  ui_->qvtkWidget->SetRenderWindow (vis_>getRenderWindow ());
  vis_->setupInteractor (ui_->qvtkWidget->GetInteractor (), ui_->qvtkWidget->GetRenderWindow ());
  vis_->getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
  ui_->qvtkWidget->update ();

*/

  // Configure the motion Type box
  ui_->motionTypeBox->addItems(motions_);

  // Connect all buttons
  connect (ui_->playButton, SIGNAL(clicked()), this, SLOT(playButtonPressed()));
  connect (ui_->saveButton, SIGNAL(clicked()), this, SLOT(saveButtonPressed()));
  connect (ui_->stopButton, SIGNAL(clicked()), this, SLOT(stopButtonPressed()));
  connect (ui_->backButton, SIGNAL(clicked()), this, SLOT(backButtonPressed()));
  connect (ui_->nextButton, SIGNAL(clicked()), this, SLOT(nextButtonPressed()));
  connect (ui_->selectFolderButton, SIGNAL(clicked()), this, SLOT(selectFolderButtonPressed()));
  connect (ui_->selectFilesButton, SIGNAL(clicked()), this, SLOT(selectFilesButtonPressed()));
  
  connect (ui_->indexSlider, SIGNAL(valueChanged(int)), this, SLOT(indexSliderValueChanged(int)));

  connect (ui_->motionTypeBox, SIGNAL(currentIndexChanged (int)), this, SLOT(motionTypeBoxCurrentIndexChanged(int)));

}

void 
PCDVideoPlayer::playButtonPressed()
{

}

void 
PCDVideoPlayer::stopButtonPressed()
{

}

void 
PCDVideoPlayer::backButtonPressed()
{

}

void 
PCDVideoPlayer::nextButtonPressed()
{

}

void 
PCDVideoPlayer::saveButtonPressed()
{
  PCL_INFO("[PCDVideoPlayer::saveButtonPressed] : (I) : called\n");
  PCL_INFO("[PCDVideoPlayer::saveButtonPressed] : (I) : Begin Motion %d\n", ui_->beginMotionRadioButton->isChecked());
  PCL_INFO("[PCDVideoPlayer::saveButtonPressed] : (I) : In Motion %d\n", ui_->inMotionRadioButton->isChecked());
  PCL_INFO("[PCDVideoPlayer::saveButtonPressed] : (I) : End Motion %d\n", ui_->endMotionRadioButton->isChecked());
}

void 
PCDVideoPlayer::selectFolderButtonPressed()
{
  dir_ = QFileDialog::getExistingDirectory(this, tr("Open Directory"), "/home", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks); 
  std::cout << "[PCDVideoPlayer::selectFolderButtonPressed] : selected : " << dir_.toAscii().data() << std::endl;

  current_frame_ = 0;



}

void 
PCDVideoPlayer::selectFilesButtonPressed()
{
  pcd_files_ = QFileDialog::getOpenFileNames(this, "Select one or more PCD files to open", "/home", "PointClouds (*.pcd)");
  nr_of_frames_ = pcd_files_.size();
  std::cout << "[PCDVideoPlayer::selectFilesButtonPressed] : selected " << nr_of_frames_ << " files" << std::endl;

  current_frame_ = 0;

  // Reset the Slider
  ui_->indexSlider->setValue(0);                // set cursor back in the beginning
  ui_->indexSlider->setRange(0,nr_of_frames_);  // rescale the slider
}

void 
PCDVideoPlayer::timeoutSlot ()
{
  /*
  if(cloud_src_present_ && cloud_src_modified_)
  {
    if(!vis_src_->updatePointCloud(cloud_src_, "cloud_src_"))
    {
      vis_src_->addPointCloud (cloud_src_, "cloud_src_");
      vis_src_->resetCameraViewpoint("cloud_src_");
    }
    cloud_src_modified_ = false;
  }
  if(cloud_dst_present_ && cloud_dst_modified_)
  {
    if(!vis_dst_->updatePointCloud(cloud_dst_, "cloud_dst_"))
    {
      vis_dst_->addPointCloud (cloud_dst_, "cloud_dst_");
      vis_dst_->resetCameraViewpoint("cloud_dst_");
    }
    cloud_dst_modified_ = false;
  }
  ui_->qvtk_widget_src->update();
  ui_->qvtk_widget_dst->update();
  */
}

void
PCDVideoPlayer::indexSliderValueChanged(int value)
{
  PCL_INFO("[PCDVideoPlayer::indexSliderValueChanged] : (I) : value %d\n", value);
}

void
PCDVideoPlayer::motionTypeBoxCurrentIndexChanged(int index)
{
  std::cout << "[PCDVideoPlayer::motionTypeBoxCurrentIndexChanged] : selected index " << index << std::endl;
}

void
print_usage ()
{
  PCL_INFO ("PCDVideoPlayer V0.1\n");
  PCL_INFO ("-------------------\n");
  PCL_INFO ("\tThe slider accepts focus on Tab and provides both a mouse wheel and a keyboard interface. The keyboard interface is the following:\n");
  PCL_INFO ("\t  Left/Right move a horizontal slider by one single step.\n");
  PCL_INFO ("\t  Up/Down move a vertical slider by one single step.\n");
  PCL_INFO ("\t  PageUp moves up one page.\n");
  PCL_INFO ("\t  PageDown moves down one page.\n");
  PCL_INFO ("\t  Home moves to the start (mininum).\n");
  PCL_INFO ("\t  End moves to the end (maximum).\n");
}

int
main (int argc, char** argv)
{
  QApplication app(argc, argv);

  PCDVideoPlayer VideoPlayer;

  VideoPlayer.show();

  return (app.exec());
}
