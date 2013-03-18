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

//PCL
#include <pcl/apps/MHMC_video_annotator/mhmc_video_annotator.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//STD
#include <iostream>
#include <fstream>

//BOOST
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>

//QT4
#include <QApplication>
#include <QMutexLocker>
#include <QEvent>
#include <QObject>
#include <QFileDialog>
#include <QGroupBox>
#include <QRadioButton>
#include <QButtonGroup>

// VTK
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>

using namespace pcl;
using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
MHMCVideoAnnotator::MHMCVideoAnnotator ()
{
  cloud_present_ = false;
  cloud_modified_ = false;
  play_mode_ = false;
  speed_counter_ = 0;
  speed_value_ = 5;

  //Create a timer
  vis_timer_ = new QTimer (this);
  vis_timer_->start (5);//5ms

  connect (vis_timer_, SIGNAL (timeout ()), this, SLOT (timeoutSlot()));

  ui_ = new Ui::MainWindow;
  ui_->setupUi (this);
  
  this->setWindowTitle ("MHMC Video Annotator");

  // Create the list of static poses
  static_motions_ << "Unclassified"     << "Neutral_pose"     << "T_pose"     << "U_pose"           << "Straight_pose"  << "Forward_pose";
  static_motions_ << "Right_angle_pose" << "Left_angle_pose"  << "Hip_pose"   << "Right_knee_pose"  << "Left_knee_pose" << "Right_knee_back";
  static_motions_ << "Left_knee_back"   << "Right_leg_front"  << "Right_front_foot" << "Left_leg_front" << "Left_front_foot";
  static_motions_ << "Right_leg_back"   << "Left_leg_back"    << "Right_stop" << "Left_stop" << "Both_stop";

  // Create the list of motions in free space
  free_motions_ << "Unclassified" << "Beckon_left" << "Beckon_right" << "Dance" << "Ballet" << "Forbid_left";
  free_motions_ << "Forbid_right" << "Frisbee_left" << "Frisbee_right" << "Front_kick_left" << "Front_kick_right";
  free_motions_ << "Jog_in_place" << "Jump_one_leg" << "Jump_two_leg" << "Jump" << "Jumping_jacks";
  free_motions_ << "Knee_sit" << "Knee_twist_right" << "Knee_twist_left" << "Leg_swing" << "Macarena";
  free_motions_ << "Ow" << "Point_left" << "Point_right" << "Rope_skip_walk" << "Rope_skip" << "Side_bend";
  free_motions_ << "Side_twists" << "Squat_jump" << "Squats" << "Thinker" << "Trow_kiss" << "Walk_turn";
  free_motions_ << "Walk" << "Wave_left" << "Wave_right" << "YMCA";

  // Create the list of motions in interaction with object
  object_motions_ << "Unclassified" << "Water" << "Cucumber" << "Pancake" << "Wipe";

  // Setup the cloud pointer
  cloud_.reset (new pcl::PointCloud<pcl::PointXYZRGBA>);

  // Set up the qvtk window
  vis_.reset (new pcl::visualization::PCLVisualizer ("", false));
  ui_->qvtkWidget->SetRenderWindow (vis_->getRenderWindow ());
  vis_->setupInteractor (ui_->qvtkWidget->GetInteractor (), ui_->qvtkWidget->GetRenderWindow ());
  vis_->getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
  ui_->qvtkWidget->update ();

  // Configure the motion Type box
  ui_->motionTypeBox->addItems(static_motions_);

  // Connect all buttons
  connect (ui_->playButton, SIGNAL(clicked()), this, SLOT(playButtonPressed()));
  connect (ui_->saveButton, SIGNAL(clicked()), this, SLOT(saveButtonPressed()));
  connect (ui_->stopButton, SIGNAL(clicked()), this, SLOT(stopButtonPressed()));
  connect (ui_->backButton, SIGNAL(clicked()), this, SLOT(backButtonPressed()));
  connect (ui_->nextButton, SIGNAL(clicked()), this, SLOT(nextButtonPressed()));
  connect (ui_->writeButton, SIGNAL(clicked()), this, SLOT(writeButtonPressed()));

  connect (ui_->selectFolderButton, SIGNAL(clicked()), this, SLOT(selectFolderButtonPressed()));
  connect (ui_->selectFilesButton, SIGNAL(clicked()), this, SLOT(selectFilesButtonPressed()));
  
  connect (ui_->staticRadioButton, SIGNAL(pressed()), this, SLOT(staticRadioButtonPressed()));
  connect (ui_->freeRadioButton, SIGNAL(pressed()), this, SLOT(freeRadioButtonPressed()));
  connect (ui_->objectRadioButton, SIGNAL(pressed()), this, SLOT(objectRadioButtonPressed()));

  connect (ui_->indexSlider, SIGNAL(valueChanged(int)), this, SLOT(indexSliderValueChanged(int)));

  // Not needed for now
  //connect (ui_->motionTypeBox, SIGNAL(currentIndexChanged (int)), this, SLOT(motionTypeBoxCurrentIndexChanged(int)));

}

void 
MHMCVideoAnnotator::staticRadioButtonPressed()
{
  // Configure the motion Type box
  ui_->motionTypeBox->clear();
  ui_->motionTypeBox->addItems(static_motions_);
}

void
MHMCVideoAnnotator::freeRadioButtonPressed()
{
  // Configure the motion Type box
  ui_->motionTypeBox->clear();
  ui_->motionTypeBox->addItems(free_motions_);
}

void
MHMCVideoAnnotator::objectRadioButtonPressed()
{
  // Configure the motion Type box
  ui_->motionTypeBox->clear();
  ui_->motionTypeBox->addItems(object_motions_);
}

void
MHMCVideoAnnotator::playButtonPressed()
{
  play_mode_ = true;
}

void 
MHMCVideoAnnotator::stopButtonPressed()
{
  play_mode_= false;
}

void 
MHMCVideoAnnotator::backButtonPressed()
{
  if(current_frame_ == 0) // Allready in the beginning
  {
    PCL_DEBUG ("[MHMCVideoAnnotator::nextButtonPressed] : reached the end\n");
    current_frame_ = nr_of_frames_-1; // reset to end
  }
  else
  {
    current_frame_--;
    cloud_modified_ = true;
    ui_->indexSlider->setSliderPosition(current_frame_);        // Update the slider position
  }
}

void 
MHMCVideoAnnotator::nextButtonPressed()
{
  if(current_frame_ == (nr_of_frames_-1)) // Reached the end
  {
    PCL_DEBUG ("[MHMCVideoAnnotator::nextButtonPressed] : reached the end\n");
    current_frame_ = 0; // reset to beginning
  }
  else
  {
    current_frame_++;
    cloud_modified_ = true;
    ui_->indexSlider->setSliderPosition(current_frame_);        // Update the slider position
  }
}

/*
 * \brief This methods saves the semantics of the current frame, nothing is stored to disk!
 */
void 
MHMCVideoAnnotator::saveButtonPressed()
{
  PCL_DEBUG ("[MHMCVideoAnnotator::saveButtonPressed] : (I) : called\n");

  if(cloud_present_)
  {
    frame_semantics_[current_frame_].motion_type_ = ui_->motionTypeBox->currentText().toStdString();
    frame_semantics_[current_frame_].motion_type_index_ = ui_->motionTypeBox->currentIndex();

    frame_semantics_[current_frame_].static_motion_ = ui_->staticRadioButton->isChecked();
    frame_semantics_[current_frame_].free_motion_ = ui_->freeRadioButton->isChecked();
    frame_semantics_[current_frame_].object_motion_ = ui_->objectRadioButton->isChecked();

    frame_semantics_[current_frame_].begin_motion_ = ui_->beginMotionRadioButton->isChecked();
    frame_semantics_[current_frame_].in_motion_ = ui_->inMotionRadioButton->isChecked();
    frame_semantics_[current_frame_].end_motion_ = ui_->endMotionRadioButton->isChecked();

    frame_semantics_[current_frame_].visible_body_parts_ =ui_->bodyPartsBox->value();

    time(&frame_semantics_[current_frame_].last_save_);

    frame_semantics_[current_frame_].annotated_ = true;
  }
  else
  {
    PCL_ERROR ("[MHMCVideoAnnotator::saveButtonPressed] : please load PCD files first!\n");
  }
}

/*
 * \brief This method saves the semantics off all the selected frames to an XML file
 */
void 
MHMCVideoAnnotator::writeButtonPressed()
{
  PCL_DEBUG ("[MHMCVideoAnnotator::writeButtonPressed] : (I) : called\n");
  QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), "day_person_trial_machine.xml", tr("XML Files (*.xml)"));

  std::ofstream outfile;

  outfile.open (fileName.toStdString().c_str());

  boost::property_tree::ptree pt;

  pt.add("version", CURRENT_VERSION);
  pt.add("nr_of_frames", nr_of_frames_);

  BOOST_FOREACH(FrameSemantics frameS, frame_semantics_)
  {
    boost::property_tree::ptree & node = pt.add("frame", "");
    node.put("filename", frameS.filename_);

    node.put("motion_type", frameS.motion_type_);
    node.put("motion_type_index", frameS.motion_type_index_);

    node.put("static_motion", frameS.static_motion_);
    node.put("free_motion", frameS.free_motion_);
    node.put("object_motion", frameS.object_motion_);

    node.put("begin_motion", frameS.static_motion_);
    node.put("in_motion", frameS.in_motion_);
    node.put("end_motion", frameS.end_motion_);

    node.put("visible_body_parts", frameS.visible_body_parts_);

    node.put("time_last_save", frameS.last_save_);

    node.put("annotated", frameS.annotated_);
  }
  write_xml(outfile,pt);
  outfile.close();
}

void
MHMCVideoAnnotator::selectFolderButtonPressed()
{
  pcd_files_.clear();     // Clear the std::vector
  pcd_paths_.clear();     // Clear the boost filesystem paths

  dir_ = QFileDialog::getExistingDirectory(this, tr("Open Directory"), "/home", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

  // GIVES ERROR
  //PCL_DEBUG ("[MHMCVideoAnnotator::selectFolderButtonPressed] : selected : %s\n", dir_.toAscii().data());

  boost::filesystem::directory_iterator end_itr;

  if(boost::filesystem::is_directory(dir_.toStdString()))
  {
    for (boost::filesystem::directory_iterator itr(dir_.toStdString()); itr != end_itr; ++itr)
    {
      std::string ext = itr->path().extension().string();
      if(ext.compare(".pcd") == 0)
      {
        pcd_files_.push_back (itr->path ().string ());
        pcd_paths_.push_back (itr->path ());
      }
      else
      {
        // Found non pcd file
        PCL_DEBUG ("[MHMCVideoAnnotator::selectFolderButtonPressed] : found a different file\n");
      }
      // PCL_DEBUG GIVES ERROR, std::cout not
      //PCL_DEBUG ("String : \t%s\n",itr->path().string());
      //PCL_DEBUG ("Extension : \ts\n",itr->path().extension());
      //PCL_DEBUG ("Filename : \ts\n",itr->path().filename());
        //std::cout << "Root_dir" << itr->path().root_directory() << std::endl;
        //std::cout << "Root_path" << itr->path().root_path() << std::endl;
        //std::cout << "Root_name" << itr->path().root_name() << std::endl;
        //std::cout << "rel_path" << itr->path().relative_path() << std::endl;
        //std::cout << "parent_path" << itr->path().parent_path() << std::endl;
    }
  }
  else
  {
    PCL_ERROR("Path is not a directory\n");
    exit(-1);
  }
  nr_of_frames_ = pcd_files_.size();
  PCL_DEBUG ("[MHMCVideoAnnotator::selectFolderButtonPressed] : found %d files\n", nr_of_frames_ );

  if(nr_of_frames_ == 0)
  {
    PCL_ERROR("Please select valid pcd folder\n");
    cloud_present_ = false;
    return;
  }
  else
  {
    // Reset the Slider
    ui_->indexSlider->setValue(0);                // set cursor back in the beginning
    ui_->indexSlider->setRange(0,nr_of_frames_-1);  // rescale the slider

    current_frame_ = 0;

    cloud_present_ = true;
    cloud_modified_ = true;

    frame_semantics_.resize (nr_of_frames_);
    // Copy the filenames to the frame semantics file names
    for(int i = 0; i< nr_of_frames_; i++)
      frame_semantics_[i].filename_ = pcd_files_[i];
  }
}

void 
MHMCVideoAnnotator::selectFilesButtonPressed()
{
  pcd_files_.clear();  // Clear the std::vector
  pcd_paths_.clear();     // Clear the boost filesystem paths

  QStringList qt_pcd_files = QFileDialog::getOpenFileNames(this, "Select one or more PCD files to open", "/home", "PointClouds (*.pcd)");
  nr_of_frames_ = qt_pcd_files.size();
  std::cout << "[MHMCVideoAnnotator::selectFilesButtonPressed] : selected " << nr_of_frames_ << " files" << std::endl;

  if(nr_of_frames_ == 0)
  {
    PCL_ERROR("Please select valid pcd files\n");
    cloud_present_ = false;
    return;
  }
  else
  {
    frame_semantics_.resize (nr_of_frames_);
    for(int i = 0; i < qt_pcd_files.size(); i++)
    {
      pcd_files_.push_back(qt_pcd_files.at(i).toStdString());
      frame_semantics_[i].filename_ = qt_pcd_files.at(i).toStdString();
    }

    current_frame_ = 0;

    // Reset the Slider
    ui_->indexSlider->setValue(0);                // set cursor back in the beginning
    ui_->indexSlider->setRange(0,nr_of_frames_-1);  // rescale the slider

    cloud_present_ = true;
    cloud_modified_ = true;
  }
}

void 
MHMCVideoAnnotator::timeoutSlot ()
{
  if(play_mode_)
  {
    if(speed_counter_ == speed_value_)
    {
      if(current_frame_ == (nr_of_frames_-1)) // Reached the end
      {
        current_frame_ = 0; // reset to beginning
      }
      else
      {
        current_frame_++;
        cloud_modified_ = true;
        ui_->indexSlider->setSliderPosition(current_frame_);        // Update the slider position
      }
    }
    else
    {
      speed_counter_++;
    }
  }

  if(cloud_present_ && cloud_modified_)
  {
    // Set buttons etc back to saved values if they exist
    if (frame_semantics_[current_frame_].annotated_) // Load from std::vector
    {
      ui_->motionTypeBox->setCurrentIndex(frame_semantics_[current_frame_].motion_type_index_);
      ui_->bodyPartsBox->setValue(frame_semantics_[current_frame_].visible_body_parts_);

      ui_->staticRadioButton->setChecked(frame_semantics_[current_frame_].static_motion_);
      ui_->freeRadioButton->setChecked(frame_semantics_[current_frame_].free_motion_);
      ui_->objectRadioButton->setChecked(frame_semantics_[current_frame_].object_motion_);

      ui_->beginMotionRadioButton->setChecked(frame_semantics_[current_frame_].begin_motion_);
      ui_->inMotionRadioButton->setChecked(frame_semantics_[current_frame_].in_motion_);
      ui_->endMotionRadioButton->setChecked(frame_semantics_[current_frame_].end_motion_);
    }
    else  // Reset
    {
      ui_->motionTypeBox->setCurrentIndex(0);
      ui_->bodyPartsBox->setValue(0);

      ui_->staticRadioButton->setChecked(false);
      ui_->freeRadioButton->setChecked(false);
      ui_->objectRadioButton->setChecked(false);

      ui_->beginMotionRadioButton->setChecked(false);
      ui_->inMotionRadioButton->setChecked(false);
      ui_->endMotionRadioButton->setChecked(false);
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (pcd_files_[current_frame_], *cloud_) == -1) //* load the file
    {
      PCL_ERROR ("[MHMCVideoAnnotator::timeoutSlot] : Couldn't read file %s\n");
    }

    if(!vis_->updatePointCloud(cloud_, "cloud_"))
    {
      vis_->addPointCloud (cloud_, "cloud_");
      vis_->resetCameraViewpoint("cloud_");
    }
    cloud_modified_ = false;
  }
  ui_->qvtkWidget->update();
}

void
MHMCVideoAnnotator::indexSliderValueChanged(int value)
{
  PCL_DEBUG ("[MHMCVideoAnnotator::indexSliderValueChanged] : (I) : value %d\n", value);
  current_frame_ = value;
  cloud_modified_ = true;
}

void
MHMCVideoAnnotator::motionTypeBoxCurrentIndexChanged(int index)
{
  std::cout << "[MHMCVideoAnnotator::motionTypeBoxCurrentIndexChanged] : selected index " << index << std::endl;
}

void
print_usage ()
{
  PCL_INFO ("MHMCVideoAnnotator V0.1\n");
  PCL_INFO ("-------------------\n");
  PCL_INFO ("\tThe slider accepts focus on Tab and provides both a mouse wheel and a keyboard interface. The keyboard interface is the following:\n");
  PCL_INFO ("\t  Left/Right move a horizontal slider by one single step.\n");
  PCL_INFO ("\t  Up/Down move a vertical slider by one single step.\n");
  PCL_INFO ("\t  PageUp moves up one page.\n");
  PCL_INFO ("\t  PageDown moves down one page.\n");
  PCL_INFO ("\t  Home moves to the start (mininum).\n");
  PCL_INFO ("\t  End moves to the end (maximum).\n");
  PCL_INFO ("\tThe Write button saves the semantics off all the selected frames to an XML file.\n");
}

int
main (int argc, char** argv)
{
  QApplication app(argc, argv);

  MHMCVideoAnnotator VideoPlayer;

  VideoPlayer.show();

  return (app.exec());
}
