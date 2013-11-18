/**
 *  Software License Agreement (BSD License)
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
 * @brief This file is the execution node of the Human Tracking 
 * @copyright Copyright (2011) Willow Garage
 * @authors Koen Buys, Anatoly Baksheev
 **/
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types_conversion.h>
#include <pcl/common/statistics/statistics.h>
#include <pcl/common/time.h>
#include <pcl/exceptions.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/gpu/people/people_detector.h>
#include <pcl/gpu/people/colormap.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/boost.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter_hist.h>
#include <pcl/tracking/impl/particle_filter_hist.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <utility>
#include <iostream>
#include <string>

namespace pc = pcl::console;
using namespace pcl::visualization;
using namespace pcl::gpu;
using namespace pcl;
using namespace std;

std::vector <double> bins(361);
int num_of_trackers = 1;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct SampledScopeTime : public StopWatch
{
  enum { EACH = 33 };
  SampledScopeTime(int& time_ms) : time_ms_(time_ms) {}
  ~SampledScopeTime()
  {
    static int i_ = 0;
    time_ms_ += getTime ();
    if (i_ % EACH == 0 && i_)
    {
      cout << "[~SampledScopeTime] : Average frame time = " << time_ms_ / EACH << "ms ( " << 1000.f * EACH / time_ms_ << "fps )" << endl;
      time_ms_ = 0;
    }
    ++i_;
  }
  private:
  int& time_ms_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

string
make_name(int counter, const char* suffix)
{
  char buf[4096];
  sprintf (buf, "./people%04d_%s.png", counter, suffix);
  return buf;
}

template<typename T> void
savePNGFile(const std::string& filename, const pcl::gpu::DeviceArray2D<T>& arr)
{
  int c;
  pcl::PointCloud<T> cloud(arr.cols(), arr.rows());
  arr.download(cloud.points, c);
  pcl::io::savePNGFile(filename, cloud);
}

template <typename T> void
savePNGFile (const std::string& filename, const pcl::PointCloud<T>& cloud)
{
  pcl::io::savePNGFile(filename, cloud);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class PeoplePCDApp
{
  public:
    typedef pcl::gpu::people::PeopleDetector PeopleDetector;
    typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    enum { COLS = 640, ROWS = 480 };

    PeoplePCDApp (pcl::Grabber& capture, bool write)
      : capture_ (capture),
        write_ (write),
        exit_ (false),
        time_ms_ (0),
        cloud_cb_ (true),
        counter_ (0),
        final_view_ ("Final labeling"),
        depth_view_ ("Depth"),
        histogram_view_ ("Colormodels tracked limbs"),
        cloud_view_ ("Pointcloud with PF"),
        hist_ref_ (num_of_trackers, std::vector<float> (361)),
        hist_ref_double_ (num_of_trackers, std::vector<double> (361)),
        histogramStatistics_ (0, 360, 361, false, true),
        tracker_list_ (num_of_trackers),
        setRef_ (false),
        color_ (num_of_trackers, std::vector<float> (3))
    {
      final_view_.setSize (COLS, ROWS);
      depth_view_.setSize (COLS, ROWS);

      final_view_.setPosition (0, 0);
      depth_view_.setPosition (650, 0);
      histogram_view_.setPosition (1300, 0);
      cloud_view_.setPosition (0, 490);

      cmap_device_.create(ROWS, COLS);
      cmap_host_.points.resize(COLS * ROWS);
      depth_device_.create(ROWS, COLS);
      image_device_.create(ROWS, COLS);

      depth_host_.points.resize(COLS * ROWS);

      rgba_host_.points.resize(COLS * ROWS);
      rgb_host_.resize(COLS * ROWS * 3);

      people::uploadColorMap(color_map_);

      color_ [0][0] = 255;
      color_ [0][1] = 0;
      color_ [0][2] = 0;
      if (num_of_trackers >= 2)
      {
      color_ [1][0] = 0;
      color_ [1][1] = 250;
      color_ [1][2] = 255;
      }
      if (num_of_trackers >= 3)
      {
        color_ [2][0] = 0;
        color_ [2][1] = 255;
        color_ [2][2] = 0;
      }
    }

    // Draw the current particles
    void
    drawParticles ()
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::PointXYZ rep_state;
      for (int i = 0; i < tracker_list_.size (); i++)
      {
        std::string particle_cloud_name = "particle_cloud_";
        std::string rep_state_name = "rep_state_";
        char number[16];
        sprintf (number, "%d", i);
        particle_cloud_name.append (number);
        rep_state_name.append (number);

        pcl::PointCloud<pcl::tracking::ParticleXYZRPY>::Ptr particles = tracker_list_[i].getParticles ();
        if (particles)
        {
          for (size_t j = 0; j < particles->points.size (); j++)
          {
            pcl::PointXYZ point;

            point.x = particles->points[j].x;
            point.y = -particles->points[j].y;
            point.z = -particles->points[j].z;
            particle_cloud->points.push_back (point);
          }

          // Set representative state
          pcl::tracking::ParticleXYZRPY tracker_result = tracker_list_[i].getRepresentativeState ();
          rep_state.x =   tracker_result.x;
          rep_state.y = - tracker_result.y;
          rep_state.z = - tracker_result.z;

          if (!cloud_view_.updatePointCloud (particle_cloud, particle_cloud_name.c_str()))
          {
            cloud_view_.addPointCloud (particle_cloud, particle_cloud_name.c_str());
            cloud_view_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color_[i][0], color_[i][1], color_[i][2], particle_cloud_name.c_str());
            cloud_view_.addSphere (rep_state, 0.05, color_[i][0], color_[i][1], color_[i][2], rep_state_name.c_str());
          }
          else
          {
            cloud_view_.updatePointCloud (particle_cloud, particle_cloud_name.c_str());
            cloud_view_.updateSphere (rep_state, 0.05, color_[i][0], color_[i][1], color_[i][2], rep_state_name.c_str());
          }
        }
        particle_cloud.points.clear ();
      }
    }

    void
    tracker_cb (const CloudConstPtr& cloud)
    {
      boost::mutex::scoped_lock lock (cloud_mutex_);

      // Compute all reference colormodels for each limb
      if (!setRef_)
      {
        for (int i = 0; i < tracker_list_.size (), i++)
        {
          //set initial state and colormodel
          Eigen::Vector3f c;
          Eigen::Affine3f trans;
          c[0] = 0; //TODO mean values of selected blob
          c[1] = 0;
          c[2] = 1.25;
          trans.translation ().matrix () = c;

          tracker_list_[i].setTrans (trans);
          tracker_list_[i].setReferenceHistogram (reference_histogram);
          std::cout << "Reference colormodel " << i << " has been set." << std::endl;
        }
        setRef_ = true;
      }
      // Start tracking
      else
      {
        for (int i = 0; i < tracker_list_.size (); i++)
        {
          tracker_list_[i].setInputCloud (cloud);
          tracker_list_[i].compute ();
        }
        drawParticles (); // TODO visualizeAndWrite () would be a better place to do this
      }

      //get actual reference colormodels of all trackers to plot as a histogram
      if (setRef_)
      {
        for (int i = 0; i < tracker_list_.size (); i++)
        {
          hist_ref_[i] = tracker_list_[i].getReferenceHistogram ();
          std::copy(hist_ref_[i].begin(), hist_ref_[i].end(), hist_ref_double_[i].begin());
        }
      }
    }

    void
    visualizeAndWrite()
    {
      const PeopleDetector::Labels& labels = people_detector_.rdf_detector_->getLabels();
      people::colorizeLabels(color_map_, labels, cmap_device_);
      //people::colorizeMixedLabels(

      int c;
      cmap_host_.width = cmap_device_.cols();
      cmap_host_.height = cmap_device_.rows();
      cmap_host_.points.resize(cmap_host_.width * cmap_host_.height);
      cmap_device_.download(cmap_host_.points, c);

      final_view_.showRGBImage<pcl::RGB>(cmap_host_);
      final_view_.spinOnce(1, true);

      if (cloud_cb_)
      {
        depth_host_.width = people_detector_.depth_device1_.cols();
        depth_host_.height = people_detector_.depth_device1_.rows();
        depth_host_.points.resize(depth_host_.width * depth_host_.height);
        people_detector_.depth_device1_.download(depth_host_.points, c);
      }

      depth_view_.showShortImage(&depth_host_.points[0], depth_host_.width, depth_host_.height, 0, 5000, true);
      depth_view_.spinOnce(1, true);

      if (write_)
      {
        PCL_VERBOSE("PeoplePCDApp::visualizeAndWrite : (I) : Writing to disk");
        if (cloud_cb_)
          savePNGFile(make_name(counter_, "ii"), cloud_host_);
        else
          savePNGFile(make_name(counter_, "ii"), rgba_host_);
        savePNGFile(make_name(counter_, "c2"), cmap_host_);
        savePNGFile(make_name(counter_, "s2"), labels);
        savePNGFile(make_name(counter_, "d1"), people_detector_.depth_device1_);
        savePNGFile(make_name(counter_, "d2"), people_detector_.depth_device2_);
      }
    }

    void
    source_cb1 (const boost::shared_ptr<const PointCloud<PointXYZRGBA> >& cloud)
    {
      {
        boost::mutex::scoped_lock lock (data_ready_mutex_);
        if (exit_)
          return;

        pcl::copyPointCloud (*cloud, cloud_host_);
      }
      data_ready_cond_.notify_one ();
    }

    void
    source_cb2 (const boost::shared_ptr<openni_wrapper::Image>& image_wrapper, const boost::shared_ptr<openni_wrapper::DepthImage>& depth_wrapper, float)
    {
      {
        boost::mutex::scoped_try_lock lock (data_ready_mutex_);

        if (exit_ || !lock)
          return;

        //getting depth
        int w = depth_wrapper->getWidth ();
        int h = depth_wrapper->getHeight ();
        int s = w * PeopleDetector::Depth::elem_size;
        const unsigned short *data = depth_wrapper->getDepthMetaData ().Data ();
        depth_device_.upload (data, s, h, w);

        depth_host_.points.resize(w *h);
        depth_host_.width = w;
        depth_host_.height = h;
        std::copy (data, data + w * h, &depth_host_.points[0]);

        //getting image
        w = image_wrapper->getWidth ();
        h = image_wrapper->getHeight ();
        s = w * PeopleDetector::Image::elem_size;

        //fill rgb array
        rgb_host_.resize (w * h * 3);
        image_wrapper->fillRGB (w, h, (unsigned char*)&rgb_host_[0]);

        // convert to rgba, TODO image_wrapper should be updated to support rgba directly
        rgba_host_.points.resize (w * h);
        rgba_host_.width = w;
        rgba_host_.height = h;
        for(int i = 0; i < rgba_host_.size (); ++i)
        {
          const unsigned char *pixel = &rgb_host_[i * 3];
          RGB& rgba = rgba_host_.points[i];
          rgba.r = pixel[0];
          rgba.g = pixel[1];
          rgba.b = pixel[2];
        }
        image_device_.upload (&rgba_host_.points[0], s, h, w);
      }
      data_ready_cond_.notify_one ();
    }

    void
    startMainLoop ()
    {
      // Initialize all PF trackers
      std::vector<double> default_step_covariance = std::vector<double> (6, 0.08 * 0.08);
      int n = 0.15;
      default_step_covariance[3] = n * n;
      default_step_covariance[4] = n * n;
      default_step_covariance[5] = n * n;

      std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
      std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

      for (int i = 0; i < tracker_list_.size (); i++)
      {
        tracker_list_[i].setTrans (Eigen::Affine3f::Identity ());
        tracker_list_[i].setStepNoiseCovariance (default_step_covariance);
        tracker_list_[i].setInitialNoiseCovariance (initial_noise_covariance);
        tracker_list_[i].setInitialNoiseMean (default_initial_mean);
        tracker_list_[i].setIterationNum (1);
        tracker_list_[i].setParticleNum (500);
        tracker_list_[i].setResampleLikelihoodThr(0.00);
        tracker_list_[i].setUseNormal (false);
        tracker_list_[i].setMotionRatio (0.5);
      }

      cloud_cb_ = false;

      PCDGrabberBase* ispcd = dynamic_cast<pcl::PCDGrabberBase*>(&capture_);
      if (ispcd)
        cloud_cb_= true;

      typedef boost::shared_ptr<openni_wrapper::DepthImage> DepthImagePtr;
      typedef boost::shared_ptr<openni_wrapper::Image> ImagePtr;

      boost::function<void (const boost::shared_ptr<const PointCloud<PointXYZRGBA> >&)> func1 = boost::bind (&PeoplePCDApp::source_cb1, this, _1);
      boost::function<void (const ImagePtr&, const DepthImagePtr&, float constant)> func2 = boost::bind (&PeoplePCDApp::source_cb2, this, _1, _2, _3);
      boost::signals2::connection c = cloud_cb_ ? capture_.registerCallback (func1) : capture_.registerCallback (func2);

      {
        boost::unique_lock<boost::mutex> lock(data_ready_mutex_);

        try
        {
          capture_.start ();
          while (!exit_ && !final_view_.wasStopped())
          {
            bool has_data = data_ready_cond_.timed_wait(lock, boost::posix_time::millisec(100));
            if(has_data)
            {
              SampledScopeTime fps(time_ms_);

              if (cloud_cb_)
                process_return_ = people_detector_.process(cloud_host_.makeShared());
              else
                process_return_ = people_detector_.process(depth_device_, image_device_);

              ++counter_;
            }

            if(has_data && (process_return_ == 2))
              visualizeAndWrite();
          }
          final_view_.spinOnce (3);
        }
        catch (const std::bad_alloc& /*e*/) { cout << "Bad alloc" << endl; }
        catch (const std::exception& /*e*/) { cout << "Exception" << endl; }

        capture_.stop ();
      }
      c.disconnect();
    }

    boost::mutex data_ready_mutex_;
    boost::condition_variable data_ready_cond_;

    pcl::Grabber& capture_;

    bool cloud_cb_;
    bool write_;
    bool exit_;
    int time_ms_;
    int counter_;
    int process_return_;
    PeopleDetector people_detector_;
    PeopleDetector::Image cmap_device_;
    pcl::PointCloud<pcl::RGB> cmap_host_;

    PeopleDetector::Depth depth_device_;
    PeopleDetector::Image image_device_;

    pcl::PointCloud<unsigned short> depth_host_;
    pcl::PointCloud<pcl::RGB> rgba_host_;
    std::vector<unsigned char> rgb_host_;

    PointCloud<PointXYZRGBA> cloud_host_;

    ImageViewer final_view_;
    ImageViewer depth_view_;
    PCLPlotter  histogram_view_;
    PCLVisualizer cloud_view_;

    DeviceArray<pcl::RGB> color_map_;

    std::vector<std::vector<float> > hist_ref_;
    std::vector<std::vector<double> > hist_ref_double_;
    std::vector<std::vector<float> > color_;
    bool setRef_;
    pcl::HistogramStatistics<pcl::PointXYZHSV> histogramStatistics_;
    std::vector<pcl::tracking::ParticleFilterTrackerHist<pcl::PointXYZRGBA, pcl::tracking::ParticleXYZRPY> > tracker_list_;
};

void print_help()
{
  cout << "\nPeople tracking app options (help):" << endl;
  cout << "\t -numTrees    \t<int> \tnumber of trees to load" << endl;
  cout << "\t -tree0       \t<path_to_tree_file>" << endl;
  cout << "\t -tree1       \t<path_to_tree_file>" << endl;
  cout << "\t -tree2       \t<path_to_tree_file>" << endl;
  cout << "\t -tree3       \t<path_to_tree_file>" << endl;
  cout << "\t -gpu         \t<GPU_device_id>" << endl;
  cout << "\t -w           \t<bool> \tWrite results to disk" << endl;
  cout << "\t -h           \tPrint this help" << endl;
  cout << "\t -dev         \t<Kinect_device_id>" << endl;
  cout << "\t -pcd         \t<path_to_pcd_file>" << endl;
  cout << "\t -oni         \t<path_to_oni_file>" << endl;
  cout << "\t -pcd_folder  \t<path_to_folder_with_pcd_files>" << endl;
}

int main(int argc, char** argv)
{
  // Answering for help
  PCL_INFO("People tracking App version 0.2\n");
  if(pc::find_switch (argc, argv, "--help") || pc::find_switch (argc, argv, "-h"))
    return print_help(), 0;

  // Selecting GPU and prining info
  int device = 0;
  pc::parse_argument (argc, argv, "-gpu", device);
  pcl::gpu::setDevice (device);
  pcl::gpu::printShortCudaDeviceInfo (device);

  bool write = 0;
  pc::parse_argument (argc, argv, "-w", write);

  // Selecting data source
  boost::shared_ptr<pcl::Grabber> capture;
  string openni_device, oni_file, pcd_file, pcd_folder;

  try
  {
    if (pc::parse_argument (argc, argv, "-dev", openni_device) > 0)
    {
      capture.reset( new pcl::OpenNIGrabber(openni_device) );
    }
    else
    if (pc::parse_argument (argc, argv, "-oni", oni_file) > 0)
    {
      capture.reset( new pcl::ONIGrabber(oni_file, true, true) );
    }
    else
    {
      capture.reset( new pcl::OpenNIGrabber() );
    }
  }
  catch (const pcl::PCLException& /*e*/) { return cout << "Can't open depth source" << endl, -1; }

  // Selecting tree files
  vector<string> tree_files;
  tree_files.push_back("Data/forest1/tree_20.txt");
  tree_files.push_back("Data/forest2/tree_20.txt");
  tree_files.push_back("Data/forest3/tree_20.txt");
  tree_files.push_back("Data/forest4/tree_20.txt");

  pc::parse_argument (argc, argv, "-tree0", tree_files[0]);
  pc::parse_argument (argc, argv, "-tree1", tree_files[1]);
  pc::parse_argument (argc, argv, "-tree2", tree_files[2]);
  pc::parse_argument (argc, argv, "-tree3", tree_files[3]);

  int num_trees = (int)tree_files.size();
  pc::parse_argument (argc, argv, "-numTrees", num_trees);

  tree_files.resize(num_trees);
  if (num_trees == 0 || num_trees > 4)
  {
    PCL_ERROR("[Main] : Invalid number of trees");
    print_help();
    return -1;
  }

  // Generate the bins TODO do this somewhere else...
  for (int i = 0; i < 361; ++i)
  {
    bins[i] = i;
  }

  try
  {
    // Loading trees
    typedef pcl::gpu::people::RDFBodyPartsDetector RDFBodyPartsDetector;
    RDFBodyPartsDetector::Ptr rdf(new RDFBodyPartsDetector(tree_files));
    PCL_VERBOSE("[Main] : Loaded files into rdf");

    // Create the app
    PeoplePCDApp app(*capture, write);
    app.people_detector_.rdf_detector_ = rdf;

    // Executing
    app.startMainLoop ();
  }
  catch (const pcl::PCLException& e) { cout << "PCLException: " << e.detailedMessage() << endl; print_help();}
  catch (const std::runtime_error& e) { cout << e.what() << endl; print_help(); }
  catch (const std::bad_alloc& /*e*/) { cout << "Bad alloc" << endl; print_help(); }
  catch (const std::exception& /*e*/) { cout << "Exception" << endl; print_help(); }

  return 0;
}
