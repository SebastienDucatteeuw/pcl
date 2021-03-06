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
#include <pcl/common/time.h>
#include <pcl/common/statistics/statistics.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/exceptions.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/gpu/people/people_detector.h>
#include <pcl/gpu/people/bodyparts_detector.h>
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
int num_of_trackers = 3;

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
    typedef Cloud::ConstPtr CloudConstPtr;

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
        prob_view_ ("Probability distribution PF labels"),
        hist_ref_ (num_of_trackers, std::vector<float> (361)),
        hist_ref_double_ (num_of_trackers, std::vector<double> (361)),
        tracker_parts_ (num_of_trackers, std::vector<int> (32)),
        tracker_list_ (num_of_trackers),
        setRef_ (num_of_trackers, false),
        color_ (num_of_trackers, std::vector<float> (3)),
        histogramStatistics_ (0, 360, 361, false, true),
        histogramCoherence_ ()
    {
      final_view_.setSize (COLS, ROWS);
      depth_view_.setSize (COLS, ROWS);
      cloud_view_.setSize (COLS, ROWS);
      prob_view_.setSize  (COLS, ROWS);

      final_view_.setPosition (1280, 0);
      depth_view_.setPosition (0, 0);
      cloud_view_.setPosition (640, 0);
      prob_view_.setPosition (1208, 520);

      histogram_view_.setShowLegend (true);
      histogram_view_.setXRange (0, 361);
      histogram_view_.setYRange (0, 0.1);

      cmap_device_.create(ROWS, COLS);
      cmap_host_.points.resize(COLS * ROWS);
      depth_device_.create(ROWS, COLS);
      image_device_.create(ROWS, COLS);
      prob_device_.create(ROWS, COLS);

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
        color_ [1][1] = 0;
        color_ [1][2] = 255;
      }
      if (num_of_trackers >= 3)
      {
        color_ [2][0] = 0;
        color_ [2][1] = 255;
        color_ [2][2] = 0;
      }

      // ---- tracker-parts map ----
      // tracker 0 tracks part 13 (Rforearm)
      tracker_parts_ [0][13] = 1;
      // tracker 1 tracks part 17 (Lforearm)
      if (num_of_trackers >= 2)
      {
        tracker_parts_ [1][17] = 1;
      }
      // tracker 2 tracks part 23 (Rchest) and part 24 (Lchest)
      if (num_of_trackers >= 3)
      {
        tracker_parts_ [2][23] = 1;
        tracker_parts_ [2][24] = 1;
      }
/*
      //fill prob_host_ with zeros
      for (int i = 0; i < prob_host_.points.size (); i++)
        prob_host_.points[i].rgb = 0;
*/
    }

    bool
    setRefDone ()
    {
      int s = 0;
      for (int i = 0; i < setRef_.size (); i++)
        if (setRef_[i] == true)
          s++;
      return (s == tracker_list_.size ());
    }

    // returns tracker tracking this label
    int
    labelTrackedBy (int label)
    {
      int i = 0;
      int tracker = -1; // this label is not tracked
      while (i < num_of_trackers)
      {
        if (tracker_parts_[i][label] == 1)
          tracker = i;
        i++;
      }
      return tracker;
    }

    void
    convertProbToRGB (pcl::PointCloud<pcl::device::prob_histogram>& histograms, int label, pcl::PointCloud<pcl::RGB>& rgb)
    {
      rgb.points.clear ();
      for(size_t t; t < histograms.points.size(); t++)
      {
        float value = histograms.points[t].probs[label];
        float  value8 = value * 255;
        char val = static_cast<char> (value8);
        pcl::RGB p;
        p.r = val; p.b = val; p.g = val;
        rgb.points.push_back(p);
      }
      rgb.width = histograms.width;
      rgb.height = histograms.height;
    }

    // Plot probabilities of each tracker following possibly multiple parts (due to similar colormodel)
    void
    convertTrackersProbToRGB (pcl::PointCloud<pcl::device::prob_histogram>& histograms, pcl::PointCloud<pcl::RGB>& rgb)
    {
      rgb.points.clear ();
      for(size_t t; t < histograms.points.size(); t++)
      {
        pcl::RGB p;
        char zero = static_cast<char> (0);
        p.r = zero; p.g = zero; p.b = zero;
        for (int label = 0; label < tracker_parts_[0].size (); label++)
        {
          float value = histograms.points[t].probs[label];
          float  value8 = value * 255;
          char val = static_cast<char> (value8);
          int tracker = labelTrackedBy (label);
          if (tracker == 0) // label tracked by tracker 0
            p.r = val;
          if (tracker == 1) // label tracked by tracker 1
            p.b = val;
          if (tracker == 2) // label tracked by tracker 2
            p.g = val;
        }
        rgb.points.push_back(p);
      }
      rgb.width = histograms.width;
      rgb.height = histograms.height;
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
        particle_cloud->points.clear ();
      }
    }

    void
    visualize()
    {
      //---- Draw final view ----
      const PeopleDetector::Labels& labels = people_detector_.rdf_detector_->getLabels();
      people::colorizeLabels(color_map_, labels, cmap_device_);
      int c;
      cmap_host_.width = cmap_device_.cols();
      cmap_host_.height = cmap_device_.rows();
      cmap_host_.points.resize(cmap_host_.width * cmap_host_.height);
      cmap_device_.download(cmap_host_.points, c);
      final_view_.showRGBImage<pcl::RGB>(cmap_host_);
      final_view_.spinOnce(1, true);

      //---- Draw depth view ----
/*
      depth_host_.width = people_detector_.depth_device1_.cols();
      depth_host_.height = people_detector_.depth_device1_.rows();
      depth_host_.points.resize(depth_host_.width * depth_host_.height);
      people_detector_.depth_device1_.download(depth_host_.points, c);
      depth_view_.showShortImage(&depth_host_.points[0], depth_host_.width, depth_host_.height, 0, 5000, true);
      depth_view_.spinOnce(1, true);
*/
      //---- Draw cloud view ----
      // 1) draw input cloud
      if (!cloud_view_.updatePointCloud (cloud_host_, "Input PointCloud"))
      {
        cloud_view_.resetCameraViewpoint ("Input PointCloud");
        cloud_view_.addPointCloud (cloud_host_, "Input PointCloud");
      }
      else
      {
        cloud_view_.updatePointCloud (cloud_host_, "Input PointCloud");
      }
      // 2) draw particles
      drawParticles ();
      cloud_view_.spinOnce(1, true);

      //---- Draw histogram view ----
      histogram_view_.clearPlots ();
      for (int i = 0; i < tracker_list_.size (); i++)
      {
        if (setRef_[i] == true)
        {
          histogram_view_.addPlotData (bins, hist_ref_double_[i]);
        }
      }
      histogram_view_.spinOnce ();

      //---- Draw probability distribution

      pcl::PointCloud<pcl::device::prob_histogram> prob_host2(people_detector_.rdf_detector_->P_l_2_.cols(), people_detector_.rdf_detector_->P_l_2_.rows());
      people_detector_.rdf_detector_->P_l_Gaus_Temp_.download(prob_host2.points, c);
      prob_host2.width = people_detector_.rdf_detector_->P_l_2_.cols();
      prob_host2.height = people_detector_.rdf_detector_->P_l_2_.rows();
      convertProbToRGB(prob_host2, 13, prob_host_); //get prob 2nd iter of Lforearm (17) background (29)
      prob_view_.showRGBImage<pcl::RGB> (prob_host_);
      prob_view_.spinOnce(1, true);
/*
      pcl::PointCloud<pcl::device::prob_histogram> prob_host2(COLS, ROWS);
      people_detector_.rdf_detector_->P_l_ext_.download(prob_host2.points, c);
      prob_host2.width = COLS;
      prob_host2.height = ROWS;
      convertTrackersProbToRGB (prob_host2, prob_host_);
      prob_view_.showRGBImage<pcl::RGB> (prob_host_);
      prob_view_.spinOnce(1, true);
      //savePNGFile ("prob_distr.png", prob_host_);
*/
/* Check normalization of pixel part proposals
int point = 0;
for (int i =0; i < prob_host2.points.size (); i++)
{
  if (prob_host2.points[i].probs[23] != 0)
    point = i;
}
std::cout << "point: " << point << std::endl;

float tot_prob = 0;
for (int i = 0; i < 32; i++)
{
  tot_prob += prob_host2.points[point].probs[i]; // cum. prob. of point
  std::cout << "prob " << i << ": " << prob_host2.points[point].probs[i] << std::endl;
}
std::cout << "tot prob" << tot_prob << std::endl;
*/
    }

    void
    processProbPF ()
    {
      int u, v;
      pcl::PointCloud<pcl::device::prob_histogram> prob_PF (COLS, ROWS);
      static const float cx = 320-.5;
      static const float cy = 240-.5;
      static const float f = 525;

      if (setRefDone ())
      {
        // build a probability distribution for each part
/*
        // plot particles only
        for (int i = 0; i < tracker_list_.size (); i++)
        {
          pcl::PointCloud<pcl::tracking::ParticleXYZRPY>::Ptr particles = tracker_list_[i].getParticles ();
          if (particles)
          {
            for (int j = 0; j < particles->points.size (); j++)
            {
              u = static_cast<int> (f*(particles->points[j].x/particles->points[j].z) + cx);
              v = static_cast<int> (f*(particles->points[j].y/particles->points[j].z) + cy);
              int index = u + v*640;
              if (index >= 0 && index < 307200)
              {
                prob_PF.points[index].probs[tracker_parts_[i]] = 1; //particles->points[j].weight;
              }
            }
          }
        }
*/
        // Apply KDE (Kernel Density Estimation)
        #ifdef _OPENMP
        #pragma omp parallel for num_threads(0)
        #endif
        for (int i = 0; i < tracker_list_.size (); i++)
        {
          int u_particle, v_particle, index;
          float a, b, prob;
          // Set the bandwidth h
          int hu = 4;
          int hv = 4;
          int margin = 3;
          float coef = 0.1592; // 1/(2*pi)
          int u_start, u_end, v_start, v_end;
          pcl::PointCloud<pcl::tracking::ParticleXYZRPY>::Ptr particles = tracker_list_[i].getParticles ();
          if (particles)
          {
            for (int j = 0; j < particles->points.size (); j++)
            {
              u_start = 0;
              u_end = people_detector_.rdf_detector_->P_l_ext_.cols();
              v_start = 0;
              v_end = people_detector_.rdf_detector_->P_l_ext_.rows();

              u_particle = static_cast<int> (f*(particles->points[j].x/particles->points[j].z) + cx);
              v_particle = static_cast<int> (f*(particles->points[j].y/particles->points[j].z) + cy);
              // Minimize for-loop cycles
              if (u_particle > margin*hu)            // check left side
                u_start = u_particle - (margin*hu);  // we can take some advantage at the left side
              if (u_particle < (u_end - margin*hu))  // check right side
                u_end = u_particle + (margin*hu);    // we can take some advantage at the right side
              if (v_particle > margin*hv)            // check top side
                v_start = v_particle - (margin*hv);  // we can take some advantage at the top side
              if (v_particle < (v_end - margin*hv))  // check bottom side
                v_end = v_particle + (margin*hv);    // we can take some advantage at the bottom side

              for (int u = u_start; u < u_end; u++)    // iter horizontal
              {
                for (int v = v_start; v < v_end; v++)  // iter vertical
                {
                  a = (u-u_particle)/hu;
                  b = (v-v_particle)/hv;
                  index = u + v*people_detector_.rdf_detector_->P_l_ext_.cols();
                  if (index >= 0 && index < prob_PF.points.size ())
                  {
                    prob = coef * std::exp(- (std::pow(a, 2) + std::pow(b, 2)) / 2);
                    for (int label = 0; label < tracker_parts_[i].size (); label++) //loop over labels to be tracked by tracker i
                    {
                      if (tracker_parts_[i][label] == 1) // assign the probs only to the labels tracked by tracker i
                      {
                        #pragma omp atomic
                        prob_PF.points[index].probs[label] += prob;
                      }
                    }
                  }
                }
              }
            }
          }
        }
        //normalize pixel part proposals
        for (int i = 0; i < prob_PF.points.size (); i++)  // loop over all pixels
        {
          //calulate sum of proposals
          float sum = 0;
          for (int j = 0; j < 32; j++)
            sum += prob_PF.points[i].probs[j];
          //normalize proposals to 0.75
          for (int j = 0; j < 32; j++)
            if (sum != 0)
              prob_PF.points[i].probs[j] *= 0.75/sum;
        }
        people_detector_.rdf_detector_->P_l_ext_.upload(prob_PF.points, COLS);
      }
    }

    void
    track ()
    {
      const people::RDFBodyPartsDetector::BlobMatrix& sorted = people_detector_.rdf_detector_->getBlobMatrix ();

      // Initialize colormodel and state for each part
      if (!setRefDone ())
      {
        pcl::PointCloud<pcl::PointXYZRGBA> segmented_cloud;
        pcl::PointCloud<pcl::PointXYZHSV> segmented_cloud_HSV;
        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
        std::vector<float> reference_histogram (361);
        std::vector<float> reference_histogram_tmp (361);

        for (int i = 0; i < tracker_list_.size (); i++)
        {
          for (int label = 0; label < tracker_parts_[i].size (); label++)
          {
            if (tracker_parts_[i][label] == 1)
            {
              if (people_detector_.t2_.parts_lid[label] != -3);
              {
                segmented_cloud.points.clear ();
                segmented_cloud_HSV.points.clear ();
                reference_histogram_tmp.clear ();

                pcl::PointIndices::Ptr indicesPtr (new pcl::PointIndices);
                indicesPtr->indices = sorted[label][people_detector_.t2_.parts_lid[label]].indices.indices;

                //calculate initial state
                Eigen::Vector4f mean = sorted[label][people_detector_.t2_.parts_lid[label]].mean; //Rforearm
                Eigen::Vector3f c;
                Eigen::Affine3f trans;
                c[0] = mean(0);
                c[1] = mean(1);
                c[2] = mean(2);
                trans.translation ().matrix () = c;

                //Replace wrong indices with zeros
                int n = 0;
                for (int j = 0; j < indicesPtr->indices.size (); j++)
                {
                  if ((indicesPtr->indices[j] > 307199) || (indicesPtr->indices[j] < 0))
                  {
                    indicesPtr->indices[j] = 0;
                  }
                }

                /* Save indices to file
                std::string filename = "/tmp/track_log.txt";
                std::ofstream writefile;
                writefile.open (filename.c_str()); //, ios::out | ios::app); //add to append the file
                writefile << "Frame number: " << counter_ << std::endl;
                for (int j = 0; j < indicesPtr->indices.size (); j++)
                {
                  writefile << indicesPtr->indices[j] << std::endl;
                }
                writefile.close ();
                */

                extract.setInputCloud (cloud_host_);
                extract.setIndices (indicesPtr);
                extract.setNegative (false);
                extract.filter (segmented_cloud);

                /* Only show Rforearm in cloud_view
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segPtr (new pcl::PointCloud<pcl::PointXYZRGBA> (segmented_cloud));
                if (!cloud_view_.updatePointCloud (segPtr, "Segmented PointCloud"))
                {
                  cloud_view_.resetCameraViewpoint ("Segmented PointCloud");
                  cloud_view_.addPointCloud (segPtr, "Segmented PointCloud");
                }
                cloud_view_.spinOnce (1, true);
                */

                PointCloudXYZRGBAtoXYZHSV (segmented_cloud, segmented_cloud_HSV);
                histogramStatistics_.computeHue (segmented_cloud_HSV, reference_histogram_tmp);

                if (counter_ < 10) //skip first 10 frames to avoid unreliable measurements
                {
                  tracker_list_[i].setReferenceHistogram (reference_histogram_tmp);
                }

                std::vector<float> reference_histogram_old = tracker_list_[i].getReferenceHistogram ();
                if (counter_ >= 10 && (setRef_[i] == false)) //build the colormodel on reliable measurements
                {
                  float alpha = 0.10;
                  for (int j = 0; j < reference_histogram.size (); j++)
                  {
                    reference_histogram[j] = static_cast<float> ( ((1-alpha) * reference_histogram_old[j]) + (alpha * reference_histogram_tmp[j]) );
                  }
                  tracker_list_[i].setReferenceHistogram (reference_histogram);
                }

                if (counter_ >= 10 && (histogramCoherence_.BhattacharyyaDistance(reference_histogram, reference_histogram_old) > 0.85 ))
                {
                  //set initial state
                  tracker_list_[i].setTrans (trans);
                  std::cout << "Reference colormodel " << i << " has been set." << std::endl;
                  setRef_[i] = true;
                }
              }
              //only learn color model from first part in tracker_parts_ list TODO Combine the colormodels and mean state of all parts tracked by one tracker
              label = tracker_parts_[i].size ();
            }
          }
        }
      }

      // Start tracking
      else
      {
        for (int i = 0; i < tracker_list_.size (); i++)
        {
          tracker_list_[i].setInputCloud (cloud_host_);
          tracker_list_[i].compute ();
        }
      }

      //get actual reference colormodels of all trackers to plot as a histogram
      for (int i = 0; i < tracker_list_.size (); i++)
      {
        if (setRef_[i] == true)
        {
          hist_ref_[i] = tracker_list_[i].getReferenceHistogram ();
          std::copy(hist_ref_[i].begin(), hist_ref_[i].end(), hist_ref_double_[i].begin());
        }
      }
    }

    void
    source_cb (const boost::shared_ptr<const PointCloud<PointXYZRGBA> >& cloud)
    {
      {
        boost::mutex::scoped_lock lock (data_ready_mutex_);
        if (exit_)
          return;

        cloud_host_ = cloud;
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

      boost::function<void (const boost::shared_ptr<const PointCloud<PointXYZRGBA> >&)> func_source_cb = boost::bind (&PeoplePCDApp::source_cb, this, _1);
      boost::signals2::connection source_connection = capture_.registerCallback (func_source_cb);

      {
        boost::unique_lock<boost::mutex> lock (data_ready_mutex_);
        try
        {
          capture_.start ();
          while (!exit_)
          {
            bool has_data = data_ready_cond_.timed_wait(lock, boost::posix_time::millisec(100));
            if(has_data)
            {
              SampledScopeTime fps(time_ms_);
              if (setRefDone ())
                process_return_ = people_detector_.processProb (cloud_host_);
              else
                process_return_ = people_detector_.process (cloud_host_);
              track ();
              processProbPF ();
              ++counter_;
            }
            if(has_data && (process_return_ == 2))
              visualize();

            if (final_view_.wasStopped() || depth_view_.wasStopped() || cloud_view_.wasStopped())
              exit_ = true;
          }
          final_view_.spinOnce (3);
        }
        catch (const std::bad_alloc& /*e*/) { cout << "Bad alloc" << endl; }
        catch (const std::exception& /*e*/) { cout << "Exception" << endl; }

        capture_.stop ();
      }
      source_connection.disconnect();
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
    PeopleDetector::Image prob_device_;
    pcl::PointCloud<pcl::RGB> prob_host_;

    PeopleDetector::Depth depth_device_;
    PeopleDetector::Image image_device_;

    pcl::PointCloud<unsigned short> depth_host_;
    pcl::PointCloud<pcl::RGB> rgba_host_;
    std::vector<unsigned char> rgb_host_;

    PointCloud<PointXYZRGBA>::ConstPtr cloud_host_;

    ImageViewer final_view_;
    ImageViewer depth_view_;
    ImageViewer prob_view_;

    PCLPlotter  histogram_view_;
    PCLVisualizer cloud_view_;

    DeviceArray<pcl::RGB> color_map_;

    std::vector<std::vector<float> > hist_ref_;
    std::vector<std::vector<double> > hist_ref_double_;
    std::vector<std::vector<float> > color_;
    std::vector<bool> setRef_;
    std::vector<std::vector<int> > tracker_parts_; // list of parts tracked by each tracker
    std::vector<pcl::tracking::ParticleFilterTrackerHist<pcl::PointXYZRGBA, pcl::tracking::ParticleXYZRPY> > tracker_list_;
    pcl::HistogramStatistics<pcl::PointXYZHSV> histogramStatistics_;
    pcl::tracking::HistogramCoherence<pcl::PointXYZRGBA, pcl::tracking::ParticleXYZRPY> histogramCoherence_;
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
