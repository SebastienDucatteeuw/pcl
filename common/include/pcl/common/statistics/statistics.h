/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 * $Id$
 *
 */

#ifndef PCL_COMMON_HISTOGRAM_H_
#define PCL_COMMON_HISTOGRAM_H_

#include <pcl/pcl_base.h>

/**
 * Histogram workflow:
 * - Histogram is created as object
 * - A pointcloud of PointT is handed as input
 * - A function ptr is asked for "process function"
 * This function gets a single pointT as argument and returns the bin index
 * The user needs to implement this function and give it as argument to the Histogram class
 */


namespace pcl {
  template <typename PointT>
  class HistogramStatistics : public pcl::PCLBase <PointT> {
    public:
      typedef float (PointT::*AttributePtr);
      typedef int (*fptr_)(PointT);
      fptr_ function_ptr_;                        // This is the method the user needs to implement

      HistogramStatistics (float min, float max, unsigned int dim, bool cumulative = false, bool normalize = false)
        :min_ (min)
        ,max_ (max)
        ,dim_ (dim)
        ,cumulative_ (cumulative)
        ,normalize_ (normalize)
      {
      }

      virtual ~HistogramStatistics ()
      {
      }

      inline void setFunctionPointer (fptr_ function)
      {
        function_ptr_ = function;
      }

      inline bool getNormalize () const
      {
        return (normalize_);
      }

      inline void setNormalize (bool normalize)
      {
        normalize_ = normalize;
      }

      inline void setMemberPtr (const AttributePtr attr)
      {
        attr_ = attr;
      }

      inline bool getCumulative () const
      {
        return cumulative_;
      }

      inline void setCumulative (bool cumulative)
      {
        cumulative_ = cumulative;
      }

      void compute (std::vector <float> &return_values)
      {
        initCompute ();

        bins_.resize (dim_);

        for (int i = 0; i < indices_->size (); ++i) {
          float val = (*input_)[(*indices_)[i]].*attr_;
          bins_[quantize (val)]++;
        }

        if (normalize_) {
          normalize ();
        }
        if (cumulative_) {
          cumulate ();
        }

        deinitCompute ();

        return_values = bins_;
        bins_.clear ();
      }

      void compute (pcl::PointCloud<PointT> input_cloud, std::vector <float> &return_values)
      {
        initCompute ();

        bins_.resize (dim_);      // Scale the bins_ vector to the actual number of bins

        entries_ = input_cloud.points.size();
        for (unsigned int i = 0; i < entries_; i++)
        {
          unsigned int bin = ftpr(input_cloud.points[i]);
          if( bin > dim_-1)
          {
            //PCL_INFO ("[HistogramStatistics::compute] : ftpr returned incorrect bin index\n");
          }
          else
          {
            bins_[bin]++;
          }
        }
        if (normalize_)
        {
          normalize ();
        }
        if (cumulative_)
        {
          cumulate ();
        }

        deinitCompute ();

        return_values = bins_;
        bins_.clear ();
      }


    protected:
      using PCLBase <PointT>::indices_;
      using PCLBase <PointT>::input_;
      using PCLBase <PointT>::initCompute;
      using PCLBase <PointT>::deinitCompute;

      void normalize ()
      {
        for (int i = 0; i < dim_; ++i)
        {
          bins_[i] /= entries_;
        }
      }

      inline void cumulate ()
      {
        for (int i = 1; i < dim_; ++i)
        {
          bins_[i] += bins_[i-1];
        }
      }

      inline unsigned int quantize (float val) const
      {
        // TODO check correctness
        return (unsigned int) (((val - min_) / (max_ - min_)) * (dim_ - 1));
      }

      bool  cumulative_;              // Indicates if we need to create a cumulative bin distribution
      bool  normalize_;               // Indicates if the bins need to be normalized at the end of compute
      float min_;
      float max_;

      unsigned int entries_;          // The number of entries over all bins before normalisation

      unsigned int dim_;              // Indicates the number of bins
      std::vector <float> bins_;      // Holds the actual bins

      AttributePtr attr_;
  };
}

#endif  //#ifndef PCL_COMMON_HISTOGRAM_H_
