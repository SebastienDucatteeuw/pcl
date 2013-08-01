#ifndef PCL_TRACKING_IMPL_HISTOGRAM_COHERENCE_H_
#define PCL_TRACKING_IMPL_HISTOGRAM_COHERENCE_H_

#include "boost/multi_array.hpp"

namespace pcl
{
  namespace tracking
  {
    /** \brief histogram coherence computes coherence between a ref hist. and a hypothesis hist. from the histogram distance between them. The histogram distance is calculated in HSV color space.
      * \ingroup tracking
      */
    template <typename StateT>
    class HistogramCoherence
    {
      typedef boost::multi_array<float, 3> array_type;
      typedef array_type::index index;

      public:

        HistogramCoherence ()
        : clusterWidth_  (51)
        , clusterHeight_ (51)
        {
          std::fill(sourceHistogram_.begin(), sourceHistogram_.end(), 0);
          std::vector <float> sourceHistogram_(361);
        }

        inline void
        setClusterWidth (int cluster_width) { clusterWidth_  = cluster_width; }

        inline void
        setClusterHeight (int cluster_height) { clusterHeight_ = cluster_height; }

        inline void
        setSourceHistogram (std::vector <float> histogram) { sourceHistogram_ = histogram; }

        inline std::vector <float>
        getSourceHistogram () { return sourceHistogram_; }

        int clusterWidth_;
        int clusterHeight_;
        std::vector <float> sourceHistogram_;

        float
        computeCoherence (StateT &target);

      private:
        /**
          * @brief calculates the BhattacharyyaDistance
          */
        float
        BhattacharyyaDistance (std::vector <float> &hist1, std::vector <float> &hist2);

        float
        ChiSquaredDistance (std::vector <float> &hist1, std::vector <float> &hist2);

        array_type
        cloud2uvmatrix (pcl::PointCloud<pcl::PointXYZRGBA> &cloud);

    };
  }
}

//#ifdef PCL_NO_PRECOMPILE
#include <pcl/tracking/impl/histogram_coherence.hpp>
//#endif

#endif // PCL_TRACKING_IMPL_HISTOGRAM_COHERENCE_H_


/*
namespace pcl
{
  namespace tracking
  {
    /** \brief histogram coherence computes coherence between a ref hist. and a hypothesis hist. form the histogram distance between them. The histogram distance is calculated in HSV color space.
      * \ingroup tracking
      */
/*
    template <typename PointInT, typename StateT>
    class HistogramCoherence : public ParticleFilterTracker<PointInT, StateT>
    {
      public:

        using Tracker<PointInT, StateT>::tracker_name_;
        using Tracker<PointInT, StateT>::search_;
        using Tracker<PointInT, StateT>::input_;
        using Tracker<PointInT, StateT>::indices_;
        using Tracker<PointInT, StateT>::getClassName;

        typedef Tracker<PointInT, StateT> BaseClass;

        typedef typename Tracker<PointInT, StateT>::PointCloudIn PointCloudIn;

        typedef typename PointCloudIn::Ptr PointCloudInPtr;
        typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

        HistogramCoherence ()
        : clusterWidth_  (51)
        , clusterHeight_ (51)
        {
          std::fill(sourceHistogram_.begin(), sourceHistogram_.end(), 0);
          std::vector <float> sourceHistogram_(361);
        }

        inline void
        setClusterWidth (int cluster_width) { clusterWidth_  = cluster_width; }

        inline void
        setClusterHeight (int cluster_height) { clusterHeight_ = cluster_height; }

        inline void
        setSourceHistogram (std::vector <float> histogram) { sourceHistogram_ = histogram; }

        inline std::vector <float>
        getSourceHistogram () { return sourceHistogram_; }

        int clusterWidth_;
        int clusterHeight_;
        std::vector <float> sourceHistogram_;

        float
        computeCoherence (StateT &target);
/*
        float
        computeCoherence (StateT &target, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);
*/
/*
    };
  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/tracking/impl/histogram_coherence.hpp>
#endif
*/
