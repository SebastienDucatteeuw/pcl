#ifndef PCL_TRACKING_HISTOGRAM_COHERENCE_H_
#define PCL_TRACKING_HISTOGRAM_COHERENCE_H_

#include <boost/multi_array.hpp>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

namespace pcl
{
  namespace tracking
  {
    /** \brief histogram coherence computes coherence between a ref hist. and a hypothesis hist. from the histogram distance between them. The histogram distance is calculated in HSV color space.
      * \ingroup tracking
      */
    template <typename PointInT, typename StateT>
    class HistogramCoherence: public PCLBase<PointInT>
    {
      protected:
        using PCLBase<PointInT>::initCompute;
        using PCLBase<PointInT>::deinitCompute;

      public:
        using PCLBase<PointInT>::indices_;
        using PCLBase<PointInT>::input_;

      public:
        HistogramCoherence ()
        : clusterWidth_  (51)
        , clusterHeight_ (51)
        , sourceHistogram_ (361)
        {
          std::fill(sourceHistogram_.begin(), sourceHistogram_.end(), 0);
        }

      public:
        inline void
        setClusterWidth (int cluster_width) { clusterWidth_  = cluster_width; } //TODO only odd numbers -> check!

        inline void
        setClusterHeight (int cluster_height) { clusterHeight_ = cluster_height; } //TODO only odd numbers -> check!

        inline void
        setSourceHistogram (std::vector <float> histogram) { sourceHistogram_ = histogram; }

        inline std::vector <float>
        getSourceHistogram () { return sourceHistogram_; }

        /**
          * @brief calculates the BhattacharyyaDistance
          */
        float
        BhattacharyyaDistance (std::vector <float> &hist1, std::vector <float> &hist2);

        float
        ChiSquaredDistance (std::vector <float> &hist1, std::vector <float> &hist2);

        boost::multi_array<float, 3>
        cloud2uvmatrix ();

        float
        computeCoherence (const StateT& target);

        int clusterWidth_;
        int clusterHeight_;
        std::vector <float> sourceHistogram_;
    };
  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/tracking/impl/histogram_coherence.hpp>
#endif

#endif // PCL_TRACKING_HISTOGRAM_COHERENCE_H_
