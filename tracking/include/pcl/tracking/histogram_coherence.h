#ifndef PCL_TRACKING_IMPL_HISTOGRAM_COHERENCE_H_
#define PCL_TRACKING_IMPL_HISTOGRAM_COHERENCE_H_

#include <pcl/tracking/coherence.h>

namespace pcl
{
  namespace tracking
  {
    /** \brief histogram coherence computes coherence between the two points form the histogram distance between them. The histogram distance is calculated in HSV color space.
      * \ingroup tracking
      */
    template <typename PointInT>
    class histogramCoherence
    {
      public:
        histogramCoherence ()
        : PointCoherence<PointInT> ()
        , cluster_width_  (51)
        , cluster_height_ (51)
        {}

        inline void
        setClusterWidth (int cluster_width) { cluster_width_  = cluster_width; }

        inline void
        setClusterHeight (int cluster_height) { cluster_hight_ = cluster_height; }

      protected:
        double
        computeCoherence (PointInT &source, PointInT &target);
    };
  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/tracking/impl/histogram_coherence.hpp>
#endif

#endif
