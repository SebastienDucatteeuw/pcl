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
        using PCLBase<PointInT>::deinitCompute;

      public:
        using PCLBase<PointInT>::indices_;
        using PCLBase<PointInT>::input_;

        HistogramCoherence ()
        : cluster_radius_  (0.05)
        , reference_histogram_ (361)
        , target_histogram_ (361)
        , update_reference_histogram_ (false)
        , update_threshold_ (0.7)
        {
          std::fill(reference_histogram_.begin(), reference_histogram_.end(), 0);
        }

        float
        compute (const StateT& target);

        inline void
        setClusterRadius (int cluster_radius) { cluster_radius_  = cluster_radius; }

        inline void
        setReferenceHistogram (const std::vector <float> &histogram) { reference_histogram_ = histogram; }

        inline std::vector <float>
        getReferenceHistogram () { return reference_histogram_; }

        inline void
        setUpdateReferenceHistogram (bool status) { update_reference_histogram_ = status; }

        inline void
        setUpdateThreshold (float threshold) { update_threshold_ = threshold; }

        /**
          * @brief calculates the BhattacharyyaDistance
          */
        float
        BhattacharyyaDistance (std::vector <float> &hist1, std::vector <float> &hist2);

        float
        ChiSquaredDistance (std::vector <float> &hist1, std::vector <float> &hist2);

        float
        computeCoherence (const StateT& target);

      protected:
        virtual bool
        initCompute ();

      private:
        float cluster_radius_;
        std::vector <float> reference_histogram_;
        std::vector <float> target_histogram_;
        bool update_reference_histogram_;
        float update_threshold_;
    };
  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/tracking/impl/histogram_coherence.hpp>
#endif

#endif // PCL_TRACKING_HISTOGRAM_COHERENCE_H_
