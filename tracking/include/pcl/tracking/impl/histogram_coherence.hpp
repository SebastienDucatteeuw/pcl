#ifndef PCL_TRACKING_IMPL_HISTOGRAM_COHERENCE_H_
#define PCL_TRACKING_IMPL_HISTOGRAM_COHERENCE_H_

#include <pcl/common/statistics/statistics.h>
#include <pcl/tracking/histogram_coherence.h>
#include <Eigen/Dense>
#include <pcl/point_types_conversion.h>
#include <pcl/search/organized.h>
#include <cmath>

    template <typename PointInT, typename StateT> float
    pcl::tracking::HistogramCoherence<PointInT, StateT>::BhattacharyyaDistance (std::vector <float> &hist1, std::vector <float> &hist2)
    {
      if (hist1.size () != hist2.size ())
      {
        //PCL_INFO ("[HistogramStatistics::BhattacharyyaDistance] : both histograms do not have the same number of bins\n");
        return 0;
      }
      else
      {
        int bins = hist1.size ();
        // Calc the Bhattacharyya coef:
        float bcoeff = 0;
        for (int i = 0; i < bins; i++)
        {
          bcoeff = bcoeff + std::sqrt(hist1[i] * hist2[i]);
        }
        // Calc the distance between the two distributions
        float bdist = std::sqrt(1 - bcoeff);

        return bdist;
      }
    }

    template <typename PointInT, typename StateT> float
    pcl::tracking::HistogramCoherence<PointInT, StateT>::ChiSquaredDistance (std::vector <float> &hist1, std::vector <float> &hist2)
    {
      if (hist1.size () != hist2.size ())
      {
        //PCL_INFO ("[HistogramStatistics::ChiSquaredDistance] : both histograms do not have the same number of bins\n");
        return 0;
      }
      else
      {
        float d = 0;
        int M = 361; int N = 361;
        int counter = 0;
        for (int i = 0; i <= 360; i++)
        {
          if((hist1[i]+hist2[i]) != 0)
          {
            /*
            1/(MN) SUM_i[((Mni - Nmi)^2)/(mi+ni)].
            M and N are the total number of entries in each histogram, mi is the number of entries in bin i of histogram M and ni is the number of entries in bin i of histogram N.
            */
            d += std::pow(M*hist1[i]-N*hist2[i],2)/(hist1[i]+hist2[i]);
            counter++;
          }
        }
        return d/(M*N); //TODO not correctly normalized...
      }
    }

    template <typename PointInT, typename StateT> float
    pcl::tracking::HistogramCoherence<PointInT, StateT>::computeCoherence (const StateT& target)
    {
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster_target (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_cluster_target_hsv (new pcl::PointCloud<pcl::PointXYZHSV>);
      float weight;

      if (!pcl::isFinite (target))
      {
        std::cout << "Target is infinite." << std::endl;
        return 0;
      }

      //search points in radius around target
      pcl::PointXYZRGBA center;
      center.x = target.x;
      center.y = target.y;
      center.z = target.z;

/*--- Bruteforce
      double distance = 0;
      for (int i = 0; i < input_->points.size (); i++)
      {
        distance = std::sqrt(std::pow(input_->points[i].x - center.x, 2) + std::pow(input_->points[i].y - center.y, 2) + std::pow(input_->points[i].z - center.z, 2));
        if (distance < 1.2)
        {
          cloud_cluster_target->points.push_back (input_->points[i]);
        }
      }
      std::cout << "aantal gevonden puntjes binnen radius: " << cloud_cluster_target->points.size () << std::endl;
*/

      pcl::search::OrganizedNeighbor<PointInT> organizedNeighborSearch (false, 1e-4f, 5);
      organizedNeighborSearch.setInputCloud(input_);
      if (!organizedNeighborSearch.isValid ())
        std::cout << "Error: Input is not organized or from projective device" << std::endl;

      double radius = 0.05;

      std::vector<int> searchIndices;
      std::vector<float> searchSquaredDistances;
      searchIndices.clear ();
      searchSquaredDistances.clear ();

      organizedNeighborSearch.radiusSearch (center, radius, searchIndices, searchSquaredDistances);
      //std::cout << "aantal gevonden puntjes binnen radius: " << searchIndices.size () << std::endl;

      if (searchIndices.size () > 0)
      {
        for (int i = 0; i < searchIndices.size (); ++i)
        {
          cloud_cluster_target->points.push_back (input_->points[searchIndices[i]]);
        }

        // Calculate histogram distance
        std::vector <float> targetHistogram(361);
        PointCloudXYZRGBAtoXYZHSV (*cloud_cluster_target, *cloud_cluster_target_hsv);

        pcl::HistogramStatistics<pcl::PointXYZHSV> obj (0, 360, 361, false, true); //TODO create object at class instantiation
        obj.computeHue (*cloud_cluster_target_hsv, targetHistogram);

        // TODO Use case structure to select the desired method to calculate the likelihood
        weight = 1-BhattacharyyaDistance(sourceHistogram_, targetHistogram);
      }
      else
        weight = 0;

      //std::cout << "Weight: " << weight << std::endl;
      return weight;
    }

template <typename PointInT, typename StateT> bool
pcl::tracking::HistogramCoherence<PointInT, StateT>::initCompute ()
{
  if (!PCLBase<PointInT>::initCompute ())
  {
    //PCL_ERROR ("[pcl::%s::initCompute] PCLBase::Init failed.\n", getClassName ().c_str ());
    std::cout << "[pcl::HistogramCoherence::initCompute] PCLBase::Init failed." << std::endl;
    return (false);
  }

  // If the dataset is empty, just return
  if (input_->points.empty ())
  {
    //PCL_ERROR ("[pcl::%s::compute] input_ is empty!\n", getClassName ().c_str ());
    std::cout << "[pcl::HistogramCoherence::initCompute] input_ is empty!" << std::endl;
    // Cleanup
    deinitCompute ();
    return (false);
  }

  return (true);
}

template <typename PointInT, typename StateT> float
pcl::tracking::HistogramCoherence<PointInT, StateT>::compute (const StateT& target)
{
  if (!initCompute ())
    return 0;

  float coherence = computeCoherence (target);
  deinitCompute ();
  return coherence;
}

#define PCL_INSTANTIATE_HistogramCoherence(T,ST) template class PCL_EXPORTS pcl::tracking::HistogramCoherence<T,ST>;

#endif // PCL_TRACKING_IMPL_HISTOGRAM_COHERENCE_H_
