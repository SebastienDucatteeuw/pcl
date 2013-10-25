#ifndef PCL_TRACKING_IMPL_HISTOGRAM_COHERENCE_H_
#define PCL_TRACKING_IMPL_HISTOGRAM_COHERENCE_H_

#include <pcl/common/statistics/statistics.h>
#include <pcl/tracking/histogram_coherence.h>
#include <Eigen/Dense>
#include <pcl/point_types_conversion.h>
//#include <pcl/pcl_macros.h> // Include PCL macros such as PCL_ERROR, etc

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

    template <typename PointInT, typename StateT> boost::multi_array<float, 3>
    pcl::tracking::HistogramCoherence<PointInT, StateT>::cloud2uvmatrix ()
    {
      initCompute ();

      static const float cx = 320-.5;
      static const float cy = 240-.5;
      static const float f  = 525;
      boost::multi_array<float, 3> uvmatrix(boost::extents[480][640][2]);
      boost::multi_array<float, 3>::index u;
      boost::multi_array<float, 3>::index v;

      //set all elements to zero
      std::fill(uvmatrix.begin()->begin()->begin(), uvmatrix.end()->end()->end(), 0);

      for (int i = 0; i < input_->points.size (); i++)
      {
        u = (int) f*(input_->points[i].x/input_->points[i].z) + cx;
        v = (int) f*(input_->points[i].y/input_->points[i].z) + cy;

        if ((input_->points[i].z < uvmatrix[u][v][1]) || (uvmatrix[u][v][0] == 0)) // save rgba value to points with zero rgba value or points with smaller z-values
        {
          uvmatrix[u][v][0] = input_->points[i].rgba;
          uvmatrix[u][v][1] = input_->points[i].z;
        }
      }

      deinitCompute ();

      return uvmatrix;
    }

    template <typename PointInT, typename StateT> float
    pcl::tracking::HistogramCoherence<PointInT, StateT>::computeCoherence (const StateT& target)
    {
      /* TODO
      - source cluster should be a histogram vector, changing/weighted over time according to the confidence about the colormodel (could be a class variable that can be initialised (only calculate color model once and adapt if necessary) or reset?)
      - target cluster should be the target_input_ cloud from the coherence_ obj.
      */

      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster_target (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_cluster_target_hsv (new pcl::PointCloud<pcl::PointXYZHSV>);

      // convert target cloud to uvrgba matrix using cloud2uvmatrix
      boost::multi_array<float, 3> target_uvmatrix = pcl::tracking::HistogramCoherence<PointInT, StateT>::cloud2uvmatrix ();

      static const float cx = 320-.5;
      static const float cy = 240-.5;
      static const float f  = 525;

      int target_cluster_u = f*(target.x/target.z) + cx;     // u coordinate of target center
      int target_cluster_v = f*(target.y/target.z) + cy;     // v coordinate of target center

      // check if cluster fits in target matrix
      if (!( ((clusterWidth_-1)/2 <= target_cluster_u) || (target_cluster_u <= 640-(clusterWidth_-1)/2) ))
      {
        //PCL_ERROR ("Invalid center u-coordinate cluster");
        std::cout << "Invalid center u-coordinate cluster" << std::endl;
      }

      if (!( ((clusterHeight_-1)/2 <= target_cluster_v) || (target_cluster_v <= 480-(clusterHeight_-1)/2) ))
      {
        //PCL_ERROR ("Invalid center v-coordinate cluster");
        std::cout << "Invalid center v-coordinate cluster" << std::endl;
      }

      // Set border size
      int u_border = (clusterWidth_ - 1)/2;
      int v_border = (clusterHeight_ - 1)/2;

      // Make target cluster
      cloud_cluster_target->width = clusterWidth_;
      cloud_cluster_target->height = clusterHeight_;
      cloud_cluster_target->points.resize (clusterWidth_ * clusterHeight_);

      int i = 0;
      for (int row = target_cluster_v - v_border; row <= target_cluster_v + v_border; row++)
      {
        for (int column = target_cluster_u - u_border; column <= target_cluster_u + u_border; column++)
        {
          cloud_cluster_target->points[i].rgba = target_uvmatrix[row][column][0];
          i++;
        }
      }

      // Calculate histogram distance
      std::vector <float> targetHistogram(361);
      PointCloudXYZRGBAtoXYZHSV (*cloud_cluster_target, *cloud_cluster_target_hsv);

      pcl::HistogramStatistics<pcl::PointXYZHSV> obj (0, 360, 361, false, true); //TODO create object at class instantiation
      obj.computeHue (*cloud_cluster_target_hsv, targetHistogram);

      // TODO Use case structure to select the desired method to calculate the likelihood
      return BhattacharyyaDistance(sourceHistogram_, targetHistogram);
    }

#define PCL_INSTANTIATE_HistogramCoherence(T,ST) template class PCL_EXPORTS pcl::tracking::HistogramCoherence<T,ST>;

#endif // PCL_TRACKING_IMPL_HISTOGRAM_COHERENCE_H_
