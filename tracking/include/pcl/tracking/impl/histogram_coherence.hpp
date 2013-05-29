#ifndef PCL_TRACKING_IMPL_HISTOGRAM_COHERENCE_H_
#define PCL_TRACKING_IMPL_HISTOGRAM_COHERENCE_H_

#include <pcl/common/statistics/statistics.h>
#include <pcl/tracking/histogram_coherence.h>
#include <Eigen/Dense>

namespace pcl
{
  namespace tracking
  {
    typedef boost::multi_array<float, 3> array_type;
    typedef array_type::index index;

    double
    BhattacharyyaDistance (std::vector <float> &hist1, std::vector <float> &hist2) // Both histograms must be normalised
    {
      if (hist1.size () != hist2.size ())
      {
        PCL_INFO ("[HistogramStatistics::BhattacharyyaDistance] : both histograms do not have the same number of bins\n");
        return 0;
      }
      else
      {
        int bins = hist1.size ();
        // Calc the Bhattacharyya coef:
        bcoeff = 0;
        for (int i = 0; i < bins; i++)
        {
          bcoeff = bcoeff + std::sqrt(hist1[i] * hist2[i]);
        }
        // Calc the distance between the two distributions
        bdist = std::sqrt(1 - bcoeff);

        return bdist;
      }
    }

    double
    ChiSquaredDistance (std::vector <float> &hist1, std::vector <float> &hist2)
    {
      if (hist1.size () != hist2.size ())
      {
        PCL_INFO ("[HistogramStatistics::ChiSquaredDistance] : both histograms do not have the same number of bins\n");
        return 0;
      }
      else
      {
        double d = 0;
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
        return d/(M*N);
      }
    }

    array_type
    cloud2uvmatrix (pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
    {
      static const float cx = 320-.5;
      static const float cy = 240-.5;
      static const float f  = 525;

      array_type uvmatrix(boost::extents[480][640][2]);
      index u;
      index v;

      //set all element to zero
      std::fill(uvmatrix.begin()->begin()->begin(), uvmatrix.end()->end()->end(), 0);

      for (int i = 0; i < cloud->points.size (); i++)
      {
        u = (int) f*(cloud->points[i].x/cloud->points[i].z) + cx;
        v = (int) f*(cloud->points[i].y/cloud->points[i].z) + cy;

        if (cloud->points[i].z < uvmatrix[u][v][1] || uvmatrix[u][v][0] == 0) // save rgba value to points with (smaller z-values or new value) projected on the same uv
        {
          uvmatrix[u][v][0] = cloud->points[i].rgba
          uvmatrix[u][v][1] = cloud->points[i].z
        }
      }
      return uvmatrix;
    }

    template <typename PointInT> double
    histogramCoherence<PointInT>::computeCoherence (PointInT &target)
    {

/* TODO
- source cluster should be a histogram vector, changing/weighted over time according to the confidence about the colormodel (could be a class variable that can be initialised (only calculate color model once and adapt if necessary) or reset?)
- target cluster should be the target_input_ cloud from the coherence_ obj.
*/
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster_source (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster_target (new pcl::PointCloud<pcl::PointXYZRGBA>);

      // convert source and target cloud to uvrgba matrices using cloud2uvmatrix
      array_type source_uvmatrix = cloud2uvmatrix (source_cloud);
      array_type target_uvmatrix = cloud2uvmatrix (target_cloud);

      int source_cluster_u = f*(source.x/source.z) + cx;     // u center of the source cluster
      int source_cluster_v = f*(source.y/source.z) + xy;     // y center of the source cluster
      int target_cluster_u = f*(target.x/target.z) + cx;     // u center of the target cluster
      int target_cluster_v = f*(target.y/target.z) + cy;     // v center of the target cluster

      // check if cluster fits in source matrix
      if (!((cluster_width_ - 1)/2 <= source_cluster_u <= 640 - (cluster_width_ - 1)/2))
      {
        PCL_INFO ("invalid center u-coordinate cluster\n");
      }

      if (!((cluster_height_ - 1)/2 <= source_cluster_v <= 480 - (cluster_height_ - 1)/2))
      {
        PCL_INFO ("invalid center v-coordinate cluster\n");
      }

      // check if cluster fits in target matrix
      if (!((cluster_width_ - 1)/2 <= target_cluster_u <= 640 - (cluster_width_ - 1)/2))
      {
        PCL_INFO ("invalid center u-coordinate cluster\n");
      }

      if (!((cluster_height_ - 1)/2 <= target_cluster_v <= 480 - (cluster_height_ - 1)/2))
      {
        PCL_INFO ("invalid center v-coordinate cluster\n");
      }

      // Set border size
      int u_border = (cluster_width_ - 1)/2;
      int v_border = (cluster_height_ - 1)/2;

      // Make source cluster
      cloud_cluster_source->width = cluster_width_;
      cloud_cluster_source->height = cluster_height_;
      cloud_cluster_source->points.resize (cluster_width_ * cluster_height_);

      int i = 0;
      for (int row = cluster_y - v_border; row <= cluster_y + v_border; row++)
      {
        for (int column = cluster_x - u_border; column <= cluster_x + u_border; column++)
        {
          cloud_cluster_source->points[i].rgba = source_uvmatrix[row][column][0];
          i++;
        }
      }

      // Make target cluster
      cloud_cluster_target->width = cluster_width_;
      cloud_cluster_target->height = cluster_height_;
      cloud_cluster_target->points.resize (cluster_width_ * cluster_height_);

      int i = 0;
      for (int row = cluster_y - v_border; row <= cluster_y + v_border; row++)
      {
        for (int column = cluster_x - u_border; column <= cluster_x + u_border; column++)
        {
          cloud_cluster_target->points[i].rgba = target_uvmatrix[row][column][0];
          i++;
        }
      }

      // Calculate histogram distance
      std::vector <float> source_hist(361);
      std::vecotr <float> target_hist(361);
      PointCloudXYZRGBAtoXYZHSV (*cloud_cluster_source, *cloud_cluster_source_hsv);
      PointCloudXYZRGBAtoXYZHSV (*cloud_cluster_target, *cloud_cluster_target_hsv);

      pcl::HistogramStatistics <pcl::PointXYZHSV> obj (0, 360, 361, false, true);
      obj.computeHue (*cloud_cluster_source_hsv, source_hist);
      obj.computeHue (*cloud_cluster_target_hsv, target_hist)

      // TODO Use case structure to select the desired method to calculate the likelihood
      return BhattacharyyaDistance(source_hist, target_hist);
    }
  }
}

#endif
