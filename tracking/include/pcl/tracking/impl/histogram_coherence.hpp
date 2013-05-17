#ifndef PCL_TRACKING_IMPL_HISTOGRAM_COHERENCE_H_
#define PCL_TRACKING_IMPL_HISTOGRAM_COHERENCE_H_

#include <pcl/common/statistics/statistics.h>
#include <pcl/tracking/histogram_coherence.h>
#include <Eigen/Dense>

namespace pcl
{
  namespace tracking
  {

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
            d = d + std::pow(M*hist1[i]-N*hist2[i],2)/(hist1[i]+hist2[i]);
            counter++;
          }
        }
        return d/(M*N);
      }
    }

    Eigen::MatrixXf
    cloud2uvmatrix (pcl::PointCloud<pcl::PointXYZRGBA>& cloud)
    {
      static const float cx = 320-.5;
      static const float cy = 240-.5;
      static const float f  = 525;

      Eigen::VectorXf u, v, r, g, b, a;

      for (int i = 0; i < cloud->points.size (); i++)
      {
        u(i,0) = f*(cloud->points[i].x/cloud->points[i].z) + cx;
        v(i,1) = f*(cloud->points[i].y/cloud->points[i].z) + cy;
        r(i,2) = cloud->points[i].r
        g(i,3) = cloud->points[i].g
        b(i,4) = cloud->points[i].b
        a(i,5) = cloud->points[i].a
      }

      Eigen::MatrixXi uvrgba(u.rows(),6);
      uvrgba.col(0) = u.cast<int>();
      uvrgba.col(1) = v.cast<int>();
      uvrgba.col(2) = r.cast<int>();
      uvrgba.col(3) = g.cast<int>();
      uvrgba.col(4) = b.cast<int>();
      uvrgba.col(5) = a.cast<int>();

      return uvrgba;
    }

    template <typename PointInT> double
    histogramCoherence<PointInT>::computeCoherence (PointInT &source, PointInT &target)
    {
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster_source (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster_target (new pcl::PointCloud<pcl::PointXYZRGBA>);

      Eigen::MatrixXi source_cluster_uv = xyz2uv (source.xyz);

      int source_cluster_x = source.x;     // x center of the source cluster
      int source_cluster_y = source.y;     // y center of the source cluster
      int target_cluster_x = target.x;     // x center of the target cluster
      int target_cluster_y = target.y;     // y center of the target cluster

      // With cloud_source as reference TODO
      if (!((cluster_width_ - 1)/2 <= source_cluster_x <= cloud_source->width - (cluster_width_ - 1)/2))
      {
        PCL_INFO ("invalid center x-coordinate cluster\n");
      }

      if (!((cluster_height_ - 1)/2 <= source_cluster_y <= cloud_source->height - (cluster_height_ - 1)/2))
      {
        PCL_INFO ("invalid center y-coordinate cluster\n");
      }

      // WITH CLOUD = TARGET CLOUD TODO
      if (!((cluster_width_ - 1)/2 <= target_cluster_x <= cloud_target->width - (cluster_width_ - 1)/2))
      {
        PCL_INFO ("invalid center x-coordinate cluster\n");
      }

      if (!((cluster_height_ - 1)/2 <= target_cluster_y <= cloud_target->height - (cluster_height_ - 1)/2))
      {
        PCL_INFO ("invalid center y-coordinate cluster\n");
      }

      // Set border size
      int y_border = (cluster_height_ - 1)/2;
      int x_border = (cluster_width_ - 1)/2;

      // Make source cluster
      cloud_cluster_source->width  = cluster_width_;
      cloud_cluster_source->height = cluster_height_;
      cloud_cluster_source->points.resize (cluster_width_ * cluster_height_);

      int i = 0;
      for (int row = cluster_y - y_border; row <= cluster_y + y_border; row++)
      {
        for (int column = cluster_x - x_border; column <= cluster_x + x_border; column++)
        {
          cloud_cluster_source->points[i] = cloud_source->points[column + row*cloud->width];
          i++;
        }
      }

      // Make target cluster
      cloud_cluster_target->width  = cluster_width_;
      cloud_cluster_target->height = cluster_height_;
      cloud_cluster_target->points.resize (cluster_width_ * cluster_height_);

      int i = 0;
      for (int row = cluster_y - y_border; row <= cluster_y + y_border; row++)
      {
        for (int column = cluster_x - x_border; column <= cluster_x + x_border; column++)
        {
          cloud_cluster_target->points[i] = cloud_target->points[column + row*cloud->width];
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

      // TODO Case structuur maken die naargelang de gevraagde methode de afstand berekend
      return BhattacharyyaDistance(source_hist, target_hist);
    }
  }
}
