#ifndef PCL_TRACKING_IMPL_PARTICLE_FILTER_HIST_H_
#define PCL_TRACKING_IMPL_PARTICLE_FILTER_HIST_H_

#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/tracking/boost.h>
#include <pcl/tracking/particle_filter_hist.h>
#include <pcl/tracking/tracking.h>

template <typename PointInT, typename StateT> bool
pcl::tracking::ParticleFilterTrackerHist<PointInT, StateT>::initCompute ()
{
  if (!Tracker<PointInT, StateT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  if (transed_reference_vector_.empty ())
  {
    // only one time allocation
    transed_reference_vector_.resize (particle_num_);
    for (int i = 0; i < particle_num_; i++)
    {
      transed_reference_vector_[i] = PointCloudInPtr (new PointCloudIn ());
    }
  }

  if (!change_detector_)
    change_detector_ = boost::shared_ptr<pcl::octree::OctreePointCloudChangeDetector<PointInT> >(new pcl::octree::OctreePointCloudChangeDetector<PointInT> (change_detector_resolution_));
  
  if (!particles_ || particles_->points.empty ())
    initParticles (true);
  return (true);
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTrackerHist<PointInT, StateT>::initParticles (bool reset)
{
  particles_.reset (new PointCloudState ());
  std::vector<double> initial_noise_mean;
  if (reset)
  {
    representative_state_.zero ();
    StateT offset = StateT::toState (trans_);
    representative_state_ = offset;
    representative_state_.weight = 1.0f / static_cast<float> (particle_num_);
  }

  // sampling...
  for ( int i = 0; i < particle_num_; i++ )
  {
    StateT p;
    p.zero ();
    p.sample (initial_noise_mean_, initial_noise_covariance_);
    p = p + representative_state_;
    p.weight = 1.0f / static_cast<float> (particle_num_);
    particles_->points.push_back (p); // update
  }
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTrackerHist<PointInT, StateT>::normalizeWeight ()
{
  double sum = 0.0;
  for ( size_t i = 0; i < particles_->points.size (); i++ )
  {
      sum += particles_->points[i].weight;
  }

  if (sum != 0.0)
  {
    for ( size_t i = 0; i < particles_->points.size (); i++ )
      particles_->points[i].weight =  particles_->points[i].weight / static_cast<float> (sum);
  }
  else
  {
    for ( size_t i = 0; i < particles_->points.size (); i++ )
      particles_->points[i].weight = 1.0f / static_cast<float> (particles_->points.size ());
  }
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTrackerHist<PointInT, StateT>::weight_histogram ()
{
  histogramCoherence_.setInputCloud (input_);
  {
    for (size_t i = 0; i < particles_->points.size (); i++)
    {
      particles_->points[i].weight = histogramCoherence_.compute (particles_->points[i]);
      //std::cout << "norm weight part 20: " << particles_->points[i].weight << std::endl;
    }
  }
  normalizeWeight ();
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTrackerHist<PointInT, StateT>::predict ()
{
  const std::vector<double> zero_mean (StateT::stateDimension (), 0.0);
  PointCloudState origparticles = *particles_;
  particles_->points.clear ();
  // the first particle, it is a just copy of the maximum result
  //StateT p = representative_state_;
  //particles_->points.push_back (p);

  // constant velocity model
  float dt = ((float)t_ - (float)tt_)/CLOCKS_PER_SEC;
  if (dt > 0) // false if t-2 not yet available
  {
    for (int i = 0; i < origparticles.points.size (); i++)
    {
      StateT p = origparticles.points[i];
      // calulate speeds vx, vy, vz save in RPY
      p.roll  = (particles_t_.points[i].x - particles_tt_.points[i].x) / dt;
      p.pitch = (particles_t_.points[i].y - particles_tt_.points[i].y) / dt;
      p.yaw   = (particles_t_.points[i].z - particles_tt_.points[i].z) / dt;
      // add noise using gaussian
      p.sample (zero_mean, step_noise_covariance_);
      // add motion
      p.x = p.x + (dt * p.roll);
      p.y = p.y + (dt * p.pitch);
      p.z = p.z + (dt * p.yaw);
      particles_->points.push_back (p);
    }
  }
  else
  {
    // constant position model
    for (int i = 0; i < origparticles.points.size (); i++)
    {
      StateT p = origparticles.points[i];
      // add noise using gaussian
      p.sample (zero_mean, step_noise_covariance_);
      particles_->points.push_back (p);
    }
  }
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTrackerHist<PointInT, StateT>::resample ()
{
  // Low Variance Sampler (p110 - Probablistic Robotics)
  PointCloudState origparticles = *particles_;
  particles_->points.clear ();
  double step = static_cast<double> (1)/ static_cast<double> (particle_num_);
  double rand = std::rand () % 101;
  double r = rand/(100*particle_num_); // [0,1/M]
  double c = origparticles.points[0].weight;
  int i = 0;
  double U = 0;
  for (int m = 0; m < particle_num_; m++)
  {
    U = r + (static_cast<double> (m) * step);
    while (U > c)
    {
      i = i+1;
      c = c + origparticles.points[i].weight;
    }
    origparticles.points[i].weight = 1.0f / static_cast<float> (particle_num_); //set new samples with equal weights
    particles_->points.push_back (origparticles.points[i]);
  }
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTrackerHist<PointInT, StateT>::computeTracking ()
{
  tt_ = t_;
  t_  = clock();
  particles_tt_ = particles_t_;
  particles_t_ = *particles_;
/*
  for (int i = 0; i < particles_->points.size (); i++ )
    std::cout << "particles_: " << particles_->points[i].weight << std::endl;
  for (int i = 0; i < particles_t_.points.size (); i++ )
    std::cout << "particles_t_: " << particles_t_.points[i].weight << std::endl;
*/

  for (int i = 0; i < iteration_num_; i++)
  {
    predict ();
    weight_histogram (); // likelihood is called in it
    resample ();
  }

  representative_state_.zero ();
  for ( size_t i = 0; i < particles_->points.size (); i++)
  {
    StateT p = particles_->points[i];
    representative_state_ = representative_state_ + p * p.weight;
  }
}
/*
prediction step                                         (proposal via motion model + noise)
weighting step                                          (correction via observation model)
resample step (roulette wheel or stochastic universal sampling)
*/

#define PCL_INSTANTIATE_ParticleFilterTrackerHistHist(T,ST) template class PCL_EXPORTS pcl::tracking::ParticleFilterTrackerHist<T,ST>;
#endif
