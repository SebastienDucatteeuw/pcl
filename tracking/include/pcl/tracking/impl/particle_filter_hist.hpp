#ifndef PCL_TRACKING_IMPL_PARTICLE_FILTER_HIST_H_
#define PCL_TRACKING_IMPL_PARTICLE_FILTER_HIST_H_

#ifndef _OPENMP
#define _OPENMP
#endif

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
    representative_state_.roll = 0; representative_state_.pitch = 0; representative_state_.yaw = 0; //set speed in x direction to zero (nonzero because of reasons)
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
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
  for ( size_t i = 0; i < particles_->points.size (); i++ )
  {
      sum += particles_->points[i].weight;
  }

  if (sum != 0.0)
  {
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
    for ( size_t i = 0; i < particles_->points.size (); i++ )
      particles_->points[i].weight =  particles_->points[i].weight / static_cast<float> (sum);
  }
  else
  {
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
    for ( size_t i = 0; i < particles_->points.size (); i++ )
      particles_->points[i].weight = 1.0f / static_cast<float> (particles_->points.size ());
  }
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTrackerHist<PointInT, StateT>::weight ()
{
  histogramCoherence_.setInputCloud (input_);
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
  for (size_t i = 0; i < particles_->points.size (); i++)
  {
    particles_->points[i].weight = histogramCoherence_.compute (particles_->points[i]);
    //std::cout << "norm weight part 20: " << particles_->points[i].weight << std::endl;
  }
  normalizeWeight ();
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTrackerHist<PointInT, StateT>::predict ()
{
  const std::vector<double> zero_mean (StateT::stateDimension (), 0.0);
  PointCloudState origparticles = *particles_;
  particles_->points.clear ();
  float dt = ((float)t_ - (float)tt_)/CLOCKS_PER_SEC;

  Eigen::Matrix<float, 6, 1> X; // state
  Eigen::Matrix<float, 6, 6> F; // motion model
  F <<  1, 0, 0, dt,  0,  0,
        0, 1, 0,  0, dt,  0,
        0, 0, 1,  0,  0, dt,
        0, 0, 0,  1,  0,  0,
        0, 0, 0,  0,  1,  0,
        0, 0, 0,  0,  0,  1;
  //Matrix<float, 6, 6> E; // covariance matrix
  Eigen::Matrix<float, 6, 1> Q; // covariances

  //std::cout << "voor: " << particles_->points[2] << std::endl;

  if (dt > 0) // false if t-2 not yet available
  {
    // constant velocity model
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
    for (int i = 0; i < origparticles.points.size (); i++)
    {
      StateT p = origparticles.points[i];
      X(0,0) = p.x;
      X(1,0) = p.y;
      X(2,0) = p.z;
      X(3,0) = p.roll;
      X(4,0) = p.pitch;
      X(5,0) = p.yaw;

      Q(0,0) = static_cast<float> (sampleNormal (zero_mean[0], step_noise_covariance_[0]));
      Q(1,0) = static_cast<float> (sampleNormal (zero_mean[1], step_noise_covariance_[1]));
      Q(2,0) = static_cast<float> (sampleNormal (zero_mean[2], step_noise_covariance_[2]));
      Q(3,0) = static_cast<float> (sampleNormal (zero_mean[3], step_noise_covariance_[3]));
      Q(4,0) = static_cast<float> (sampleNormal (zero_mean[4], step_noise_covariance_[4]));
      Q(5,0) = static_cast<float> (sampleNormal (zero_mean[5], step_noise_covariance_[5]));

      X = F*X + F*Q;

      p.x       = X(0,0);
      p.y       = X(1,0);
      p.z       = X(2,0);
      p.roll    = X(3,0);
      p.pitch   = X(4,0);
      p.yaw     = X(5,0);
      particles_->points.push_back (p);
    }
  }
  else
  {
    // constant position model
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
    for (int i = 0; i < origparticles.points.size (); i++)
    {
      StateT p = origparticles.points[i];
      // add noise using gaussian
      p.sample (zero_mean, step_noise_covariance_);
      particles_->points.push_back (p);
    }
  }

  //std::cout << "na: " << particles_->points[2] << std::endl;

/*
  if (dt > 0) // false if t-2 not yet available
  {
    // constant velocity model
    for (int i = 0; i < origparticles.points.size (); i++)
    {
      StateT p = origparticles.points[i];
      p.x = p.x + motion_ratio_ * (dt*p.roll);
      p.y = p.y + motion_ratio_ * (dt*p.pitch);
      p.z = p.z + motion_ratio_ * (dt*p.yaw);
      // add noise using gaussian
      p.sample (zero_mean, step_noise_covariance_);
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

  // constant velocity model
  //dt = 0; //uncomment to use const pos model
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
      p.x = p.x + (dt * motion_ratio_ * p.roll);
      p.y = p.y + (dt * motion_ratio_ * p.pitch);
      p.z = p.z + (dt * motion_ratio_ * p.yaw);
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
*/
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

  // PF
  predict ();
  weight (); // likelihood is called in it
  resample ();

  // PF calculate result
  representative_state_.zero ();
  for ( size_t i = 0; i < particles_->points.size (); i++)
  {
    StateT p = particles_->points[i];
    representative_state_ = representative_state_ + p * p.weight;
  }
/*
  // update colormodel
  histogramCoherence_.setInputCloud (input_);
  histogramCoherence_.setClusterRadius (0.03);
  histogramCoherence_.setUpdateReferenceHistogram (true);
  histogramCoherence_.compute (representative_state_);
  histogramCoherence_.setUpdateReferenceHistogram (false);
  histogramCoherence_.setClusterRadius (0.02);
*/
}
/*
prediction step       (proposal via motion model + noise)
weighting step        (correction via observation model)
resample step         (roulette wheel or stochastic universal sampling)
*/

#define PCL_INSTANTIATE_ParticleFilterTrackerHistHist(T,ST) template class PCL_EXPORTS pcl::tracking::ParticleFilterTrackerHist<T,ST>;
#endif
