#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  // Initially set to false until first measurement
  is_initialized_ = false;

  // State dimension
  n_x_ = 5;

  // Augmented dimension
  n_aug_ = 7;

  // Spreading parameter
  lambda_ = 0;

  // Matrix to hold sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

  // Vector for weights
  weights_ = VectorXd(2*n_aug_+1);

  // Noise matrices
  R_radar = MatrixXd(3,3);
  R_laser = MatrixXd(2,2);  

  // Start time
  time_us_ = 0;

  // NIS
  NIS_radar_ = 0;
  NIS_laser_ = 0;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

//   if (!is_initialized_) {
//     if(meas_package.sensor_type_ == MeasurementPackage::LASER) {
//       // Initialize state.
//       x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 4, 0.5, 0.0;

//       // State covariance matrix
//       P_ << std_laspx_*std_laspx_, 0, 0, 0, 0,
//             0, std_laspy_*std_laspy_, 0, 0, 0,
//             0, 0, 1, 0, 0,
//             0, 0, 0, 1, 0,
//             0, 0, 0, 0, 1;

//       // Create R for update noise later
//       R_laser << std_laspx_*std_laspx_, 0,
//                  0, std_laspy_*std_laspy_;
//     } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
//       double rho = meas_package.raw_measurements_(0);
//       double phi = meas_package.raw_measurements_(1);
//       double rhodot = meas_package.raw_measurements_(2);
      
//       // Convert from polar to cartesian
//       x_ << rho*cos(phi), rho*sin(phi), 4, rhodot*cos(phi), rhodot*sin(phi);
      
//       // State covariance matrix
//       P_ << std_radr_*std_radr_, 0, 0, 0, 0,
//             0, std_radr_*std_radr_, 0, 0, 0,
//             0, 0, 1, 0, 0,
//             0, 0, 0, std_radphi_, 0,
//             0, 0, 0, 0, std_radphi_;
      
//       // Create R for update noise later
//       R_radar << std_radr_*std_radr_, 0, 0,
//                  0, std_radphi_*std_radphi_, 0,
//                  0, 0, std_radrd_*std_radrd_;

//       }
//       // Done
//       is_initialized_ = true;
//       time_us_ = meas_package.timestamp_;
//       return;
//     }
//     // Calculate delta_t and store current time
//     double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
//     time_us_ = meas_package.timestamp_;

//     // Prediction
//     Prediction(delta_t);

//     // Measurement updates
//     if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
//       UpdateLidar(meas_package);
//     } else {
//       UpdateRadar(meas_package);
//     }
// }
    
  if (!is_initialized_) {
    // First measurement
    x_ << 1, 1, 1, 1, 0.1;

    // Initialize covariance matrix
    P_ << 0.15, 0, 0, 0, 0,
          0, 0.15, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

    // Initialize time stamp
    time_us_ = meas_package.timestamp_;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
      // Convert from polar to cartesian
      float ro     = meas_package.raw_measurements_(0);
      float phi    = meas_package.raw_measurements_(1);
      float ro_dot = meas_package.raw_measurements_(2);
      x_(0) = ro * cos(phi);
      x_(1) = ro * sin(phi);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
    }

    // Done
    is_initialized_ = true;

    return;
  }

  // Calculate delta_t and store current time
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  // Prediction
  Prediction(delta_t);

  // Measurement updates
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  } else {
    UpdateLidar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  // This section generates Sigma points
  // Define spreading parameter
  lambda_ = 3 - n_x_;

  // Create sigma point matrix
  MatrixXd Xsig_ = MatrixXd(n_x_, 2*n_x_+1);

  // Square root of P
  MatrixXd A_ = P_.llt().matrixL();

  // Sigma points
  Xsig_.col(0) = x_;
  for (int i = 0; i<n_x_; i++) {
    Xsig_.col(i+1) = x_ + sqrt(lambda_+n_x_)*A_.col(i);
    Xsig_.col(i+1+n_x_) = x_ - sqrt(lambda_+n_x_)*A_.col(i);
  }
  
  // This section augments Sigma points
  // Spreading parameter
  lambda_ = 3 - n_aug_;

  // Augmented mean vector
  VectorXd x_aug_ = VectorXd(n_aug_);

  // Augmented state covariance matrix
  MatrixXd P_aug_ = MatrixXd(n_aug_, n_aug_);

  // Sigma point matrix
  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2*n_aug_+1);
  
  // Augmented mean state
  x_aug_.head(5) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;

  // Augmented covariance matrix
  MatrixXd Q = MatrixXd(2,2);
  Q << std_a_*std_a_, 0,
       0, std_yawdd_*std_yawdd_;
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(5, 5) = P_;
  P_aug_.bottomRightCorner(2, 2) = Q;
  
  // Square root matrix
  MatrixXd A_aug = P_aug_.llt().matrixL();

  // Augmented sigma points
  Xsig_aug_.col(0) = x_aug_;
  for(int i = 0; i<n_aug_; i++) {
    Xsig_aug_.col(i+1) = x_aug_ + sqrt(lambda_+n_aug_)*A_aug.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug_ - sqrt(lambda_+n_aug_)*A_aug.col(i);
  }
  
  // This section predicts sigma points and then converts to mean/covariance
  // Predict sigma points
  VectorXd vec1 = VectorXd(5);
  VectorXd vec2 = VectorXd(5);
  
  // for(int i = 0; i < 2*n_aug_+1; i++) {
  //   double px   = Xsig_aug_(0, i);
  //   double py   = Xsig_aug_(1, i);
  //   double v    = Xsig_aug_(2, i);
  //   double yaw  = Xsig_aug_(3, i);
  //   double yawd = Xsig_aug_(4, i);
  //   double v1   = Xsig_aug_(5, i);
  //   double v2   = Xsig_aug_(6, i);
    
  //   VectorXd orig = Xsig_aug_.head(5, i);
    
  //   if(yawd > .001) {
  //     vec1 << (v/yawd)*(sin(yaw+yawd*delta_t) - sin(yaw)),
  //             (v/yawd)*(-cos(yaw+yawd*delta_t) + cos(yaw)),
  //             0,
  //             yawd * delta_t,
  //             0;
  //   } else {
  //     // Avoid division by zero
  //     vec1 << v*cos(yaw)*delta_t,
  //             v*sin(yaw)*delta_t,
  //             0,
  //             yawd*delta_t,
  //             0;
  //   }
  //   vec2 << .5*delta_t*delta_t*cos(yaw)*v1,
  //           .5*delta_t*delta_t*sin(yaw)*v1,
  //           delta_t*v1,
  //           .5*delta_t*delta_t*v2,
  //           delta_t*v2;
    
  //   // Write predicted
  //   Xsig_pred_.col(i) << orig + vec1 + vec2;

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    //extract values for better readability
    double p_x      = Xsig_aug_(0, i);
    double p_y      = Xsig_aug_(1, i);
    double v        = Xsig_aug_(2, i);
    double yaw      = Xsig_aug_(3, i);
    double yawd     = Xsig_aug_(4, i);
    double nu_a     = Xsig_aug_(5, i);
    double nu_yawdd = Xsig_aug_(6, i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else {
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  // Create vector for predicted state
  VectorXd x_pred = VectorXd(n_x_);

  // Create covariance matrix for prediction
  MatrixXd P_pred = MatrixXd(n_x_, n_x_);
  
  x_pred.fill(0.0);
  P_pred.fill(0.0);
  
  for(int i = 0; i<2*n_aug_+1; i++) {
  
    if (i == 0) {
      weights_(i) = lambda_ / (lambda_ + n_aug_);
    } else {
      weights_(i) = .5 / (lambda_ + n_aug_);
    }
    
    // Predict state mean
    x_pred += weights_(i) * Xsig_pred_.col(i);
  }
  
  for (int i = 0; i<2*n_aug_+1; i++) {
    
    // Predict state covariance
    VectorXd x_diff = Xsig_pred_.col(i) - x_pred;
    
    // Normalize
    if (x_diff(3) > M_PI) {
      x_diff(3) -= 2. * M_PI;
    } else if (x_diff(3) < -M_PI) {
      x_diff(3) += 2. * M_PI;
    }
    P_pred += weights_(i) * x_diff * x_diff.transpose();
  }
  
  x_ = x_pred;
  P_ = P_pred;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  // Set measurement dimension
  int n_z = 2;

  // Create example matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);

  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  // create example matrix for predicted measurement covariance
  MatrixXd S = MatrixXd(n_z, n_z);

  //calculate cross correlation matrix
  Zsig.fill(0.0);
  z_pred.fill(0.0);
  S.fill(0.0);
  
  for (int i = 0; i<2*n_aug_+1; i++) {
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    
    Zsig.col(i) << px,
                   py;
    
    // Calculate mean predicted measurement
    z_pred += weights_(i) * Zsig.col(i);
  }
  
  // Calculate measurement covariance matrix S
  for (int i = 0; i<2*n_aug_+1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S += weights_(i) * z_diff * z_diff.transpose();
  }
  
  // Add noise to S
  S += R_laser;
  
  // Create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);

  double meas_px = meas_package.raw_measurements_(0);
  double meas_py = meas_package.raw_measurements_(1);

  z << meas_px,
       meas_py;

  // Create cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);

  // Calculate cross correlation matrix
  for (int i = 0; i<2*n_aug_+1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    
    // Normalize
    if (x_diff(3) > M_PI) {
      x_diff(3) -= 2. * M_PI;
    } else if (x_diff(3) < -M_PI) {
      x_diff(3) += 2. * M_PI;
    }
    
    VectorXd z_diff = Zsig.col(i) - z_pred;

    Tc += weights_(i) * x_diff * z_diff.transpose();

  }
  
  // Residual
  VectorXd z_diff = z - z_pred;

  // NIS
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
  
  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  
  // Update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  // Set measurement dimension
  int n_z = 3;

  // Create example matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);

  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  // create example matrix for predicted measurement covariance
  MatrixXd S = MatrixXd(n_z, n_z);

  //calculate cross correlation matrix
  Zsig.fill(0.0);
  z_pred.fill(0.0);
  S.fill(0.0);
  double rho = 0;
  double phi = 0;
  double rhodot = 0;
  
  for (int i = 0; i < 2*n_aug_+1; i++) {
    // Sigma to measurement points
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);
    double yaw_d = Xsig_pred_(4, i);
    
    rho = sqrt(px*px+py*py);
    phi = atan2(py,px);
    rhodot = (px*cos(yaw)*v+py*sin(yaw)*v) / rho;
    
    Zsig.col(i) << rho,
                   phi,
                   rhodot;
    
    // Calculate mean predicted measuremENT
    z_pred += weights_(i) * Zsig.col(i);
  }
  
  // Calculate measurement covariance matrix S
  for (int i = 0; i<2*n_aug_+1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    if (z_diff(1) > M_PI) {
      z_diff(1) -= 2. * M_PI;
    } else if (z_diff(1) < - M_PI) {
      z_diff(1) += 2. * M_PI;
    }
    S += weights_(i) * z_diff * z_diff.transpose();
  }
  
  // Add noise to S
  S += R_radar;
  
  // Create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  double meas_rho = meas_package.raw_measurements_(0);
  double meas_phi = meas_package.raw_measurements_(1);
  double meas_rhodot = meas_package.raw_measurements_(2);
  
  z << meas_rho,
       meas_phi,
       meas_rhodot;
  
  // Create cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  
  // Calculate cross correlation matrix
  for (int i = 0; i<2*n_aug_+1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // Normalize
    if (x_diff(3) > M_PI) {
      x_diff(3) -= 2. * M_PI;
    } else if (x_diff(3) < -M_PI) {
      x_diff(3) += 2. * M_PI;
    }
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // Normalize
    if (z_diff(1) > M_PI) {
      z_diff(1) -= 2. * M_PI;
    } else if (z_diff(1) < -M_PI) {
      z_diff(1) += 2. * M_PI;
    }
    Tc += weights_(i) * x_diff * z_diff.transpose();
    
  }
  
  // Residual
  VectorXd z_diff = z - z_pred;
  
  // Normalize
  if (z_diff(1) > M_PI) {
    z_diff(1) -= 2. * M_PI;
  } else if (z_diff(1) < -M_PI) {
    z_diff(1) += 2. * M_PI;
  }
  
  // NIS
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  
  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // Update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
}
