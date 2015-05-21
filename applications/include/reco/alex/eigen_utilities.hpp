#ifndef EIGEN_UTILITIES_HPP_
#define EIGEN_UTILITIES_HPP_

#include <iostream>
#include <fstream>
#include <assert.h>
#include <random>
#include <eigen3/Eigen/Dense>

namespace Eigen
{
  /** \brief Write matrix to a file in binary mode
   * \param filename output file name
   * \param matrix matrix
   * \note http://stackoverflow.com/questions/25389480/how-to-write-read-an-eigen-matrix-from-binary-file
   */
  template<class Matrix>
  inline
  bool writeBinary(const std::string filename, const Matrix& matrix)
  {
    std::ofstream out(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
    if (out.is_open())
    {
      typename Matrix::Index rows=matrix.rows(), cols=matrix.cols();
      out.write((char*) (&rows), sizeof(typename Matrix::Index));
      out.write((char*) (&cols), sizeof(typename Matrix::Index));
      out.write((char*) matrix.data(), rows*cols*sizeof(typename Matrix::Scalar) );
      out.close();
      return true;
    }
    else
    {
      std::cout << "[Eigen::writeBinary] Colud not open file '" << filename << "' for writing\n";
      return false;
    }
  }
  
  /** \brief Read a matrix from a binary file
   * \param filename input file name
   * \param matrix matrix
   * \note http://stackoverflow.com/questions/25389480/how-to-write-read-an-eigen-matrix-from-binary-file
   */  
  template<class Matrix>
  inline
  bool readBinary(const std::string filename, Matrix& matrix)
  {
    std::ifstream in(filename.c_str(), std::ios::in | std::ios::binary);
    if (in.is_open())
    {
      typename Matrix::Index rows=0, cols=0;
      in.read((char*) (&rows),sizeof(typename Matrix::Index));
      in.read((char*) (&cols),sizeof(typename Matrix::Index));
      matrix.resize(rows, cols);
      in.read( (char *) matrix.data() , rows*cols*sizeof(typename Matrix::Scalar) );
      in.close();
      return true;
    }
    else
    {
      std::cout << "[Eigen::readBinary] Colud not open file '" << filename << "' for reading\n";
    }
  }
  
  /** \brief Count the number of values that are within the spcified bin range.
    * If a value is outside the provided range it is discarded.
    * Analog of MATLAB histc
    * (http://www.mathworks.com/help/matlab/ref/histc.html)
    * \param[in] x input vector
    * \param[in] bin_ranges number of bins in the histogram (monotonically non-decreasing)
    * \param[out] bin_counts output histogram
    * \note value a is assigned to bin b if a \in [b_min, b_max)
    */  
  inline
  void histc(const VectorXd &x, const VectorXd &bin_ranges, VectorXd &bin_counts)
  {
    // Check input
    if (x.size() < 1 || bin_ranges.size() < 1)
    {
      std::cout << "[Eigen::histc] input vector and bin ranges size has to be greater than 0\n";
      exit (EXIT_FAILURE);
    }
    
    // Count
    size_t out_of_range_count = 0;
    bin_counts = VectorXd::Zero(bin_ranges.size());
    for (size_t i=0; i < x.size(); i++)
    {
      // If current values is less than the smallest bin range skip it
      if (x[i] < bin_ranges[0])
        continue;
      
      // If current values is greater or equal to the largest bin range
      if (x[i] >= bin_ranges[bin_ranges.size()-1])
      {
        bin_counts[bin_ranges.size()-1]++;
        continue;
      }
      
      // Go over all other bin ranges
      for (size_t curBin=0; curBin < bin_ranges.size()-1; curBin++)
      {
        if (x[i] < bin_ranges[curBin+1])
        {
          bin_counts[curBin]++;
          break;
        }
      }
    }
  }  
  
  /** \brief Compute the histgroam of the input vector
    * \param[in] vector_in input vector
    * \param[in] nbins number of bins in the histogram
    * \param[out] hist_out output histogram
    * \param[out] bin_centers center value of each bin
    * \
    */  
  inline
  void hist(const VectorXd &x, int nbins, VectorXd &hist_out, VectorXd &bin_centers)
  {
    // Check that number of bins is at least 1
    if (nbins < 1)
    {
      std::cout << "[Eigen::hist] histogram requires at least one bin\n";
      exit(EXIT_FAILURE);
    }
    
    // Create bins
    double minVal = x.minCoeff();
    double maxVal = x.maxCoeff();
    double step   = (maxVal - minVal) / static_cast<double>(nbins);
    VectorXd bin_ranges   = VectorXd::LinSpaced(nbins, minVal, maxVal-step);
    bin_centers           = VectorXd::LinSpaced(nbins, minVal+step/2, maxVal-step/2);    
    
    // Count
    histc(x, bin_ranges, hist_out);
/*    
    
    // Calculate histogram
    hist_out = VectorXd::Zero(nbins);
    
    for (size_t i = 0; i < vector_in.size(); i++)
    {
      int curBin = static_cast<int>(floor((vector_in[i] - minVal) / step));
      curBin = std::min<int>(curBin, nbins-1);
      hist_out[curBin]++;
    }  */
  }
  
  /** \brief Convert an Eigen column vector to an std vector of the same type
    * \param vector_in input Eigen vector
    * \return return std vector
    * \note Data is copied
    */    
  template <class Scalar>
  inline
  std::vector<Scalar> toStdVector(const Matrix< Scalar, Dynamic, 1> &vector_in)
  {
    std::vector<Scalar> vector_out(vector_in.size());
    for (size_t i=0; i < vector_in.size(); i++)
      vector_out[i] = vector_in[i];
    
    return vector_out;
  }
  
  /** \brief Compute the pdf at a given set of values using normal distribution
    * \param x values
    * \param mu distribution mean
    * \param sigma distribution standard deviation
    * \return pdf values of the input values
    */  
  template <typename Scalar>  
  inline
  Eigen::Matrix< Scalar, Eigen::Dynamic, 1 > normpdf(const Eigen::Matrix< Scalar, Eigen::Dynamic, 1 > x, const Scalar mu, const Scalar sigma)
  {
    Eigen::Matrix< Scalar, Eigen::Dynamic, 1 > pdf (x.size());
    
    for (size_t i = 0; i < x.size(); i++)
      pdf(i) = std::exp ( - (x(i)-mu)*(x(i)-mu) / (2 * sigma*sigma) ) / (sigma * std::sqrt(2*M_PI));
    
    return pdf;
  }  
  
  /** \brief Generate samples from Multivariate Notmal Distribution
    * \param num_samples number of samples to generate
    * \param mu mean
    * \param covar covariance
    * \return matrix where each column is a sample
    */    
  inline
  Eigen::MatrixXd mvnrnd(const int num_samples, const Eigen::VectorXd &mean, const Eigen::MatrixXd &covar)
  {
    // Check that covariance matrix is square
    if (covar.cols() != covar.rows())
    {
      std::cout << "[Eigen::mvnrnd] covariance matrix must be square\n";
      exit(EXIT_FAILURE);
    }
    
    // Check that covariance and mean have same dimensions
    if (covar.cols() != mean.size())
    {
      std::cout << "[Eigen::mvnrnd] covariance and mean must have same dimensions\n";
      exit(EXIT_FAILURE);
    }
    
    // Check that covariance matrix is symmetric
    if (covar != covar.transpose())
    {
      std::cout << "[Eigen::mvnrnd] covariance matrix must by symmetric\n";
      exit(EXIT_FAILURE);
    }
      
    // Decompose the covariance matrix
    int nDim = mean.size();        
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
    if (eigenSolver.info() != Eigen::Success)
    {
      std::cout << "[Eigen::mvnrnd] Something went wrong in eigen decomposition\n";
      exit(EXIT_FAILURE);        
    }
    // If there are negative eigenvalues then covariance matrix is not positive semidefinite
    else if (eigenSolver.eigenvalues().minCoeff() < 0)
    {
      std::cout << "[Eigen::mvnrnd] Covariance matrix is not positive semidefinite\n";
      exit(EXIT_FAILURE);        
    }
    
    Eigen::MatrixXd transform(nDim, nDim);
    transform = eigenSolver.eigenvectors() 
              * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    
    // Generate random samples ~N(0,1)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> distribution(0.0,1.0);
       
    Eigen::MatrixXd samples(nDim, num_samples);
    for (size_t col = 0; col < num_samples; col++)
      for (size_t i=0; i < nDim; i++)
        samples(i, col) = distribution(gen);
    
    // Transform samples
    samples = transform * samples + mean.replicate(1, num_samples);
      
    return samples;
  }
  
  /** \brief Compute rowwise mean of the sample
    * \param samples matrix containing the samples. Each column is a sample
    * \return mean of the sample
    */
  template <class Scalar>
  inline
  Matrix< Scalar, Dynamic, 1> mean(const Matrix< Scalar, Dynamic, Dynamic> &samples)
  {
    return samples.rowwise().mean();
  }

  /** \brief Compute the covariance of sample.
    * \param samples matrix containing the samples. Each column is a sample
    * \return covariance of the sample
    */
  template <class Scalar>
  inline
  Matrix< Scalar, Dynamic, Dynamic> cov(const Matrix< Scalar, Dynamic, Dynamic> &samples)
  {
//     Eigen::VectorXd sample_mean = Eigen::mean(samples);
//     Eigen::MatrixXd centered = samples - sample_mean.replicate(1, samples.cols());
//     return centered * centered.adjoint() / (samples.cols() - 1);
    Matrix< Scalar, Dynamic, Dynamic> centered = samples.colwise() - samples.rowwise().mean();
    return centered * centered.adjoint() / (samples.cols() - 1);    
  }  
 
  /** \brief Compute an intersection point between a line anf a plane
    * \param[in] line_point1 first point of the line
    * \param[in] line_point2 second point of the line
    * \param[in] plane plane parameters (ax + by + cz + d = 0)
    * \return point where line and plane intersect
    */
  inline
  Eigen::Vector3f linePlaneIntersection(const Eigen::Vector3f &line_point1, const Eigen::Vector3f &line_point2, const Eigen::Vector4f &plane)
  {
    Eigen::Vector3f line_direction = line_point2 - line_point1;
    Eigen::Vector3f plane_normal = Eigen::Vector3f(plane[0], plane[1], plane[2]);
    Eigen::Vector3f plane_point = plane_normal * (-plane[3]);
    float d = plane_normal.dot(plane_point - line_point1) / line_direction.dot(plane_normal);
    
    return line_point1 + d * line_direction;
  }
  
  /** \brief Find a clockwise angle between two 3D vectors
    * \param[in] v1 first vector
    * \param[in] v2 second vector
    * \param[in] normal plane normal
    * \return clockwise angle between two vectors
    * \note assumes right handed coordinate system
    */
  inline
  float vectorAngleSigned (const Eigen::Vector3f &v1, const Eigen::Vector3f &v2, const Eigen::Vector3f &normal)
  {
    float cos = v1.dot(v2);
    float sin = normal.dot(v1.cross(v2));
    return std::atan2(sin, cos);
  }  
  
  /** \brief Get distance between point and a plane
    * \param[in] point point
    * \param[in] plane_coefficients coefficients of the equation of the plane
    * \return distance between point and plane
    * \note distance is signed and depends on the plane normal
    */
  inline
  float pointToPlaneDistance (const Eigen::Vector3f &point, const Eigen::VectorXf &plane_coefficients)
  {
    Eigen::Vector3f planeToPointVector = point - Eigen::Vector3f( - plane_coefficients[3] / plane_coefficients[0], 0.0f, 0.0f);
    Eigen::Vector3f planeNormal = plane_coefficients.head(3);
    return planeToPointVector.dot(planeNormal);
  }

  /** \brief Get distance between point and a plane
    * \param[in] point point
    * \param[in] plane_point  a point on the plane
    * \param[in] plane_normal plane normal
    * \return distance between point and plane
    * \note distance is signed and depends on the plane normal
    */
  inline
  float pointToPlaneDistance (const Eigen::Vector3f &point, const Eigen::Vector3f &plane_point, const Eigen::Vector3f &plane_normal)
  {
    Eigen::Vector3f planeToPointVector = point - plane_point;
    return planeToPointVector.dot(plane_normal);
  }
  
  /** \brief Project point on a plane
    * \param[in] point point to be projected
    * \param[in] plane_coefficients coefficients of the equation of the plane
    * \return point projected onto a plane
    */
  inline
  Eigen::Vector3f projectPointToPlane (const Eigen::Vector3f &point, const Eigen::VectorXf &plane_coefficients)
  {
    Eigen::Vector3f planeToPointVector = point - Eigen::Vector3f( - plane_coefficients[3] / plane_coefficients[0], 0.0f, 0.0f);
    Eigen::Vector3f planeNormal = plane_coefficients.head(3);
    return point - planeNormal * (planeToPointVector.dot(planeNormal));
  }  
  
  /** \brief Project point on a plane
    * \param[in] point point to be projected
    * \param[in] plane_point  a point on the plane
    * \param[in] plane_normal plane normal
    * \return point projected onto a plane
    */
  inline
  Eigen::Vector3f projectPointToPlane (const Eigen::Vector3f &point, const Eigen::Vector3f &plane_point, const Eigen::Vector3f &plane_normal)
  {
    Eigen::Vector3f planeToPointVector = point - plane_point;
    return point - plane_normal * (planeToPointVector.dot(plane_normal));
  }
  
  /** \brief Calculate the remainder of division of two numbers. The remainder is always positive.
   *  \param[in] numer    numenator
   *  \param[in] denom    denominator
   *  \return remainder
   */
  template<class T>
  inline
  T remainder (const T &numer, const T &denom)
  {
    float result = std::fmod(numer, denom);
    if (result < 0)
      result = result + std::abs(denom);
    
    return result;
  }
  
  /** \brief Get counter clockwise difference between two angles (in radians)
    * \param[in] angle_start    start angle
    * \param[in] angle_end      end angle 
    * \return angular distance from start angle to end angle
    */
  inline
  float angleDifferenceCCw (const float &start_angle, const float &end_angle)
  {
    return Eigen::remainder(end_angle - start_angle, static_cast<float>(2 * M_PI));
  }
  
  /** \brief Find a rotation matrix that aligns two vectors. Note that transformation matrix is not unique.
   * \param target_normal target normal
   * \param source_normal source normal
   * \return 3x3 rotation matrix
   * \note: http://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
   */
  template <typename Scalar>
  inline
  Eigen::Matrix<Scalar,3,3> alignVectors(const Eigen::Matrix<Scalar,1,3> source_vector, const Eigen::Matrix<Scalar,1,3> target_vector)
  {
    Eigen::Matrix<Scalar,1,3> source_normal = source_vector / source_vector.norm();
    Eigen::Matrix<Scalar,1,3> target_normal = target_vector / target_vector.norm();
    
    Eigen::Matrix<Scalar,1,3> k = -target_normal.cross(source_normal);           // Unit vector representing the axis of rotation between source and target
    Scalar sinTheta = k.norm();                                                 // Rotation angle sine
    Scalar cosTheta = target_normal.dot(source_normal);                         // Rotation angle cosince
    
    Eigen::Matrix<Scalar,3,3> K;
    K <<    0 , -k(2),  k(1),
          k(2),     0, -k(0),
         -k(1),  k(0),     0;
         
    Eigen::Matrix<Scalar,3,3> R;
    R = Eigen::Matrix<Scalar,3,3>::Identity() + K + (1 - cosTheta) * K * K / sinTheta / sinTheta;
    
    return R;    
    
//     Scalar dotProd                  = target_normal.dot(source_normal);
//     dotProd                         = std::max(std::min(dotProd, static_cast<Scalar>(1.0)), static_cast<Scalar>(-1.0));
//     Scaalr angle                    = std::acos<Scalar>(dotProd);
//     Eigen::Matrix<Scalar,1,3> axis  = source_normal.cross(target_normal);
//     axis.normalize();
//     Eigen::Matrix<Scalar,3,3> R     = Eigen::AngleAxis<Scalar>(angle, axis).toRotationMatrix();
//     return R;
  }
}

#endif // EIGEN_UTILITIES_HPP_