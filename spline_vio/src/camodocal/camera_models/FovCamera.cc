#include "spline_vio/camodocal/camera_models/FovCamera.h"

#include <cmath>
#include <cstdio>
#include <Eigen/Dense>
#include <iomanip>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "spline_vio/camodocal/gpl/gpl.h"

namespace camodocal
{

FovCamera::Parameters::Parameters()
 : Camera::Parameters(FOV)
 ,m_fx(0.0)
 , m_fy(0.0)
 , m_cx(0.0)
 , m_cy(0.0)
 , m_w(0.0)
{

}

FovCamera::Parameters::Parameters(const std::string& cameraName,
                                      int width, int height,
                                      double fx, double fy, double cx, double cy,
                                      double w)
 : Camera::Parameters(FOV, cameraName, width, height)
 , m_fx(fx)
 , m_fy(fy)
 , m_cx(cx)
 , m_cy(cy)
,  m_w(w)
{
}



double&
FovCamera::Parameters::fx(void)
{
    return m_fx;
}

double&
FovCamera::Parameters::fy(void)
{
    return m_fy;
}

double&
FovCamera::Parameters::cx(void)
{
    return m_cx;
}

double&
FovCamera::Parameters::cy(void)
{
    return m_cy;
}

double&
FovCamera::Parameters::w(void)
{
    return m_w;
}


double
FovCamera::Parameters::fx(void) const
{
    return m_fx;
}

double
FovCamera::Parameters::fy(void) const
{
    return m_fy;
}

double
FovCamera::Parameters::cx(void) const
{
    return m_cx;
}

double
FovCamera::Parameters::cy(void) const
{
    return m_cy;
}

double
FovCamera::Parameters::w(void) const
{
    return m_w;
}

bool
FovCamera::Parameters::readFromYamlFile(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        return false;
    }

    if (!fs["model_type"].isNone())
    {
        std::string sModelType;
        fs["model_type"] >> sModelType;

        if (sModelType.compare("FOV") != 0)
        {
            return false;
        }
    }

    m_modelType = FOV;
    fs["camera_name"] >> m_cameraName;
    m_imageWidth = static_cast<int>(fs["image_width"]);
    m_imageHeight = static_cast<int>(fs["image_height"]);

    cv::FileNode n = fs["distortion_parameters"];
    m_w = static_cast<double>(n["w"]);

    n = fs["projection_parameters"];
    m_fx = static_cast<double>(n["fx"]);
    m_fy = static_cast<double>(n["fy"]);
    m_cx = static_cast<double>(n["cx"]);
    m_cy = static_cast<double>(n["cy"]);

    return true;
}

void
FovCamera::Parameters::writeToYamlFile(const std::string& filename) const
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    fs << "model_type" << "FOV";
    fs << "camera_name" << m_cameraName;
    fs << "image_width" << m_imageWidth;
    fs << "image_height" << m_imageHeight;

    // radial distortion: k1, k2
    // tangential distortion: p1, p2
    fs << "distortion_parameters";
    fs << "{" << "w" << m_w << "}";

    // projection: fx, fy, cx, cy
    fs << "projection_parameters";
    fs << "{" << "fx" << m_fx
              << "fy" << m_fy
              << "cx" << m_cx
              << "cy" << m_cy << "}";

    fs.release();
}

FovCamera::Parameters&
FovCamera::Parameters::operator=(const FovCamera::Parameters& other)
{
    if (this != &other)
    {
        m_modelType = other.m_modelType;
        m_cameraName = other.m_cameraName;
        m_imageWidth = other.m_imageWidth;
        m_imageHeight = other.m_imageHeight;
        m_w = other.m_w;

        m_fx = other.m_fx;
        m_fy = other.m_fy;
        m_cx = other.m_cx;
        m_cy = other.m_cy;
    }

    return *this;
}

std::ostream&
operator<< (std::ostream& out, const FovCamera::Parameters& params)
{
    out << "Camera Parameters:" << std::endl;
    out << "    model_type " << "FOV" << std::endl;
    out << "   camera_name " << params.m_cameraName << std::endl;
    out << "   image_width " << params.m_imageWidth << std::endl;
    out << "  image_height " << params.m_imageHeight << std::endl;

    // radial distortion: k1, k2
    // tangential distortion: p1, p2
    out << "Distortion Parameters" << std::endl;
    out << "            w " << params.m_w << std::endl;

    // projection: fx, fy, cx, cy
    out << "Projection Parameters" << std::endl;
    out << "            fx " << params.m_fx << std::endl
        << "            fy " << params.m_fy << std::endl
        << "            cx " << params.m_cx << std::endl
        << "            cy " << params.m_cy << std::endl;

    return out;
}

FovCamera::FovCamera()
 : m_inv_K11(1.0)
 , m_inv_K13(0.0)
 , m_inv_K22(1.0)
 , m_inv_K23(0.0)
 , m_noDistortion(true)
{

}

FovCamera::FovCamera(const std::string& cameraName,
                             int imageWidth, int imageHeight,
                             double fx, double fy, double cx, double cy,
                            double w)
 : mParameters(cameraName, imageWidth, imageHeight,
               fx, fy, cx, cy,w)
{
    if (w == 0.0)
    {
        m_noDistortion = true;
    }
    else
    {
        m_noDistortion = false;
    }

    // Inverse camera projection matrix parameters
    m_inv_K11 = 1.0 / mParameters.fx();
    m_inv_K13 = -mParameters.cx() / mParameters.fx();
    m_inv_K22 = 1.0 / mParameters.fy();
    m_inv_K23 = -mParameters.cy() / mParameters.fy();
}

FovCamera::FovCamera(const FovCamera::Parameters& params)
 : mParameters(params)
{
    if (mParameters.w() == 0.0)
    {
        m_noDistortion = true;
    }
    else
    {
        m_noDistortion = false;
    }

    // Inverse camera projection matrix parameters
    m_inv_K11 = 1.0 / mParameters.fx();
    m_inv_K13 = -mParameters.cx() / mParameters.fx();
    m_inv_K22 = 1.0 / mParameters.fy();
    m_inv_K23 = -mParameters.cy() / mParameters.fy();
}

Camera::ModelType
FovCamera::modelType(void) const
{
    return mParameters.modelType();
}

const std::string&
FovCamera::cameraName(void) const
{
    return mParameters.cameraName();
}

int
FovCamera::imageWidth(void) const
{
    return mParameters.imageWidth();
}

int
FovCamera::imageHeight(void) const
{
    return mParameters.imageHeight();
}

void
FovCamera::estimateIntrinsics(const cv::Size& boardSize,
                                  const std::vector< std::vector<cv::Point3f> >& objectPoints,
                                  const std::vector< std::vector<cv::Point2f> >& imagePoints) {
  OBSOLETE_IMPL_EXIT();
}



/**
 * \brief Lifts a point from the image plane to the unit sphere
 *
 * \param p image coordinates
 * \param P coordinates of the point on the sphere
 */
void
FovCamera::liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const
{
    liftProjective(p, P);

    P.normalize();
}

/**
 * \brief Lifts a point from the image plane to its projective ray
 *
 * \param p image coordinates
 * \param P coordinates of the projective ray
 */
void
FovCamera::liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const
{
    //=================
    // fisheye FOV model:
    //  r_u = tan(r_d*w)/(2*tan(w/2))
    //================
    double mx_d, my_d, mx_u, my_u;
    double w = mParameters.w();
    // Lift points to normalised plane
    mx_d = m_inv_K11 * p(0) + m_inv_K13;
    my_d = m_inv_K22 * p(1) + m_inv_K23;

    if (m_noDistortion)
    {
        mx_u = mx_d;
        my_u = my_d;
    }
    else
    {
        // Recursive distortion model
        double r_d = sqrt(mx_d*mx_d + my_d*my_d);
        double r_u = tan(r_d * w) / (2.0 * tan(w/2));

        double ratio = r_u / r_d;

        mx_u = mx_d * ratio;
        my_u = my_d * ratio;
    }

    // Obtain a projective ray
    P << mx_u, my_u, 1.0;
}


/**
 * \brief Project a 3D point (\a x,\a y,\a z) to the image plane in (\a u,\a v)
 *
 * \param P 3D point coordinates
 * \param p return value, contains the image point coordinates
 */
void
FovCamera::spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const
{
    OBSOLETE_IMPL_EXIT();
    Eigen::Vector2d p_u, p_d;
    // Project points to the normalised plane
    p_u << P(0) / P(2), P(1) / P(2);

    if (m_noDistortion)
    {
        p_d = p_u;
    }
    else
    {
        // Apply distortion
        Eigen::Vector2d d_u;
        distortion(p_u, d_u);
        p_d = d_u;
    }

    // Apply generalised projection matrix
    p << mParameters.fx() * p_d(0) + mParameters.cx(),
         mParameters.fy() * p_d(1) + mParameters.cy();
}



void
FovCamera::spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
                        Eigen::Matrix<double,2,3>& J) const
{
    // handle singularity
    if (fabs(P[2]) < 1.0e-12) {
        return ;
    }

    // projection
    Eigen::Vector2d imagePointUndistorted;
    const double rz = 1.0 / P[2];
    double rz2 = rz * rz;
    imagePointUndistorted[0] = P[0] * rz;
    imagePointUndistorted[1] = P[1] * rz;

    Eigen::Matrix<double, 2, 3> pointJacobianProjection;
    Eigen::Matrix2Xd intrinsicsJacobianProjection;
    Eigen::Matrix2d distortionJacobian;
    Eigen::Matrix2Xd intrinsicsJacobianDistortion;
    Eigen::Vector2d imagePoint2;


    // get point Jacobian

    distortion(imagePointUndistorted,imagePoint2,distortionJacobian);

    double fu_ = mParameters.fx();
    double fv_ = mParameters.fy();
    double cu_ = mParameters.cx();
    double cv_ = mParameters.cy();



    // compute the point Jacobian in any case

    J(0, 0) = fu_ * distortionJacobian(0, 0) * rz;
    J(0, 1) = fu_ * distortionJacobian(0, 1) * rz;
    J(0, 2) = -fu_
              * (P[0] * distortionJacobian(0, 0)
                 + P[1] * distortionJacobian(0, 1)) * rz2;
    J(1, 0) = fv_ * distortionJacobian(1, 0) * rz;
    J(1, 1) = fv_ * distortionJacobian(1, 1) * rz;
    J(1, 2) = -fv_
              * (P[0] * distortionJacobian(1, 0)
                 + P[1] * distortionJacobian(1, 1)) * rz2;

    // scale and offset
    p[0] = fu_ * imagePoint2[0] + cu_;
    p[1] = fv_ * imagePoint2[1] + cv_;

}


/**
 * \brief Projects an undistorted 2D point p_u to the image plane
 *
 * \param p_u 2D point coordinates
 * \return image point coordinates
 */
void
FovCamera::undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const
{
    OBSOLETE_IMPL_EXIT();
    Eigen::Vector2d p_d;

    if (m_noDistortion)
    {
        p_d = p_u;
    }
    else
    {
        // Apply distortion
        Eigen::Vector2d d_u;
        distortion(p_u, d_u);
        p_d = d_u;
    }

    // Apply generalised projection matrix
    p << mParameters.fx() * p_d(0) + mParameters.cx(),
         mParameters.fy() * p_d(1) + mParameters.cy();
}

/**
 * \brief Apply distortion to input point (from the normalised plane)
 *
 * \param p_u undistorted coordinates of point on the normalised plane
 * \return to obtain the distorted point: p_d = p_u + d_u
 */
void
FovCamera::distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const
{
    double w_ = mParameters.w();
    const double r_u = p_u.norm();

    const double tanwhalf = tan(w_ / 2.);

    const double atan_wrd = atan(2. * tanwhalf * r_u);
    double r_rd;

    if (w_ * w_ < 1e-5) {
        // Limit w_ > 0.
        r_rd = 1.0;
    } else {
        if (r_u * r_u < 1e-5) {
            // Limit r_u > 0.
            r_rd = 2. * tanwhalf / w_;
        } else {
            r_rd = atan_wrd / (r_u * w_);
        }
    }

        d_u = p_u*r_rd;
}

/**
 * \brief Apply distortion to input point (from the normalised plane)
 *        and calculate Jacobian
 *
 * \param p_u undistorted coordinates of point on the normalised plane
 * \return to obtain the distorted point: p_d = p_u + d_u
 */
void
FovCamera::distortion(const Eigen::Vector2d& pointUndistorted, Eigen::Vector2d& pointDistorted,
                          Eigen::Matrix2d& pointJacobian) const
{
    double w_ = mParameters.w();

    const Eigen::Vector2d & y= pointUndistorted;
    const double r_u = y.norm();
    const double r_u_cubed = r_u * r_u * r_u;
    const double tanwhalf = tan(w_ / 2.);
    const double tanwhalfsq = tanwhalf * tanwhalf;
    const double atan_wrd = atan(2. * tanwhalf * r_u);
    double r_rd;

    if (w_ * w_ < 1e-5) {
        // Limit w_ > 0.
        r_rd = 1.0;
    } else {
        if (r_u * r_u < 1e-5) {
            // Limit r_u > 0.
            r_rd = 2. * tanwhalf / w_;
        } else {
            r_rd = atan_wrd / (r_u * w_);
        }
    }

    const double u = y(0);
    const double v = y(1);

    pointDistorted = pointUndistorted*r_rd;

    Eigen::Matrix2d & J = pointJacobian;
    J.setZero();

    if (w_ * w_ < 1e-5) {
        J.setIdentity();
    } else if (r_u * r_u < 1e-5) {
        J.setIdentity();
        // The coordinates get multiplied by an expression not depending on r_u.
        J *= (2. * tanwhalf / w_);
    } else {
        const double duf_du = (atan_wrd) / (w_ * r_u)
                              - (u * u * atan_wrd) / (w_ * r_u_cubed)
                              + (2 * u * u * tanwhalf)
                                / (w_ * (u * u + v * v) * (4 * tanwhalfsq * (u * u + v * v) + 1));
        const double duf_dv = (2 * u * v * tanwhalf)
                              / (w_ * (u * u + v * v) * (4 * tanwhalfsq * (u * u + v * v) + 1))
                              - (u * v * atan_wrd) / (w_ * r_u_cubed);
        const double dvf_du = (2 * u * v * tanwhalf)
                              / (w_ * (u * u + v * v) * (4 * tanwhalfsq * (u * u + v * v) + 1))
                              - (u * v * atan_wrd) / (w_ * r_u_cubed);
        const double dvf_dv = (atan_wrd) / (w_ * r_u)
                              - (v * v * atan_wrd) / (w_ * r_u_cubed)
                              + (2 * v * v * tanwhalf)
                                / (w_ * (u * u + v * v) * (4 * tanwhalfsq * (u * u + v * v) + 1));

        J << duf_du, duf_dv,
                dvf_du, dvf_dv;
    }



}



void
FovCamera::initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale) const
{
    OBSOLETE_IMPL_EXIT();
    cv::Size imageSize(mParameters.imageWidth(), mParameters.imageHeight());

    cv::Mat mapX = cv::Mat::zeros(imageSize, CV_32F);
    cv::Mat mapY = cv::Mat::zeros(imageSize, CV_32F);

    for (int v = 0; v < imageSize.height; ++v)
    {
        for (int u = 0; u < imageSize.width; ++u)
        {
            double mx_u = m_inv_K11 / fScale * u + m_inv_K13 / fScale;
            double my_u = m_inv_K22 / fScale * v + m_inv_K23 / fScale;

            Eigen::Vector3d P;
            P << mx_u, my_u, 1.0;

            Eigen::Vector2d p;
            spaceToPlane(P, p);

            mapX.at<float>(v,u) = p(0);
            mapY.at<float>(v,u) = p(1);
        }
    }

    cv::convertMaps(mapX, mapY, map1, map2, CV_32FC1, false);
}

cv::Mat
FovCamera::initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
                                       float fx, float fy,
                                       cv::Size imageSize,
                                       float cx, float cy,
                                       cv::Mat rmat) const
{
    OBSOLETE_IMPL_EXIT();
    if (imageSize == cv::Size(0, 0))
    {
        imageSize = cv::Size(mParameters.imageWidth(), mParameters.imageHeight());
    }

    cv::Mat mapX = cv::Mat::zeros(imageSize.height, imageSize.width, CV_32F);
    cv::Mat mapY = cv::Mat::zeros(imageSize.height, imageSize.width, CV_32F);

    Eigen::Matrix3f R, R_inv;
    cv::cv2eigen(rmat, R);
    R_inv = R.inverse();

    // assume no skew
    Eigen::Matrix3f K_rect;

    if (cx == -1.0f || cy == -1.0f)
    {
        K_rect << fx, 0, imageSize.width / 2,
                  0, fy, imageSize.height / 2,
                  0, 0, 1;
    }
    else
    {
        K_rect << fx, 0, cx,
                  0, fy, cy,
                  0, 0, 1;
    }

    if (fx == -1.0f || fy == -1.0f)
    {
        K_rect(0,0) = mParameters.fx();
        K_rect(1,1) = mParameters.fy();
    }

    Eigen::Matrix3f K_rect_inv = K_rect.inverse();

    for (int v = 0; v < imageSize.height; ++v)
    {
        for (int u = 0; u < imageSize.width; ++u)
        {
            Eigen::Vector3f xo;
            xo << u, v, 1;

            Eigen::Vector3f uo = R_inv * K_rect_inv * xo;

            Eigen::Vector2d p;
            spaceToPlane(uo.cast<double>(), p);

            mapX.at<float>(v,u) = p(0);
            mapY.at<float>(v,u) = p(1);
        }
    }

    cv::convertMaps(mapX, mapY, map1, map2, CV_32FC1, false);

    cv::Mat K_rect_cv;
    cv::eigen2cv(K_rect, K_rect_cv);
    return K_rect_cv;
}

int
FovCamera::parameterCount(void) const
{
    return 5;
}

const FovCamera::Parameters&
FovCamera::getParameters(void) const
{
    return mParameters;
}

void
FovCamera::setParameters(const FovCamera::Parameters& parameters)
{
    mParameters = parameters;

    if (mParameters.w() == 0.0)
    {
        m_noDistortion = true;
    }
    else
    {
        m_noDistortion = false;
    }

    m_inv_K11 = 1.0 / mParameters.fx();
    m_inv_K13 = -mParameters.cx() / mParameters.fx();
    m_inv_K22 = 1.0 / mParameters.fy();
    m_inv_K23 = -mParameters.cy() / mParameters.fy();
}

void
FovCamera::readParameters(const std::vector<double>& parameterVec)
{
    OBSOLETE_IMPL_EXIT();
}

void
FovCamera::writeParameters(std::vector<double>& parameterVec) const
{

    parameterVec.resize(parameterCount());
    parameterVec.at(0) = mParameters.fx();
    parameterVec.at(1) = mParameters.fy();
    parameterVec.at(2) = mParameters.cx();
    parameterVec.at(3) = mParameters.cy();
    parameterVec.at(4) = mParameters.w();

}

void
FovCamera::writeParametersToYamlFile(const std::string& filename) const
{
    mParameters.writeToYamlFile(filename);
}

std::string
FovCamera::parametersToString(void) const
{
    std::ostringstream oss;
    oss << mParameters;

    return oss.str();
}

}
