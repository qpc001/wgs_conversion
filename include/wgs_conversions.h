/*
 * Header file for wgs_conversions.cpp
 *
 * Dan Pierce
 * 2017-03-13
 */

#ifndef WGS_CONVERTION_H
#define WGS_CONVERTION_H
#include <iostream>
#include <math.h>
#include "Geocentric/LocalCartesian.hpp"

typedef double array_type[3];
typedef double matrix_type[3][3];

/*! Primary class for wgs_conversions */
class WgsConversions{

  public:
    WgsConversions();
    ~WgsConversions();

    enum Compute_Method{selfFunc,geoLib};

    ////////////////////////! 实现方法1： 自行函数实现////////////////////////////////
    /*! Convert to/from ENU/LLA (requires reference LLA) */
    bool enu2lla(double lla[3], const double enu[3], const double ref_lla[3]);
    bool lla2enu(double enu[3], const double lla[3], const double ref_lla[3]);

    /*! Convert to/from ECEF/LLA */
    bool xyz2lla(double lla[3], const double xyz[3]);
    bool lla2xyz(double xyz[3], const double lla[3]);

    /*! Convert to/from ENU/ECEF (requires reference LLA) */
    bool enu2xyz(double xyz[3], const double enu[3], const double ref_lla[3]);
    bool xyz2enu(double enu[3], const double xyz[3], const double ref_lla[3]);

    /*! Convert velocities (or delta positions) to/from ENU/ECEF (requires reference LLA) */
    void enu2xyz_vel(double xyz_vel[3], const double enu_vel[3], const double ref_lla[3]);
    void xyz2enu_vel(double enu_vel[3], const double xyz_vel[3], const double ref_lla[3]);

    /*! Convert position/velocity covariance to/from ENU/ECEF (requires reference LLA) */
    void enu2xyz_cov(double xyz_Cov[3][3], const double enu_Cov[3][3], const double ref_lla[3]);
    void xyz2enu_cov(double enu_Cov[3][3], const double xyz_Cov[3][3], const double ref_lla[3]);
    
    void enu2xyz_cov(double xyz_cov[9], const double enu_cov[9], const double ref_lla[3]);
    void xyz2enu_cov(double enu_cov[9], const double xyz_cov[9], const double ref_lla[3]);


    /////////////////////////! 实现方法2： 使用geo库////////////////////////////////
    void InitOriginPosition(double latitude,double longitude,double altitude) {
        if(origin_position_inited){
           return;
        }
        geo_converter.Reset(latitude, longitude, altitude);
        origin_position_inited = true;
    }

    /**
     * @brief lla2enu
     * 输入经纬度，得到ENU坐标（相对于原点的）
     */
    void lla2enu(double enu[3], const double lla[3]) {
        if (!origin_position_inited) {
            std::cout << "[Warnning] GeoConverter has not set origin position"<<std::endl;
        }

        geo_converter.Forward(lla[0], lla[1], lla[2], enu[0], enu[1], enu[2]);
    }


  private:

    /*! Rotation matrix about a given axis */
    void rot(double R[3][3], const double angle, const int axis);
    
    /*! Rotation matrix from ECEF to ENU frame */
    void rot3d(double R[3][3], const double reflat, const double reflon);
    
    /*! Multiply 3x3 matrix times another 3x3 matrix C=AB */
    void matrixMultiply(double C[3][3], const double A[3][3], const double B[3][3]);
    
    /*! Multiply 3x3 matrix times a 3x1 vector c=Ab */
    void matrixMultiply(double c[3], const double A[3][3], const double b[3]);
    
    /*! Transpose a 3x3 matrix At = A' */
    void transposeMatrix(double At[3][3], const double A[3][3]);

    //! 实现方法2： 使用geo库
    bool origin_position_inited = false;
    GeographicLib::LocalCartesian geo_converter;
    Compute_Method method_=Compute_Method::geoLib;
    
};
#endif
