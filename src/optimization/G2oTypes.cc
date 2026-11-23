/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "optimization/G2oTypes.h"
#include "core/ImuTypes.h"
#include "core/Converter.h"
#include "database/Map.h"

#include <cmath>
#include <fstream>
#include <iostream>

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"

namespace ORB_SLAM3
{

ImuCamPose::ImuCamPose(KeyFrame *pKF):its(0)
{
    // Load IMU pose
    twb = pKF->GetImuPosition().cast<double>();
    Rwb = pKF->GetImuRotation().cast<double>();

    // Load camera poses
    int num_cams;
    if(pKF->mpCamera2)
        num_cams=2;
    else
        num_cams=1;

    tcw.resize(num_cams);
    Rcw.resize(num_cams);
    tcb.resize(num_cams);
    Rcb.resize(num_cams);
    Rbc.resize(num_cams);
    tbc.resize(num_cams);
    pCamera.resize(num_cams);

    // Left camera
    tcw[0] = pKF->GetTranslation().cast<double>();
    Rcw[0] = pKF->GetRotation().cast<double>();
    tcb[0] = pKF->mImuCalib.mTcb.translation().cast<double>();
    Rcb[0] = pKF->mImuCalib.mTcb.rotationMatrix().cast<double>();
    Rbc[0] = Rcb[0].transpose();
    tbc[0] = pKF->mImuCalib.mTbc.translation().cast<double>();
    pCamera[0] = pKF->mpCamera;
    bf = pKF->mbf * ExtrinsicDrift::BaselineScale();

    if(num_cams>1)
    {
        Eigen::Matrix4d Trl = pKF->GetRelativePoseTrl().matrix().cast<double>();
        Rcw[1] = Trl.block<3,3>(0,0) * Rcw[0];
        tcw[1] = Trl.block<3,3>(0,0) * tcw[0] + Trl.block<3,1>(0,3);
        tcb[1] = Trl.block<3,3>(0,0) * tcb[0] + Trl.block<3,1>(0,3);
        Rcb[1] = Trl.block<3,3>(0,0) * Rcb[0];
        Rbc[1] = Rcb[1].transpose();
        tbc[1] = -Rbc[1] * tcb[1];
        pCamera[1] = pKF->mpCamera2;
    }

    // For posegraph 4DoF
    Rwb0 = Rwb;
    DR.setIdentity();
}

ImuCamPose::ImuCamPose(Frame *pF):its(0)
{
    // Load IMU pose
    twb = pF->GetImuPosition().cast<double>();
    Rwb = pF->GetImuRotation().cast<double>();

    // Load camera poses
    int num_cams;
    if(pF->mpCamera2)
        num_cams=2;
    else
        num_cams=1;

    tcw.resize(num_cams);
    Rcw.resize(num_cams);
    tcb.resize(num_cams);
    Rcb.resize(num_cams);
    Rbc.resize(num_cams);
    tbc.resize(num_cams);
    pCamera.resize(num_cams);

    // Left camera
    tcw[0] = pF->GetPose().translation().cast<double>();
    Rcw[0] = pF->GetPose().rotationMatrix().cast<double>();
    tcb[0] = pF->mImuCalib.mTcb.translation().cast<double>();
    Rcb[0] = pF->mImuCalib.mTcb.rotationMatrix().cast<double>();
    Rbc[0] = Rcb[0].transpose();
    tbc[0] = pF->mImuCalib.mTbc.translation().cast<double>();
    pCamera[0] = pF->mpCamera;
    bf = pF->mbf * ExtrinsicDrift::BaselineScale();

    if(num_cams>1)
    {
        Eigen::Matrix4d Trl = pF->GetRelativePoseTrl().matrix().cast<double>();
        Rcw[1] = Trl.block<3,3>(0,0) * Rcw[0];
        tcw[1] = Trl.block<3,3>(0,0) * tcw[0] + Trl.block<3,1>(0,3);
        tcb[1] = Trl.block<3,3>(0,0) * tcb[0] + Trl.block<3,1>(0,3);
        Rcb[1] = Trl.block<3,3>(0,0) * Rcb[0];
        Rbc[1] = Rcb[1].transpose();
        tbc[1] = -Rbc[1] * tcb[1];
        pCamera[1] = pF->mpCamera2;
    }

    // For posegraph 4DoF
    Rwb0 = Rwb;
    DR.setIdentity();
}

ImuCamPose::ImuCamPose(Eigen::Matrix3d &_Rwc, Eigen::Vector3d &_twc, KeyFrame* pKF): its(0)
{
    // This is only for posegrpah, we do not care about multicamera
    tcw.resize(1);
    Rcw.resize(1);
    tcb.resize(1);
    Rcb.resize(1);
    Rbc.resize(1);
    tbc.resize(1);
    pCamera.resize(1);

    tcb[0] = pKF->mImuCalib.mTcb.translation().cast<double>();
    Rcb[0] = pKF->mImuCalib.mTcb.rotationMatrix().cast<double>();
    Rbc[0] = Rcb[0].transpose();
    tbc[0] = pKF->mImuCalib.mTbc.translation().cast<double>();
    twb = _Rwc * tcb[0] + _twc;
    Rwb = _Rwc * Rcb[0];
    Rcw[0] = _Rwc.transpose();
    tcw[0] = -Rcw[0] * _twc;
    pCamera[0] = pKF->mpCamera;
    bf = pKF->mbf * ExtrinsicDrift::BaselineScale();

    // For posegraph 4DoF
    Rwb0 = Rwb;
    DR.setIdentity();
}

void ImuCamPose::SetParam(const std::vector<Eigen::Matrix3d> &_Rcw, const std::vector<Eigen::Vector3d> &_tcw, const std::vector<Eigen::Matrix3d> &_Rbc,
              const std::vector<Eigen::Vector3d> &_tbc, const double &_bf)
{
    Rbc = _Rbc;
    tbc = _tbc;
    Rcw = _Rcw;
    tcw = _tcw;
    const int num_cams = Rbc.size();
    Rcb.resize(num_cams);
    tcb.resize(num_cams);

    for(int i=0; i<tcb.size(); i++)
    {
        Rcb[i] = Rbc[i].transpose();
        tcb[i] = -Rcb[i]*tbc[i];
    }
    Rwb = Rcw[0].transpose()*Rcb[0];
    twb = Rcw[0].transpose()*(tcb[0]-tcw[0]);

    bf = _bf;
}

Eigen::Vector2d ImuCamPose::Project(const Eigen::Vector3d &Xw, int cam_idx) const
{
    Eigen::Vector3d Xc = Rcw[cam_idx] * Xw + tcw[cam_idx];

    return pCamera[cam_idx]->project(Xc);
}

Eigen::Vector3d ImuCamPose::ProjectStereo(const Eigen::Vector3d &Xw, int cam_idx) const
{
    Eigen::Vector3d Pc = Rcw[cam_idx] * Xw + tcw[cam_idx];
    Eigen::Vector3d pc;
    double invZ = 1/Pc(2);
    pc.head(2) = pCamera[cam_idx]->project(Pc);
    pc(2) = pc(0) - bf*invZ;
    return pc;
}

bool ImuCamPose::isDepthPositive(const Eigen::Vector3d &Xw, int cam_idx) const
{
    return (Rcw[cam_idx].row(2) * Xw + tcw[cam_idx](2)) > 0.0;
}

void ImuCamPose::Update(const double *pu)
{
    Eigen::Vector3d ur, ut;
    ur << pu[0], pu[1], pu[2];
    ut << pu[3], pu[4], pu[5];

    // Update body pose
    twb += Rwb * ut;
    Rwb = Rwb * ExpSO3(ur);

    // Normalize rotation after 5 updates
    its++;
    if(its>=3)
    {
        NormalizeRotation(Rwb);
        its=0;
    }

    // Update camera poses
    const Eigen::Matrix3d Rbw = Rwb.transpose();
    const Eigen::Vector3d tbw = -Rbw * twb;

    for(int i=0; i<pCamera.size(); i++)
    {
        Rcw[i] = Rcb[i] * Rbw;
        tcw[i] = Rcb[i] * tbw + tcb[i];
    }

}

void ImuCamPose::UpdateW(const double *pu)
{
    Eigen::Vector3d ur, ut;
    ur << pu[0], pu[1], pu[2];
    ut << pu[3], pu[4], pu[5];


    const Eigen::Matrix3d dR = ExpSO3(ur);
    DR = dR * DR;
    Rwb = DR * Rwb0;
    // Update body pose
    twb += ut;

    // Normalize rotation after 5 updates
    its++;
    if(its>=5)
    {
        DR(0,2) = 0.0;
        DR(1,2) = 0.0;
        DR(2,0) = 0.0;
        DR(2,1) = 0.0;
        NormalizeRotation(DR);
        its = 0;
    }

    // Update camera pose
    const Eigen::Matrix3d Rbw = Rwb.transpose();
    const Eigen::Vector3d tbw = -Rbw * twb;

    for(int i=0; i<pCamera.size(); i++)
    {
        Rcw[i] = Rcb[i] * Rbw;
        tcw[i] = Rcb[i] * tbw+tcb[i];
    }
}

InvDepthPoint::InvDepthPoint(double _rho, double _u, double _v, KeyFrame* pHostKF): u(_u), v(_v), rho(_rho),
    fx(pHostKF->fx), fy(pHostKF->fy), cx(pHostKF->cx), cy(pHostKF->cy), bf(pHostKF->mbf)
{
}

void InvDepthPoint::Update(const double *pu)
{
    rho += *pu;
}


bool VertexPose::read(std::istream& is)
{
    std::vector<Eigen::Matrix<double,3,3> > Rcw;
    std::vector<Eigen::Matrix<double,3,1> > tcw;
    std::vector<Eigen::Matrix<double,3,3> > Rbc;
    std::vector<Eigen::Matrix<double,3,1> > tbc;

    const int num_cams = _estimate.Rbc.size();
    for(int idx = 0; idx<num_cams; idx++)
    {
        for (int i=0; i<3; i++){
            for (int j=0; j<3; j++)
                is >> Rcw[idx](i,j);
        }
        for (int i=0; i<3; i++){
            is >> tcw[idx](i);
        }

        for (int i=0; i<3; i++){
            for (int j=0; j<3; j++)
                is >> Rbc[idx](i,j);
        }
        for (int i=0; i<3; i++){
            is >> tbc[idx](i);
        }

        float nextParam;
        for(size_t i = 0; i < _estimate.pCamera[idx]->size(); i++){
            is >> nextParam;
            _estimate.pCamera[idx]->setParameter(nextParam,i);
        }
    }

    double bf;
    is >> bf;
    _estimate.SetParam(Rcw,tcw,Rbc,tbc,bf);
    updateCache();
    
    return true;
}

bool VertexPose::write(std::ostream& os) const
{
    std::vector<Eigen::Matrix<double,3,3> > Rcw = _estimate.Rcw;
    std::vector<Eigen::Matrix<double,3,1> > tcw = _estimate.tcw;

    std::vector<Eigen::Matrix<double,3,3> > Rbc = _estimate.Rbc;
    std::vector<Eigen::Matrix<double,3,1> > tbc = _estimate.tbc;

    const int num_cams = tcw.size();

    for(int idx = 0; idx<num_cams; idx++)
    {
        for (int i=0; i<3; i++){
            for (int j=0; j<3; j++)
                os << Rcw[idx](i,j) << " ";
        }
        for (int i=0; i<3; i++){
            os << tcw[idx](i) << " ";
        }

        for (int i=0; i<3; i++){
            for (int j=0; j<3; j++)
                os << Rbc[idx](i,j) << " ";
        }
        for (int i=0; i<3; i++){
            os << tbc[idx](i) << " ";
        }

        for(size_t i = 0; i < _estimate.pCamera[idx]->size(); i++){
            os << _estimate.pCamera[idx]->getParameter(i) << " ";
        }
    }

    os << _estimate.bf << " ";

    return os.good();
}


void EdgeMono::linearizeOplus()
{
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
    const g2o::VertexSBAPointXYZ* VPoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

    const Eigen::Matrix3d &Rcw = VPose->estimate().Rcw[cam_idx];
    const Eigen::Vector3d &tcw = VPose->estimate().tcw[cam_idx];
    const Eigen::Vector3d Xc = Rcw*VPoint->estimate() + tcw;
    const Eigen::Vector3d Xb = VPose->estimate().Rbc[cam_idx]*Xc+VPose->estimate().tbc[cam_idx];
    const Eigen::Matrix3d &Rcb = VPose->estimate().Rcb[cam_idx];

    const Eigen::Matrix<double,2,3> proj_jac = VPose->estimate().pCamera[cam_idx]->projectJac(Xc);
    _jacobianOplusXi = -proj_jac * Rcw;

    Eigen::Matrix<double,3,6> SE3deriv;
    double x = Xb(0);
    double y = Xb(1);
    double z = Xb(2);

    SE3deriv << 0.0, z,   -y, 1.0, 0.0, 0.0,
            -z , 0.0, x, 0.0, 1.0, 0.0,
            y ,  -x , 0.0, 0.0, 0.0, 1.0;

    _jacobianOplusXj = proj_jac * Rcb * SE3deriv; // TODO optimize this product
}

void EdgeMonoOnlyPose::linearizeOplus()
{
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);

    const Eigen::Matrix3d &Rcw = VPose->estimate().Rcw[cam_idx];
    const Eigen::Vector3d &tcw = VPose->estimate().tcw[cam_idx];
    const Eigen::Vector3d Xc = Rcw*Xw + tcw;
    const Eigen::Vector3d Xb = VPose->estimate().Rbc[cam_idx]*Xc+VPose->estimate().tbc[cam_idx];
    const Eigen::Matrix3d &Rcb = VPose->estimate().Rcb[cam_idx];

    Eigen::Matrix<double,2,3> proj_jac = VPose->estimate().pCamera[cam_idx]->projectJac(Xc);

    Eigen::Matrix<double,3,6> SE3deriv;
    double x = Xb(0);
    double y = Xb(1);
    double z = Xb(2);
    SE3deriv << 0.0, z,   -y, 1.0, 0.0, 0.0,
            -z , 0.0, x, 0.0, 1.0, 0.0,
            y ,  -x , 0.0, 0.0, 0.0, 1.0;
    _jacobianOplusXi = proj_jac * Rcb * SE3deriv; // symbol different becasue of update mode
}

void EdgeStereo::linearizeOplus()
{
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
    const g2o::VertexSBAPointXYZ* VPoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

    const Eigen::Matrix3d &Rcw = VPose->estimate().Rcw[cam_idx];
    const Eigen::Vector3d &tcw = VPose->estimate().tcw[cam_idx];
    const Eigen::Vector3d Xc = Rcw*VPoint->estimate() + tcw;
    const Eigen::Vector3d Xb = VPose->estimate().Rbc[cam_idx]*Xc+VPose->estimate().tbc[cam_idx];
    const Eigen::Matrix3d &Rcb = VPose->estimate().Rcb[cam_idx];
    const double bf = VPose->estimate().bf;
    const double inv_z2 = 1.0/(Xc(2)*Xc(2));

    Eigen::Matrix<double,3,3> proj_jac;
    proj_jac.block<2,3>(0,0) = VPose->estimate().pCamera[cam_idx]->projectJac(Xc);
    proj_jac.block<1,3>(2,0) = proj_jac.block<1,3>(0,0);
    proj_jac(2,2) += bf*inv_z2;

    _jacobianOplusXi = -proj_jac * Rcw;

    Eigen::Matrix<double,3,6> SE3deriv;
    double x = Xb(0);
    double y = Xb(1);
    double z = Xb(2);

    SE3deriv << 0.0, z,   -y, 1.0, 0.0, 0.0,
            -z , 0.0, x, 0.0, 1.0, 0.0,
            y ,  -x , 0.0, 0.0, 0.0, 1.0;

    _jacobianOplusXj = proj_jac * Rcb * SE3deriv;
}

void EdgeStereoOnlyPose::linearizeOplus()
{
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);

    const Eigen::Matrix3d &Rcw = VPose->estimate().Rcw[cam_idx];
    const Eigen::Vector3d &tcw = VPose->estimate().tcw[cam_idx];
    const Eigen::Vector3d Xc = Rcw*Xw + tcw;
    const Eigen::Vector3d Xb = VPose->estimate().Rbc[cam_idx]*Xc+VPose->estimate().tbc[cam_idx];
    const Eigen::Matrix3d &Rcb = VPose->estimate().Rcb[cam_idx];
    const double bf = VPose->estimate().bf;
    const double inv_z2 = 1.0/(Xc(2)*Xc(2));

    Eigen::Matrix<double,3,3> proj_jac;
    proj_jac.block<2,3>(0,0) = VPose->estimate().pCamera[cam_idx]->projectJac(Xc);
    proj_jac.block<1,3>(2,0) = proj_jac.block<1,3>(0,0);
    proj_jac(2,2) += bf*inv_z2;

    Eigen::Matrix<double,3,6> SE3deriv;
    double x = Xb(0);
    double y = Xb(1);
    double z = Xb(2);
    SE3deriv << 0.0, z,   -y, 1.0, 0.0, 0.0,
            -z , 0.0, x, 0.0, 1.0, 0.0,
            y ,  -x , 0.0, 0.0, 0.0, 1.0;
    _jacobianOplusXi = proj_jac * Rcb * SE3deriv;
}

VertexVelocity::VertexVelocity(KeyFrame* pKF)
{
    setEstimate(pKF->GetVelocity().cast<double>());
}

VertexVelocity::VertexVelocity(Frame* pF)
{
    setEstimate(pF->GetVelocity().cast<double>());
}

VertexGyroBias::VertexGyroBias(KeyFrame *pKF)
{
    setEstimate(pKF->GetGyroBias().cast<double>());
}

VertexGyroBias::VertexGyroBias(Frame *pF)
{
    Eigen::Vector3d bg;
    bg << pF->mImuBias.bwx, pF->mImuBias.bwy,pF->mImuBias.bwz;
    setEstimate(bg);
}

VertexAccBias::VertexAccBias(KeyFrame *pKF)
{
    setEstimate(pKF->GetAccBias().cast<double>());
}

VertexAccBias::VertexAccBias(Frame *pF)
{
    Eigen::Vector3d ba;
    ba << pF->mImuBias.bax, pF->mImuBias.bay,pF->mImuBias.baz;
    setEstimate(ba);
}



EdgeInertial::EdgeInertial(IMU::Preintegrated *pInt):JRg(pInt->JRg.cast<double>()),
    JVg(pInt->JVg.cast<double>()), JPg(pInt->JPg.cast<double>()), JVa(pInt->JVa.cast<double>()),
    JPa(pInt->JPa.cast<double>()), mpInt(pInt), dt(pInt->dT)
{
    // This edge links 6 vertices
    resize(6);
    g << 0, 0, -IMU::GRAVITY_VALUE;

    Matrix9d Info = pInt->C.block<9,9>(0,0).cast<double>().inverse();
    Info = (Info+Info.transpose())/2;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,9,9> > es(Info);
    Eigen::Matrix<double,9,1> eigs = es.eigenvalues();
    for(int i=0;i<9;i++)
        if(eigs[i]<1e-12)
            eigs[i]=0;
    Info = es.eigenvectors()*eigs.asDiagonal()*es.eigenvectors().transpose();
    setInformation(Info);
}




void EdgeInertial::computeError()
{
    // TODO Maybe Reintegrate inertial measurments when difference between linearization point and current estimate is too big
    const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV1= static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyroBias* VG1= static_cast<const VertexGyroBias*>(_vertices[2]);
    const VertexAccBias* VA1= static_cast<const VertexAccBias*>(_vertices[3]);
    const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
    const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);
    const IMU::Bias b1(VA1->estimate()[0],VA1->estimate()[1],VA1->estimate()[2],VG1->estimate()[0],VG1->estimate()[1],VG1->estimate()[2]);
    const Eigen::Matrix3d dR = mpInt->GetDeltaRotation(b1).cast<double>();
    const Eigen::Vector3d dV = mpInt->GetDeltaVelocity(b1).cast<double>();
    const Eigen::Vector3d dP = mpInt->GetDeltaPosition(b1).cast<double>();

    const Eigen::Vector3d er = LogSO3(dR.transpose()*VP1->estimate().Rwb.transpose()*VP2->estimate().Rwb);
    const Eigen::Vector3d ev = VP1->estimate().Rwb.transpose()*(VV2->estimate() - VV1->estimate() - g*dt) - dV;
    const Eigen::Vector3d ep = VP1->estimate().Rwb.transpose()*(VP2->estimate().twb - VP1->estimate().twb
                                                               - VV1->estimate()*dt - g*dt*dt/2) - dP;

    _error << er, ev, ep;
}

void EdgeInertial::linearizeOplus()
{
    const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV1= static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyroBias* VG1= static_cast<const VertexGyroBias*>(_vertices[2]);
    const VertexAccBias* VA1= static_cast<const VertexAccBias*>(_vertices[3]);
    const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
    const VertexVelocity* VV2= static_cast<const VertexVelocity*>(_vertices[5]);
    const IMU::Bias b1(VA1->estimate()[0],VA1->estimate()[1],VA1->estimate()[2],VG1->estimate()[0],VG1->estimate()[1],VG1->estimate()[2]);
    const IMU::Bias db = mpInt->GetDeltaBias(b1);
    Eigen::Vector3d dbg;
    dbg << db.bwx, db.bwy, db.bwz;

    const Eigen::Matrix3d Rwb1 = VP1->estimate().Rwb;
    const Eigen::Matrix3d Rbw1 = Rwb1.transpose();
    const Eigen::Matrix3d Rwb2 = VP2->estimate().Rwb;

    const Eigen::Matrix3d dR = mpInt->GetDeltaRotation(b1).cast<double>();
    const Eigen::Matrix3d eR = dR.transpose()*Rbw1*Rwb2;
    const Eigen::Vector3d er = LogSO3(eR);
    const Eigen::Matrix3d invJr = InverseRightJacobianSO3(er);

    // Jacobians wrt Pose 1
    _jacobianOplus[0].setZero();
     // rotation
    _jacobianOplus[0].block<3,3>(0,0) = -invJr*Rwb2.transpose()*Rwb1; // OK
    _jacobianOplus[0].block<3,3>(3,0) = Sophus::SO3d::hat(Rbw1*(VV2->estimate() - VV1->estimate() - g*dt)); // OK
    _jacobianOplus[0].block<3,3>(6,0) = Sophus::SO3d::hat(Rbw1*(VP2->estimate().twb - VP1->estimate().twb
                                                   - VV1->estimate()*dt - 0.5*g*dt*dt)); // OK
    // translation
    _jacobianOplus[0].block<3,3>(6,3) = -Eigen::Matrix3d::Identity(); // OK

    // Jacobians wrt Velocity 1
    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3,3>(3,0) = -Rbw1; // OK
    _jacobianOplus[1].block<3,3>(6,0) = -Rbw1*dt; // OK

    // Jacobians wrt Gyro 1
    _jacobianOplus[2].setZero();
    _jacobianOplus[2].block<3,3>(0,0) = -invJr*eR.transpose()*RightJacobianSO3(JRg*dbg)*JRg; // OK
    _jacobianOplus[2].block<3,3>(3,0) = -JVg; // OK
    _jacobianOplus[2].block<3,3>(6,0) = -JPg; // OK

    // Jacobians wrt Accelerometer 1
    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3,3>(3,0) = -JVa; // OK
    _jacobianOplus[3].block<3,3>(6,0) = -JPa; // OK

    // Jacobians wrt Pose 2
    _jacobianOplus[4].setZero();
    // rotation
    _jacobianOplus[4].block<3,3>(0,0) = invJr; // OK
    // translation
    _jacobianOplus[4].block<3,3>(6,3) = Rbw1*Rwb2; // OK

    // Jacobians wrt Velocity 2
    _jacobianOplus[5].setZero();
    _jacobianOplus[5].block<3,3>(3,0) = Rbw1; // OK
}

EdgeInertialGS::EdgeInertialGS(IMU::Preintegrated *pInt):JRg(pInt->JRg.cast<double>()),
    JVg(pInt->JVg.cast<double>()), JPg(pInt->JPg.cast<double>()), JVa(pInt->JVa.cast<double>()),
    JPa(pInt->JPa.cast<double>()), mpInt(pInt), dt(pInt->dT)
{
    // This edge links 8 vertices
    resize(8);
    gI << 0, 0, -IMU::GRAVITY_VALUE;

    Matrix9d Info = pInt->C.block<9,9>(0,0).cast<double>().inverse();
    Info = (Info+Info.transpose())/2;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,9,9> > es(Info);
    Eigen::Matrix<double,9,1> eigs = es.eigenvalues();
    for(int i=0;i<9;i++)
        if(eigs[i]<1e-12)
            eigs[i]=0;
    Info = es.eigenvectors()*eigs.asDiagonal()*es.eigenvectors().transpose();
    setInformation(Info);
}



void EdgeInertialGS::computeError()
{
    // TODO Maybe Reintegrate inertial measurments when difference between linearization point and current estimate is too big
    const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV1= static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyroBias* VG= static_cast<const VertexGyroBias*>(_vertices[2]);
    const VertexAccBias* VA= static_cast<const VertexAccBias*>(_vertices[3]);
    const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
    const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);
    const VertexGDir* VGDir = static_cast<const VertexGDir*>(_vertices[6]);
    const VertexScale* VS = static_cast<const VertexScale*>(_vertices[7]);
    const IMU::Bias b(VA->estimate()[0],VA->estimate()[1],VA->estimate()[2],VG->estimate()[0],VG->estimate()[1],VG->estimate()[2]);
    g = VGDir->estimate().Rwg*gI;
    const double s = VS->estimate();
    const Eigen::Matrix3d dR = mpInt->GetDeltaRotation(b).cast<double>();
    const Eigen::Vector3d dV = mpInt->GetDeltaVelocity(b).cast<double>();
    const Eigen::Vector3d dP = mpInt->GetDeltaPosition(b).cast<double>();

    const Eigen::Vector3d er = LogSO3(dR.transpose()*VP1->estimate().Rwb.transpose()*VP2->estimate().Rwb);
    const Eigen::Vector3d ev = VP1->estimate().Rwb.transpose()*(s*(VV2->estimate() - VV1->estimate()) - g*dt) - dV;
    const Eigen::Vector3d ep = VP1->estimate().Rwb.transpose()*(s*(VP2->estimate().twb - VP1->estimate().twb - VV1->estimate()*dt) - g*dt*dt/2) - dP;

    _error << er, ev, ep;
}

void EdgeInertialGS::linearizeOplus()
{
    const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV1= static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyroBias* VG= static_cast<const VertexGyroBias*>(_vertices[2]);
    const VertexAccBias* VA= static_cast<const VertexAccBias*>(_vertices[3]);
    const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
    const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);
    const VertexGDir* VGDir = static_cast<const VertexGDir*>(_vertices[6]);
    const VertexScale* VS = static_cast<const VertexScale*>(_vertices[7]);
    const IMU::Bias b(VA->estimate()[0],VA->estimate()[1],VA->estimate()[2],VG->estimate()[0],VG->estimate()[1],VG->estimate()[2]);
    const IMU::Bias db = mpInt->GetDeltaBias(b);

    Eigen::Vector3d dbg;
    dbg << db.bwx, db.bwy, db.bwz;

    const Eigen::Matrix3d Rwb1 = VP1->estimate().Rwb;
    const Eigen::Matrix3d Rbw1 = Rwb1.transpose();
    const Eigen::Matrix3d Rwb2 = VP2->estimate().Rwb;
    const Eigen::Matrix3d Rwg = VGDir->estimate().Rwg;
    Eigen::MatrixXd Gm = Eigen::MatrixXd::Zero(3,2);
    Gm(0,1) = -IMU::GRAVITY_VALUE;
    Gm(1,0) = IMU::GRAVITY_VALUE;
    const double s = VS->estimate();
    const Eigen::MatrixXd dGdTheta = Rwg*Gm;
    const Eigen::Matrix3d dR = mpInt->GetDeltaRotation(b).cast<double>();
    const Eigen::Matrix3d eR = dR.transpose()*Rbw1*Rwb2;
    const Eigen::Vector3d er = LogSO3(eR);
    const Eigen::Matrix3d invJr = InverseRightJacobianSO3(er);

    // Jacobians wrt Pose 1
    _jacobianOplus[0].setZero();
     // rotation
    _jacobianOplus[0].block<3,3>(0,0) = -invJr*Rwb2.transpose()*Rwb1;
    _jacobianOplus[0].block<3,3>(3,0) = Sophus::SO3d::hat(Rbw1*(s*(VV2->estimate() - VV1->estimate()) - g*dt));
    _jacobianOplus[0].block<3,3>(6,0) = Sophus::SO3d::hat(Rbw1*(s*(VP2->estimate().twb - VP1->estimate().twb
                                                   - VV1->estimate()*dt) - 0.5*g*dt*dt));
    // translation
    _jacobianOplus[0].block<3,3>(6,3) = Eigen::DiagonalMatrix<double,3>(-s,-s,-s);

    // Jacobians wrt Velocity 1
    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3,3>(3,0) = -s*Rbw1;
    _jacobianOplus[1].block<3,3>(6,0) = -s*Rbw1*dt;

    // Jacobians wrt Gyro bias
    _jacobianOplus[2].setZero();
    _jacobianOplus[2].block<3,3>(0,0) = -invJr*eR.transpose()*RightJacobianSO3(JRg*dbg)*JRg;
    _jacobianOplus[2].block<3,3>(3,0) = -JVg;
    _jacobianOplus[2].block<3,3>(6,0) = -JPg;

    // Jacobians wrt Accelerometer bias
    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3,3>(3,0) = -JVa;
    _jacobianOplus[3].block<3,3>(6,0) = -JPa;

    // Jacobians wrt Pose 2
    _jacobianOplus[4].setZero();
    // rotation
    _jacobianOplus[4].block<3,3>(0,0) = invJr;
    // translation
    _jacobianOplus[4].block<3,3>(6,3) = s*Rbw1*Rwb2;

    // Jacobians wrt Velocity 2
    _jacobianOplus[5].setZero();
    _jacobianOplus[5].block<3,3>(3,0) = s*Rbw1;

    // Jacobians wrt Gravity direction
    _jacobianOplus[6].setZero();
    _jacobianOplus[6].block<3,2>(3,0) = -Rbw1*dGdTheta*dt;
    _jacobianOplus[6].block<3,2>(6,0) = -0.5*Rbw1*dGdTheta*dt*dt;

    // Jacobians wrt scale factor
    _jacobianOplus[7].setZero();
    _jacobianOplus[7].block<3,1>(3,0) = Rbw1*(VV2->estimate()-VV1->estimate());
    _jacobianOplus[7].block<3,1>(6,0) = Rbw1*(VP2->estimate().twb-VP1->estimate().twb-VV1->estimate()*dt);
}

EdgePriorPoseImu::EdgePriorPoseImu(ConstraintPoseImu *c)
{
    resize(4);
    Rwb = c->Rwb;
    twb = c->twb;
    vwb = c->vwb;
    bg = c->bg;
    ba = c->ba;
    setInformation(c->H);
}

void EdgePriorPoseImu::computeError()
{
    const VertexPose* VP = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV = static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyroBias* VG = static_cast<const VertexGyroBias*>(_vertices[2]);
    const VertexAccBias* VA = static_cast<const VertexAccBias*>(_vertices[3]);

    const Eigen::Vector3d er = LogSO3(Rwb.transpose()*VP->estimate().Rwb);
    const Eigen::Vector3d et = Rwb.transpose()*(VP->estimate().twb-twb);
    const Eigen::Vector3d ev = VV->estimate() - vwb;
    const Eigen::Vector3d ebg = VG->estimate() - bg;
    const Eigen::Vector3d eba = VA->estimate() - ba;

    _error << er, et, ev, ebg, eba;
}

void EdgePriorPoseImu::linearizeOplus()
{
    const VertexPose* VP = static_cast<const VertexPose*>(_vertices[0]);
    const Eigen::Vector3d er = LogSO3(Rwb.transpose()*VP->estimate().Rwb);
    _jacobianOplus[0].setZero();
    _jacobianOplus[0].block<3,3>(0,0) = InverseRightJacobianSO3(er);
    _jacobianOplus[0].block<3,3>(3,3) = Rwb.transpose()*VP->estimate().Rwb;
    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3,3>(6,0) = Eigen::Matrix3d::Identity();
    _jacobianOplus[2].setZero();
    _jacobianOplus[2].block<3,3>(9,0) = Eigen::Matrix3d::Identity();
    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3,3>(12,0) = Eigen::Matrix3d::Identity();
}

void EdgePriorAcc::linearizeOplus()
{
    // Jacobian wrt bias
    _jacobianOplusXi.block<3,3>(0,0) = Eigen::Matrix3d::Identity();

}

void EdgePriorGyro::linearizeOplus()
{
    // Jacobian wrt bias
    _jacobianOplusXi.block<3,3>(0,0) = Eigen::Matrix3d::Identity();

}

// SO3 FUNCTIONS
Eigen::Matrix3d ExpSO3(const Eigen::Vector3d &w)
{
    return ExpSO3(w[0],w[1],w[2]);
}

Eigen::Matrix3d ExpSO3(const double x, const double y, const double z)
{
    const double d2 = x*x+y*y+z*z;
    const double d = sqrt(d2);
    Eigen::Matrix3d W;
    W << 0.0, -z, y,z, 0.0, -x,-y,  x, 0.0;
    if(d<1e-5)
    {
        Eigen::Matrix3d res = Eigen::Matrix3d::Identity() + W +0.5*W*W;
        return NormalizeRotation(res);
    }
    else
    {
        Eigen::Matrix3d res =Eigen::Matrix3d::Identity() + W*sin(d)/d + W*W*(1.0-cos(d))/d2;
        return NormalizeRotation(res);
    }
}

Eigen::Vector3d LogSO3(const Eigen::Matrix3d &R)
{
    const double tr = R(0,0)+R(1,1)+R(2,2);
    Eigen::Vector3d w;
    w << (R(2,1)-R(1,2))/2, (R(0,2)-R(2,0))/2, (R(1,0)-R(0,1))/2;
    const double costheta = (tr-1.0)*0.5f;
    if(costheta>1 || costheta<-1)
        return w;
    const double theta = acos(costheta);
    const double s = sin(theta);
    if(fabs(s)<1e-5)
        return w;
    else
        return theta*w/s;
}

Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d &v)
{
    return InverseRightJacobianSO3(v[0],v[1],v[2]);
}

Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y, const double z)
{
    const double d2 = x*x+y*y+z*z;
    const double d = sqrt(d2);

    Eigen::Matrix3d W;
    W << 0.0, -z, y,z, 0.0, -x,-y,  x, 0.0;
    if(d<1e-5)
        return Eigen::Matrix3d::Identity();
    else
        return Eigen::Matrix3d::Identity() + W/2 + W*W*(1.0/d2 - (1.0+cos(d))/(2.0*d*sin(d)));
}

Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d &v)
{
    return RightJacobianSO3(v[0],v[1],v[2]);
}

Eigen::Matrix3d RightJacobianSO3(const double x, const double y, const double z)
{
    const double d2 = x*x+y*y+z*z;
    const double d = sqrt(d2);

    Eigen::Matrix3d W;
    W << 0.0, -z, y,z, 0.0, -x,-y,  x, 0.0;
    if(d<1e-5)
    {
        return Eigen::Matrix3d::Identity();
    }
    else
    {
        return Eigen::Matrix3d::Identity() - W*(1.0-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
    }
}

Eigen::Matrix3d Skew(const Eigen::Vector3d &w)
{
    Eigen::Matrix3d W;
    W << 0.0, -w[2], w[1],w[2], 0.0, -w[0],-w[1],  w[0], 0.0;
    return W;
}

void EdgeTagCornerInertial::linearizeOplus()
{
    const auto *vTag = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
    const auto *vPose = static_cast<const VertexPose *>(_vertices[1]);
    const ImuCamPose &imu = vPose->estimate();

    const Eigen::Vector3d Xw = vTag->estimate().map(X_t);
    const Eigen::Matrix3d &Rcw = imu.Rcw[cam_idx];
    const Eigen::Vector3d &tcw = imu.tcw[cam_idx];
    const Eigen::Vector3d Xc = Rcw * Xw + tcw;
    const Eigen::Vector3d Xb = imu.Rbc[cam_idx] * Xc + imu.tbc[cam_idx];
    const Eigen::Matrix3d &Rcb = imu.Rcb[cam_idx];

    GeometricCamera *cam = pCamera ? pCamera : imu.pCamera[cam_idx];
    const Eigen::Matrix<double, 2, 3> proj_jac = cam->projectJac(Xc);

    // MapTag: g2o VertexSE3Expmap 左扰动，e = u - π
    Eigen::Matrix<double, 3, 6> J_tag;
    const double xw = Xw(0), yw = Xw(1), zw = Xw(2);
    J_tag << 0.0, zw, -yw, 1.0, 0.0, 0.0,
            -zw, 0.0,  xw, 0.0, 1.0, 0.0,
             yw, -xw, 0.0, 0.0, 0.0, 1.0;
    _jacobianOplusXi = -proj_jac * Rcw * J_tag;

    // KF: VertexPose 右扰动（与 EdgeMono 相同）
    Eigen::Matrix<double, 3, 6> SE3deriv;
    const double x = Xb(0), y = Xb(1), z = Xb(2);
    SE3deriv << 0.0, z,  -y, 1.0, 0.0, 0.0,
               -z,  0.0,  x, 0.0, 1.0, 0.0,
                y, -x,  0.0, 0.0, 0.0, 1.0;
    _jacobianOplusXj = proj_jac * Rcb * SE3deriv;
}

void EdgeTagCornerPose4DoF::linearizeOplus()
{
    const auto *vPose = static_cast<const VertexPose4DoF *>(_vertices[0]);
    const ImuCamPose &imu = vPose->estimate();
    const Eigen::Matrix3d &Rcw = imu.Rcw[cam_idx];
    const Eigen::Vector3d &tcw = imu.tcw[cam_idx];
    const Eigen::Vector3d Xc = Rcw * Xw + tcw;

    GeometricCamera *cam = imu.pCamera[cam_idx];
    const Eigen::Matrix<double, 2, 3> proj_jac = cam->projectJac(Xc);
    const Eigen::Matrix<double, 2, 3> J_w = proj_jac * Rcw;

    const Eigen::Vector3d rel = Xw - imu.twb;
    const Eigen::Vector3d ez_cross_rel(-rel.y(), rel.x(), 0.0);
    _jacobianOplusXi.col(0) = J_w * ez_cross_rel;
    _jacobianOplusXi.block<2, 3>(0, 1) = J_w;
}

bool ExtrinsicDrift::enabled_ = false;
bool ExtrinsicDrift::factory_set_ = false;
Sophus::SE3d ExtrinsicDrift::Tbc_factory_;
Sophus::SE3d ExtrinsicDrift::Tlr_factory_;
Sophus::SE3d ExtrinsicDrift::Tbc_;
Sophus::SE3d ExtrinsicDrift::Tlr_;
double ExtrinsicDrift::sigma_rot_bc_ = 0.5 * M_PI / 180.0;
double ExtrinsicDrift::sigma_trans_bc_ = 0.005;
double ExtrinsicDrift::sigma_rot_lr_ = 0.3 * M_PI / 180.0;
double ExtrinsicDrift::sigma_trans_lr_ = 0.002;
double ExtrinsicDrift::baseline_scale_ = 1.0;
int ExtrinsicDrift::n_solves_ = 0;

namespace {

class VertexExtrinsicDrift : public g2o::BaseVertex<12, Eigen::Matrix<double, 12, 1>>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexExtrinsicDrift() { setToOriginImpl(); }
    bool read(std::istream&) override { return false; }
    bool write(std::ostream&) const override { return false; }
    void setToOriginImpl() override { _estimate.setZero(); }
    void oplusImpl(const double* update) override
    {
        for(int i = 0; i < 12; ++i)
            _estimate[i] += update[i];
    }
};

Eigen::Matrix<double, 6, 1> ComposeLog(const Sophus::SE3d& T_factory,
                                       const Sophus::SE3d& T_current,
                                       const Eigen::Matrix<double, 6, 1>& inc)
{
    const Sophus::SE3d T = T_factory.inverse() * (T_current * Sophus::SE3d::exp(inc));
    return T.log();
}

class EdgeExtrinsicPrior : public g2o::BaseUnaryEdge<12, Eigen::Matrix<double, 12, 1>, VertexExtrinsicDrift>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeExtrinsicPrior(const Sophus::SE3d& Tbc_factory, const Sophus::SE3d& Tbc,
                       const Sophus::SE3d& Tlr_factory, const Sophus::SE3d& Tlr)
        : Tbc_factory_(Tbc_factory), Tbc_(Tbc), Tlr_factory_(Tlr_factory), Tlr_(Tlr) {}

    void computeError() override
    {
        const auto inc = static_cast<const VertexExtrinsicDrift*>(_vertices[0])->estimate();
        _error.head<6>() = ComposeLog(Tbc_factory_, Tbc_, inc.head<6>());
        _error.tail<6>() = ComposeLog(Tlr_factory_, Tlr_, inc.segment<6>(6));
    }
    bool read(std::istream&) override { return false; }
    bool write(std::ostream&) const override { return false; }

    Sophus::SE3d Tbc_factory_, Tbc_, Tlr_factory_, Tlr_;
};

class EdgeDriftMono : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexExtrinsicDrift>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeDriftMono(const Eigen::Vector3d& Xw, const Sophus::SE3d& Twb, GeometricCamera* cam,
                  bool right, const Sophus::SE3d& Tbc, const Sophus::SE3d& Tlr)
        : Xw_(Xw), Twb_(Twb), cam_(cam), right_(right), Tbc_(Tbc), Tlr_(Tlr) {}

    void computeError() override
    {
        const auto inc = static_cast<const VertexExtrinsicDrift*>(_vertices[0])->estimate();
        const Sophus::SE3d Tbc = Tbc_ * Sophus::SE3d::exp(inc.head<6>());
        const Sophus::SE3d Tlr = Tlr_ * Sophus::SE3d::exp(inc.segment<6>(6));
        Sophus::SE3d Tcw = Tbc.inverse() * Twb_.inverse();
        if(right_)
            Tcw = Tlr.inverse() * Tcw;
        const Eigen::Vector3d Xc = Tcw.rotationMatrix() * Xw_ + Tcw.translation();
        if(Xc.z() <= 1e-6)
        {
            _error.setConstant(1e3);
            return;
        }
        _error = _measurement - cam_->project(Xc);
    }
    bool read(std::istream&) override { return false; }
    bool write(std::ostream&) const override { return false; }

    Eigen::Vector3d Xw_;
    Sophus::SE3d Twb_;
    GeometricCamera* cam_;
    bool right_;
    Sophus::SE3d Tbc_, Tlr_;
};

class EdgeDriftStereo : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexExtrinsicDrift>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeDriftStereo(const Eigen::Vector3d& Xw, const Sophus::SE3d& Twb, GeometricCamera* cam,
                    double fx, const Sophus::SE3d& Tbc, const Sophus::SE3d& Tlr)
        : Xw_(Xw), Twb_(Twb), cam_(cam), fx_(fx), Tbc_(Tbc), Tlr_(Tlr) {}

    void computeError() override
    {
        const auto inc = static_cast<const VertexExtrinsicDrift*>(_vertices[0])->estimate();
        const Sophus::SE3d Tbc = Tbc_ * Sophus::SE3d::exp(inc.head<6>());
        const Sophus::SE3d Tlr = Tlr_ * Sophus::SE3d::exp(inc.segment<6>(6));
        const Sophus::SE3d Tcw = Tbc.inverse() * Twb_.inverse();
        const Eigen::Vector3d Xc = Tcw.rotationMatrix() * Xw_ + Tcw.translation();
        if(Xc.z() <= 1e-6)
        {
            _error.setConstant(1e3);
            return;
        }
        const Eigen::Vector2d uv = cam_->project(Xc);
        const double bf = fx_ * std::max(1e-6, Tlr.translation().norm());
        _error[0] = _measurement[0] - uv[0];
        _error[1] = _measurement[1] - uv[1];
        _error[2] = _measurement[2] - (uv[0] - bf / Xc.z());
    }
    bool read(std::istream&) override { return false; }
    bool write(std::ostream&) const override { return false; }

    Eigen::Vector3d Xw_;
    Sophus::SE3d Twb_;
    GeometricCamera* cam_;
    double fx_;
    Sophus::SE3d Tbc_, Tlr_;
};

double ReadSigma(cv::FileStorage& fs, const char* key, double fallback)
{
    cv::FileNode node = fs[key];
    if(node.empty())
        return fallback;
    const double v = static_cast<double>(node);
    if(!(v > 0.0) || !std::isfinite(v))
    {
        std::cerr << "ERROR: " << key << " must be finite and > 0, got " << v << std::endl;
        exit(1);
    }
    return v;
}

}  // namespace

void ExtrinsicDrift::LoadSettings(const std::string& settingsPath)
{
    cv::FileStorage fs(settingsPath, cv::FileStorage::READ);
    if(!fs.isOpened())
        return;
    cv::FileNode node = fs["Extrinsic.drift_enable"];
    if(node.empty())
    {
        enabled_ = false;
        std::cout << "Extrinsic.drift_enable: 0" << std::endl;
        return;
    }
    const int v = static_cast<int>(node);
    if(v != 0 && v != 1)
    {
        std::cerr << "ERROR: Extrinsic.drift_enable must be 0 or 1, got " << v << std::endl;
        exit(1);
    }
    enabled_ = v == 1;
    sigma_rot_bc_ = ReadSigma(fs, "Extrinsic.imu_cam_rot_sigma_deg", 0.5) * M_PI / 180.0;
    sigma_trans_bc_ = ReadSigma(fs, "Extrinsic.imu_cam_trans_sigma_m", 0.005);
    sigma_rot_lr_ = ReadSigma(fs, "Extrinsic.stereo_rot_sigma_deg", 0.3) * M_PI / 180.0;
    sigma_trans_lr_ = ReadSigma(fs, "Extrinsic.stereo_trans_sigma_m", 0.002);
    std::cout << "Extrinsic.drift_enable: " << (enabled_ ? 1 : 0)
              << " imu_cam_sigma_deg/m: " << (sigma_rot_bc_ * 180.0 / M_PI) << " / " << sigma_trans_bc_
              << " stereo_sigma_deg/m: " << (sigma_rot_lr_ * 180.0 / M_PI) << " / " << sigma_trans_lr_
              << std::endl;
}

bool ExtrinsicDrift::Enabled()
{
    return enabled_;
}

double ExtrinsicDrift::BaselineScale()
{
    return enabled_ ? baseline_scale_ : 1.0;
}

void ExtrinsicDrift::ApplyToTracker(IMU::Calib* calib, Sophus::SE3f& Tlr)
{
    if(!enabled_ || !factory_set_ || !calib)
        return;
    calib->mTbc = Tbc_.cast<float>();
    calib->mTcb = Tbc_.inverse().cast<float>();
    calib->mbIsSet = true;
    Tlr = Tlr_.cast<float>();
}

void ExtrinsicDrift::Optimize(const std::vector<KeyFrame*>& keyframes)
{
    if(!enabled_)
        return;
    KeyFrame* seed = nullptr;
    for(KeyFrame* kf : keyframes)
    {
        if(kf && !kf->isBad() && kf->mImuCalib.mbIsSet)
        {
            seed = kf;
            break;
        }
    }
    if(!seed)
        return;
    if(!factory_set_)
    {
        Tbc_factory_ = seed->mImuCalib.mTbc.cast<double>();
        Tlr_factory_ = seed->GetRelativePoseTlr().cast<double>();
        Tbc_ = Tbc_factory_;
        Tlr_ = Tlr_factory_;
        baseline_scale_ = 1.0;
        if(Tlr_factory_.translation().norm() < 1e-8)
            return;
        factory_set_ = true;
    }

    struct Item
    {
        Eigen::Vector3d Xw;
        Sophus::SE3d Twb;
        Eigen::Vector3d obs;
        int kind = 0;
        GeometricCamera* cam = nullptr;
        double info = 1.0;
        double fx = 0.0;
    };
    std::vector<Item> items;
    items.reserve(4096);
    for(KeyFrame* kf : keyframes)
    {
        if(!kf || kf->isBad() || !kf->mImuCalib.mbIsSet)
            continue;
        const Sophus::SE3d Twb = kf->GetImuPose().cast<double>();
        const std::vector<MapPoint*> points = kf->GetMapPointMatches();
        for(size_t i = 0; i < points.size(); ++i)
        {
            MapPoint* mp = points[i];
            if(!mp || mp->isBad())
                continue;
            Item item;
            item.Xw = mp->GetWorldPos().cast<double>();
            item.Twb = Twb;
            item.fx = kf->fx;
            if(i < kf->mvKeysUn.size() && kf->mvuRight[i] < 0)
            {
                item.kind = 0;
                item.cam = kf->mpCamera;
                item.obs << kf->mvKeysUn[i].pt.x, kf->mvKeysUn[i].pt.y, 0.0;
                item.info = kf->mvInvLevelSigma2[kf->mvKeysUn[i].octave];
                items.push_back(item);
            }
            else if(i < kf->mvKeysUn.size() && kf->mvuRight[i] >= 0)
            {
                item.kind = 2;
                item.cam = kf->mpCamera;
                item.obs << kf->mvKeysUn[i].pt.x, kf->mvKeysUn[i].pt.y, kf->mvuRight[i];
                item.info = kf->mvInvLevelSigma2[kf->mvKeysUn[i].octave];
                items.push_back(item);
            }
            if(kf->NLeft != -1 && i >= static_cast<size_t>(kf->NLeft) &&
               i < static_cast<size_t>(kf->NLeft) + kf->mvKeysRight.size())
            {
                const size_t ir = i - static_cast<size_t>(kf->NLeft);
                Item right = item;
                right.kind = 1;
                right.cam = kf->mpCamera2;
                right.obs << kf->mvKeysRight[ir].pt.x, kf->mvKeysRight[ir].pt.y, 0.0;
                right.info = kf->mvInvLevelSigma2[kf->mvKeysRight[ir].octave];
                items.push_back(right);
            }
        }
    }
    if(items.size() < 30)
        return;
    if(items.size() > 2000)
    {
        const size_t stride = items.size() / 2000;
        std::vector<Item> kept;
        kept.reserve(2000);
        for(size_t i = 0; i < items.size(); i += stride)
            kept.push_back(items[i]);
        items.swap(kept);
    }

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType* linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX* solverPtr = new g2o::BlockSolverX(linearSolver);
    optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(solverPtr));

    VertexExtrinsicDrift* vertex = new VertexExtrinsicDrift();
    vertex->setId(0);
    optimizer.addVertex(vertex);

    Eigen::Matrix<double, 12, 12> priorInfo = Eigen::Matrix<double, 12, 12>::Zero();
    const double irot_bc = 1.0 / (sigma_rot_bc_ * sigma_rot_bc_);
    const double itrans_bc = 1.0 / (sigma_trans_bc_ * sigma_trans_bc_);
    const double irot_lr = 1.0 / (sigma_rot_lr_ * sigma_rot_lr_);
    const double itrans_lr = 1.0 / (sigma_trans_lr_ * sigma_trans_lr_);
    for(int i = 0; i < 3; ++i)
    {
        priorInfo(i, i) = itrans_bc;
        priorInfo(3 + i, 3 + i) = irot_bc;
        priorInfo(6 + i, 6 + i) = itrans_lr;
        priorInfo(9 + i, 9 + i) = irot_lr;
    }
    EdgeExtrinsicPrior* prior = new EdgeExtrinsicPrior(Tbc_factory_, Tbc_, Tlr_factory_, Tlr_);
    prior->setVertex(0, vertex);
    prior->setInformation(priorInfo);
    optimizer.addEdge(prior);

    const double thMono = std::sqrt(5.991);
    const double thStereo = std::sqrt(7.815);
    for(const Item& item : items)
    {
        if(item.kind == 2)
        {
            EdgeDriftStereo* edge = new EdgeDriftStereo(item.Xw, item.Twb, item.cam, item.fx, Tbc_, Tlr_);
            edge->setVertex(0, vertex);
            edge->setMeasurement(item.obs);
            edge->setInformation(Eigen::Matrix3d::Identity() * item.info);
            g2o::RobustKernelHuber* kernel = new g2o::RobustKernelHuber;
            kernel->setDelta(thStereo);
            edge->setRobustKernel(kernel);
            optimizer.addEdge(edge);
        }
        else
        {
            EdgeDriftMono* edge = new EdgeDriftMono(item.Xw, item.Twb, item.cam, item.kind == 1, Tbc_, Tlr_);
            edge->setVertex(0, vertex);
            edge->setMeasurement(item.obs.head<2>());
            edge->setInformation(Eigen::Matrix2d::Identity() * item.info);
            g2o::RobustKernelHuber* kernel = new g2o::RobustKernelHuber;
            kernel->setDelta(thMono);
            edge->setRobustKernel(kernel);
            optimizer.addEdge(edge);
        }
    }

    optimizer.initializeOptimization();
    optimizer.optimize(8);
    const Eigen::Matrix<double, 12, 1> inc = vertex->estimate();
    if(!inc.allFinite())
        return;

    Tbc_ = Tbc_ * Sophus::SE3d::exp(inc.head<6>());
    Tlr_ = Tlr_ * Sophus::SE3d::exp(inc.segment<6>(6));
    const double base = Tlr_factory_.translation().norm();
    baseline_scale_ = base > 1e-8 ? Tlr_.translation().norm() / base : 1.0;

    Map* map = seed->GetMap();
    if(map)
    {
        const Sophus::SE3f Tbc = Tbc_.cast<float>();
        const Sophus::SE3f Tlr = Tlr_.cast<float>();
        for(KeyFrame* kf : map->GetAllKeyFrames())
        {
            if(!kf || !kf->mImuCalib.mbIsSet)
                continue;
            kf->SetRelativePoseTlr(Tlr);
            kf->ApplyBodyCameraExtrinsic(Tbc);
        }
    }

    ++n_solves_;
    const Eigen::Matrix<double, 6, 1> ebc = (Tbc_factory_.inverse() * Tbc_).log();
    const Eigen::Matrix<double, 6, 1> elr = (Tlr_factory_.inverse() * Tlr_).log();
    std::ofstream file("extrinsic_Tbc.txt");
    if(file)
    {
        const Eigen::Matrix4d M = Tbc_.matrix();
        for(int r = 0; r < 4; ++r)
            for(int c = 0; c < 4; ++c)
                file << M(r, c) << (r == 3 && c == 3 ? "" : " ");
        file << "\n";
    }
    if(n_solves_ <= 3 || n_solves_ % 50 == 0)
    {
        std::cout << "[ExtrinsicDrift] solves=" << n_solves_
                  << " imu_cam_dt_mm=" << ebc.head<3>().norm() * 1000.0
                  << " imu_cam_dR_deg=" << ebc.tail<3>().norm() * 180.0 / M_PI
                  << " stereo_dt_mm=" << elr.head<3>().norm() * 1000.0
                  << " stereo_dR_deg=" << elr.tail<3>().norm() * 180.0 / M_PI
                  << " baseline_scale=" << baseline_scale_
                  << std::endl;
    }
}

}
