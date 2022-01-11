using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;


namespace Partical {
    public class ImplicitEuler : MonoBehaviour
    {
        public double eps = 0.1; // < 1
        public double iMax = 30;
        private Cloth cloth;
        private Vector<double> dv;          // null vector is a good choice for initial guess
        private Matrix<double> M;
        private Matrix<double> dfDv;
        private Matrix<double> dfDx;
        private Vector<double> f0;
        private Vector<double> dfDxMultiplyV;
        private Vector<double> g;
        void Start()
        {
            cloth = gameObject.AddComponent<Cloth>();
            cloth.numberOfParticleInOneSide = 10;
            cloth.particleDistance = 2;
        }


        void Update()
        {
            if (dv == null) {
                Debug.Log(cloth.nParticles);
                InitDvVariable();
            }
            for (int i = 0; i < cloth.nParticles; i++) {
                cloth.particles[i].F = cloth.particles[i].m * g;
            }
            for (int i = 0; i < cloth.nSprings; i++) {
                ClothParticle pi = cloth.springs[i].p1;
                ClothParticle pj = cloth.springs[i].p2;
                Vector<double> xij = pi.x - pj.x;
                // (1-L/|xij|)*xij*k
                Vector<double> fij = (1 - cloth.springs[i].r / xij.L2Norm()) * xij * cloth.springs[i].ks;
                // dij = (vij)*kd
                Vector<double> dij = (pi.v - pj.v) * cloth.springs[i].kd;
                pi.F += -fij - dij;
                pj.F += fij + dij;
            }
            // 清除鎖定位置的力
            for (int i = 0; i < cloth.nParticles; i++) {
                if (cloth.particles[i].IsPin) {
                    cloth.particles[i].F.Clear();
                    // cloth.particles[i].v.Clear();
                }
            }
            double dt = Time.deltaTime;
            // for (int i = 0; i < cloth.nParticles; i++) {
            //     if (cloth.particles[i].IsPin){
            //         cloth.particles[i].v.Clear();
            //         cloth.particles[i].F.Clear();
            //     }
            // }
            ComputeJacobians();
            UpdateDv(dt);
            // 清除鎖定位置的速度
            for (int i = 0; i < cloth.nParticles; i++) {
                if (cloth.particles[i].IsPin)
                    cloth.particles[i].v.Clear();
            }
            // for (int i = 0; i < cloth.nParticles; i++) {
            //     if (cloth.particles[i].v.L2Norm() > 100) {
            //         cloth.particles[i].v /= cloth.particles[i].v.L2Norm();
            //         cloth.particles[i].v *= 100;
            //     }
            // }
            // 計算位移
            for (int i = 0; i < cloth.nParticles; i++) {
                cloth.particles[i].x += cloth.particles[i].v * dt;
            }
            // for (int i = 0; i < cloth.nSprings; i++) {
            //     ClothParticle pi = cloth.springs[i].p1;
            //     ClothParticle pj = cloth.springs[i].p2;
            //     Vector<double> xij = pi.x - pj.x;
            //     Debug.Log(String.Format("更新完長度: {0}", xij.L2Norm()));
            // }
        }
        private void InitDvVariable() {
            dv = Utility.CreateVectord3n(cloth.nParticles);
            M = Utility.CreateMatrixd3nx3n(cloth.nParticles, true);
            SetMassMatrix(M);
            dfDv = Utility.CreateMatrixd3nx3n(cloth.nParticles, false);
            dfDx = Utility.CreateMatrixd3nx3n(cloth.nParticles, false);
            f0 = Utility.CreateVectord3n(cloth.nParticles);
            dfDxMultiplyV = Utility.CreateVectord3n(cloth.nParticles);
            g = Utility.CreateVector3d(0, -9.8, 0);
        }
        private void UpdateDv(double dt) {
            dv.Clear();
            // A * dv = b
            SetDfDv(dfDv);
            SetDfDx(dfDx);
            SetFroce0(f0);
            GetDfDxMultiplyV(dfDxMultiplyV);
            Matrix<double> A = M - dt * dfDv - dt * dt * dfDx;  // M - dt*df/dv - dt^2*df/dx
            Vector<double> b = dt * (f0 + dt * dfDxMultiplyV);  // dt*(f0 + dt*df/dx*v0)
            Vector<double> r = b - A * dv;                      // a vector
            Vector<double> d = r.Clone();
            double eps0 = Utility.InnerProduct(r, r);         // dot product, a scalar
            double epsNew = eps0;
            int i;
            for (i = 0; i < iMax && (epsNew > eps * eps0 || epsNew > eps); i++)
            {
                // Debug.Log(String.Format("{0}: {1} > {2}", i, epsNew, eps * eps0));
                Vector<double> q = A * d;
                double alpha = epsNew / Utility.InnerProduct(d, q);
                dv +=  alpha * d;
                r -= alpha * q;
                double epsOld = epsNew;
                epsNew = Utility.InnerProduct(r, r);
                double beta = epsNew / epsOld;
                d = r + beta * d;
            }
            // Debug.Log(String.Format("{0}: {1} <= {2}", i, epsNew, eps * eps0));
            for (i = 0; i < cloth.nParticles; i++) {
                cloth.particles[i].v += Utility.GetVector3FromVector(dv, i);
            }
        }

        private void ComputeJacobians()
        {
            for (int i = 0; i < cloth.nSprings; i++)
            {
                Vector<double> dx = cloth.springs[i].p1.x - cloth.springs[i].p2.x;
                Matrix<double> dxtdx = Utility.OuterProduct(dx, dx);
                Matrix<double> I3x3 = Utility.GetIdentity();


                double dxdxt = Utility.InnerProduct(dx, dx);
                if (dxdxt != 0) dxdxt = 1.0 / dxdxt;
                double l = dx.L2Norm();
                if (l != 0) l = 1.0 / l;

                // { (xij*xij^t)/(xij^t*xij) + [I-(xij*xij^t)/(xij^t*xij)]*(1-L/|xij|) } * k
                cloth.springs[i].Jx = (dxtdx * dxdxt + (I3x3 - dxtdx * dxdxt) * (1 - cloth.springs[i].r * l)) * cloth.springs[i].ks;
                // I * kd
                cloth.springs[i].Jv = Utility.GetIdentity() * cloth.springs[i].kd;
            }
        }

        private void GetDfDxMultiplyV(Vector<double> dfDxMultiplyV)
        {
            dfDxMultiplyV.Clear();
            for (int i = 0; i < cloth.nSprings; i++)
            {
                Vector<double> temp = cloth.springs[i].Jx * (cloth.springs[i].p1.v - cloth.springs[i].p2.v);
                Utility.PutVector3IntoVector(dfDxMultiplyV, cloth.springs[i].p1.index, -temp);
                Utility.PutVector3IntoVector(dfDxMultiplyV, cloth.springs[i].p2.index, temp);
            }
        }

        private void SetMassMatrix(Matrix<double> M)
        {
            M.Clear();
            for (int i = 0; i < cloth.nParticles; i++)
            {
                Utility.PutVector3IntoMatrix(
                    M, i, i, Utility.CreateVector3d(cloth.particles[i].m, cloth.particles[i].m, cloth.particles[i].m));
            }
        }

        private void SetFroce0(Vector<double> f0)
        {
            f0.Clear();
            for (int i = 0; i < cloth.nParticles; i++)
            {
                Utility.PutVector3IntoVector(f0, i, cloth.particles[i].F);
            }
        }

        private void SetDfDx(Matrix<double> dfDx)
        {
            dfDx.Clear();
            for (int i = 0; i < cloth.nSprings; i++)
            {
                Utility.PutMatrix3IntoMatrix(dfDx, cloth.springs[i].p1.index, cloth.springs[i].p2.index, cloth.springs[i].Jx);
                Utility.PutMatrix3IntoMatrix(dfDx, cloth.springs[i].p2.index, cloth.springs[i].p1.index, cloth.springs[i].Jx);
                Utility.PutMatrix3IntoMatrix(dfDx, cloth.springs[i].p1.index, cloth.springs[i].p1.index, -cloth.springs[i].Jx);
                Utility.PutMatrix3IntoMatrix(dfDx, cloth.springs[i].p2.index, cloth.springs[i].p2.index, -cloth.springs[i].Jx);
            }
        }

        private void SetDfDv(Matrix<double> dfDv)
        {
            dfDv.Clear();
            for (int i = 0; i < cloth.nSprings; i++)
            {
                Utility.PutMatrix3IntoMatrix(dfDv, cloth.springs[i].p1.index, cloth.springs[i].p2.index, cloth.springs[i].Jv);
                Utility.PutMatrix3IntoMatrix(dfDv, cloth.springs[i].p2.index, cloth.springs[i].p1.index, cloth.springs[i].Jv);
                Utility.PutMatrix3IntoMatrix(dfDv, cloth.springs[i].p1.index, cloth.springs[i].p1.index, -cloth.springs[i].Jv);
                Utility.PutMatrix3IntoMatrix(dfDv, cloth.springs[i].p2.index, cloth.springs[i].p2.index, -cloth.springs[i].Jv);
            }
        }
    }
}