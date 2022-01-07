using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

namespace Partical
{
    public class OneSpring : MonoBehaviour
    {
        public int numberOfParticleInOneSide = 2;
        public int particleDistance = 5;
        private int nParticles;
        private int nSprings;
        private List<ClothParticle> particles = new List<ClothParticle>();
        private List<ClothSpring> springs = new List<ClothSpring>();
        public double eps = 0.1e-45; // < 1
        public double iMax = 1000;
        void Start()
        {
            Vector<double> pos = Utility.CreateVector3d();
            for (int i = 0; i < numberOfParticleInOneSide; i++) {
                GameObject point = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                point.name = String.Format("point{0}", i);
                point.transform.parent = transform;
                point.AddComponent<Rigidbody>();
                ClothParticle particle = point.AddComponent<ClothParticle>();
                particle.index = i;
                particles.Add(particle);
                pos.CopyTo(particle.x);
                pos[0] += particleDistance;
                pos[1] = 5;
                nParticles++;
            }
            particles[0].IsPin = true;
            for (int i = 0; i < numberOfParticleInOneSide - 1; i++) {
                ClothSpring spring = particles[i].gameObject.AddComponent<ClothSpring>();
                spring.Setup(particles[i], particles[i+1], particleDistance);
                springs.Add(spring);
                nSprings++;
            }
            InitDvVariable();
        }

        void Update()
        {
            for (int i = 0; i < nParticles; i++) {
                particles[i].F = particles[i].m * g;
            }
            for (int i = 0; i < nSprings; i++) {
                ClothParticle pi = springs[i].p1;
                ClothParticle pj = springs[i].p2;
                Vector<double> xij = pi.x - pj.x;
                // (1-L/|xij|)*xij*k
                Vector<double> fij = (1 - springs[i].r / xij.L2Norm()) * xij * springs[i].ks;
                // dij = (vij)*kd
                Vector<double> dij = (pi.v - pj.v) * springs[i].kd;
                pi.F += -fij - dij;
                pj.F += fij + dij;
            }
            // 清除鎖定位置的力
            for (int i = 0; i < nParticles; i++) {
                particles[i].F.Clear();
            }
            double dt = Time.deltaTime;
            ComputeJacobians();
            UpdateDv(dt);
            // 清除鎖定位置的速度
            for (int i = 0; i < nParticles; i++) {
                particles[i].v.Clear();
            }
            // 計算位移
            for (int i = 0; i < nParticles; i++) {
                particles[i].x += particles[i].v * dt;
            }
            // for (int i = 0; i < nSprings; i++) {
            //     ClothParticle pi = springs[i].p1;
            //     ClothParticle pj = springs[i].p2;
            //     Vector<double> xij = pi.x - pj.x;
            //     Debug.Log(String.Format("更新完長度: {0}", xij.L2Norm()));
            // }
        }

        private Vector<double> dv;          // null vector is a good choice for initial guess
        Matrix<double> M;
        Matrix<double> dfDv;
        Matrix<double> dfDx;
        Vector<double> f0;
        Vector<double> dfDxMultiplyV;
        Vector<double> g;
        
        private void InitDvVariable() {
            dv = Utility.CreateVectord3n(nParticles);
            M = Utility.CreateMatrixd3nx3n(nParticles, true);
            SetMassMatrix(M);
            dfDv = Utility.CreateMatrixd3nx3n(nParticles, false);
            dfDx = Utility.CreateMatrixd3nx3n(nParticles, false);
            f0 = Utility.CreateVectord3n(nParticles);
            dfDxMultiplyV = Utility.CreateVectord3n(nParticles);
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
            for (i = 0; i < nParticles; i++) {
                particles[i].v += Utility.GetVector3FromVector(dv, i);
            }
        }

        private void ComputeJacobians()
        {
            for (int i = 0; i < nSprings; i++)
            {
                Vector<double> dx = springs[i].p1.x - springs[i].p2.x;
                Matrix<double> dxtdx = Utility.OuterProduct(dx, dx);
                Matrix<double> I3x3 = Utility.GetIdentity();


                double dxdxt = Utility.InnerProduct(dx, dx);
                if (dxdxt != 0) dxdxt = 1.0 / dxdxt;
                double l = dx.L2Norm();
                if (l != 0) l = 1.0 / l;

                // { (xij*xij^t)/(xij^t*xij) + [I-(xij*xij^t)/(xij^t*xij)]*(1-L/|xij|) } * k
                springs[i].Jx = (dxtdx * dxdxt + (I3x3 - dxtdx * dxdxt) * (1 - springs[i].r * l)) * springs[i].ks;
                // I * kd
                springs[i].Jv = Utility.GetIdentity() * springs[i].kd;
            }
        }

        private void GetDfDxMultiplyV(Vector<double> dfDxMultiplyV)
        {
            dfDxMultiplyV.Clear();
            for (int i = 0; i < nSprings; i++)
            {
                Vector<double> temp = springs[i].Jx * (springs[i].p1.v - springs[i].p2.v);
                Utility.PutVector3IntoVector(dfDxMultiplyV, springs[i].p1.index, -temp);
                Utility.PutVector3IntoVector(dfDxMultiplyV, springs[i].p2.index, temp);
            }
        }

        private void SetMassMatrix(Matrix<double> M)
        {
            M.Clear();
            for (int i = 0; i < nParticles; i++)
            {
                Utility.PutVector3IntoMatrix(
                    M, i, i, Utility.CreateVector3d(particles[i].m, particles[i].m, particles[i].m));
            }
        }

        private void SetFroce0(Vector<double> f0)
        {
            f0.Clear();
            for (int i = 0; i < nParticles; i++)
            {
                Utility.PutVector3IntoVector(f0, i, particles[i].F);
            }
        }

        private void SetDfDx(Matrix<double> dfDx)
        {
            dfDx.Clear();
            for (int i = 0; i < nSprings; i++)
            {
                Utility.PutMatrix3IntoMatrix(dfDx, springs[i].p1.index, springs[i].p2.index, springs[i].Jx);
                Utility.PutMatrix3IntoMatrix(dfDx, springs[i].p2.index, springs[i].p1.index, springs[i].Jx);
                Utility.PutMatrix3IntoMatrix(dfDx, springs[i].p1.index, springs[i].p1.index, -springs[i].Jx);
                Utility.PutMatrix3IntoMatrix(dfDx, springs[i].p2.index, springs[i].p2.index, -springs[i].Jx);
            }
        }

        private void SetDfDv(Matrix<double> dfDv)
        {
            dfDv.Clear();
            for (int i = 0; i < nSprings; i++)
            {
                Utility.PutMatrix3IntoMatrix(dfDv, springs[i].p1.index, springs[i].p2.index, springs[i].Jv);
                Utility.PutMatrix3IntoMatrix(dfDv, springs[i].p2.index, springs[i].p1.index, springs[i].Jv);
                Utility.PutMatrix3IntoMatrix(dfDv, springs[i].p1.index, springs[i].p1.index, -springs[i].Jv);
                Utility.PutMatrix3IntoMatrix(dfDv, springs[i].p2.index, springs[i].p2.index, -springs[i].Jv);
            }
        }
    }
}
