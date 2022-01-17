using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;


namespace Partical {
    // 這是實作 Implicit Euler 的做法
    // Implicit 參考：https://hugi.scene.org/online/hugi28/hugi%2028%20-%20coding%20corner%20uttumuttu%20implementing%20the%20implicit%20euler%20method%20for%20mass-spring%20systems.htm
    public class ImplicitEuler : MonoBehaviour
    {
        private Cloth cloth;
        private Vector<double> dv;
        private Matrix<double> M;
        private Matrix<double> dfDv;
        private Matrix<double> dfDx;
        private Vector<double> f0;
        private Vector<double> dfDxMultiplyV;
        private Vector<double> g = Utility.CreateVector3d(0, -9.8, 0);
        void Start()
        {
            cloth = gameObject.AddComponent<Cloth>();
        }

        void Update()
        {
            if (dv == null) {
                InitDvVariable();
            }
            // 計算當前的力
            // 計算重力
            for (int i = 0; i < cloth.nParticles; i++) {
                cloth.particles[i].F = g;
            }
            // 計算彈簧的伸縮力跟阻力 
            for (int i = 0; i < cloth.nSprings; i++) {
                ClothParticle pi = cloth.springs[i].p1;
                ClothParticle pj = cloth.springs[i].p2;
                Vector<double> xij = pi.x - pj.x;
                // (1 - L / |xij|) * xij * k
                Vector<double> fij = (1 - cloth.springs[i].r / xij.L2Norm()) * xij * cloth.springs[i].ks;
                // dij = (vij) * kd
                Vector<double> dij = (pi.v - pj.v) * cloth.springs[i].kd;
                pi.F += -fij - dij;
                pj.F += fij + dij;
            }
            // 清除被鎖定位置的粒子的力
            for (int i = 0; i < cloth.nParticles; i++) {
                if (cloth.particles[i].IsPin) {
                    cloth.particles[i].F.Clear();
                }
            }
            double dt = Time.deltaTime;
            // 更新相關的參數，詳細看網站
            ComputeJacobians();
            UpdateV(dt);
            // 利用新的 V，計算新的位置
            for (int i = 0; i < cloth.nParticles; i++) {
                cloth.particles[i].x += cloth.particles[i].v * dt;
            }
        }
        private void InitDvVariable() {
            // 初始化相關的參數的向量戶矩陣
            dv = Utility.CreateVectord3n(cloth.nParticles);
            M = Utility.CreateMatrixd3nx3n(cloth.nParticles, true);
            SetMassMatrix(M);
            dfDv = Utility.CreateMatrixd3nx3n(cloth.nParticles, false);
            dfDx = Utility.CreateMatrixd3nx3n(cloth.nParticles, false);
            f0 = Utility.CreateVectord3n(cloth.nParticles);
            dfDxMultiplyV = Utility.CreateVectord3n(cloth.nParticles);
        }
        private void UpdateV(double dt) {
            dv.Clear();
            // 解 A * dv = b 當中的 dv
            SetDfDv(dfDv);
            SetDfDx(dfDx);
            SetFroce0(f0);
            GetDfDxMultiplyV(dfDxMultiplyV);
            Matrix<double> A = M - dt * dfDv - dt * dt * dfDx;  // M - dt*df/dv - dt^2*df/dx
            Vector<double> b = dt * (f0 + dt * dfDxMultiplyV);  // dt*(f0 + dt*df/dx*v0)

            dv = A.Solve(b);

            // 算得新的速率 (v1 = v0 + dv)
            for (int i = 0; i < cloth.nParticles; i++) {
                if (cloth.particles[i].IsPin)
                    cloth.particles[i].v.Clear();
                else
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