using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;


namespace Partical {
    // 這是實作 Explicit Euler 的做法
    // 彈簧特性可以參考這個影片：https://www.youtube.com/watch?v=kyQP4t_wOGI&t=531s&ab_channel=Gonkee
    // 跟這個網站：https://www.khanacademy.org/computing/pixar/simulation/hair-simulation-101/v/sim2-fix
    public class ExplicitEuler : MonoBehaviour
    {
        private Cloth cloth;
        private Vector<double> g = Utility.CreateVector3d(0, -9.8, 0);
        private Matrix<double> M;
        private Vector<double> f0;
        void Start()
        {
            cloth = gameObject.AddComponent<Cloth>();
        }

        void Update()
        {
            if (f0 == null) {
                // 初始化參數
                f0 = Utility.CreateVectord3n(cloth.nParticles);
                M = Utility.CreateMatrixd3nx3n(cloth.nParticles, true);
                SetMassMatrix(M);
            }
            // 計算當下的力
            double dt = Time.deltaTime;
            for (int i = 0; i < cloth.nParticles; i++) {
                cloth.particles[i].F = g;
            }
            for (int i = 0; i < cloth.nSprings; i++) {
                ClothParticle pi = cloth.springs[i].p1;
                ClothParticle pj = cloth.springs[i].p2;
                Vector<double> xij = pi.x - pj.x;

                // damping: dij = (vij) * kd
                Vector<double> dij = (pi.v - pj.v) * cloth.springs[i].kd;
                // stiffness: (1 - L / |xij|) * xij * k
                Vector<double> fij = (1 - cloth.springs[i].r / xij.L2Norm()) * xij * cloth.springs[i].ks;
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
            SetFroce0(f0);
            Vector<double> dv = M.Inverse() * f0 * dt;
            // 更新速度及位置
            for (int i = 0; i < cloth.nParticles; i++) {
                if (cloth.particles[i].IsPin) {
                    cloth.particles[i].v.Clear();
                }
                else {
                    cloth.particles[i].v += Utility.GetVector3FromVector(dv, i);
                    cloth.particles[i].x += cloth.particles[i].v * dt;
                }
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

        private void SetMassMatrix(Matrix<double> M)
        {
            M.Clear();
            for (int i = 0; i < cloth.nParticles; i++)
            {
                Utility.PutVector3IntoMatrix(
                    M, i, i, Utility.CreateVector3d(cloth.particles[i].m, cloth.particles[i].m, cloth.particles[i].m));
            }
        }
    }
}