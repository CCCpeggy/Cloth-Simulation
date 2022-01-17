using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;


namespace Partical
{
    // 用 CPU 運算 Runge-Kutta 4
    // Runge-Kutta 主要是在趨近斜率
    // Runge-Kutta 可以參考：https://zh.wikipedia.org/wiki/%E9%BE%99%E6%A0%BC%EF%BC%8D%E5%BA%93%E5%A1%94%E6%B3%95
    // TODO: Runge-Kutta 2，公式：https://mathworld.wolfram.com/Runge-KuttaMethod.html
    public class ExplicitEuler_RK4 : MonoBehaviour
    {
        private Cloth cloth;
        private Vector<double> g = Utility.CreateVector3d(0, -9.8, 0);
        private Matrix<double> M;
        private Vector<double> f0;
        Vector<double> dv, v, x;
        void Start()
        {
            cloth = gameObject.AddComponent<Cloth>();
        }

        void Update()
        {
            if (f0 == null)
            {
                f0 = Utility.CreateVectord3n(cloth.nParticles);
                M = Utility.CreateMatrixd3nx3n(cloth.nParticles, true);
                SetMassMatrix(M);
                dv = Utility.CreateVectord3n(cloth.nParticles);
                x = Utility.CreateVectord3n(cloth.nParticles);
                v = Utility.CreateVectord3n(cloth.nParticles);
            }
            const double n1_6 = 1.0 / 6;
            double dt = Time.deltaTime;

            for (int i = 0; i < cloth.nParticles; i++)
            {
                Utility.PutVector3IntoVector(x, i, cloth.particles[i].x);
                Utility.PutVector3IntoVector(v, i, cloth.particles[i].v);
            }

            // 差別主要在這邊
            dv.Clear();
            // dv = ComputeDv(dt);
            dv += n1_6 * ComputeDv(dt);
            dv += n1_6 * 2 * ComputeDv(dt / 2);
            dv += n1_6 * 2 * ComputeDv(dt / 2);
            dv += n1_6 * ComputeDv(dt);
            for (int i = 0; i < cloth.nParticles; i++)
            {
                if (cloth.particles[i].IsPin)
                    cloth.particles[i].v.Clear();
                else
                    cloth.particles[i].v += Utility.GetVector3FromVector(dv, i);
                cloth.particles[i].x += cloth.particles[i].v * dt;
            }
        }
        private void SetFroce0(Vector<double> f0, List<Vector<double>> f)
        {
            f0.Clear();
            for (int i = 0; i < cloth.nParticles; i++)
            {
                Utility.PutVector3IntoVector(f0, i, f[i]);
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

        // 概念上跟 ExplicitEuler 是一樣的，只是因為不能直接更新 Particle 的參數，所以另外用其他變數暫存
        private Vector<double> ComputeDv(double dt)
        {
            // 當前的 f
            List<Vector<double>> fTmp = new List<Vector<double>>();
            for (int i = 0; i < cloth.nParticles; i++)
            {
                fTmp.Add(g);
            }
            for (int i = 0; i < cloth.nSprings; i++)
            {
                int idx1 = cloth.springs[i].p1.index;
                int idx2 = cloth.springs[i].p2.index;
                Vector<double> xij = Utility.GetVector3FromVector(x, idx1) - Utility.GetVector3FromVector(x, idx2);
                Vector<double> vij = Utility.GetVector3FromVector(v, idx1) - Utility.GetVector3FromVector(v, idx2);

                // dij = (vij) * kd
                Vector<double> dij = vij * cloth.springs[i].kd;
                // (1 - L / |xij|) * xij * k
                Vector<double> fij = (1 - cloth.springs[i].r / xij.L2Norm()) * xij * cloth.springs[i].ks;
                fTmp[idx1] += -fij - dij;
                fTmp[idx2] += fij + dij;
            }
            // 清除鎖定位置的力
            for (int i = 0; i < cloth.nParticles; i++)
            {
                if (cloth.particles[i].IsPin) fTmp[i].Clear();
            }
            SetFroce0(f0, fTmp);
            Vector<double> dv = M.Inverse() * f0 * dt;
            for (int i = 0; i < cloth.nParticles; i++)
            {
                if (cloth.particles[i].IsPin)
                {
                    dv[i * 3] = 0; dv[i * 3 + 1] = 0; dv[i * 3 + 2] = 0;
                }
            }
            v += dv;
            x += v * dt;
            return dv;
        }
    }
}