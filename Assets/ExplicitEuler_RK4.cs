using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;


namespace Partical
{
    public class ExplicitEuler_RK4 : MonoBehaviour
    {
        private Cloth cloth;
        private Vector<double> g = Utility.CreateVector3d(0, -9.8, 0);
        private Matrix<double> M;
        private Vector<double> f0;
        Vector<double> v;
        Vector<double> x;
        void Start()
        {
            cloth = gameObject.AddComponent<Cloth>();
            cloth.speed = 0.3;
        }

        void Update()
        {
            if (f0 == null)
            {
                f0 = Utility.CreateVectord3n(cloth.nParticles);
                M = Utility.CreateMatrixd3nx3n(cloth.nParticles, true);
                SetMassMatrix(M);
                x = Utility.CreateVectord3n(cloth.nParticles);
                v = Utility.CreateVectord3n(cloth.nParticles);
            }
            const double n1_6 = 1.0 / 6;
            double dt = Time.deltaTime * cloth.speed;

            for (int i = 0; i < cloth.nParticles; i++)
            {
                Utility.PutVector3IntoVector(x, i, cloth.particles[i].x);
                Utility.PutVector3IntoVector(v, i, cloth.particles[i].v);
            }

            Get(dt);
            for (int i = 0; i < cloth.nParticles; i++)
                cloth.particles[i].x = n1_6 * Utility.GetVector3FromVector(x, i);
            Get(dt / 2);
            for (int i = 0; i < cloth.nParticles; i++) 
                cloth.particles[i].x += n1_6 * 2 * Utility.GetVector3FromVector(x, i);
            Get(dt / 2);
            for (int i = 0; i < cloth.nParticles; i++) 
                cloth.particles[i].x += n1_6 * 2 * Utility.GetVector3FromVector(x, i);
            Get(dt);
            for (int i = 0; i < cloth.nParticles; i++) 
                cloth.particles[i].x += n1_6 * Utility.GetVector3FromVector(x, i);
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

        private void Get(double dt)
        {
            List<Vector<double>> fTmp = new List<Vector<double>>();
            for (int i = 0; i < cloth.nParticles; i++)
            {
                fTmp.Add(cloth.particles[i].m * g);
            }
            for (int i = 0; i < cloth.nSprings; i++)
            {
                int idx1 = cloth.springs[i].p1.index;
                int idx2 = cloth.springs[i].p2.index;
                Vector<double> xij = Utility.GetVector3FromVector(x, idx1) - Utility.GetVector3FromVector(x, idx2);
                Vector<double> vij = Utility.GetVector3FromVector(v, idx1) - Utility.GetVector3FromVector(v, idx2);

                Vector<double> dij = vij * cloth.springs[i].kd;
                Vector<double> fij = (1 - cloth.springs[i].r / xij.L2Norm()) * xij * cloth.springs[i].ks;
                fTmp[idx1] += -fij - dij;
                fTmp[idx2] += fij + dij;
            }
            // 清除鎖定位置的力
            for (int i = 0; i < cloth.nParticles; i++)
            {
                if (cloth.particles[i].IsPin)
                {
                    fTmp[i].Clear();
                }
            }
            SetFroce0(f0, fTmp);
            Vector<double> dv = M.Inverse() * f0 * dt;
            for (int i = 0; i < cloth.nParticles; i++)
            {
                if (cloth.particles[i].IsPin)
                {
                    dv[i * 3] = 0;
                    dv[i * 3 + 1] = 0;
                    dv[i * 3 + 2] = 0;
                }
            }
            v += dv;
            x += dv * dt;
        }
    }
}