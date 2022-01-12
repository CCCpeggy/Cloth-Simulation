using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;


namespace Partical {
    public class ExplicitEuler : MonoBehaviour
    {
        private Cloth cloth;
        private Vector<double> g = Utility.CreateVector3d(0, -9.8, 0);
        private Matrix<double> M;
        private Vector<double> f0;
        void Start()
        {
            cloth = gameObject.AddComponent<Cloth>();
            cloth.speed = 0.25;
        }

        void Update()
        {
            if (f0 == null) {
                f0 = Utility.CreateVectord3n(cloth.nParticles);
                M = Utility.CreateMatrixd3nx3n(cloth.nParticles, true);
                SetMassMatrix(M);
            }
            double dt = Time.deltaTime * cloth.speed;
            for (int i = 0; i < cloth.nParticles; i++) {
                cloth.particles[i].F = cloth.particles[i].m * g;
                // cloth.particles[i].F.Clear();
            }
            for (int i = 0; i < cloth.nSprings; i++) {
                ClothParticle pi = cloth.springs[i].p1;
                ClothParticle pj = cloth.springs[i].p2;
                Vector<double> xij = pi.x - pj.x;

                Vector<double> dij = (pi.v - pj.v) * cloth.springs[i].kd;
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
            for (int i = 0; i < cloth.nParticles; i++) {
                cloth.particles[i].v += Utility.GetVector3FromVector(dv, i);
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