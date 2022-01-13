using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

namespace Partical
{
    public class Cloth : MonoBehaviour
    {
        public int numberOfParticleInOneSide = 10;
        public float particleDistance = 2;
        public int nParticles;
        public double speed = 1;
        public int nSprings;
        public List<ClothParticle> particles = new List<ClothParticle>();
        public List<ClothSpring> springs = new List<ClothSpring>();
        void Start()
        {
            Vector<double> pos = Utility.CreateVector3d();
            for (int i = 0; i < numberOfParticleInOneSide; i++) {
                pos[0] = 0;
                for (int j = 0; j < numberOfParticleInOneSide; j++) {
                    GameObject point = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    point.name = String.Format("point{0}", i);
                    point.transform.parent = transform;
                    // point.AddComponent<Rigidbody>();
                    ClothParticle particle = point.AddComponent<ClothParticle>();
                    particle.index = i * numberOfParticleInOneSide + j;
                    particles.Add(particle);
                    pos.CopyTo(particle.x);
                    pos[0] += particleDistance;
                    nParticles++;
                }
                pos[1] -= particleDistance;
            }
            // 水平第一排固定
            // for (int i = 0; i < numberOfParticleInOneSide; i++) {
            //     particles[i].IsPin = true;
            // }
            // 固定左上右上
            particles[0].IsPin = true;
            particles[numberOfParticleInOneSide - 1].IsPin = true;
            // Structural 垂直
            for (int i = 0; i < numberOfParticleInOneSide - 1; i++) {
                for (int j = 0; j < numberOfParticleInOneSide; j++) {
                    int index = i * numberOfParticleInOneSide + j;
                    AddStructuralSpring(index, index + numberOfParticleInOneSide);
                }
            }
            // Structural 水平
            for (int i = 0; i < numberOfParticleInOneSide; i++) {
                for (int j = 0; j < numberOfParticleInOneSide - 1; j++) {
                    int index = i * numberOfParticleInOneSide + j;
                    AddStructuralSpring(index, index + 1);
                }
            }
            // Sheer
            for (int i = 0; i < numberOfParticleInOneSide - 1; i++) {
                for (int j = 0; j < numberOfParticleInOneSide - 1; j++) {
                    int index = i * numberOfParticleInOneSide + j;
                    AddSheerSpring(index, index + numberOfParticleInOneSide + 1);
                    AddSheerSpring(index + 1, index + numberOfParticleInOneSide);
                }
            }
            // Flexion 垂直
            for (int i = 0; i < numberOfParticleInOneSide - 2; i++) {
                for (int j = 0; j < numberOfParticleInOneSide; j++) {
                    int index = i * numberOfParticleInOneSide + j;
                    AddFlexionSpring(index, index + numberOfParticleInOneSide * 2);
                }
            }
            // Flexion 水平
            for (int i = 0; i < numberOfParticleInOneSide; i++) {
                for (int j = 0; j < numberOfParticleInOneSide - 2; j++) {
                    int index = i * numberOfParticleInOneSide + j;
                    AddFlexionSpring(index, index + 2);
                }
            }
        }
        public void Update() {
            double dt = Time.deltaTime * speed;
            // 清除鎖定位置的速度
            // for (int i = 0; i < nParticles; i++) {
            //     if (particles[i].IsPin)
            //         particles[i].v.Clear();
            // }
            // 計算位移
            // for (int i = 0; i < nParticles; i++) {
            //     particles[i].x += particles[i].v * dt;
            // }
            // for (int i = 0; i < cloth.nSprings; i++) {
            //     ClothParticle pi = cloth.springs[i].p1;
            //     ClothParticle pj = cloth.springs[i].p2;
            //     Vector<double> xij = pi.x - pj.x;
            //     Debug.Log(String.Format("更新完長度: {0}", xij.L2Norm()));
            // }
        }

        void AddStructuralSpring(int i, int j) {
            ClothSpring spring = new GameObject().AddComponent<ClothSpring>();
            spring.transform.parent = particles[i].transform;
            spring.Setup(particles[i], particles[j], particleDistance);
            spring.ks = 10;
            spring.kd = 1;
            springs.Add(spring);
            nSprings++;
        }
        double sqrt = Math.Sqrt(2);
        void AddSheerSpring(int i, int j) {
            ClothSpring spring = new GameObject().AddComponent<ClothSpring>();
            spring.transform.parent = particles[i].transform;
            spring.Setup(particles[i], particles[j], particleDistance * sqrt);
            spring.ks = 10;
            spring.kd = 1;
            springs.Add(spring);
            nSprings++;
        }
        void AddFlexionSpring(int i, int j) {
            ClothSpring spring = new GameObject().AddComponent<ClothSpring>();
            spring.transform.parent = particles[i].transform;
            spring.Setup(particles[i], particles[j], particleDistance * 2);
            spring.ks = 10;
            spring.kd = 1;
            springs.Add(spring);
            nSprings++;
        }

        
    }
}
