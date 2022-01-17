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
        public int nParticles;
        public int nSprings;
        public List<ClothParticle> particles = new List<ClothParticle>();
        public List<ClothSpring> springs = new List<ClothSpring>();
        public double eps = 0.1; // < 1
        public double iMax = 30;
        void Start()
        {
            Vector<double> pos = Utility.CreateVector3d();
            for (int i = 0; i < numberOfParticleInOneSide; i++) {
                GameObject point = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                point.name = String.Format("point{0}", i);
                point.transform.parent = transform;
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
        }

    }
}
