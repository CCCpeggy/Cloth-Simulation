using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Partical
{
    struct ComputeSpring
    {
        public float kd;
        public float ks;
        public float r;
    };
    public class ExplicitEulerGPU : MonoBehaviour
    {
        public ComputeShader clothShader;
        private Cloth cloth;
        private ComputeBuffer xBuffer, vBuffer, fBuffer, pinBuffer, springBuffer;
        private ComputeBuffer colorBuffer;
        ComputeSpring[] computeSpring;
        int[] pin;
        Vector3[] x, v, f, color;
        int forceKernel;
        int positionKernel;
        public Shader shader;
        void Start()
        {
            cloth = gameObject.AddComponent<Cloth>();
        }

        void Update()
        {
            if (x == null) {
                Init();
                for (int i = 0; i < cloth.nParticles; i++)
                {
                    cloth.particles[i].gameObject.GetComponent<MeshRenderer>().material = new Material( shader );
                }
            }
            UpdateDt(Time.deltaTime);
            // for (int i = 0; i < 4; i++) {
            //     UpdateDt(Time.deltaTime/4);
            // }
            // for (int i = 0; i < cloth.nParticles; i++) {
            //     RaycastHit hit; 
            //     ClothParticle clothPartical = cloth.particles[i];
            //     Vector3 x = Utility.ConvertToVector3(cloth.particles[i].x);
            //     if (Physics.Raycast(x, Vector3.down, out hit, 0.01f))
            //     {
            //         Vector3 v = Utility.ConvertToVector3(cloth.particles[i].v);
            //         v = Vector3.Reflect(v, hit.normal);
            //         Utility.CreateVector3d(v.x, v.y, v.z).CopyTo(cloth.particles[i].v);
            //         x = hit.point;
            //         Utility.CreateVector3d(x.x, x.y, x.z).CopyTo(cloth.particles[i].x);
            //         // Debug.DrawRay(transform.position, transform.TransformDirection(Vector3.forward) * hit.distance, Color.yellow);
            //         Debug.Log("Did Hit");
            //     }
            // }
        }

        void UpdateDt(float dt) {
            // x = cloth.particles.Select(x => Utility.ConvertToVector3(x.x)).ToArray();
            // xBuffer.SetData(x);
            // v = cloth.particles.Select(x => Utility.ConvertToVector3(x.v)).ToArray();
            // vBuffer.SetData(v);
            // Euler
            clothShader.SetFloat("dt", dt);
            clothShader.Dispatch(forceKernel, cloth.nParticles / 256 + 1, 1, 1);
            clothShader.Dispatch(positionKernel, cloth.nParticles / 256 + 1, 1, 1);
            xBuffer.GetData(x);
            colorBuffer.GetData(color);
            for (int i = 0; i < cloth.nParticles; i++)
            {
                Utility.CreateVector3d(x[i].x, x[i].y, x[i].z).CopyTo(cloth.particles[i].x);
                cloth.particles[i].gameObject.GetComponent<MeshRenderer>().material.color = new Color(color[i][0], color[i][1], color[i][2]);
            }

            // Vector3[] tmpX = new Vector3[cloth.nParticles];
            // clothShader.SetFloat("dt", dt);
            // clothShader.Dispatch(forceKernel, cloth.nParticles / 256 + 1, 1, 1);
            // clothShader.Dispatch(positionKernel, cloth.nParticles / 256 + 1, 1, 1);
            // xBuffer.GetData(x);
            // for (int i = 0; i < cloth.nParticles; i++)
            // {
            //     tmpX[i] += x[i] / 6;
            // }
            // clothShader.SetFloat("dt", dt / 2);
            // clothShader.Dispatch(forceKernel, cloth.nParticles / 256 + 1, 1, 1);
            // clothShader.Dispatch(positionKernel, cloth.nParticles / 256 + 1, 1, 1);
            // xBuffer.GetData(x);
            // for (int i = 0; i < cloth.nParticles; i++)
            // {
            //     tmpX[i] += x[i] / 3;
            // }
            // clothShader.SetFloat("dt", dt / 2);
            // clothShader.Dispatch(forceKernel, cloth.nParticles / 256 + 1, 1, 1);
            // clothShader.Dispatch(positionKernel, cloth.nParticles / 256 + 1, 1, 1);
            // xBuffer.GetData(x);
            // for (int i = 0; i < cloth.nParticles; i++)
            // {
            //     tmpX[i] += x[i] / 3;
            // }
            // clothShader.SetFloat("dt", dt);
            // clothShader.Dispatch(forceKernel, cloth.nParticles / 256 + 1, 1, 1);
            // clothShader.Dispatch(positionKernel, cloth.nParticles / 256 + 1, 1, 1);
            // xBuffer.GetData(x);
            // for (int i = 0; i < cloth.nParticles; i++)
            // {
            //     tmpX[i] += x[i] / 6;
            // }
            // for (int i = 0; i < cloth.nParticles; i++)
            // {
            //     Utility.CreateVector3d(tmpX[i].x, tmpX[i].y, tmpX[i].z).CopyTo(cloth.particles[i].x);
            // }
        }

        void Init() {
            pin = cloth.particles.Select(x => x.IsPin?1:0).ToArray();
            x = cloth.particles.Select(x => Utility.ConvertToVector3(x.x)).ToArray();
            v = cloth.particles.Select(x => Utility.ConvertToVector3(x.v)).ToArray();
            f = new Vector3[cloth.nParticles];
            color = new Vector3[cloth.nParticles];

            computeSpring = new ComputeSpring[cloth.nParticles * cloth.nParticles];
            for (int i = 0; i < cloth.nSprings; i++)
            {
                int idx1 = cloth.springs[i].p1.index;
                int idx2 = cloth.springs[i].p2.index;
                computeSpring[idx1 * cloth.nParticles + idx2].kd = (float)cloth.springs[i].kd;
                computeSpring[idx1 * cloth.nParticles + idx2].ks = (float)cloth.springs[i].ks;
                computeSpring[idx1 * cloth.nParticles + idx2].r = (float)cloth.springs[i].r;
                computeSpring[idx2 * cloth.nParticles + idx1].kd = (float)cloth.springs[i].kd;
                computeSpring[idx2 * cloth.nParticles + idx1].ks = (float)cloth.springs[i].ks;
                computeSpring[idx2 * cloth.nParticles + idx1].r = (float)cloth.springs[i].r;
            }

            int count = cloth.nParticles;
            xBuffer = new ComputeBuffer(count, 12);
            colorBuffer = new ComputeBuffer(count, 12);
            vBuffer = new ComputeBuffer(count, 12);
            fBuffer = new ComputeBuffer(count, 12);
            pinBuffer = new ComputeBuffer(count, 4);
            springBuffer  = new ComputeBuffer(count * count, 12);
            
            xBuffer.SetData(x);
            colorBuffer.SetData(color);
            vBuffer.SetData(v);
            fBuffer.SetData(f);
            pinBuffer.SetData(pin);
            springBuffer.SetData(computeSpring);

            forceKernel = clothShader.FindKernel("UpdateForce");
            positionKernel = clothShader.FindKernel("UpdatePosition");
            clothShader.SetBuffer(forceKernel, "x", xBuffer);
            clothShader.SetBuffer(forceKernel, "v", vBuffer);
            clothShader.SetBuffer(forceKernel, "f", fBuffer);
            clothShader.SetBuffer(forceKernel, "springs", springBuffer);

            clothShader.SetBuffer(positionKernel, "pin", pinBuffer);
            clothShader.SetBuffer(positionKernel, "x", xBuffer);
            clothShader.SetBuffer(positionKernel, "color", colorBuffer);
            clothShader.SetBuffer(positionKernel, "v", vBuffer);
            clothShader.SetBuffer(positionKernel, "f", fBuffer);

            // clothShader.SetInt("dim", cloth.numberOfParticleInOneSide);
            clothShader.SetInt("nParticals", cloth.nParticles);
            clothShader.SetInt("nSprings", cloth.nSprings);
        }
        void OnDestroy() {
            if (xBuffer != null) {
                xBuffer.Release();
            }
            if (colorBuffer != null) {
                colorBuffer.Release();
            }

            if (vBuffer != null) {
                vBuffer.Release();
            }

            if (fBuffer != null) {
                fBuffer.Release();
            }

            if (pinBuffer != null) {
                pinBuffer.Release();
            }

            if (springBuffer != null) {
                springBuffer.Release();
            }
        }
    }

}