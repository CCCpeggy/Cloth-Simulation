using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;


namespace Partical
{
    // 包裝彈簧傳進 Compute Shader
    struct ComputeSpring
    {
        public float kd;
        public float ks;
        public float r;
    };
    // 將 Explicit Euler 的內容移植到 GPU 做計算，另外也計算了 Collision
    public class ExplicitEulerGPU : MonoBehaviour
    {
        public ComputeShader clothShader;   // 計算的 shader
        private Cloth cloth;
        private ComputeBuffer xBuffer, vBuffer, fBuffer, pinBuffer, springBuffer, colorBuffer;  // GPU data buffer
        ComputeSpring[] computeSpring;      // 彈簧
        int[] pin;                          // 位置是否固定
        Vector3[] x, v, f, color;           // 位置, 速度, 力, 顏色
        int forceKernel;                    // compute shader 的 kernel 編號
        int positionKernel;                 // compute shader 的 kernel 編號
        void Start()
        {
            cloth = gameObject.AddComponent<Cloth>();
        }

        void Update()
        {
            if (x == null) {
                InitComputeShaderData();
            }
            UpdateDt(Time.deltaTime);
        }

        void UpdateDt(float dt) {
            // x = cloth.particles.Select(x => Utility.ConvertToVector3(x.x)).ToArray();
            // xBuffer.SetData(x);
            // v = cloth.particles.Select(x => Utility.ConvertToVector3(x.v)).ToArray();
            // vBuffer.SetData(v);
            // Euler
            clothShader.SetFloat("dt", dt);
            // 計算力
            clothShader.Dispatch(forceKernel, cloth.nParticles / 256 + 1, 1, 1);
            // 計算位移
            clothShader.Dispatch(positionKernel, cloth.nParticles / 256 + 1, 1, 1);
            // 取得位置、顏色
            xBuffer.GetData(x);
            colorBuffer.GetData(color);
            // 將位置與資料給到 Particle
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

        // 初始化 Compute Shader 的資料
        void InitComputeShaderData() {
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

        // 釋放 Buffer 的資料
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