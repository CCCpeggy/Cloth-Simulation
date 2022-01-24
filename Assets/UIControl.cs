using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using TMPro;

public class UIControl : MonoBehaviour
{
    enum Integrator
    {
        IMPLICIT,
        EXPLICIT,
        RK2,
        RK4,
        GPU
    }

    GameObject cloth;
    public GameObject GPU;
    public GameObject MassBar;
    public TMP_Text MassText;
    public GameObject ParticleBar;
    public TMP_Text ParticleText;
    static public int num = 10;
    Integrator integrator;

    private void Start()
    {
        MassText.GetComponent<TMP_Text>().text = MassBar.GetComponent<Slider>().value.ToString();
        num = (int)ParticleBar.GetComponent<Slider>().value;
        ParticleText.GetComponent<TMP_Text>().text = ParticleBar.GetComponent<Slider>().value.ToString();
    }

    public void ImplicitEulerClick()
    {
        if (cloth != null)
            Destroy(cloth);
        cloth = new GameObject();
        cloth.AddComponent<Partical.ImplicitEuler>();
        integrator = Integrator.IMPLICIT;
    }

    public void ExplicitEulerClick()
    {
        if (cloth != null)
            Destroy(cloth);
        cloth = new GameObject();
        cloth.AddComponent<Partical.ExplicitEuler>();
        integrator = Integrator.EXPLICIT;
    }

    public void RK2Click()
    {
        if (cloth != null)
            Destroy(cloth);
        cloth = new GameObject();
        cloth.AddComponent<Partical.ExplicitEuler_RK2>();
        integrator = Integrator.RK2;
    }

    public void RK4Click()
    {
        if (cloth != null)
            Destroy(cloth);
        cloth = new GameObject();
        cloth.AddComponent<Partical.ExplicitEuler_RK4>();
        integrator = Integrator.RK4;
    }

    public void GPUClick()
    {
        if (cloth != null)
            Destroy(cloth);
        cloth = new GameObject();
        cloth.AddComponent<Partical.ExplicitEulerGPU>();
        cloth.GetComponent<Partical.ExplicitEulerGPU>().Sphere = GPU.GetComponent<Partical.ExplicitEulerGPU>().Sphere;
        cloth.GetComponent<Partical.ExplicitEulerGPU>().clothShader = GPU.GetComponent<Partical.ExplicitEulerGPU>().clothShader;
        integrator = Integrator.GPU;
    }

    public void MassChange() 
    {
        cloth.GetComponent<Partical.Cloth>().M = MassBar.GetComponent<Slider>().value;
        MassText.GetComponent<TMP_Text>().text = MassBar.GetComponent<Slider>().value.ToString();
    }

    public void ParticlesChange()
    {
        if (cloth != null)
            Destroy(cloth);
        num = (int)ParticleBar.GetComponent<Slider>().value;
        switch(integrator)
        {
            case Integrator.IMPLICIT:
                cloth = new GameObject();
                cloth.AddComponent<Partical.ImplicitEuler>();
                break;
            case Integrator.EXPLICIT:
                cloth = new GameObject();
                cloth.AddComponent<Partical.ExplicitEuler>();
                break;
            case Integrator.RK2:
                cloth = new GameObject();
                cloth.AddComponent<Partical.ExplicitEuler_RK2>();
                break;
            case Integrator.RK4:
                cloth = new GameObject();
                cloth.AddComponent<Partical.ExplicitEuler_RK4>();
                break;
            case Integrator.GPU:
                cloth = new GameObject();
                cloth.AddComponent<Partical.ExplicitEulerGPU>();
                break;
        }

        ParticleText.GetComponent<TMP_Text>().text = ParticleBar.GetComponent<Slider>().value.ToString();
    }
}
