using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using Newtonsoft;

[Serializable]
public class MapSettings {

    public List<List<double>> bounding_polygon;

    // TODO: Improve this to have a list of those
    public List<List<double>> obstacle_1;
    public List<List<double>> obstacle_2;
    public List<List<double>> obstacle_3;
    public List<List<double>> obstacle_4;
    public List<List<double>> obstacle_5;
    public List<List<double>> obstacle_6;
    public List<List<double>> obstacle_7;
    public List<List<double>> obstacle_8;
    public List<List<double>> obstacle_9;
    public List<List<double>> obstacle_10;

    public List<double> pos_start;
    public double vehicle_L;
    public double vehicle_a_max;
    public double vehicle_dt;
    public double vehicle_omega_max;
    public double vehicle_phi_max;
    public double vehicle_t;
    public double vehicle_vmax;
    public List<double> vel_goal;
    public List<double> vel_start;

    // Use this for initialization
    void Start () {

    }
	
	// Update is called once per frame
	void Update () {
		
	}
}
