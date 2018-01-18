using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json;

public class Run : MonoBehaviour {

    public string path = "P1";

    // Use this for initialization
    void Start() {
        MapSettings settings = new MapSettings();
        string json = LoadTextfile(path);
        //string new_json = group_obstacles(json);
        //settings = JsonUtility.FromJson<MapSettings>(new_json);
        settings = JsonConvert.DeserializeObject<MapSettings>(json);
        Debug.Log(settings.bounding_polygon.Count);

        Object wallPrefab = Resources.Load("wall");
        Instantiate(wallPrefab, new Vector3(100, 20, 100), Quaternion.identity);
    }

    // Update is called once per frame
    void Update() {

    }

    public static string LoadTextfile(string file) {
        TextAsset targetFile = Resources.Load<TextAsset>(file);
        return targetFile.text;
    }

    public string group_obstacles(string json) {
        string new_json;
        int idx1 = json.IndexOf("obstacle_1") - 1;
        new_json = json.Insert(idx1, "\"obstacles\": {");
        int idx2 = new_json.LastIndexOf(",", new_json.IndexOf("pos_goal") - 1);
        new_json = new_json.Insert(idx2, "}");
        return new_json;
    }

    public void drawExternalWall(MapSettings settings) {
        List<List<double>> polygon = settings.bounding_polygon;
        int vertices = polygon.Count;

    }
}
