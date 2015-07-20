using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class SAICWaypointsContainer : MonoBehaviour {

	public List<Transform> waypoints = new List<Transform>();

	void Start () {
	
	}
	
	// Update is called once per frame
	void Update () {
	
	}

	void OnDrawGizmos() {
		
		for(int i = 0; i < waypoints.Count; i ++){
			if(i < waypoints.Count - 1){
				if(waypoints[i] && waypoints[i+1]){
					Gizmos.color = new Color(0.0f, 1.0f, 1.0f, 0.3f);
					Gizmos.DrawSphere (waypoints[i+1].transform.position, 2);
					Gizmos.DrawWireSphere (waypoints[i+1].transform.position, 20f);
					if (waypoints.Count > 0) {
						Gizmos.color = Color.green;
						if(i < waypoints.Count - 1)
							Gizmos.DrawLine(waypoints[i].position, waypoints[i+1].position); 
						else Gizmos.DrawLine(waypoints[i].position, waypoints[0].position); 
					}
				}
			}
		}
		
	}

}
