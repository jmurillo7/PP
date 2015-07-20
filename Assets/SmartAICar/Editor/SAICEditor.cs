using UnityEngine;
using System.Collections;
using UnityEditor;
using CurveExtended;

[CustomEditor(typeof(SAICSmartAICar)), CanEditMultipleObjects]
public class SAICEditor : Editor {

	SAICSmartAICar aiScript;
	
	void Awake () {

		aiScript = (SAICSmartAICar)target;
	
	}

	public override void OnInspectorGUI () {

		serializedObject.Update();

		if(GUILayout.Button("Create Wheel Colliders")){
			
			WheelCollider[] wheelColliders = aiScript.gameObject.GetComponentsInChildren<WheelCollider>();
			
			if(wheelColliders.Length >= 1)
				Debug.LogError("Your Vehicle has Wheel Colliders already!");
			else
				aiScript.CreateWheelColliders();
			
		}

		DrawDefaultInspector();
	
		if(GUI.changed){
			EngineCurveInit();
		}

		serializedObject.ApplyModifiedProperties();

	}

	public void EngineCurveInit (){
		
		if(aiScript.totalGears <= 0){
			Debug.LogError("You are trying to set your vehicle gear to 0 or below! Why you trying to do this???");
			return;
		}
		
		aiScript.gearSpeed = new float[aiScript.totalGears];
		aiScript.engineTorqueCurve = new AnimationCurve[aiScript.totalGears];
		aiScript.currentGear = 0;
		
		for(int i = 0; i < aiScript.engineTorqueCurve.Length; i ++){
			aiScript.engineTorqueCurve[i] = new AnimationCurve(new Keyframe(0, 1));
		}
		
		for(int i = 0; i < aiScript.totalGears; i ++){
			
			aiScript.gearSpeed[i] = Mathf.Lerp(0, aiScript.maxSpeed / 1.5f, ((float)i/(float)(aiScript.totalGears - 0)));
			
			if(i != 0){
				aiScript.engineTorqueCurve[i].MoveKey(0, new Keyframe(0, .5f));
				aiScript.engineTorqueCurve[i].AddKey(KeyframeUtil.GetNew(Mathf.Lerp(0, aiScript.maxSpeed, ((float)i/(float)(aiScript.totalGears - 0))), 1f, CurveExtended.TangentMode.Smooth));
				aiScript.engineTorqueCurve[i].AddKey(aiScript.maxSpeed, 0);
				aiScript.engineTorqueCurve[i].postWrapMode = WrapMode.Clamp;
				aiScript.engineTorqueCurve[i].UpdateAllLinearTangents();
			}else{
				aiScript.engineTorqueCurve[i].AddKey(KeyframeUtil.GetNew(Mathf.Lerp(25, aiScript.maxSpeed, ((float)i/(float)(aiScript.totalGears - 1))), 1.25f, TangentMode.Linear));
				aiScript.engineTorqueCurve[i].AddKey(KeyframeUtil.GetNew(Mathf.Lerp(25f, aiScript.maxSpeed, ((float)(i+1)/(float)(aiScript.totalGears - 1))), 0, TangentMode.Linear));
				aiScript.engineTorqueCurve[i].AddKey(aiScript.maxSpeed, 0);
				aiScript.engineTorqueCurve[i].UpdateAllLinearTangents();
			}
			
		}
		
	}

}
