using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
	
[RequireComponent (typeof (Rigidbody))]

//Smart AI Car V2.4

public class SAICSmartAICar : MonoBehaviour {

	private Rigidbody rigid;
	
	// Wheel transforms of the vehicle.
	public Transform FrontLeftWheelTransform;
	public Transform FrontRightWheelTransform;
	public Transform RearLeftWheelTransform;
	public Transform RearRightWheelTransform;

	//Wheel colliders of the vehicle.
	public WheelCollider FrontLeftWheelCollider;
	public WheelCollider FrontRightWheelCollider;
	public WheelCollider RearLeftWheelCollider;
	public WheelCollider RearRightWheelCollider;
	public float antiRoll = 10000.0f;

	//Waypoint Container.
	public SAICWaypointsContainer waypointsContainer;

	//  Raycast distances.
	public LayerMask raycastLayers;
	public int wideRayDistance = 20;
	public int tightRayDistance = 20;
	private float newInputSteer = 0f;
	private bool  raycasting = false;
	private float resetTime = 0f;
		
	//Center of mass.
	public Transform COM;
	
	// Maximum and minimum engine RPM for adjusting engine audio pitch level and gear ratio.
	private bool  reversing = false;
	
	// We need an array for waypoints, and decide to which waypoint will be our target waypoint.

	private int currentWaypoint = 0;
	
	// Steer, motor, and brake inputs.
	private float steerInput = 0f;
	private float motorInput = 0f;
	private float brakeInput = 0f;
	
	// Counts laps and how many waypoints passed.
	public int lap = 0;
	public int totalWaypointPassed = 0;
	public int nextWaypointPassRadius = 20;
	
	private NavMeshAgent navigator;
	private GameObject navigatorObject;

	// Set wheel drive of the vehicle.
	public enum WheelType{FWD, RWD, AWD};
	public WheelType _wheelTypeChoise;

	//Vehicle Mecanim
	[HideInInspector]
	public AnimationCurve[] engineTorqueCurve;
	public float engineTorque = 750.0f;
	public float brakeTorque = 1000f;
	public float minEngineRPM = 1000.0f;
	public float maxEngineRPM = 6000.0f;
	public float maxSpeed = 180.0f;
	[HideInInspector]
	public float speed = 0f;
	private float engineRPM = 0f;
	public float steerAngle = 20.0f;
	public float highSpeedSteerAngle = 10.0f;
	public float highSpeedSteerAngleAtSpeed = 80.0f;
	private float defsteerAngle;

	//Gears.
	public int currentGear;
	public int totalGears = 6;
	private int _totalGears
	{
		get
		{
			return totalGears - 1;
		}
	}
	[HideInInspector]
	public bool changingGear = false;
	public float gearShiftRate = 10.0f;
	[HideInInspector]
	public float[] gearSpeed;
	
	private float acceleration;
	private float lastVelocity;

	//Sounds
	private AudioSource engineAudio;
	public AudioClip engineClip;
	private AudioSource skidAudio;
	public AudioClip skidClip;
	private AudioSource crashAudio;
	public AudioClip[] crashClips;
	
	// Each wheel transform's rotation value.
	private float rotationValueFL, rotationValueFR, rotationValueRL, rotationValueRR;
	private float[] rotationValueExtra;
	
	public GameObject wheelSlipPrefab;
	private List <GameObject> wheelParticles = new List<GameObject>();
	
	public GameObject chassis;
	private float horizontalLean = 0.0f;
	private float verticalLean = 0.0f;
	public bool GUIenabled = false;
	
	void Start (){

		rigid = GetComponent<Rigidbody>();
		if(!waypointsContainer)
			waypointsContainer = FindObjectOfType(typeof(SAICWaypointsContainer)) as SAICWaypointsContainer;

		SoundsInitialize(); 
		if(wheelSlipPrefab)
			SmokeInit();

		navigatorObject = new GameObject("Navigator");
		navigatorObject.transform.parent = transform;
		navigatorObject.transform.localPosition = Vector3.zero;
		navigatorObject.AddComponent<NavMeshAgent>();
		navigatorObject.GetComponent<NavMeshAgent>().radius = 1;
		navigatorObject.GetComponent<NavMeshAgent>().speed = 5f;
		navigatorObject.GetComponent<NavMeshAgent>().height = 1;
		navigatorObject.GetComponent<NavMeshAgent>().avoidancePriority = 99;
		navigator = navigatorObject.GetComponent<NavMeshAgent>();
			
		// Lower the center of mass for make more stable car.
		rigid.centerOfMass = new Vector3(COM.localPosition.x * transform.localScale.x , COM.localPosition.y * transform.localScale.y , COM.localPosition.z * transform.localScale.z);
		rigid.maxAngularVelocity = 3f;
		defsteerAngle = steerAngle;
		
	}

	public void CreateWheelColliders (){
		
		List <Transform> allWheelTransforms = new List<Transform>();
		allWheelTransforms.Add(FrontLeftWheelTransform); allWheelTransforms.Add(FrontRightWheelTransform); allWheelTransforms.Add(RearLeftWheelTransform); allWheelTransforms.Add(RearRightWheelTransform);
		
		if(allWheelTransforms[0] == null){
			Debug.LogError("You haven't choose your Wheel Transforms. Please select all of your Wheel Transforms before creating Wheel Colliders. Script needs to know their positions, aye?");
			return;
		}
		
		transform.rotation = Quaternion.identity;
		
		GameObject WheelColliders = new GameObject("Wheel Colliders");
		WheelColliders.transform.parent = transform;
		WheelColliders.transform.rotation = transform.rotation;
		WheelColliders.transform.localPosition = Vector3.zero;
		WheelColliders.transform.localScale = Vector3.one;
		
		foreach(Transform wheel in allWheelTransforms){
			
			GameObject wheelcollider = new GameObject(wheel.transform.name); 
			
			wheelcollider.transform.position = wheel.transform.position;
			wheelcollider.transform.rotation = transform.rotation;
			wheelcollider.transform.name = wheel.transform.name;
			wheelcollider.transform.parent = WheelColliders.transform;
			wheelcollider.transform.localScale = Vector3.one;
			wheelcollider.layer = LayerMask.NameToLayer("Wheel");
			wheelcollider.AddComponent<WheelCollider>();
			wheelcollider.GetComponent<WheelCollider>().radius = (wheel.GetComponent<MeshRenderer>().bounds.size.y / 2f) / transform.localScale.y;
			
			wheelcollider.AddComponent<SAICWheelSkidmarks>();
			wheelcollider.GetComponent<SAICWheelSkidmarks>().vehicle = gameObject;
			
			JointSpring spring = wheelcollider.GetComponent<WheelCollider>().suspensionSpring;
			
			spring.spring = 35000f;
			spring.damper = 2000f;
			
			wheelcollider.GetComponent<WheelCollider>().suspensionSpring = spring;
			wheelcollider.GetComponent<WheelCollider>().suspensionDistance = .25f;
			wheelcollider.GetComponent<WheelCollider>().forceAppPointDistance = .25f;
			wheelcollider.GetComponent<WheelCollider>().mass = 100f;
			wheelcollider.GetComponent<WheelCollider>().wheelDampingRate = 1f;
			
			WheelFrictionCurve sidewaysFriction = wheelcollider.GetComponent<WheelCollider>().sidewaysFriction;
			WheelFrictionCurve forwardFriction = wheelcollider.GetComponent<WheelCollider>().forwardFriction;
			
			forwardFriction.extremumSlip = .4f;
			forwardFriction.extremumValue = 1;
			forwardFriction.asymptoteSlip = .8f;
			forwardFriction.asymptoteValue = .75f;
			forwardFriction.stiffness = 1.75f;
			
			sidewaysFriction.extremumSlip = .25f;
			sidewaysFriction.extremumValue = 1;
			sidewaysFriction.asymptoteSlip = .5f;
			sidewaysFriction.asymptoteValue = .75f;
			sidewaysFriction.stiffness = 2f;
			
			wheelcollider.GetComponent<WheelCollider>().sidewaysFriction = sidewaysFriction;
			wheelcollider.GetComponent<WheelCollider>().forwardFriction = forwardFriction;
			
		}
		
		WheelCollider[] allWheelColliders = new WheelCollider[allWheelTransforms.Count];
		allWheelColliders = GetComponentsInChildren<WheelCollider>();
		
		FrontLeftWheelCollider = allWheelColliders[0];
		FrontRightWheelCollider = allWheelColliders[1];
		RearLeftWheelCollider = allWheelColliders[2];
		RearRightWheelCollider = allWheelColliders[3];
		
	}

	public AudioSource CreateAudioSource(string audioName, float minDistance, float volume, AudioClip audioClip, bool loop, bool playNow, bool destroyAfterFinished){
		
		GameObject audioSource = new GameObject(audioName);
		audioSource.transform.position = transform.position;
		audioSource.transform.rotation = transform.rotation;
		audioSource.transform.parent = transform;
		audioSource.AddComponent<AudioSource>();
		audioSource.GetComponent<AudioSource>().minDistance = minDistance;
		audioSource.GetComponent<AudioSource>().volume = volume;
		audioSource.GetComponent<AudioSource>().clip = audioClip;
		audioSource.GetComponent<AudioSource>().loop = loop;
		audioSource.GetComponent<AudioSource>().spatialBlend = 1f;
		
		if(playNow)
			audioSource.GetComponent<AudioSource>().Play();
		
		if(destroyAfterFinished)
			Destroy(audioSource, audioClip.length);
		
		return audioSource.GetComponent<AudioSource>();
		
	}
	
	void SoundsInitialize(){
		
		engineAudio = CreateAudioSource("engineSound", 5f, 1f, engineClip, true, true, false);
		skidAudio = CreateAudioSource("skidSound", 5f, 0f, skidClip, true, true, false);
		
	}

	public void SmokeInit (){
		
		string wheelSlipPrefabName = wheelSlipPrefab.name+"(Clone)";
		
		for(int i = 0; i < 4; i++){
			Instantiate(wheelSlipPrefab, transform.position, transform.rotation);
		}
		
		foreach(GameObject go in GameObject.FindObjectsOfType(typeof(GameObject)))
		{
			if(go.name == wheelSlipPrefabName)
				wheelParticles.Add (go);
		} 
		
		wheelParticles[0].transform.position = FrontRightWheelCollider.transform.position;
		wheelParticles[1].transform.position = FrontLeftWheelCollider.transform.position;
		wheelParticles[2].transform.position = RearRightWheelCollider.transform.position;
		wheelParticles[3].transform.position = RearLeftWheelCollider.transform.position;
		
		wheelParticles[0].transform.parent = FrontRightWheelCollider.transform;
		wheelParticles[1].transform.parent = FrontLeftWheelCollider.transform;
		wheelParticles[2].transform.parent = RearRightWheelCollider.transform;
		wheelParticles[3].transform.parent = RearLeftWheelCollider.transform;
		
	}
	
	void  OnGUI (){
	
		if(GUIenabled) {
			
			GUI.backgroundColor = Color.black;
			float guiWidth = Screen.width/2 - 200;
			
			GUI.Box(new Rect(Screen.width-410 - guiWidth, 10, 400, 220), "");
			GUI.Label(new Rect(Screen.width-400 - guiWidth, 10, 400, 150), "Engine RPM : " + Mathf.CeilToInt(engineRPM));
			GUI.Label(new Rect(Screen.width-400 - guiWidth, 30, 400, 150), "speed : " + Mathf.CeilToInt(speed));
			if(_wheelTypeChoise == WheelType.FWD){
				GUI.Label(new Rect(Screen.width-400 - guiWidth, 50, 400, 150), "Left Wheel RPM : " + Mathf.CeilToInt(FrontLeftWheelCollider.rpm));
				GUI.Label(new Rect(Screen.width-400 - guiWidth, 70, 400, 150), "Right Wheel RPM : " + Mathf.CeilToInt(FrontRightWheelCollider.rpm));
				GUI.Label(new Rect(Screen.width-400 - guiWidth, 90, 400, 150), "Left Wheel Torque : " + Mathf.CeilToInt(FrontLeftWheelCollider.motorTorque));
				GUI.Label(new Rect(Screen.width-400 - guiWidth, 110, 400, 150), "Right Wheel Torque : " + Mathf.CeilToInt(FrontRightWheelCollider.motorTorque));
				GUI.Label(new Rect(Screen.width-400 - guiWidth, 130, 400, 150), "Left Wheel brake : " + Mathf.CeilToInt(FrontLeftWheelCollider.brakeTorque));
				GUI.Label(new Rect(Screen.width-400 - guiWidth, 150, 400, 150), "Right Wheel brake : " + Mathf.CeilToInt(FrontRightWheelCollider.brakeTorque));
				GUI.Label(new Rect(Screen.width-400 - guiWidth, 170, 400, 150), "Steer Angle : " + Mathf.CeilToInt(FrontLeftWheelCollider.steerAngle));
			}
			if(_wheelTypeChoise == WheelType.RWD || _wheelTypeChoise == WheelType.AWD){
				GUI.Label(new Rect(Screen.width-400 - guiWidth, 50, 400, 150), "Left Wheel RPM : " + Mathf.CeilToInt(RearLeftWheelCollider.rpm));
				GUI.Label(new Rect(Screen.width-400 - guiWidth, 70, 400, 150), "Right Wheel RPM : " + Mathf.CeilToInt(RearRightWheelCollider.rpm));
				GUI.Label(new Rect(Screen.width-400 - guiWidth, 90, 400, 150), "Left Wheel Torque : " + Mathf.CeilToInt(RearLeftWheelCollider.motorTorque));
				GUI.Label(new Rect(Screen.width-400 - guiWidth, 110, 400, 150), "Right Wheel Torque : " + Mathf.CeilToInt(RearRightWheelCollider.motorTorque));
				GUI.Label(new Rect(Screen.width-400 - guiWidth, 130, 400, 150), "Left Wheel brake : " + Mathf.CeilToInt(RearLeftWheelCollider.brakeTorque));
				GUI.Label(new Rect(Screen.width-400 - guiWidth, 150, 400, 150), "Right Wheel brake : " + Mathf.CeilToInt(RearRightWheelCollider.brakeTorque));
				GUI.Label(new Rect(Screen.width-400 - guiWidth, 170, 400, 150), "Steer Angle : " + Mathf.CeilToInt(FrontLeftWheelCollider.steerAngle));
			}
			
			GUI.backgroundColor = Color.blue;
			GUI.Button (new Rect(Screen.width-30 - guiWidth, 165, 10, Mathf.Clamp(((-motorInput + brakeInput) * 100), -100, 0)), "");
			
			GUI.backgroundColor = Color.red;
			GUI.Button (new Rect(Screen.width-45 - guiWidth, 165, 10, Mathf.Clamp((-brakeInput * 100), -100, 0)), "");
			
		}
		
	}

	void Update(){

		WheelAlign();
		SkidAudio();
		ShiftGears();

		if(chassis)
			Chassis();
		if(wheelSlipPrefab)
			SmokeInstantiateRate();

	}
		
	void  FixedUpdate (){
	
		Engine();
		Navigation();
		FixedRaycasts();
		Resetting();
		AntiRollBars();

	}

	void Engine(){
		
		//speed.
		speed = rigid.velocity.magnitude * 3.0f;
		
		//Acceleration Calculation.
		acceleration = 0f;
		acceleration = (transform.InverseTransformDirection(rigid.velocity).z - lastVelocity) / Time.fixedDeltaTime;
		lastVelocity = transform.InverseTransformDirection(rigid.velocity).z;
		
		//Drag Limit Depends On Vehicle Acceleration.
		rigid.drag = Mathf.Clamp((acceleration / 50f), 0f, 1f);
		
		//Steer Limit.
		steerAngle = Mathf.Lerp(defsteerAngle, highSpeedSteerAngle, (speed / highSpeedSteerAngleAtSpeed));

		FrontLeftWheelCollider.steerAngle = Mathf.Clamp((steerAngle * steerInput), -steerAngle, steerAngle);
		FrontRightWheelCollider.steerAngle = Mathf.Clamp((steerAngle * steerInput), -steerAngle, steerAngle);
		
		//Engine RPM.
		engineRPM = Mathf.Clamp((((Mathf.Abs((RearLeftWheelCollider.rpm + RearRightWheelCollider.rpm)) * gearShiftRate) + minEngineRPM)) / (currentGear+1), minEngineRPM, maxEngineRPM);

		//Engine Audio Volume.
		if(engineAudio){
			engineAudio.GetComponent<AudioSource>().pitch = Mathf.Lerp (engineAudio.GetComponent<AudioSource>().pitch, Mathf.Lerp (1f, 2f, (engineRPM - minEngineRPM / 1.5f) / (maxEngineRPM + minEngineRPM)), Time.deltaTime * 5);
			if(!changingGear)
				engineAudio.GetComponent<AudioSource>().volume = Mathf.Lerp (engineAudio.GetComponent<AudioSource>().volume, Mathf.Clamp (motorInput - brakeInput, .35f, .85f), Time.deltaTime*  5);
			else
				engineAudio.GetComponent<AudioSource>().volume = Mathf.Lerp (engineAudio.GetComponent<AudioSource>().volume, .35f, Time.deltaTime*  5);
		}

		//Applying WheelCollider Motor Torques Depends On Wheel Type Choice.
		switch(_wheelTypeChoise){

		case WheelType.FWD:
			FrontLeftWheelCollider.motorTorque = ApplyWheelTorque();
			FrontRightWheelCollider.motorTorque = ApplyWheelTorque();
			break;
		case WheelType.RWD:
			RearLeftWheelCollider.motorTorque = ApplyWheelTorque();
			RearRightWheelCollider.motorTorque = ApplyWheelTorque();
			break;
		case WheelType.AWD:
			FrontLeftWheelCollider.motorTorque = ApplyWheelTorque();
			FrontRightWheelCollider.motorTorque = ApplyWheelTorque();
			RearLeftWheelCollider.motorTorque = ApplyWheelTorque();
			RearRightWheelCollider.motorTorque = ApplyWheelTorque();
			break;

		}

		// Apply the brake torque values to the rear wheels.
		FrontLeftWheelCollider.brakeTorque = brakeTorque * brakeInput;
		FrontRightWheelCollider.brakeTorque = brakeTorque * brakeInput;
		//RearLeftWheelCollider.brakeTorque = (brakeTorque) * brakeInput;
		//RearRightWheelCollider.brakeTorque = (brakeTorque) * brakeInput;

	}

	public float ApplyWheelTorque(){

		float torque = 0;

		if(changingGear){
			torque = 0;
		}else{
			if(!reversing)
				torque = (engineTorque * Mathf.Clamp(motorInput - (brakeInput / 1.5f), 0f, 1f)) * engineTorqueCurve[currentGear].Evaluate(speed);
			else
				torque = (-engineTorque * Mathf.Clamp(motorInput, 0f, 1f)) * engineTorqueCurve[currentGear].Evaluate(speed);
		}

		return torque;

	}

	public void AntiRollBars (){
		
		WheelHit FrontWheelHit;
		
		float travelFL = 1.0f;
		float travelFR = 1.0f;
		
		bool groundedFL= FrontLeftWheelCollider.GetGroundHit(out FrontWheelHit);
		
		if (groundedFL)
			travelFL = (-FrontLeftWheelCollider.transform.InverseTransformPoint(FrontWheelHit.point).y - FrontLeftWheelCollider.radius) / FrontLeftWheelCollider.suspensionDistance;
		
		bool groundedFR= FrontRightWheelCollider.GetGroundHit(out FrontWheelHit);
		
		if (groundedFR)
			travelFR = (-FrontRightWheelCollider.transform.InverseTransformPoint(FrontWheelHit.point).y - FrontRightWheelCollider.radius) / FrontRightWheelCollider.suspensionDistance;
		
		float antiRollForceFront= (travelFL - travelFR) * antiRoll;
		
		if (groundedFL)
			rigid.AddForceAtPosition(FrontLeftWheelCollider.transform.up * -antiRollForceFront, FrontLeftWheelCollider.transform.position); 
		if (groundedFR)
			rigid.AddForceAtPosition(FrontRightWheelCollider.transform.up * antiRollForceFront, FrontRightWheelCollider.transform.position); 
		
		WheelHit RearWheelHit;
		
		float travelRL = 1.0f;
		float travelRR = 1.0f;
		
		bool groundedRL= RearLeftWheelCollider.GetGroundHit(out RearWheelHit);
		
		if (groundedRL)
			travelRL = (-RearLeftWheelCollider.transform.InverseTransformPoint(RearWheelHit.point).y - RearLeftWheelCollider.radius) / RearLeftWheelCollider.suspensionDistance;
		
		bool groundedRR= RearRightWheelCollider.GetGroundHit(out RearWheelHit);
		
		if (groundedRR)
			travelRR = (-RearRightWheelCollider.transform.InverseTransformPoint(RearWheelHit.point).y - RearRightWheelCollider.radius) / RearRightWheelCollider.suspensionDistance;
		
		float antiRollForceRear= (travelRL - travelRR) * antiRoll;
		
		if (groundedRL)
			rigid.AddForceAtPosition(RearLeftWheelCollider.transform.up * -antiRollForceRear, RearLeftWheelCollider.transform.position); 
		if (groundedRR)
			rigid.AddForceAtPosition(RearRightWheelCollider.transform.up * antiRollForceRear, RearRightWheelCollider.transform.position);
		
		if (groundedRR && groundedRL)
			rigid.AddRelativeTorque((Vector3.up * (steerInput)) * 5000f);
		
	}

	public void ShiftGears (){
			
		if(currentGear < _totalGears && !changingGear){
			if(speed > gearSpeed[currentGear + 1] && RearLeftWheelCollider.rpm >= 0){
				StartCoroutine("ChangingGear", currentGear + 1);
			}
		}
		
		if(currentGear > 0){
			if(engineRPM < minEngineRPM + 500 && !changingGear){
				
				for(int i = 0; i < gearSpeed.Length; i++){
					if(speed > gearSpeed[i])
						StartCoroutine("ChangingGear", i);
				}
				
			}
		}
		
	}
	
	IEnumerator ChangingGear(int gear){
		
		changingGear = true;
		
		yield return new WaitForSeconds(.5f);
		changingGear = false;
		currentGear = gear;
		
	}
	
	void  Navigation (){
			
		// Next waypoint's position.
		Vector3 nextWaypointPosition = transform.InverseTransformPoint( new Vector3(waypointsContainer.waypoints[currentWaypoint].position.x, transform.position.y, waypointsContainer.waypoints[currentWaypoint].position.z));

		navigator.SetDestination(waypointsContainer.waypoints[currentWaypoint].position);
		navigator.transform.localPosition = Vector3.zero;

		//Steering Input.
		if(!reversing)
			steerInput = Mathf.Lerp (steerInput, Mathf.Clamp((transform.InverseTransformDirection(navigator.desiredVelocity).x + newInputSteer), -1f, 1f), Time.deltaTime * 20f);
		else
			steerInput = Mathf.Clamp((-transform.InverseTransformDirection(navigator.desiredVelocity).x + newInputSteer), -1f, 1f);

		if(speed >= 10)
			brakeInput = Mathf.Lerp(0f, 1f, (Mathf.Abs(steerInput)));
		else
			brakeInput = 0f;

		motorInput = Mathf.Clamp(transform.InverseTransformDirection(navigator.desiredVelocity).z, 0.85f, 1f) ;

		// Checks for the distance to next waypoint. If it is less than written value, then pass to next waypoint.
		if ( nextWaypointPosition.magnitude < nextWaypointPassRadius ) {
				currentWaypoint ++;
				totalWaypointPassed ++;
			
		// If all waypoints are passed, sets the current waypoint to first waypoint and increase lap.
			if ( currentWaypoint >= waypointsContainer.waypoints.Count ) {
				currentWaypoint = 0;
				lap ++;
			}
		}
			
	}
		
	void Resetting (){

		if(speed <= 15)
			resetTime += Time.deltaTime;
		else
			resetTime = 0;

		Vector3 thisT = ( new Vector3( transform.localEulerAngles.x, transform.localEulerAngles.y, transform.localEulerAngles.z));
			
		if(resetTime >= 3)
			reversing = true;
		else
			reversing = false;
				
		if(thisT.z < 300 && thisT.z > 60 && speed <= 5)
			transform.localEulerAngles = new Vector3( transform.localEulerAngles.x, transform.localEulerAngles.y, 0);
			
	}
		
	void  WheelAlign (){
		
		RaycastHit hit;
		WheelHit CorrespondingGroundHit;
		
		//Front Left Wheel Transform.
		Vector3 ColliderCenterPointFL = FrontLeftWheelCollider.transform.TransformPoint( FrontLeftWheelCollider.center );
		FrontLeftWheelCollider.GetGroundHit( out CorrespondingGroundHit );
		
		if ( Physics.Raycast( ColliderCenterPointFL, -FrontLeftWheelCollider.transform.up, out hit, (FrontLeftWheelCollider.suspensionDistance + FrontLeftWheelCollider.radius) * transform.localScale.y) ) {
			if(hit.transform.gameObject.layer != LayerMask.NameToLayer("Vehicle")){
				FrontLeftWheelTransform.transform.position = hit.point + (FrontLeftWheelCollider.transform.up * FrontLeftWheelCollider.radius) * transform.localScale.y;
				float extension = (-FrontLeftWheelCollider.transform.InverseTransformPoint(CorrespondingGroundHit.point).y - FrontLeftWheelCollider.radius) / FrontLeftWheelCollider.suspensionDistance;
				Debug.DrawLine(CorrespondingGroundHit.point, CorrespondingGroundHit.point + FrontLeftWheelCollider.transform.up * (CorrespondingGroundHit.force / 8000), extension <= 0.0 ? Color.magenta : Color.white);
				Debug.DrawLine(CorrespondingGroundHit.point, CorrespondingGroundHit.point - FrontLeftWheelCollider.transform.forward * CorrespondingGroundHit.forwardSlip, Color.green);
				Debug.DrawLine(CorrespondingGroundHit.point, CorrespondingGroundHit.point - FrontLeftWheelCollider.transform.right * CorrespondingGroundHit.sidewaysSlip, Color.red);
			}
		}else{
			FrontLeftWheelTransform.transform.position = ColliderCenterPointFL - (FrontLeftWheelCollider.transform.up * FrontLeftWheelCollider.suspensionDistance) * transform.localScale.y;
		}

		FrontLeftWheelTransform.transform.rotation = FrontLeftWheelCollider.transform.rotation * Quaternion.Euler( rotationValueFL, FrontLeftWheelCollider.steerAngle, FrontLeftWheelCollider.transform.rotation.z);
		rotationValueFL += FrontLeftWheelCollider.rpm * ( 6 ) * Time.deltaTime;
		
		
		//Front Right Wheel Transform.
		Vector3 ColliderCenterPointFR = FrontRightWheelCollider.transform.TransformPoint( FrontRightWheelCollider.center );
		FrontRightWheelCollider.GetGroundHit( out CorrespondingGroundHit );
		
		if ( Physics.Raycast( ColliderCenterPointFR, -FrontRightWheelCollider.transform.up, out hit, (FrontRightWheelCollider.suspensionDistance + FrontRightWheelCollider.radius) * transform.localScale.y ) ) {
			if(hit.transform.gameObject.layer != LayerMask.NameToLayer("Vehicle")){
				FrontRightWheelTransform.transform.position = hit.point + (FrontRightWheelCollider.transform.up * FrontRightWheelCollider.radius) * transform.localScale.y;
				float extension = (-FrontRightWheelCollider.transform.InverseTransformPoint(CorrespondingGroundHit.point).y - FrontRightWheelCollider.radius) / FrontRightWheelCollider.suspensionDistance;
				Debug.DrawLine(CorrespondingGroundHit.point, CorrespondingGroundHit.point + FrontRightWheelCollider.transform.up * (CorrespondingGroundHit.force / 8000), extension <= 0.0 ? Color.magenta : Color.white);
				Debug.DrawLine(CorrespondingGroundHit.point, CorrespondingGroundHit.point - FrontRightWheelCollider.transform.forward * CorrespondingGroundHit.forwardSlip, Color.green);
				Debug.DrawLine(CorrespondingGroundHit.point, CorrespondingGroundHit.point - FrontRightWheelCollider.transform.right * CorrespondingGroundHit.sidewaysSlip, Color.red);
			}
		}else{
			FrontRightWheelTransform.transform.position = ColliderCenterPointFR - (FrontRightWheelCollider.transform.up * FrontRightWheelCollider.suspensionDistance) * transform.localScale.y;
		}

		rotationValueFR += FrontRightWheelCollider.rpm * ( 6 ) * Time.deltaTime;
		FrontRightWheelTransform.transform.rotation = FrontRightWheelCollider.transform.rotation * Quaternion.Euler( rotationValueFR, FrontRightWheelCollider.steerAngle, FrontRightWheelCollider.transform.rotation.z);

		//Rear Left Wheel Transform.
		Vector3 ColliderCenterPointRL = RearLeftWheelCollider.transform.TransformPoint( RearLeftWheelCollider.center );
		RearLeftWheelCollider.GetGroundHit( out CorrespondingGroundHit );
		
		if ( Physics.Raycast( ColliderCenterPointRL, -RearLeftWheelCollider.transform.up, out hit, (RearLeftWheelCollider.suspensionDistance + RearLeftWheelCollider.radius) * transform.localScale.y ) ) {
			if(hit.transform.gameObject.layer != LayerMask.NameToLayer("Vehicle")){
				RearLeftWheelTransform.transform.position = hit.point + (RearLeftWheelCollider.transform.up * RearLeftWheelCollider.radius) * transform.localScale.y;
				float extension = (-RearLeftWheelCollider.transform.InverseTransformPoint(CorrespondingGroundHit.point).y - RearLeftWheelCollider.radius) / RearLeftWheelCollider.suspensionDistance;
				Debug.DrawLine(CorrespondingGroundHit.point, CorrespondingGroundHit.point + RearLeftWheelCollider.transform.up * (CorrespondingGroundHit.force / 8000), extension <= 0.0 ? Color.magenta : Color.white);
				Debug.DrawLine(CorrespondingGroundHit.point, CorrespondingGroundHit.point - RearLeftWheelCollider.transform.forward * CorrespondingGroundHit.forwardSlip, Color.green);
				Debug.DrawLine(CorrespondingGroundHit.point, CorrespondingGroundHit.point - RearLeftWheelCollider.transform.right * CorrespondingGroundHit.sidewaysSlip, Color.red);
			}
		}else{
			RearLeftWheelTransform.transform.position = ColliderCenterPointRL - (RearLeftWheelCollider.transform.up * RearLeftWheelCollider.suspensionDistance) * transform.localScale.y;
		}
		rotationValueRL += RearLeftWheelCollider.rpm * ( 6 ) * Time.deltaTime;
		RearLeftWheelTransform.transform.rotation = RearLeftWheelCollider.transform.rotation * Quaternion.Euler( rotationValueRL, 0, RearLeftWheelCollider.transform.rotation.z);
		
		RearLeftWheelCollider.GetGroundHit( out CorrespondingGroundHit );

		//Rear Right Wheel Transform.
		Vector3 ColliderCenterPointRR = RearRightWheelCollider.transform.TransformPoint( RearRightWheelCollider.center );
		RearRightWheelCollider.GetGroundHit( out CorrespondingGroundHit );
		
		if ( Physics.Raycast( ColliderCenterPointRR, -RearRightWheelCollider.transform.up, out hit, (RearRightWheelCollider.suspensionDistance + RearRightWheelCollider.radius) * transform.localScale.y ) ) {
			if(hit.transform.gameObject.layer != LayerMask.NameToLayer("Vehicle")){
				RearRightWheelTransform.transform.position = hit.point + (RearRightWheelCollider.transform.up * RearRightWheelCollider.radius) * transform.localScale.y;
				float extension = (-RearRightWheelCollider.transform.InverseTransformPoint(CorrespondingGroundHit.point).y - RearRightWheelCollider.radius) / RearRightWheelCollider.suspensionDistance;
				Debug.DrawLine(CorrespondingGroundHit.point, CorrespondingGroundHit.point + RearRightWheelCollider.transform.up * (CorrespondingGroundHit.force / 8000), extension <= 0.0 ? Color.magenta : Color.white);
				Debug.DrawLine(CorrespondingGroundHit.point, CorrespondingGroundHit.point - RearRightWheelCollider.transform.forward * CorrespondingGroundHit.forwardSlip, Color.green);
				Debug.DrawLine(CorrespondingGroundHit.point, CorrespondingGroundHit.point - RearRightWheelCollider.transform.right * CorrespondingGroundHit.sidewaysSlip, Color.red);
			}
		}else{
			RearRightWheelTransform.transform.position = ColliderCenterPointRR - (RearRightWheelCollider.transform.up * RearRightWheelCollider.suspensionDistance) * transform.localScale.y;
		}
		RearRightWheelTransform.transform.rotation = RearRightWheelCollider.transform.rotation * Quaternion.Euler( rotationValueRR, 0, RearRightWheelCollider.transform.rotation.z);
		rotationValueRR += RearRightWheelCollider.rpm * ( 6 ) * Time.deltaTime;
		RearRightWheelCollider.GetGroundHit( out CorrespondingGroundHit );
		
	}
		
	void FixedRaycasts(){

		Vector3 fwd = transform.TransformDirection ( new Vector3(0, 0, 1));
		RaycastHit hit;
	
		bool  tightTurn = false;
		bool  wideTurn = false;
		bool  tightTurn1 = false;
		bool  wideTurn1 = false;
		
		// New input steers effected by fixed raycasts.
		float newinputSteer1 = 0.0f;
		float newinputSteer2 = 0.0f;
		float newinputSteer3 = 0.0f;
		float newinputSteer4 = 0.0f;
		
		// Drawing Rays.
		Debug.DrawRay (transform.position, Quaternion.AngleAxis(25, transform.up) * fwd * wideRayDistance, Color.white);
		Debug.DrawRay (transform.position, Quaternion.AngleAxis(-25, transform.up) * fwd * wideRayDistance, Color.white);
		
		Debug.DrawRay (transform.position, Quaternion.AngleAxis(7, transform.up) * fwd * tightRayDistance, Color.white);
		Debug.DrawRay (transform.position, Quaternion.AngleAxis(-7, transform.up) * fwd * tightRayDistance, Color.white);
		
		// Wide Raycasts.
		if (Physics.Raycast (transform.position, Quaternion.AngleAxis(25, transform.up) * fwd, out hit, wideRayDistance, raycastLayers)) {
			Debug.DrawRay (transform.position, Quaternion.AngleAxis(25, transform.up) * fwd * wideRayDistance, Color.red);
			newinputSteer1 = Mathf.Lerp (-.5f, 0, (hit.distance / wideRayDistance));
			wideTurn = true;
		}
		
		else{
			newinputSteer1 = 0;
			wideTurn = false;
		}
		
		if (Physics.Raycast (transform.position, Quaternion.AngleAxis(-25, transform.up) * fwd, out hit, wideRayDistance, raycastLayers)) {
			Debug.DrawRay (transform.position, Quaternion.AngleAxis(-25, transform.up) * fwd * wideRayDistance, Color.red);
			newinputSteer4 = Mathf.Lerp (.5f, 0, (hit.distance / wideRayDistance));
			wideTurn1 = true;
		}
		
		else{
			newinputSteer4 = 0;
			wideTurn1 = false;
		}
		
		// Tight Raycasts.
		if (Physics.Raycast (transform.position, Quaternion.AngleAxis(7, transform.up) * fwd, out hit, tightRayDistance, raycastLayers)) {
			Debug.DrawRay (transform.position, Quaternion.AngleAxis(7, transform.up) * fwd * tightRayDistance , Color.red);
			newinputSteer3 = Mathf.Lerp (-1, 0, (hit.distance / tightRayDistance));
			tightTurn = true;
		}
		
		else{
			newinputSteer3 = 0;
			tightTurn = false;
		}
		
		if (Physics.Raycast (transform.position, Quaternion.AngleAxis(-7, transform.up) * fwd, out hit, tightRayDistance, raycastLayers)) {
			Debug.DrawRay (transform.position, Quaternion.AngleAxis(-7, transform.up) * fwd * tightRayDistance, Color.red);
			newinputSteer2 = Mathf.Lerp (1, 0, (hit.distance / tightRayDistance));
			tightTurn1 = true;
		}
		
		else{
			newinputSteer2 = 0;
			tightTurn1 = false;
		}

		if(wideTurn || wideTurn1 || tightTurn || tightTurn1)
			raycasting = true;
		else
			raycasting = false;

		if(raycasting)
			newInputSteer = (newinputSteer1 + newinputSteer2 + newinputSteer3 + newinputSteer4);
		else
			newInputSteer = 0;

	}
		
	public void SkidAudio (){
		
		if(!skidClip)
			return;
		
		WheelHit CorrespondingGroundHitF;
		FrontRightWheelCollider.GetGroundHit( out CorrespondingGroundHitF );
		
		WheelHit CorrespondingGroundHitR;
		RearRightWheelCollider.GetGroundHit( out CorrespondingGroundHitR );
		
		if(Mathf.Abs(CorrespondingGroundHitF.sidewaysSlip) > .25f || Mathf.Abs(CorrespondingGroundHitR.forwardSlip) > .5f || Mathf.Abs(CorrespondingGroundHitF.forwardSlip) > .5f){
			if(rigid.velocity.magnitude > 1f)
				skidAudio.volume = Mathf.Abs(CorrespondingGroundHitF.sidewaysSlip) + ((Mathf.Abs(CorrespondingGroundHitF.forwardSlip) + Mathf.Abs(CorrespondingGroundHitR.forwardSlip)) / 4f);
			else
				skidAudio.volume -= Time.deltaTime;
		}else{
			skidAudio.volume -= Time.deltaTime;
		}
		
	}
		
	void SmokeInstantiateRate () {
		
		WheelHit CorrespondingGroundHit;
		
		if(wheelParticles.Count > 0){
			
			FrontRightWheelCollider.GetGroundHit( out CorrespondingGroundHit );
			if(Mathf.Abs(CorrespondingGroundHit.sidewaysSlip) > .25f || Mathf.Abs(CorrespondingGroundHit.forwardSlip) > .5f ) 
				wheelParticles[0].GetComponent<ParticleEmitter>().emit = true;
			else wheelParticles[0].GetComponent<ParticleEmitter>().emit = false;
			
			FrontLeftWheelCollider.GetGroundHit( out CorrespondingGroundHit );
			if(Mathf.Abs(CorrespondingGroundHit.sidewaysSlip) > .25f || Mathf.Abs(CorrespondingGroundHit.forwardSlip) > .5f ) 
				wheelParticles[1].GetComponent<ParticleEmitter>().emit = true;
			else wheelParticles[1].GetComponent<ParticleEmitter>().emit = false;
			
			RearRightWheelCollider.GetGroundHit( out CorrespondingGroundHit );
			if(Mathf.Abs(CorrespondingGroundHit.sidewaysSlip) > .25f || Mathf.Abs(CorrespondingGroundHit.forwardSlip) > .5f ) 
				wheelParticles[2].GetComponent<ParticleEmitter>().emit = true;
			else wheelParticles[2].GetComponent<ParticleEmitter>().emit = false;
			
			RearLeftWheelCollider.GetGroundHit( out CorrespondingGroundHit );
			if(Mathf.Abs(CorrespondingGroundHit.sidewaysSlip) > .25f || Mathf.Abs(CorrespondingGroundHit.forwardSlip) > .5f ) 
				wheelParticles[3].GetComponent<ParticleEmitter>().emit = true;
			else wheelParticles[3].GetComponent<ParticleEmitter>().emit = false;
			
		}
		
	}

	public void Chassis (){
		
		verticalLean = Mathf.Clamp(Mathf.Lerp (verticalLean, rigid.angularVelocity.x * 4f, Time.deltaTime * 3f), -3.0f, 3.0f);
		
		WheelHit CorrespondingGroundHit;
		FrontRightWheelCollider.GetGroundHit(out CorrespondingGroundHit);
		
		float normalizedLeanAngle = Mathf.Clamp(CorrespondingGroundHit.sidewaysSlip, -1f, 1f);
		
		if(normalizedLeanAngle > 0f)
			normalizedLeanAngle = 1;
		else
			normalizedLeanAngle = -1;
		
		horizontalLean = Mathf.Clamp(Mathf.Lerp (horizontalLean, (Mathf.Abs (transform.InverseTransformDirection(rigid.angularVelocity).y) * -normalizedLeanAngle) * 4f, Time.deltaTime * 3f), -3.0f, 3.0f);
		
		Quaternion target = Quaternion.Euler(verticalLean, chassis.transform.localRotation.y + (rigid.angularVelocity.z), horizontalLean);
		chassis.transform.localRotation = target;
		
		rigid.centerOfMass = new Vector3((COM.localPosition.x) * transform.localScale.x , (COM.localPosition.y) * transform.localScale.y , (COM.localPosition.z) * transform.localScale.z);
		
	}
		
	void OnCollisionEnter( Collision collision ){
		
		if (collision.contacts.Length > 0){
			
			if(collision.relativeVelocity.magnitude > 5 && crashClips.Length > 0){
				if (collision.contacts[0].thisCollider.gameObject.layer != LayerMask.NameToLayer("Wheel") ){
					crashAudio = CreateAudioSource("crashSound", 5f, 1f, crashClips[UnityEngine.Random.Range(0, crashClips.Length)], false, true, true);
				}
			}
			
		}
		
	}

	void  OnTriggerStay ( Collider other  ){
		
		if(other.gameObject.GetComponent<SAICBrakeZone>()){
			if(other.gameObject.GetComponent<SAICBrakeZone>()){
				if(speed >= other.gameObject.GetComponent<SAICBrakeZone>().targetSpeed)
					brakeInput = 1; 
				else
					brakeInput = 0;
			}else{
				if(speed > 30)
					brakeInput = 1;
				else
					brakeInput = 0;
			}
		}
		
	}
		
	void  OnTriggerExit ( Collider other  ){
			
		if(other.gameObject.tag == "BrakeZone"){
			brakeInput = 0;
		}
			
	}
		
}