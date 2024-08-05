using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;					// for collision


	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	bool Collision_Detection(Vector3 x_i, Vector3 v_i, Vector3 P, Vector3 N)
	{
		// In this function, detect the collision with the plane <P, N>
		// If collision happens, update v and w
		if (Vector3.Dot(x_i - P, N) < 0 && Vector3.Dot(v_i, N) < 0)
		{
			return true;
		}
		return false;
	}

	public static Matrix4x4 Subtract(Matrix4x4 a, Matrix4x4 b)
    {
        Matrix4x4 result = new Matrix4x4();

        // Iterate through each element and perform subtraction
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                result[i, j] = a[i, j] - b[i, j];
            }
        }

        return result;
    }

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		MeshFilter meshFilter = GetComponent<MeshFilter>();
        Mesh mesh = meshFilter.mesh;
		Vector3[] vertices = mesh.vertices;

		Vector3 x = transform.position;
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
		Vector3 X = Vector3.zero;
		int num = 0;

        foreach (Vector3 vertex in vertices)
        {
            Vector3 x_i = R.MultiplyPoint3x4(vertex) + x;
			Vector3 v_i = v + Vector3.Cross(w, R.MultiplyPoint3x4(vertex));
			if (Collision_Detection(x_i, v_i, P, N))
			{
				X += x_i;
				num++;
			}
        }

		if (num == 0) return;

		// at this point, X is actually sum of x_i + Rr_i
		X = X / num;
		X = X - x;
		// after subtracting x, X is now Rr_i, so need to multiply R
		Vector3 V = v + Vector3.Cross(w, X);
		Vector3 Vn = Vector3.Dot(V, N) * N;
		Vector3 Vt = V - Vn;
		Matrix4x4 I = R * I_ref * R.inverse;

		float a = Mathf.Max(1 - 0.5f * (1 + restitution) * Vn.magnitude / Vt.magnitude, 0);
		Vector3 V_new = - restitution * Vn + a * Vt;

		Matrix4x4 K = Matrix4x4.identity;
		for(int i = 0; i < 4; i++)
		{
			K[i, i] /= mass;
		}
		
		K = Subtract(K, Get_Cross_Matrix(X) * I.inverse * Get_Cross_Matrix(X));
		Vector3 J = K.inverse * (V_new - V);

		v += J / mass;
		w += I.inverse.MultiplyVector(Vector3.Cross(X, J));
		
		// 衰减碰撞因子
		if (v.magnitude < 0.01f)
		{
			restitution = 0;
		}
		Debug.Log("J / mass: " + J / mass);
		Debug.Log("I.inverse.MultiplyVector(Vector3.Cross(R.MultiplyPoint3x4(X), J)): " + I.inverse.MultiplyVector(Vector3.Cross(X, J)));
        
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5f, 2f, 0);
			w = new Vector3 (2f, 0, 0);
			launched=true;
		}

		// Part I: Update velocities
		if(launched)
		{
			v = v + new Vector3(0, -9.8f, 0) * dt;
			w = w * angular_decay;

			// Part II: Collision Impulse
			Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
			Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

			// Part III: Update position & orientation
			//Update linear status
			Vector3 x    = transform.position;
			v.x = v.x * linear_decay;
			v.z = v.z * linear_decay;
			x += v * dt;
			//Update angular status
			Quaternion q = transform.rotation;
			Quaternion deltaRotation = Quaternion.Euler(w * Mathf.Rad2Deg * dt);
			q = q * deltaRotation;

			// Part IV: Assign to the object
			transform.position = x;
			transform.rotation = q;
		}
	}
}
