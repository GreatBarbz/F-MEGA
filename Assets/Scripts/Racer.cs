using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Racer : MonoBehaviour
{
    public LayerMask hoverMask;
    public float hoverDistance;
    public float hoverHeight;
    public float hoverSpeed;

    private Rigidbody rb;

   
    
    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void Update()
    {
        Ray ray = new Ray(transform.position, -transform.up);

        RaycastHit hit;
       
        if (Physics.Raycast(ray, out hit, hoverDistance, hoverMask))
        {
            if(rb.useGravity == true)
                rb.useGravity = false;

            MeshCollider meshCollider = hit.collider as MeshCollider;

            Mesh mesh = meshCollider.sharedMesh;

            Vector3[] normals = mesh.normals;
            int[] triangles = mesh.triangles;

            // Extract local space normals of the triangle we hit
            Vector3 n0 = normals[triangles[hit.triangleIndex * 3 + 0]];
            Vector3 n1 = normals[triangles[hit.triangleIndex * 3 + 1]];
            Vector3 n2 = normals[triangles[hit.triangleIndex * 3 + 2]];

            // interpolate using the barycentric coordinate of the hitpoint
            Vector3 baryCenter = hit.barycentricCoordinate;

            // Use barycentric coordinate to interpolate normal
            Vector3 interpolatedNormal = n0 * baryCenter.x + n1 * baryCenter.y + n2 * baryCenter.z;

            // normalize the interpolated normal
            interpolatedNormal = interpolatedNormal.normalized * hoverHeight;

            // Transform local space normals to world space
            //Transform hitTransform = hit.collider.transform;
            //interpolatedNormal = hitTransform.TransformDirection(interpolatedNormal);

            Vector3 target = hit.point + interpolatedNormal;

            // Display with Debug.DrawLine
            Debug.DrawRay(hit.point, interpolatedNormal, Color.red);

            rb.MovePosition(Vector3.MoveTowards(transform.position, target, hoverSpeed * Time.deltaTime));
        }
        else
        {
            if (rb.useGravity == false)
                rb.useGravity = true;
        }
    }
}
