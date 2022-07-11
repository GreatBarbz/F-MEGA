using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting.Dependencies.NCalc;
using UnityEngine;

public class Racer : MonoBehaviour
{
    public Vector3 hoverPos;
    
    public LayerMask groundMask;
    
    public float pGain = 20; // the proportional gain
    public float iGain = 0.5f; // the integral gain
    public float dGain = 0.5f; // differential gain
    private Vector3 integrator = Vector3.zero; // error accumulator
    private Vector3 lastError = Vector3.zero; 
    public Vector3 curPos = Vector3.zero; // actual Pos
    public Vector3 force = Vector3.zero; // current force
    
    public float pGainR = 20; // the proportional gain
    public float iGainR = 0.5f; // the integral gain
    public float dGainR = 0.5f; // differential gain
    private Vector3 integratorR = Vector3.zero; // error accumulator
    private Vector3 lastErrorR = Vector3.zero; 
    public Vector3 forceR = Vector3.zero;
    
    [SerializeField] 
    private float hoverRange, hoverDistance;
    public float hoverSpeed;

    [SerializeField] 
    private bool grounded;

    public float gravityStrenght;
    private float currentGravity;

    [SerializeField] 
    private float speed,turnSpeed;

    private Ray ray;
    private Rigidbody _rb;

    public bool landed;
    

    void Awake()
    {
        _rb = GetComponent<Rigidbody>();
    }

    private void Start()
    {
        hoverPos = transform.position;
    }

    void Update()
    {
        
    }
    
    void FixedUpdate()
    {
        Gravity();
        
        Hover();
         
         
        Torque();
        
       
        LinearMovements();
    }

    private void Gravity()
    {
        if (!grounded)
        {
           _rb.AddForce(Vector3.down * gravityStrenght);
           landed = false;
        }
        else
        {
             var locVel = transform.InverseTransformDirection(_rb.velocity);
             locVel = Vector3.zero;
            _rb.velocity = transform.TransformDirection(locVel);

            if (!landed)
            {
                var prevVel = _rb.velocity.magnitude;

                _rb.velocity = Vector3.zero;
                
                _rb.AddForce(transform.forward * prevVel);
                landed = true;
            }
        }
    }
    
    private void Hover()
    {
        ray = new Ray(transform.position, -transform.up * hoverRange);
        RaycastHit hit;
        if (Physics.Raycast(ray, out hit, hoverRange, groundMask))
        {
            grounded = true;
            
            var targetRot = Quaternion.FromToRotation(transform.up, hit.normal) * transform.rotation;
            
            PIDrot(transform.rotation.eulerAngles,targetRot.eulerAngles, Time.deltaTime);

            transform.rotation = Quaternion.Lerp(transform.rotation, targetRot, Time.deltaTime * hoverSpeed / 10);
            
            hoverPos = hit.point + hit.normal * hoverDistance;
            
            PIDpos();
            
            _rb.MovePosition(Vector3.MoveTowards(transform.position, transform.position + force, hoverSpeed * Time.deltaTime));
            
            Debug.DrawLine(hit.point, hoverPos, Color.red);
        }
        else
        {
            grounded = false;
            Debug.DrawRay(transform.position, -transform.up * hoverRange, Color.green);
        }
    }

    private void Torque()
    {
        var torque = Input.GetAxis("Horizontal") * turnSpeed * Time.fixedDeltaTime;
        
        var locVel = _rb.angularVelocity;

        locVel = torque * transform.up;

        _rb.angularVelocity = locVel;
    }

    private void LinearMovements()
    {
        var force = Input.GetAxis("Vertical") * speed * Time.fixedDeltaTime;
        
        _rb.AddForce(transform.forward * force);

        var locVel = _rb.velocity;
    }

    private void PIDpos()
    {
        curPos = transform.position;
        var error = hoverPos - curPos; // generate the error signal
        integrator += error; // integrate error
        var diff = (error - lastError); // differentiate error
        lastError = error;
        // calculate the force summing the 3 errors with respective gains:
        force = error * pGain + integrator * iGain + diff * dGain;
    }

    private void PIDrot(Vector3 actual, Vector3 target, float time)
    {
        var error = target - actual;
        integratorR += error * time;
        var diff = (error - lastErrorR) / time;
        lastErrorR = error;

        forceR = error * pGainR + integratorR * iGainR + diff * dGainR;
    }
}
