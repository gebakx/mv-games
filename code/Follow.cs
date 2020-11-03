using UnityEngine;
using UnityEngine.AI;
using PathCreation;
using System.Collections;

public class Follow : MonoBehaviour
{
    public GameObject robber;
    public GameObject treasure;
    public NavMeshAgent agent;
    public PathCreator pathCreator;
    public EndOfPathInstruction endOfPathInstruction;
    public float speed = 5;
    float distanceTravelled;

    void Start()
    {
        if (pathCreator != null)
        {
            distanceTravelled = pathCreator.path.GetClosestDistanceAlongPath(transform.position); 
            agent.destination = pathCreator.path.GetPointAtDistance(distanceTravelled, endOfPathInstruction);
        };
    }

    void Update()
    {
        if (Vector3.Distance(treasure.transform.position, robber.transform.position) < 10f)
        {
            agent.destination = robber.transform.position;
            agent.isStopped = false;
        }        
        else
            if (agent.remainingDistance > 0.2f)
            {
                distanceTravelled = pathCreator.path.GetClosestDistanceAlongPath(transform.position); 
                agent.destination = pathCreator.path.GetPointAtDistance(distanceTravelled, endOfPathInstruction);
            }
            else
            {
                agent.isStopped = true;
                if (pathCreator != null)
                {
                    distanceTravelled += speed * Time.deltaTime;
                    transform.position = pathCreator.path.GetPointAtDistance(distanceTravelled, endOfPathInstruction);
                    transform.rotation = pathCreator.path.GetRotationAtDistance(distanceTravelled, endOfPathInstruction);
                }        
            }
    }
}
