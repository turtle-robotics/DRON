using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

public class ROS2Manager : MonoBehaviour
{
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<std_msgs.msg.String> chatter_pub;
    private ISubscription<std_msgs.msg.Float64> chatter_sub;
    private ISubscription<std_msgs.Float32MultiArray> temp_sub;

    // Start is called before the first frame update
    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();

        if (ros2Unity.Ok()) {
            ros2Node = ros2Unity.CreateNode("ROS2UnityListenerNode");
            //chatter_pub = ros2Node.CreatePublisher<std_msgs.msg.String>("chatter");
            chatter_sub = ros2Node.CreateSubscription<std_msgs.msg.Float64>("/temperature", msg => Debug.Log("Unity listener heard: [" + msg.Data + "]"));
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (ros2Unity.Ok()) {
            //std_msgs.msg.String msg = new std_msgs.msg.String();
            //msg.Data = "Hello Ros2ForUnity!";
            //chatter_pub.Publish(msg);

            
        }
    }
}
