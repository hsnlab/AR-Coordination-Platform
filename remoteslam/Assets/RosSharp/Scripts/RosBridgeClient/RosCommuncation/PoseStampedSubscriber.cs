/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class PoseStampedSubscriber : Subscriber<Messages.Geometry.PoseStamped>
    {
        public Transform PublishedTransform;

        public static Vector3 position;
        public static Quaternion rotation;
        private bool isMessageReceived;
        bool inited = false;

        public static string RosTopic = "";

        private void Awake()
        {
            this.Topic = RosTopic;
        }

        protected override void Start()
        {
            base.Start();
        }
		
        private void Update()
        {
            if (!inited)
            {
                base.Start();
                inited = true;
            }
        }

        public Quaternion Ros2Unity(Quaternion quaternion)
        {
            return new Quaternion(quaternion.y, -quaternion.z, -quaternion.x, quaternion.w);
        }

        protected override void ReceiveMessage(Messages.Geometry.PoseStamped message)
        {
            Vector3 pos = GetPosition(message).Ros2Unity();
            rotation = GetRotation(message).Ros2Unity();
            position = pos;
            Debug.Log("PoseStamped received. Position: "+pos.ToString());
            long timestamp = (long)message.header.stamp.secs * 1000000000 + message.header.stamp.nsecs;
            if (NativeCameraHandler.measure_is_running)
            {
                NativeCameraHandler.SetPositionTimestamp(timestamp, pos,rotation);
                NativeCameraHandler.LogString("TimeMeasure: PoseReceived: " + timestamp + " , " + NativeCameraHandler.GetTime());
            }


            isMessageReceived = true;
        }

        private void ProcessMessage()
        {
            PublishedTransform.position = position;
            PublishedTransform.rotation = rotation;
        }

        private Vector3 GetPosition(Messages.Geometry.PoseStamped message)
        {
            return new Vector3(
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z);
        }

        private Quaternion GetRotation(Messages.Geometry.PoseStamped message)
        {
            return new Quaternion(
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w);
        }
    }
}