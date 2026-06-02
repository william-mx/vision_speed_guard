## Enriching ROS2 Bagfiles with Speed Limits

To generate reusable control data, we replay recorded bagfiles and run our speed controller node.  
The node computes a speed limit from the perception input and publishes it as a ROS topic.

### Key Idea

We use:

* `ros2 bag play --clock` to replay the original bag with its recorded timestamps
* `use_sim_time=true` so the speed controller uses the bag time
* topic remapping to publish the generated speed limit to `/true/speed_limit`
* `ros2 bag record --use-sim-time` to record the new topic with the correct timestamps

This keeps the generated speed limit **time-aligned with the original bagfile**.

---

### Commands

Run the speed controller and remap its speed limit output:

```bash
ros2 run vision_speed_guard speed_guard --ros-args \
  -p use_sim_time:=true \
  -r /speed_limit:=/true/speed_limit
````

Record the original relevant topics and the generated speed limit:

```bash
ros2 bag record --use-sim-time -s mcap \
  -o src/yolo_vision/bagfiles/cp_signs_speed_augmented \
  /rc/ackermann_cmd \
  /camera/camera/color/image_raw \
  /detections_2d \
  /label_mapping \
  /mask \
  /yolo_overlay \
  /tf_static \
  /true/speed_limit
```

Replay the original bagfile with simulated time:

```bash
ros2 bag play -r 0.5 /ros2_ws/src/yolo_vision/bagfiles/cp_signs_augmented --clock
```

---

### Why we do this

We generate the speed limit once and save it into a new bagfile.
This allows us to test the drive manager and downstream control logic without running the speed controller every time.

The remapping makes it clear that `/true/speed_limit` is the recorded reference speed limit generated from the perception policy.

---

### Result

We obtain a new bagfile that contains:

* original driving commands, for example `/rc/ackermann_cmd`
* perception input, for example `/detections_2d`
* generated speed limit output `/true/speed_limit`

All topics use **consistent simulated timestamps**, as if they were recorded live together.

