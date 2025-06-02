# Testing & Troubleshooting

## 🛠️ Common Issues & Fixes

Below are some known issues you might encounter when working with ARI, along with suggested troubleshooting steps.

### Robot Not Moving After Goal Sent

If a goal has been sent to `move_base` but ARI isn’t moving, it’s likely due to a crash or malfunction.  
**Fix:** Check the Web Commander interface at `http://<ARI_IP>:8080` to inspect running processes and ensure the cameras and other critical modules are functioning properly.

### Multibody System Not Responding

If the multibody system isn't functioning when ROI, ID, and cropped image messages are being sent:  
**Fix:** Verify that these messages are published with correct real-time timestamps. Proper synchronization is essential for the system to work.

### Face Recognition Errors (YOLO)

Face recognition using YOLO is not foolproof—blurry frames can lead to incorrect person entries, causing issues over time.  
**Fix:**

1. SSH into the robot: `ssh pal@ari`
2. Navigate to the folder: `deployed_ws/lib/dd2414_human_detection/`
3. Delete the existing JSON file.  
A new file will be generated on the next face recognition cycle, but note that this will reset all previously stored person data.

### Head stops moving

When a frame is published to the look\_at topic which the robot can’t turn the head to, expressive\_eyes crashes. This can happen when ARI is moving while turning the head.

**Fix:**

- Check the Web Commander interface at `http://<ARI_IP>:8080` for expressive\_eyes. If it has stopped running, restart it.

### Navigation in Narrow Spaces

To navigate tight areas like office doors, ARI’s obstacle radius is dynamically reduced in designated "door zones."  
**Note:**

- Ensure all narrow doors are marked as door zones in the map (“door\_XX”).
- This allows ARI to turn fully and pass through safely without collisions.

### Accessibility Limitations

Most rooms were accessible via ramps; however, the **MOCAP room** contains a step that ARI cannot cross—even with a ramp.

### Microphone Limitations & Speaker Detection

ARI’s microphones are arranged in a circular array at the front. This setup struggles with locating speakers behind the robot, especially in small, echo-prone rooms.  
**Solution:**

- Speaker detection is handled via **multimodal integration**.
- ARI uses the microphones only to detect the presence of speech and estimate the turning direction.
- She stops turning once a person is visually detected using the head camera.

* * *
