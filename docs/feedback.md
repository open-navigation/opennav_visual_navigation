# Appraisal/Feedback for NVIDIA

NV feedback

  - Too tied into NV tooling
    - launch uses an NV wrapper that is not clear how to decipher. Also hard to integrate with as a result for anyone else.
  - Software is too single-purpose specific
    - None works without Nova
    - Only works if exactly 1 workflow is followed setting env var or configs for applications (fragile)
    - Things are nested in uninutitive ways that make it impossible for reuse. Recorder launches teleop of robot base. perceptor includes Nav2 + configs can't rip out. Hardware is launched when enabling perception stack (backwards) that make it inoperable for anyone else - can't do anything without restarting.
    - Can't really be reapplied (at least not easily) to any other stiuation but the one NV created to make the demo
  - Docs are twisted, interconnected and non-linear, it took me hours of back and forth to extract information and still feel like its not telling me the full story on configurations, setup, how to work with it outside of the 1 situation using the nova platform, etc

Familiar Feedback

- Docs indeed internecine. 
- DisplayPort passthrough on robot front-panel may not be compatible with all end-user monitors. 
- System should default to exposing AP mode for initial network configuration. 
- Add nmcli commands for CLI-based WLAN config.
- TensorRT models/engines may need to be rebuilt as a result of package version drift between the Docker images and the onboard system installed packages. 
- Heavy dependencies on (seemingly) unrelated libraries and programs 
  - More to the point, it requires an end-user to be familiar with large parts of the NVIDIA overall tech stack in order to effectively use the software beyond anything but a rudimentary canned demo. 
- TBD: Try -v /etc/localtime:/etc/localtime:ro as a way to check against some of the drift problems between system and container, the launch.log for the Perceptor demo has messages about timestamps being off, cameras not in sync. 

--> Had to change from a codebase intended as a forkable platform that folks could modify for their situation as a working demo to build off of and INSTEAD make it a technology demonstration of only the 1 situation we have with the Nova Carter to show its possible, but only in this narrow situation. Tech demo to show its possible rather than a usable platform for users to build from for their unique needs.