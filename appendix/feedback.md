# Appraisal/Feedback for NVIDIA

NV feedback
  - Too tied into NV tooling
    - launch uses an NV wrapper that is not clear how to decipher. Also hard to integrate with as a result for anyone else.
  - Software is too single-purpose specific
    - Only works on Nova
    - Only works if exactly 1 workflow is followed setting env var or configs for applications (fragile)
    - Things are nested in uninutitive ways that make it impossible for reuse.
    - Can't really be reapplied (at least not easily) to any other situation but the one NV created to make the demo
  - Docs are interconnected and non-linear, it takes hours of back and forth to extract information and still feel like its not telling me the full story on configurations, setup, how to work with it
  - TensorRT models/engines may need to be rebuilt as a result of package version drift between the Docker images and the onboard system installed packages. 
  - it requires an end-user to be familiar with large parts of the NVIDIA overall tech stack in order to effectively use the software beyond anything but a rudimentary canned demo. 
