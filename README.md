# Sasha handover
Action server for grasped object handover.
 
Handover topic is ```/handover```

Messages can be found in the [grasping_pipeline_msgs](https://github.com/v4r-tuwien/grasping_pipeline_msgs) repository.
``` 
Goal:
float32 force_thresh 
Result:
Empty
```

Action Goal is a threshhold for the force sensor in the wrist. If set to zero, it uses the default value of 0.2, which is already pretty low. 

    