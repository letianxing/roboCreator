# Execution notes

The initial scan process was stopped after checking its exact owned PID/command, because it repeatedly recomputed fixed-body bounds and expensive full intersections and had not completed a candidate. No result was declared complete. The revised scan caches only fixed-object bounds and invariant within-head pairs; changing head/body and pitch-housing pairs remain checked. Exact distances reject separated pairs before Boolean volume calculation. Sources were not changed.

The first swept-opening run was stopped after four saved neutral-yaw samples, with `complete:false` retained. The earlier full hardware trial already revealed camera/neck/UAC-carrier interference at larger pitch. A separate final trial changes the proposed pitch origin and camera neutral placement and adds a camera opening; it does not overwrite the interrupted sample evidence.

Superseded process stopping used exact owned command matching. By that time the generic scan had already finished (9x13 pitch samples and20 yaw/pitch probes, with its own complete flag); only the first hardware process was still running and was stopped after23 saved cases, retaining `complete:false`. The first hardware study had demonstrated failures at larger pitch. Work proceeds on the separate final manufacturer-geometry variant. No unfinished scan is promoted to a passed design.
