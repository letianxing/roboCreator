# v26 STEP repair artifacts

- `repaired/`: current four repaired native FCStd and full STEP assemblies.
- `repair_results.json`: paired-geometry checks, comparison metrics, source/output hashes.
- `verification.json`: independent fresh-load and interface checks; read explicit statuses.
- `diagnosis.json`: original fault localization and unsuccessful automatic-healing trials.
- `phase0_attempt/`: earlier pose-dependent result, not the selected output.
- Other trial STEP/BREP/JSON files: retained diagnostics, not released models.

Run `09_cad/repair_step_v26.py`, then `11_verification/check_step_repair_v26.py` using existing FreeCAD Python after source CAD regeneration. Input geometry is frozen to v25's two28mm eye-spacing rods. Source files are not overwritten. Manufacturing/CAE/physical motion approval remains separate.
