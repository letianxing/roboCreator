import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'11_verification/yoke_c3d10_v10'
r=json.loads((OUT/'audit.json').read_text())['meshes']
x=[v['h_mm'] for v in r]
fig,axes=plt.subplots(1,2,figsize=(11,4.5))
axes[0].plot(x,[v['max_u_mm'] for v in r],'o-',label='Maximum displacement')
axes[0].plot(x,[-v['load_weighted_uz_mm'] for v in r],'s-',label='Load-weighted vertical displacement')
axes[0].set_ylabel('Displacement / mm');axes[0].legend(fontsize=8)
axes[1].plot(x,[v['peak_vm_MPa'] for v in r],'o-',color='#bf4e35')
axes[1].set_ylabel('Peak integration-point von Mises / MPa')
for a in axes:
    a.invert_xaxis();a.grid(alpha=.3);a.set_xlabel('Target element size / mm (refining to the right)')
fig.suptitle('v0.9 yoke / C3D10 / assumed 100 N / idealized face clamp + radial journal')
fig.tight_layout(rect=[0,.08,1,.92])
fig.text(.04,.025,'Displacement stabilizes; local peak stress and real joint compliance are NOT validated.',fontsize=9)
fig.savefig(OUT/'convergence.png',dpi=160);plt.close(fig)
print(OUT/'convergence.png')
