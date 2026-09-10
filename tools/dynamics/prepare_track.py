#!/usr/bin/env python3
"""Separate terrain-derived road profile for dynamic experiments; preserve M1–5 track."""
import json
from pathlib import Path
import numpy as np
from scipy.ndimage import gaussian_filter1d
ROOT=Path(__file__).resolve().parents[2]
def main():
 path=ROOT/'configs/tracks/spa-francorchamps.json';j=json.loads(path.read_text());points=j['control_points'];xy=np.array([[p['x_m'],p['y_m']] for p in points]);z=np.array([p['elevation_m'] for p in points]);spacing=float(np.mean(np.linalg.norm(np.roll(xy,-1,axis=0)-xy,axis=1)))
 # 30 m DSM cannot support 10 m paving undulations. Gaussian sigma=35 m attenuates
 # sub-resolution artifacts; no adjustment to XY or enforced published length.
 sigma_m=35.;filtered=gaussian_filter1d(z,sigma_m/spacing,mode='wrap',truncate=4.)
 for p,height in zip(points,filtered):p['elevation_m']=round(float(height),6)
 j['elevation']['dynamic_filter']={'type':'periodic Gaussian','sigma_m':sigma_m,'mean_node_spacing_m':spacing,'source_config':path.name,'rms_height_change_m':float(np.sqrt(np.mean((filtered-z)**2))),'max_height_change_m':float(np.max(np.abs(filtered-z))),'source_height_range_m':float(np.ptp(z)),'filtered_height_range_m':float(np.ptp(filtered)),'reason':'Suppress short-wavelength terrain/vegetation DSM artifacts exposed by unilateral contact. This is not surveyed paving or verified Spa crest curvature.'}
 j['elevation']['limitation']+=' Additional dynamic filtering changes crest shape; no real-car airborne prediction is justified.'
 j['source_length_validation']=j.pop('length_validation')
 out=path.with_name('spa-francorchamps-dynamic.json');out.write_text(json.dumps(j,indent=2)+'\n')
 # Measure the actual production spline after filtering; never reuse the source length.
 from native import Native
 with Native(track='spa-francorchamps') as n:length=n.snapshot(True)['length']
 reference=j['source_length_validation']['reference_length_m']
 j['length_validation']={**j['source_length_validation'],'imported_length_m':length,'absolute_difference_m':abs(length-reference),'percentage_difference':abs(length-reference)/reference*100,'measurement':'production C++ 3D periodic spline after explicitly documented dynamic elevation filter; no forced scaling'}
 out.write_text(json.dumps(j,indent=2)+'\n');print(json.dumps({'filter':j['elevation']['dynamic_filter'],'length':j['length_validation']},indent=2))
if __name__=='__main__':main()
