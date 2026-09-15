#!/usr/bin/env python3
"""Download a small, local, CC0 environment texture set; verify publisher hashes."""
import hashlib
import json
import urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'apps/dashboard/public/assets/environment'
def fetch(url):
 return urllib.request.urlopen(urllib.request.Request(url,headers={'User-Agent':'ApexLab asset import / research visualization'}),timeout=90).read()
def main():
 OUT.mkdir(parents=True,exist_ok=True);tree=json.loads(fetch('https://api.polyhaven.com/files/pine_tree_01'));sky=json.loads(fetch('https://api.polyhaven.com/files/kloofendal_48d_partly_cloudy_puresky'))
 selected={f'{key}.{"png" if key=="twig_alpha" else "jpg"}':tree[key]['1k']['png' if key=='twig_alpha' else 'jpg'] for key in ['twig_diff','twig_alpha','twig_nor_gl','bark_diff']}
 selected['sky.hdr']=sky['hdri']['1k']['hdr'];manifest=[]
 for name,source in selected.items():
  path=OUT/name;data=path.read_bytes() if path.exists() else fetch(source['url'])
  if hashlib.md5(data).hexdigest()!=source['md5']:raise RuntimeError('Asset checksum mismatch: '+name)
  path.write_bytes(data);manifest.append({'file':name,'bytes':len(data),'md5':source['md5'],'url':source['url'],'license':'CC0-1.0','publisher':'Poly Haven'})
 (ROOT/'assets/external/polyhaven-environment/asset-manifest.json').write_text(json.dumps({'assets':manifest,'tree_geometry':'ApexLab authored foliage cards using Poly Haven pine twig maps; not the 17-million-triangle source tree mesh','source_pages':['https://polyhaven.com/a/pine_tree_01','https://polyhaven.com/a/kloofendal_48d_partly_cloudy_puresky'],'license_url':'https://polyhaven.com/license'},indent=2)+'\n')
 print('Downloaded',sum(a['bytes'] for a in manifest),'bytes')
if __name__=='__main__':main()
