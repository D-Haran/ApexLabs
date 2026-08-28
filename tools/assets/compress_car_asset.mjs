// Lossless EXT_meshopt_compression with a required decoder and no duplicated fallback payload.
import {readFile,writeFile} from 'node:fs/promises';
import {MeshoptEncoder,MeshoptDecoder} from '../../apps/dashboard/node_modules/meshoptimizer/index.module.js';
await Promise.all([MeshoptEncoder.ready,MeshoptDecoder.ready]);
for(const path of process.argv.slice(2)) {
 const file=await readFile(path),jl=file.readUInt32LE(12),doc=JSON.parse(file.subarray(20,20+jl)),bin=file.subarray(28+jl);
 if(doc.extensionsRequired?.includes('EXT_meshopt_compression'))continue;
 const chunks=[];let offset=0,uncompressed=0;
 const push=(bytes)=>{const pad=Buffer.alloc((4-offset%4)%4);chunks.push(pad);offset+=pad.length;const start=offset;chunks.push(bytes);offset+=bytes.length;return start;};
 for(let i=0;i<doc.bufferViews.length;i++) {
  const view=doc.bufferViews[i],bytes=bin.subarray(view.byteOffset??0,(view.byteOffset??0)+view.byteLength),a=doc.accessors.find(a=>a.bufferView===i);
  if(a) {
   const stride=view.byteLength/a.count,mode=a.componentType===5125?'INDICES':'ATTRIBUTES';
   const packed=MeshoptEncoder.encodeGltfBuffer(bytes,a.count,stride,mode);
   const decoded=new Uint8Array(bytes.length);MeshoptDecoder.decodeGltfBuffer(decoded,a.count,stride,packed,mode);
   if(!Buffer.from(decoded).equals(bytes))throw new Error('Compression roundtrip mismatch');
   view.extensions={EXT_meshopt_compression:{buffer:0,byteOffset:push(packed),byteLength:packed.length,byteStride:stride,count:a.count,mode}};
   view.buffer=1;view.byteOffset=uncompressed;uncompressed+=view.byteLength;
  }else {view.byteOffset=push(bytes);view.buffer=0;}
 }
 doc.extensionsUsed=[...new Set([...(doc.extensionsUsed??[]),'EXT_meshopt_compression'])];doc.extensionsRequired=[...new Set([...(doc.extensionsRequired??[]),'EXT_meshopt_compression'])];
 doc.buffers=[{byteLength:offset},{byteLength:uncompressed,extensions:{EXT_meshopt_compression:{fallback:true}}}];
 const json=Buffer.from(JSON.stringify(doc));const jp=Buffer.alloc((4-json.length%4)%4,32);const bp=Buffer.alloc((4-offset%4)%4);const b=Buffer.concat([...chunks,bp]);const j=Buffer.concat([json,jp]);const head=Buffer.alloc(20);head.writeUInt32LE(0x46546c67);head.writeUInt32LE(2,4);head.writeUInt32LE(28+j.length+b.length,8);head.writeUInt32LE(j.length,12);head.writeUInt32LE(0x4e4f534a,16);const bh=Buffer.alloc(8);bh.writeUInt32LE(b.length);bh.writeUInt32LE(0x004e4942,4);
 const output=Buffer.concat([head,j,bh,b]);await writeFile(path,output);
 const mp=path.replace(/\.glb$/,'.json'),manifest=JSON.parse(await readFile(mp,'utf8'));manifest.uncompressedProcessedBytes=manifest.processedBytes;manifest.processedBytes=output.length;manifest.compression='lossless EXT_meshopt_compression, each buffer verified byte-for-byte';await writeFile(mp,JSON.stringify(manifest,null,2)+'\n');console.log(path,file.length,'->',output.length);
}
