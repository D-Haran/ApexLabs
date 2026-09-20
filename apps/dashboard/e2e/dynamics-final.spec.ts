import {test,expect} from '@playwright/test';
import {writeFileSync} from 'node:fs';
test('Spa comparison, synchronized seek, optimization UI, gamepad and rendering measurements',async({page})=>{
 await page.setViewportSize({width:1672,height:944});const errors:string[]=[];page.on('pageerror',e=>errors.push(e.message));
 await page.goto('/?drive');await expect(page.getByRole('button',{name:'Start driving'})).toBeEnabled({timeout:45000});
 await page.getByLabel('Session view').selectOption('optimized');
 await page.getByLabel('Replay time').fill('175');await page.getByLabel('Replay time').dispatchEvent('input');await page.waitForTimeout(1800);
 await page.screenshot({path:'../../docs/screenshots/m6-target-comparison.png'});
 await page.getByLabel('Tire forces',{exact:true}).check();await page.getByRole('button',{name:'Inspect FL tire'}).click();
 await page.getByLabel('Force frame').selectOption('World');await page.screenshot({path:'../../docs/screenshots/m6-engineering-overlay.png'});
 await page.getByRole('button',{name:'Close tire'}).click();await page.getByLabel('Session view').selectOption('compare');await page.getByLabel('Replay time').fill('175');await page.getByLabel('Replay time').dispatchEvent('input');await page.waitForTimeout(800);
 await expect(page.locator('.drive-delta')).toContainText('-');await expect(page.getByText('vs reference lap',{exact:true})).toBeVisible();
 await page.screenshot({path:'../../docs/screenshots/m6-reference-optimized.png'});await page.locator('.drive-bottom').screenshot({path:'../../docs/screenshots/m6-telemetry-comparison.png'});await page.locator('.drive-map').screenshot({path:'../../docs/screenshots/m6-track-map-comparison.png'});
 await page.getByRole('button',{name:'IMPROVE',exact:true}).click();await page.getByLabel('Optimization objective').selectOption('custom');await page.screenshot({path:'../../docs/screenshots/m6-optimization-panel.png'});
 const before=await page.getByLabel('Replay time').inputValue();await page.locator('.drive-trace svg').first().click({position:{x:100,y:20}});expect(await page.getByLabel('Replay time').inputValue()).not.toEqual(before);
 const seek=[];for(const t of [40,80,120]){const started=Date.now();await page.getByLabel('Replay time').fill(String(t));await page.getByLabel('Replay time').dispatchEvent('input');await page.evaluate(()=>new Promise<void>(r=>requestAnimationFrame(()=>requestAnimationFrame(()=>r()))));seek.push(Date.now()-started);}
 const metrics=await page.evaluate(()=>Reflect.get(window,'__apexDrivePerformance'));writeFileSync('../../data/generated/dynamics/render-performance.json',JSON.stringify({viewport:[1672,944],...metrics,seek_to_two_frames_ms:seek,mode:'Headless Chromium on development host; not a manual hardware display benchmark'},null,2));
 await page.getByLabel('Session view').selectOption('drive');await page.getByRole('button',{name:'Reset session'}).click();
 await page.evaluate(()=>{Object.defineProperty(navigator,'getGamepads',{configurable:true,value:()=>[{axes:[-.4,0],buttons:Array.from({length:8},(_,i)=>({value:i===7?.6:0,pressed:i===7})),connected:true,mapping:'standard'}]});});
 await page.getByRole('button',{name:'Start driving'}).click();await page.waitForTimeout(1000);
 const state=await page.request.get('http://127.0.0.1:8765/state').then(r=>r.json());expect(state.throttle).toBeCloseTo(.6);expect(state.steering).toBeGreaterThan(.1);
 await page.getByRole('button',{name:'Pause driving'}).click();expect(errors).toEqual([]);
});
