// Run with Node and Playwright available: node tests/test_dashboard.cjs
const { chromium } = require('playwright');
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');

(async () => {
    const browser = await chromium.launch({headless:true,args:['--no-sandbox']});
    try {
        const page = await browser.newPage({viewport:{width:1000,height:1000}});
        const errors=[]; page.on('pageerror',e=>errors.push(e.message));
        let disconnected=false,stale=false,freeze=false,tick=1000;
        const state={ready:true,paused:false,state:'Following',reason:'Tracking white line',
            bits:4,white_bits:4,inverted:false,left:40,right:40,error:0,raw:[3900,3800,100,3800,3900],values:[40,70,980,70,40],
            black:[4095,4095,4095,4095,4095],white:[0,0,0,0,0],kp:.9,ki:0,kd:6.5,
            threshold:820,hysteresis:20,cruise:40,turn:35,black_captured:false,white_captured:false,
            ap:'192.168.4.1',sta:'0.0.0.0',events:[{ms:1000,text:'Tracking white line'}]};
        let settingsBody;
        await page.route('http://maze.test/**',async route=>{
            const url=new URL(route.request().url());
            if(url.pathname==='/') return route.fulfill({contentType:'text/html',body:fs.readFileSync(path.join(__dirname,'../frontend/index.html'),'utf8')});
            if(url.pathname==='/api/state') {
                if(disconnected) return route.abort('failed');
                if(!freeze) tick+=200;
                return route.fulfill({contentType:'application/json',body:JSON.stringify({...state,ms:tick,age_ms:stale?2000:2})});
            }
            const body=route.request().postDataJSON();
            if(url.pathname==='/api/run') { state.paused=!body.running;state.state=state.paused?'Paused':'Following'; }
            if(url.pathname==='/api/settings') {settingsBody=body;Object.assign(state,body);}
            if(url.pathname==='/api/calibrate') state[body.surface+'_captured']=true;
            return route.fulfill({contentType:'text/plain',body:'Applied and saved'});
        });
        await page.goto('http://maze.test/');
        await page.waitForFunction(()=>document.querySelector('#connection').textContent.startsWith('Live'));
        assert.equal(await page.locator('.sensor').count(),5);
        assert.equal(await page.locator('.detect.white').count(),1);
        assert.equal(await page.locator('#run').isEnabled(),true);
        state.inverted=true; state.white_bits=27;
        await page.waitForFunction(()=>document.querySelector('#lineMode').textContent.startsWith('BLACK LINE'));
        assert.equal(await page.locator('.detect.white').count(),4);
        assert.equal(await page.locator('.detect').nth(2).textContent(),'BLACK · LINE');
        state.inverted=false; state.white_bits=4;
        await page.locator('#kp').fill('1.2');
        await page.waitForTimeout(450); assert.equal(await page.locator('#kp').inputValue(),'1.2');
        await page.locator('#apply').click();
        await page.waitForFunction(()=>document.querySelector('#message').textContent==='Applied and saved');
        assert.equal(settingsBody.kp,1.2);
        await page.locator('summary').click();
        assert.equal(await page.locator('#black').isDisabled(),true);
        await page.locator('#run').click();
        await page.waitForFunction(()=>document.querySelector('#state').textContent==='Paused');
        assert.equal(await page.locator('#black').isEnabled(),true);
        await page.locator('#black').click();
        await page.waitForFunction(()=>document.querySelector('#capture').textContent.includes('black yes'));
        await page.locator('#run').click();
        await page.waitForFunction(()=>document.querySelector('#state').textContent==='Following');
        stale=true;
        await page.waitForFunction(()=>document.querySelector('#connection').textContent==='Stale controller data');
        assert.equal(await page.locator('#run').isDisabled(),true);
        stale=false;freeze=true;
        await page.waitForTimeout(1400);
        assert.equal(await page.locator('#connection').textContent(),'Stale controller data');
        freeze=false;disconnected=true;
        await page.waitForFunction(()=>document.querySelector('#connection').textContent.includes('Disconnected'));
        disconnected=false;
        Object.assign(state,{state:'Searching left',reason:'No white: continuous left search',bits:0,left:-35,right:35,
            raw:[3900,3800,3950,3800,3900],values:[40,70,35,70,40]});
        await page.waitForFunction(()=>document.querySelector('#state').textContent==='Searching left');
        assert.equal(await page.locator('#left').textContent(),'35% reverse');
        for(const width of [1000,390]) {
            await page.setViewportSize({width,height:1100});
            assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),true);
            if(process.env.MAZE_SCREENSHOT_DIR) await page.screenshot({path:path.join(process.env.MAZE_SCREENSHOT_DIR,`maze-${width}.png`),fullPage:true});
        }
        assert.deepEqual(errors,[]);
        console.log('PASS: live readings, PID edit preservation/submission, pause/resume, calibration gating, stale/frozen/disconnected recovery and desktop/mobile layout');
    } finally { await browser.close(); }
})().catch(e=>{console.error(e);process.exitCode=1;});
