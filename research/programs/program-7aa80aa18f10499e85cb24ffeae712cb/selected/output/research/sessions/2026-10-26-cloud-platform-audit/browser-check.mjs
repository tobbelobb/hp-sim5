import puppeteer from '/workspace/hp-sim5/node_modules/puppeteer/lib/esm/puppeteer/puppeteer.js';
import fs from 'node:fs/promises';
import path from 'node:path';

const url = new URL(process.argv[2]);
if (url.protocol !== 'http:' || url.hostname !== '127.0.0.1' || !url.pathname.startsWith('/hp-sim5/')) {
  throw new Error('Use the exact owned loopback browser-service URL');
}
const label = process.argv[3] || 'browser';
if (!/^[a-z0-9-]+$/.test(label)) throw new Error('Use a simple artifact directory label');
const directory = path.join('/workspace/hp-sim5/output/research/sessions/2026-10-26-cloud-platform-audit', label);
await fs.mkdir(directory, {recursive: true});
let stop = false;
for (const signal of ['SIGTERM', 'SIGINT']) process.on(signal, () => {stop = true;});
const state = {kind: 'parent-owned headless Chromium connection', process_id: process.pid, started_at: new Date().toISOString(), errors: []};
const save = () => fs.writeFile(path.join(directory, 'result.json'), JSON.stringify(state, null, 2));
let browser;
try {
  browser = await puppeteer.launch({headless: true, executablePath: process.env.PUPPETEER_EXECUTABLE_PATH || '/usr/bin/chromium',
    args: ['--no-sandbox', '--enable-unsafe-swiftshader']});
  const page = await browser.newPage();
  await page.setViewport({width: 1280, height: 900});
  page.on('pageerror', error => {state.errors.push(error.message); void save();});
  await page.goto(url.href, {waitUntil: 'domcontentloaded', timeout: 45000});
  await page.waitForSelector('canvas', {timeout: 30000});
  state.title = await page.title();
  state.canvas_count = await page.evaluate(() => document.querySelectorAll('canvas').length);
  state.connected_at = new Date().toISOString();
  await page.screenshot({path: path.join(directory, 'initial.png')});
  await save();
  console.log(JSON.stringify({outcome: 'connected', title: state.title, canvas_count: state.canvas_count}));
  const deadline = Date.now() + 25 * 60 * 1000;
  while (!stop && Date.now() < deadline) {
    try {await fs.stat(path.join(directory, 'stop')); break;} catch (error) {if (error.code !== 'ENOENT') throw error;}
    await new Promise(resolve => setTimeout(resolve, 500));
  }
  await page.screenshot({path: path.join(directory, 'final.png')});
  state.outcome = 'closed';
} catch (error) {
  state.outcome = 'failed';
  state.error = error.message;
  process.exitCode = 1;
} finally {
  if (browser) await browser.close();
  state.closed_at = new Date().toISOString();
  await save();
}
