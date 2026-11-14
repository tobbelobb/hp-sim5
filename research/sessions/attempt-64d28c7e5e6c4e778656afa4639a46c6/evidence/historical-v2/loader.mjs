import {registerHooks} from 'node:module';
import fs from 'node:fs';
import {createHash} from 'node:crypto';
const spec=JSON.parse(fs.readFileSync(new URL('./snapshot.json',import.meta.url)));
const source=fs.readFileSync(new URL('./wrapper.js',import.meta.url),'utf8');
if(createHash('sha256').update(source).digest('hex')!==spec.wrapper_sha256) throw Error('Snapshot wrapper was modified');
registerHooks({load(url,context,nextLoad){
 if(url===spec.original_wrapper_url){
  fs.appendFileSync(new URL('./loads.jsonl',import.meta.url),JSON.stringify({pid:process.pid,url,wrapper_sha256:spec.wrapper_sha256})+'\n');
  return {format:'module',source,shortCircuit:true};
 }
 return nextLoad(url,context);
}});
