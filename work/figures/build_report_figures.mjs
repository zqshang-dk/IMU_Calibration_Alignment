/** Read-only input bridge. Run from any directory: node build_report_figures.mjs
 * Node handles file I/O; Python computes and renders exclusively in memory.
 * Outputs are confined to this script's directory. No src/config/result changes.
 */
import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import {fileURLToPath} from 'node:url';
import {spawnSync} from 'node:child_process';
const out = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(out, '../..');
const args = process.argv.slice(2);
const figures = ["acc_compensation_x","acc_compensation_y","acc_compensation_z","acc_mean_residuals","gyro_rotation_windows","gyro_compensation","alignment_observations","alignment_second","alignment_epoch","alignment_averaging"];
if (args.length && (args.length !== 2 || args[0] !== '--figure' || !figures.includes(args[1]))) {
  throw new Error('Usage: node build_report_figures.mjs [--figure NAME]');
}
const inputs = {};
const hashes = {};
for (const dir of ['work/asc/calibration', 'work/asc/alignment']) {
  for (const name of fs.readdirSync(path.join(root, dir)).filter(n => n.endsWith('.ASC'))) {
    const rel = `${dir}/${name}`;
    const bytes = fs.readFileSync(path.join(root, rel));
    hashes[rel] = crypto.createHash('sha256').update(bytes).digest('hex');
    inputs[name] = bytes.toString('utf8');
  }
}
for (const name of fs.readdirSync(path.join(root, 'src')).filter(n => /\.(cpp|h|txt|py)$/.test(n))) {
  const rel = `src/${name}`;
  hashes[rel] = crypto.createHash('sha256').update(fs.readFileSync(path.join(root,rel))).digest('hex');
}
const run = spawnSync(process.env.IMU_REPORT_PYTHON || 'python', [path.join(out, 'report_figures.py'), ...args], {
  input: JSON.stringify(inputs), encoding: 'utf8', maxBuffer: 64 * 1024 * 1024,
  env: {...process.env, PYTHONIOENCODING:'utf-8', PYTHONDONTWRITEBYTECODE:'1'},
});
if (run.stderr) process.stderr.write(run.stderr);
if (run.status !== 0) throw new Error(`Analysis failed: ${run.status}`);
const result = JSON.parse(run.stdout);
for (const [name, base64] of Object.entries(result.images)) {
  if (!/^[a-z0-9_]+\.png$/.test(name)) throw new Error('Unsafe output name');
  fs.writeFileSync(path.join(out,name), Buffer.from(base64,'base64'));
}
delete result.images;
result.input_sha256 = hashes;
if (!args.length) {
  fs.writeFileSync(path.join(out,'report_statistics.json'), JSON.stringify(result,null,2)+'\n');
}
console.log(args.length ? `Generated ${args[1]}.png` : `Generated ${figures.length} figures and report_statistics.json`);
