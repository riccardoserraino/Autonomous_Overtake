// Build: bundle src/main.js (+ three.js) into ONE self-contained HTML file.
import { build } from 'esbuild';
import fs from 'fs';
const r = await build({ entryPoints: ['src/main.js'], bundle: true, minify: true, format: 'iife', target: 'es2020', write: false, legalComments: 'none' });
let js = r.outputFiles[0].text.replace(/<\/script/gi, '<\\/script');
const html = fs.readFileSync('src/index.html', 'utf8').replace('<script>/*BUNDLE*/</script>', () => '<script>' + js + '</script>');
fs.mkdirSync('dist', { recursive: true });
fs.writeFileSync('dist/webdrivesim.html', html);
console.log('dist/webdrivesim.html', (html.length / 1024).toFixed(0), 'kB');
