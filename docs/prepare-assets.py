#!/usr/bin/env python3
"""Copy the locked npm runtimes into the static files served by Sphinx."""
from pathlib import Path
import shutil
import subprocess

root = Path(__file__).resolve().parent
if shutil.which('npm') is None or shutil.which('node') is None:
    raise SystemExit('Install Node.js 22 or newer, including npm, then run make start again.')
major = int(subprocess.check_output(['node', '--version'], text=True).strip().lstrip('v').split('.')[0])
if major < 22:
    raise SystemExit('The docs require Node.js 22 or newer.')
stamp = root / 'node_modules/.docs-lock'
lock = root / 'package-lock.json'
if not stamp.exists() or stamp.read_bytes() != lock.read_bytes():
    subprocess.run(['npm', 'ci', '--ignore-scripts', '--no-audit', '--no-fund'], cwd=root, check=True)
    stamp.write_bytes(lock.read_bytes())
for name in ('mermaid', 'd3'):
    source = root / 'node_modules' / name / 'dist'
    destination = root / 'build/browser-assets' / name
    shutil.copytree(source, destination, dirs_exist_ok=True)
print('Mermaid and D3 are ready to serve locally.')
