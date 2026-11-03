#!/usr/bin/env python3
"""Replay project compile commands and retain auditable clang-tidy diagnostics."""
import argparse
import concurrent.futures
import hashlib
import json
import pathlib
import re
import shlex
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('build', type=pathlib.Path)
    parser.add_argument('--output', required=True, type=pathlib.Path)
    parser.add_argument('--jobs', type=int, default=2)
    parser.add_argument('--file', action='append', help='Restrict to repository-relative source (repeatable)')
    args = parser.parse_args()
    root = pathlib.Path(__file__).resolve().parents[1]
    build = args.build.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    commands = json.loads((build / 'compile_commands.json').read_text())
    selected = []
    for entry in commands:
        source = pathlib.Path(entry['file']).resolve()
        if any(source.is_relative_to(root / part) for part in ('tests', 'examples')) or source.is_relative_to(build / 'header_tests'):
            if not args.file or str(source.relative_to(root)) in args.file:
                selected.append(entry)
    if (output / 'results.json').exists():
        parser.error('output already contains a completed scan; choose a new output directory')
    (output / 'commands.json').write_text(json.dumps(selected, indent=2) + '\n')
    (output / 'policy.yaml').write_bytes((root / '.clang-tidy').read_bytes())
    subprocess.run(['clang-tidy', '--version'], stdout=(output / 'version.txt').open('w'), check=True)

    def scan(item):
        index, entry = item
        original = entry.get('arguments') or shlex.split(entry['command'])
        flags = []
        skip = False
        for flag in original[1:]:
            if skip:
                skip = False
            elif flag == '-o':
                skip = True
            elif flag != '-c' and flag != entry['file']:
                flags.append(flag)
        prefix = output / f'{index:04d}'
        command = ['clang-tidy', entry['file'], '--quiet',
                   f'--config-file={root / ".clang-tidy"}',
                   f'--header-filter=^{re.escape(str(root))}/include/trackingLib/',
                   f'--export-fixes={prefix}.yaml', '--', *flags]
        result = subprocess.run(command, cwd=entry['directory'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        prefix.with_suffix('.log').write_text(result.stdout)
        diagnostics = []
        for match in re.finditer(r'^(.+?):(\d+):(\d+): (warning|error|fatal error): (.*?) \[([^\]]+)\]$', result.stdout, re.M):
            file, line, column, severity, message, check = match.groups()
            path = pathlib.Path(file)
            location = str(path.relative_to(root)) if path.is_relative_to(root) else file
            key = [location, int(line), int(column), check, message]
            diagnostics.append(dict(id=hashlib.sha256(json.dumps(key).encode()).hexdigest()[:12], file=location,
                                    line=int(line), column=int(column), check=check, message=message, severity=severity))
        return dict(index=index, file=entry['file'], command=command, directory=entry['directory'],
                    exit_code=result.returncode, compiler_failure='[clang-diagnostic-error]' in result.stdout or 'fatal error:' in result.stdout,
                    diagnostics=diagnostics)

    # Target output paths do not affect analysis. Keep all commands in the manifest,
    # but analyze identical source/flags/directory combinations once.
    representatives = {}
    aliases = {}
    for index, entry in enumerate(selected):
        command = entry.get('arguments') or shlex.split(entry['command'])
        semantic = []
        skip = False
        for flag in command:
            if skip:
                skip = False
            elif flag == '-o':
                skip = True
            else:
                semantic.append(flag)
        key = (entry['directory'], tuple(semantic))
        aliases[index] = representatives.setdefault(key, index)
    scheduled = sorted(set(aliases.values()))
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        completed = list(pool.map(scan, ((i, selected[i]) for i in scheduled)))
    by_index = {r['index']: r for r in completed}
    results = []
    for index, representative in aliases.items():
        result = dict(by_index[representative], index=index, analysis_index=representative)
        results.append(result)
    (output / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
    unique = {}
    for result in results:
        for diagnostic in result['diagnostics']:
            finding = unique.setdefault(diagnostic['id'], {**diagnostic, 'occurrences': 0, 'invocations': []})
            finding['occurrences'] += 1
            finding['invocations'].append(result['index'])
    (output / 'findings.json').write_text(json.dumps(list(unique.values()), indent=2) + '\n')
    print(json.dumps(dict(invocations=len(results), nonzero=sum(r['exit_code'] != 0 for r in results),
                          compiler_failures=sum(r['compiler_failure'] for r in results), unique=len(unique),
                          occurrences=sum(f['occurrences'] for f in unique.values()))))
    return int(any(r['exit_code'] != 0 for r in results))


if __name__ == '__main__':
    raise SystemExit(main())
