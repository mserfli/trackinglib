#!/usr/bin/env python3
"""Aggregate completed clang-tidy inventories without discarding provenance."""
import argparse
import collections
import hashlib
import json
import os
import pathlib
import re


def triage(check, file):
    if check.startswith(('bugprone-', 'clang-diagnostic-')):
        return ('Correctness and safety', 'First', 'Inspect control flow and fix the cause; add regression coverage for behavior changes', 'Review lifetime, exception paths, and numerical behavior')
    if check.startswith('portability-'):
        return ('Portability and numeric behavior', 'First', 'Use portable C++17 expressions preserving numerical types', 'Review precision and representation')
    if check.startswith('performance-'):
        return ('Performance', 'Second', 'Remove avoidable work; preserve public enum representation where ABI-sensitive', 'Review ownership, lifetimes, and public representation')
    if file.startswith(('tests/', 'examples/', 'generated/')):
        return ('Test/example-specific', 'Third', 'Apply local conventions; document intentional test idioms', 'Preserve assertions and demonstration behavior')
    if check.startswith(('modernize-', 'misc-const-')):
        return ('API and modernization', 'Second', 'Use equivalent C++17 idiom while preserving signatures and semantics', 'Review overloads, special members, and constness')
    return ('Readability and conventions', 'Third', 'Simplify locally or justify a narrow exception', 'Preserve evaluation order and algorithm structure')


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inventories', nargs='+', type=pathlib.Path)
    parser.add_argument('--output', required=True, type=pathlib.Path)
    args = parser.parse_args()
    findings = {}
    scans = []
    for inventory in args.inventories:
        results = json.loads((inventory / 'results.json').read_text())
        configuration = str(inventory.parent.parent)
        scans.append(dict(configuration=configuration, invocations=len(results),
                          nonzero=sum(r['exit_code'] != 0 for r in results),
                          failed=sum(r['compiler_failure'] or r['exit_code'] < 0 or (r['exit_code'] and not r['diagnostics']) for r in results)))
        for invocation in results:
            for diagnostic in invocation['diagnostics']:
                check = diagnostic['check'].replace(',-warnings-as-errors', '')
                file = re.sub(r'^build_tidy[^/]*/header_tests/', 'generated/header_tests/', diagnostic['file'])
                key = [file, diagnostic['line'], diagnostic['column'], check, diagnostic['message']]
                identifier = 'TIDY-' + hashlib.sha256(json.dumps(key).encode()).hexdigest()[:12]
                category, priority, fix, risk = triage(check, file)
                finding = findings.setdefault(identifier, dict(id=identifier, file=file, line=diagnostic['line'],
                    column=diagnostic['column'], check=check, message=diagnostic['message'], category=category,
                    priority=priority, proposed_fix=fix, risk=risk, validation='Affected clang-tidy invocations, build/test matrix and standalone headers',
                    disposition='open', completion_evidence=[], occurrences=0, configurations=[], provenance=[]))
                finding['occurrences'] += 1
                if configuration not in finding['configurations']:
                    finding['configurations'].append(configuration)
                finding['provenance'].append(dict(configuration=configuration, invocation=invocation['index'],
                    analysis_index=invocation.get('analysis_index', invocation['index']),
                    raw_log=str(inventory / (f"{invocation.get('analysis_index', invocation['index']):04d}.log"))))
    args.output.mkdir(parents=True, exist_ok=True)
    ordered = sorted(findings.values(), key=lambda f: (f['priority'], f['file'], f['line'], f['check'], f['message']))
    (args.output / 'findings.json').write_text(json.dumps(ordered, indent=2) + '\n')
    counts = dict(unique=len(ordered), occurrences=sum(f['occurrences'] for f in ordered), scans=scans)
    for group in ('check', 'category'):
        counts[group] = dict(collections.Counter(f[group] for f in ordered).most_common())
    counts['subsystem'] = dict(collections.Counter('/'.join(f['file'].split('/')[:3]) if f['file'].startswith('include/') else f['file'].split('/')[0] for f in ordered).most_common())
    (args.output / 'summary.json').write_text(json.dumps(counts, indent=2) + '\n')
    lines = ['# clang-tidy baseline issue inventory', '', f"{counts['unique']} unique findings; {counts['occurrences']} occurrences across {len(scans)} configurations.", '',
             'Full messages, risk assessments, proposed fixes, validation and invocation provenance are retained in [findings.json](findings.json). Raw logs and exported YAML remain under the analysis directories.', '',
             '| ID | Location | Check | Priority | Disposition |', '| --- | --- | --- | --- | --- |']
    for f in ordered:
        target = os.path.relpath(root / f['file'], args.output.resolve())
        location = f"[{f['file']}:{f['line']}]({target}#L{f['line']})" if not f['file'].startswith('generated/') else f"`{f['file']}:{f['line']}`"
        lines.append(f"| {f['id']} | {location} | {f['check']} | {f['priority']} | {f['disposition']} |")
    (args.output / 'inventory.md').write_text('\n'.join(lines) + '\n')
    print(json.dumps(counts, indent=2))


if __name__ == '__main__':
    main()
