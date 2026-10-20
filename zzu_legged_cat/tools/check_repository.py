"""Check repository documentation and simulator-independent import boundaries."""

import ast
from pathlib import Path
import re
from urllib.parse import unquote, urlsplit


ROOT = Path(__file__).resolve().parents[1]
FORBIDDEN = {'mujoco', 'mjbatch', 'gymnasium', 'stable_baselines3'}
LINK = re.compile(r'\[[^\]\n]*\]\(([^\s)]+)\)')


def check_imports():
    package = ROOT / 'scripts/wheel_legged'
    sources = [*package.joinpath('core').glob('*.py'),
               *package.joinpath('policies').glob('*.py'),
               package / 'interfaces.py', package / 'runtime.py']
    errors = []
    for source in sources:
        for node in ast.walk(ast.parse(source.read_text(), filename=str(source))):
            names = []
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or '']
                if node.level and (node.module or '').split('.')[0] == 'backends':
                    errors.append(f'{source.relative_to(ROOT)}:{node.lineno}: backend import')
            for name in names:
                if name.split('.')[0] in FORBIDDEN or '.backends' in name:
                    errors.append(f'{source.relative_to(ROOT)}:{node.lineno}: forbidden import {name}')
    return errors, len(sources)


def check_links():
    documents = [ROOT / 'README.md', *ROOT.joinpath('docs').rglob('*.md'),
                 *ROOT.joinpath('control').rglob('README.md'),
                 *ROOT.joinpath('scripts').rglob('README.md')]
    errors, checked, local_artifacts = [], 0, 0
    for source in documents:
        in_code = False
        for lineno, line in enumerate(source.read_text().splitlines(), 1):
            if line.lstrip().startswith(('```', '~~~')):
                in_code = not in_code
                continue
            if in_code:
                continue
            for match in LINK.finditer(line):
                target = match.group(1).strip('<>')
                parsed = urlsplit(target)
                if parsed.scheme or target.startswith(('/', '#')):
                    continue
                resolved = (source.parent / unquote(parsed.path)).resolve()
                if resolved.is_relative_to(ROOT / 'logs'):
                    local_artifacts += 1
                    continue
                checked += 1
                if not resolved.exists():
                    errors.append(f'{source.relative_to(ROOT)}:{lineno}: missing {target}')
    # All archived Markdown records must be discoverable from the shared index.
    archive = ROOT / 'docs/logs'
    index = (archive / 'README.md').read_text()
    for record in archive.glob('*.md'):
        if record.name != 'README.md' and f']({record.name})' not in index:
            errors.append(f'docs/logs/README.md: missing index entry for {record.name}')
    return errors, checked, local_artifacts


def main():
    import_errors, sources = check_imports()
    link_errors, links, artifacts = check_links()
    errors = import_errors + link_errors
    for error in errors:
        print(error)
    print(f'Checked {sources} portable Python modules and {links} repository links; '
          f'{artifacts} optional local artifact references; {len(errors)} errors.')
    return bool(errors)


if __name__ == '__main__':
    raise SystemExit(main())
