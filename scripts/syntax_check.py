#!/usr/bin/env python3
"""
ROS 2 パッケージの Python/YAML ファイル構文チェック
"""

import ast
import sys
from pathlib import Path

import yaml


def check_python(base: Path) -> list[str]:
    """Python ファイルの構文チェック"""
    errors = []
    for f in sorted(base.rglob('*.py')):
        try:
            ast.parse(f.read_text())
        except SyntaxError as e:
            errors.append(f'{f}: {e}')
    return errors


def check_yaml(base: Path) -> list[str]:
    """YAML ファイルの構文チェック"""
    errors = []
    for f in sorted(base.rglob('*.yaml')):
        try:
            yaml.safe_load(f.read_text())
        except yaml.YAMLError as e:
            errors.append(f'{f}: {e}')
    return errors


def main():
    base = Path(__file__).resolve().parent.parent / 'weeder_ws' / 'src' / 'grass_chopper'

    print(f'=== 構文チェック: {base} ===')

    py_errors = check_python(base)
    yaml_errors = check_yaml(base)

    if py_errors:
        print('\n--- Python 構文エラー ---')
        for e in py_errors:
            print(f'  ✗ {e}')

    if yaml_errors:
        print('\n--- YAML 構文エラー ---')
        for e in yaml_errors:
            print(f'  ✗ {e}')

    total = len(py_errors) + len(yaml_errors)
    if total == 0:
        py_count = len(list(base.rglob('*.py')))
        yaml_count = len(list(base.rglob('*.yaml')))
        print(f'  ✓ Python: {py_count} ファイル OK')
        print(f'  ✓ YAML: {yaml_count} ファイル OK')
        return 0

    print(f'\n合計 {total} 件のエラー')
    return 1


if __name__ == '__main__':
    sys.exit(main())
