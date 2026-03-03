.PHONY: check-syntax

## 構文チェック: Python + YAML
check-syntax:
	python3 scripts/syntax_check.py
