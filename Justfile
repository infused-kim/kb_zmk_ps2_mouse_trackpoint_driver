# Dev recipes for this module. The checks themselves are defined once in
# .pre-commit-config.yaml and run through prek, on commit and here.

_default:
    @just --list

# Run all checks on all files. Hooks that fix or format files change them.
check:
    mise exec -- prek run --all-files

# Run only the Python script checks (ruff and ty) on all files
check-scripts:
    mise exec -- prek run --all-files ruff-check ruff-format ty
