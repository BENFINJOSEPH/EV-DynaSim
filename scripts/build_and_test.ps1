# Build the C++ engine, run all tests and regenerate every result.
# Run from the repository root in "Developer PowerShell for VS 2022":
#     .\scripts\build_and_test.ps1
$ErrorActionPreference = "Stop"

Write-Host "== C++: configure, build, unit tests =="
cmake -S cpp -B build
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure

Write-Host "== Python: lint, tests (includes C++ vs Python comparison) =="
.\.venv\Scripts\python.exe -m ruff check .
.\.venv\Scripts\python.exe -m pytest

Write-Host "== Results and figures =="
.\.venv\Scripts\evdynasim.exe all --config configs/generic_ev.yaml --out results
Write-Host "Done. Open results\ and results\validation\*\report.md"
