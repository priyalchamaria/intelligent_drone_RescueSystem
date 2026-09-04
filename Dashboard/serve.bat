@echo off
rem Serves this folder so the dashboard can fetch data/state.json.
rem Browsers block fetch against file:// URLs, so opening index.html directly will not work.
cd /d "%~dp0"
echo Dashboard at http://localhost:8000  (Ctrl+C to stop)
python -m http.server 8000
