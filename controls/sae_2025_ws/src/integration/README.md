# PennAiR Auton Deploy

React + FastAPI dashboard for Pi connectivity, WiFi control, and build deploy/rollback.

## Structure

```
integration/
├── app.py              # FastAPI backend
├── frontend/           # React frontend
│   ├── src/
│   │   ├── App.jsx     # Main component
│   │   ├── App.css     # Styles
│   │   ├── main.jsx    # Entry point
│   │   └── index.css   # Global styles
│   ├── index.html
│   ├── vite.config.js
│   └── package.json
```

## Setup & Run

### 1. Install Dependencies (first time)

```bash
cd /Users/ethanyu/VSCodeProjects/monorepo/controls/sae_2025_ws/src/integration/frontend
npm install
```

### 2. Launch Backend + Frontend Together

```bash
cd /Users/ethanyu/VSCodeProjects/monorepo/controls/sae_2025_ws/src/integration
./launch.sh
```

This starts:
- Backend API on `http://localhost:8080`
- Frontend dev server on `http://localhost:3000`

Stop both with `Ctrl+C` in the terminal running `launch.sh`.

### Conda Workflow (optional)

If you use conda for backend dependencies, activate your env first:

```bash
conda activate <your-env>
pip install "fastapi[standard]" python-multipart httpx
cd /Users/ethanyu/VSCodeProjects/monorepo/controls/sae_2025_ws/src/integration
./launch.sh
```

When a conda env is active, `launch.sh` uses `python app.py` for the backend.
If no conda env is active, it uses `uv run app.py`.

## Building for Production

```bash
cd frontend
npm run build
```

This creates a `dist/` folder with optimized static files you can serve with any web server.
