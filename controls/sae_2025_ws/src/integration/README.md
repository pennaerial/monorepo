# Launch Interface

Simple React + FastAPI web interface for selecting git commits and running bash scripts.

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

### 1. Start the Backend (Terminal 1)

```bash
cd /Users/matthewkuo/Desktop/monorepo/controls/sae_2025_ws/src/integration

# Install Python dependencies
pip install fastapi uvicorn

# Run the API server
python app.py
```

Backend will run on http://localhost:8000

### 2. Start the Frontend (Terminal 2)

```bash
cd /Users/matthewkuo/Desktop/monorepo/controls/sae_2025_ws/src/integration/frontend

# Install dependencies (first time only)
npm install

# Run the dev server
npm run dev
```

Frontend will run on http://localhost:3000

### 3. Open Browser

Go to http://localhost:3000

## Configure Your Launch Script

Edit `app.py` line 48 to point to your actual bash script:

```python
script_path = os.path.join(REPO_PATH, "launch.sh")
```

Your script will receive:
- First argument: commit hash (e.g., `a1b2c3d`)
- Remaining arguments: space-separated parameters from the input field

Example:
```bash
./launch.sh a1b2c3d --config=dev --verbose --mode=test
```

## Building for Production

```bash
cd frontend
npm run build
```

This creates a `dist/` folder with optimized static files you can serve with any web server.
