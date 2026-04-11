# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "fastapi[standard]",
#     "python-multipart",
#     "httpx",
#     "zeroconf",
# ]
# ///

from pathlib import Path
import sys

BASE_DIR = Path(__file__).resolve().parent
for package_root in (BASE_DIR.parent / "uav", BASE_DIR.parent / "sim"):
    package_root_str = str(package_root)
    if package_root_str not in sys.path:
        sys.path.insert(0, package_root_str)

if __package__:
    from .backend import create_app
else:
    from backend import create_app

app = create_app(BASE_DIR)


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8080)
