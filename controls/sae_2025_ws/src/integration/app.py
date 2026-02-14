# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "fastapi[standard]",
#     "python-multipart",
#     "httpx",
# ]
# ///

from pathlib import Path

if __package__:
    from .backend import create_app
else:
    from backend import create_app

app = create_app(Path(__file__).parent)


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8080)
