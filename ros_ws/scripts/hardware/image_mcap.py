#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["mcap==1.6.0", "zstandard==0.25.0"]
# ///
"""Pack, verify, or restore an image using indexed MCAP attachments.

Run from the monorepo root; uv installs the declared dependencies automatically:
  uv run ros_ws/scripts/hardware/image_mcap.py pack image.img backup.mcap
  uv run ros_ws/scripts/hardware/image_mcap.py verify backup.mcap
  uv run ros_ws/scripts/hardware/image_mcap.py restore backup.mcap restored.img
"""

import argparse
import hashlib
import json
import os
import time
from pathlib import Path

import zstandard as zstd
from mcap.reader import make_reader
from mcap.records import Attachment
from mcap.stream_reader import StreamReader
from mcap.writer import Writer

MANIFEST = "disk-image-manifest.json"
FORMAT = "disk-image-mcap-v1"


def json_bytes(value):
    return json.dumps(value, sort_keys=True, indent=2).encode("utf-8")


def publish_file(partial, destination):
    # Linking publishes atomically without ever replacing an existing file.
    os.link(partial, destination)
    partial.unlink()


class PartsSink:
    def __init__(self, writer, part_bytes, timestamp):
        self.writer = writer
        self.part_bytes = part_bytes
        self.timestamp = timestamp
        self.buffer = bytearray()
        self.parts = []

    def write(self, data):
        view = memoryview(data)
        offset = 0
        while offset < len(view):
            count = min(self.part_bytes - len(self.buffer), len(view) - offset)
            self.buffer.extend(view[offset : offset + count])
            offset += count
            if len(self.buffer) == self.part_bytes:
                self.flush_part()
        return len(data)

    def flush(self):
        pass

    def flush_part(self):
        if not self.buffer:
            return
        name = f"image.zst.part-{len(self.parts):06d}"
        data = bytes(self.buffer)
        self.buffer.clear()
        self.writer.add_attachment(
            create_time=self.timestamp,
            log_time=self.timestamp,
            name=name,
            media_type="application/octet-stream",
            data=data,
        )
        self.parts.append(
            {"name": name, "size": len(data), "sha256": hashlib.sha256(data).hexdigest()}
        )


def pack(source, destination, part_bytes):
    if not 65536 <= part_bytes <= 64 * 1024 * 1024:
        raise ValueError("Attachment size must be between 64 KiB and 64 MiB")
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        raise FileExistsError(destination)
    partial = destination.with_name(destination.name + ".partial")
    started = time.time_ns()
    original_hash = hashlib.sha256()
    count = 0
    with source.open("rb") as image, partial.open("xb") as output:
        before = os.fstat(image.fileno())
        writer = Writer(output, enable_data_crcs=True)
        writer.start()
        schema = writer.register_schema(
            name="DiskImageBackup",
            encoding="jsonschema",
            data=json_bytes({"type": "object", "additionalProperties": True}),
        )
        channel = writer.register_channel(
            topic="/backup/manifest",
            message_encoding="json",
            schema_id=schema,
        )
        writer.add_message(
            channel,
            started,
            json_bytes(
                {"state": "started", "filename": source.name, "original_size": before.st_size}
            ),
            started,
        )
        sink = PartsSink(writer, part_bytes, started)
        compressor = zstd.ZstdCompressor(level=3, threads=2, write_checksum=True)
        next_report = time.monotonic() + 15
        with compressor.stream_writer(sink, closefd=False) as compressed:
            while data := image.read(1024 * 1024):
                original_hash.update(data)
                count += len(data)
                compressed.write(data)
                if time.monotonic() >= next_report:
                    print(f"Packed {count / 1e9:.2f} / {before.st_size / 1e9:.2f} GB", flush=True)
                    next_report = time.monotonic() + 15
        sink.flush_part()
        after = os.fstat(image.fileno())
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            raise RuntimeError("Source changed while packaging")
        if count != before.st_size:
            raise RuntimeError("Source size changed while packaging")
        manifest = {
            "format": FORMAT,
            "filename": source.name,
            "original_size": count,
            "original_sha256": original_hash.hexdigest(),
            "source_mtime_ns": before.st_mtime_ns,
            "compression": "zstd",
            "compressed_size": sum(part["size"] for part in sink.parts),
            "parts": sink.parts,
            "restore": "python3 image_mcap.py restore BACKUP.mcap RESTORED.img",
        }
        finished = time.time_ns()
        writer.add_attachment(
            create_time=finished,
            log_time=finished,
            name=MANIFEST,
            media_type="application/json",
            data=json_bytes(manifest),
        )
        writer.add_attachment(
            create_time=finished,
            log_time=finished,
            name="image_mcap.py",
            media_type="text/x-python",
            data=Path(__file__).read_bytes(),
        )
        writer.add_metadata(
            "disk_image_backup",
            {
                "filename": source.name,
                "original_size": str(count),
                "original_sha256": original_hash.hexdigest(),
                "compression": "zstd",
                "parts": str(len(sink.parts)),
                "format": FORMAT,
            },
        )
        writer.add_message(
            channel, finished, json_bytes({"state": "complete", **manifest}), finished
        )
        writer.finish()
        output.flush()
        os.fsync(output.fileno())
    publish_file(partial, destination)
    destination.with_suffix(".manifest.json").write_bytes(json_bytes(manifest) + b"\n")
    print(
        f"Created {destination}: {destination.stat().st_size:,} bytes, "
        f"{len(sink.parts)} image parts",
        flush=True,
    )
    print(f"Original SHA-256: {manifest['original_sha256']}", flush=True)


class HashSink:
    def __init__(self, output, limit):
        self.output = output
        self.limit = limit
        self.size = 0
        self.digest = hashlib.sha256()

    def write(self, data):
        if self.size + len(data) > self.limit:
            raise ValueError("Decompressed image exceeds the manifest size")
        if self.output is not None:
            self.output.write(data)
        self.size += len(data)
        self.digest.update(data)
        return len(data)

    def flush(self):
        if self.output is not None:
            self.output.flush()


def read_attachment(stream, index):
    if index.length > 65 * 1024 * 1024:
        raise ValueError("Unexpectedly large attachment")
    stream.seek(index.offset)
    record = next(StreamReader(stream, skip_magic=True, record_size_limit=65 * 1024 * 1024).records)
    if not isinstance(record, Attachment) or record.name != index.name:
        raise ValueError("Attachment does not match its index")
    if len(record.data) != index.data_size:
        raise ValueError("Attachment size does not match its index")
    return record.data


def verify_or_restore(source, destination=None):
    partial = None
    output = None
    if destination is not None:
        if destination.exists():
            raise FileExistsError(destination)
        partial = destination.with_name(destination.name + ".partial")
        output = partial.open("xb")
    try:
        with source.open("rb") as stream:
            summary = make_reader(stream).get_summary()
            if summary is None:
                raise ValueError("An indexed MCAP is required")
            indexes = {}
            for index in summary.attachment_indexes:
                if index.name in indexes:
                    raise ValueError("Duplicate attachment name")
                indexes[index.name] = index
            manifest = json.loads(read_attachment(stream, indexes[MANIFEST]))
            if manifest["format"] != FORMAT or manifest["compression"] != "zstd":
                raise ValueError("Unrecognized image backup format")
            sink = HashSink(output, manifest["original_size"])
            compressed_size = 0
            with zstd.ZstdDecompressor().stream_writer(sink, closefd=False) as decompressed:
                for position, part in enumerate(manifest["parts"]):
                    if part["name"] != f"image.zst.part-{position:06d}":
                        raise ValueError("Unexpected part order in manifest")
                    data = read_attachment(stream, indexes[part["name"]])
                    if (
                        len(data) != part["size"]
                        or hashlib.sha256(data).hexdigest() != part["sha256"]
                    ):
                        raise ValueError(f"Corrupt attachment: {part['name']}")
                    compressed_size += len(data)
                    decompressed.write(data)
            if compressed_size != manifest["compressed_size"]:
                raise ValueError("Compressed size mismatch")
            if (
                sink.size != manifest["original_size"]
                or sink.digest.hexdigest() != manifest["original_sha256"]
            ):
                raise ValueError("Restored image size or SHA-256 mismatch")
            if output is not None:
                output.flush()
                os.fsync(output.fileno())
                output.close()
                output = None
                publish_file(partial, destination)
            print(
                f"Verified {sink.size:,} original bytes; SHA-256 {sink.digest.hexdigest()}",
                flush=True,
            )
            if destination is not None:
                print(f"Restored {destination}", flush=True)
            return manifest
    finally:
        if output is not None:
            output.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    packing = sub.add_parser("pack")
    packing.add_argument("source", type=Path)
    packing.add_argument("destination", type=Path)
    packing.add_argument("--part-bytes", type=int, default=64 * 1024 * 1024)
    checking = sub.add_parser("verify")
    checking.add_argument("source", type=Path)
    restoring = sub.add_parser("restore")
    restoring.add_argument("source", type=Path)
    restoring.add_argument("destination", type=Path)
    args = parser.parse_args()
    if args.command == "pack":
        pack(args.source, args.destination, args.part_bytes)
    else:
        verify_or_restore(args.source, getattr(args, "destination", None))


if __name__ == "__main__":
    main()
