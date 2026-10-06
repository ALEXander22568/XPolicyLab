"""Download an explicitly selected release; no default/private URL is assumed."""
import argparse
import hashlib
import json
import os
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--revision", required=True, help="Immutable Hugging Face revision")
    parser.add_argument("--destination", type=Path, required=True)
    args = parser.parse_args()
    for key in ("http_proxy", "https_proxy", "all_proxy", "HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY"):
        os.environ.pop(key, None)
    os.environ["TORCH_ALLOW_TF32_CUBLAS_OVERRIDE"] = "0"
    from huggingface_hub import snapshot_download
    files = ("model.pt", "config.yaml", "dataset_stats.json", "README.md")
    snapshot_download(repo_id=args.repo_id, revision=args.revision,
                      local_dir=str(args.destination), allow_patterns=[*files, "manifest.json", "SHA256SUMS"])
    manifest = json.loads((args.destination / "manifest.json").read_text())
    for name in files:
        digest = hashlib.sha256()
        with (args.destination / name).open("rb") as handle:
            for chunk in iter(lambda: handle.read(8 * 1024 * 1024), b""):
                digest.update(chunk)
        if digest.hexdigest() != manifest["sha256"][name]:
            raise ValueError(f"Checkpoint bundle checksum mismatch: {name}")
    print("Verified checkpoint bundle:", args.destination)


if __name__ == "__main__":
    main()
